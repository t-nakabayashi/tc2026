"""時刻順にGNSS/LIOを融合し、既存走行スタックへENU poseを配信する."""
from collections import deque
import copy
from dataclasses import fields
import json
import math
import time
from pathlib import Path

import numpy as np

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Joy
from std_msgs.msg import Bool
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from .gnss_dropout import GnssDropoutHold
from rtk_gps_um982_msgs.msg import RtkStatus
from tc_diagnostics import ASPECT_QUALITY, ERROR, OK, WARN, DiagnosticReporter, report_alive
from tc_geo_msgs.msg import FusionState

from geo_pose_converter.geo_core import LlhPoint, ProjectionConfig, llh_to_enu
from .mount_core import base_from_sensor, horizontal_lever
from .baseline_core import BaselineConfig
from .motion_guard_core import MotionGuard
from .fusion_core import FusionConfig, FusionFilter, wrap


def seconds(stamp) -> float:
    """ROS時刻を秒へ変換する."""
    return stamp.sec+stamp.nanosec*1e-9


class FusionNode(Node):
    """真値・建物配置を購読せず、受信機の品質情報だけを用いる."""

    def __init__(self) -> None:
        super().__init__('gnss_lio_fusion')
        defaults = dict(origin_latitude=0., origin_longitude=0., origin_altitude=0.,
                        map_yaw_offset_rad=0., buffer_s=.35, output_log='', baseline_cache='',
                        master_height_m=.7, lio_height_m=.6, gnss_heading_offset_deg=0.,
                        lio_yaw_offset_deg=0., wheel_fallback_enabled=True,
                        lio_forward_m=0., lio_left_m=0., lio_mount_roll_deg=0.,
                        lio_mount_pitch_deg=0., lio_mount_yaw_deg=0.,
                        master_forward_m=0., master_left_m=0., publish_base_tf=False,
                        require_gravity_alignment=False)
        self.values = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        if not all(math.isfinite(self.values[k]) for k in [
                'master_height_m', 'lio_height_m', 'gnss_heading_offset_deg', 'lio_yaw_offset_deg',
                'lio_forward_m', 'lio_left_m', 'lio_mount_roll_deg', 'lio_mount_pitch_deg',
                'lio_mount_yaw_deg', 'master_forward_m', 'master_left_m']):
            raise ValueError('取付位置・方位補正は有限値が必要')
        self.projection = ProjectionConfig(**{k: self.values[k] for k in
            ['origin_latitude', 'origin_longitude', 'origin_altitude', 'map_yaw_offset_rad']})
        def config(cls, prefix: str):
            return cls(**{f.name: self.declare_parameter(prefix+f.name,
                          getattr(cls(), f.name)).value for f in fields(cls)})
        baseline = config(BaselineConfig, 'baseline.')
        self.cache = Path(self.values['baseline_cache']) if self.values['baseline_cache'] else None
        if self.cache and self.cache.exists():
            try:
                value = float(json.loads(self.cache.read_text())['reference_m'])
                if baseline.minimum_m < value < baseline.maximum_m:
                    baseline.initial_m = value
            except (ValueError, KeyError, OSError):
                self.get_logger().warn('baseline保存値を読めないため既定の参考値を使用する')
        self.filter = FusionFilter(config(FusionConfig, ''), baseline)
        self.alignment_ready = False
        self.alignment_received = -math.inf
        self.create_subscription(Bool, '/lio/alignment_ready', self.on_alignment, 10)
        if not 0 < self.values['buffer_s'] <= 2.:
            raise ValueError('buffer_sは0より大きく2秒以下とする')
        self.lios = deque()
        self.wheels = deque()
        self.motion = MotionGuard()
        self.events = []
        self.statuses = {}
        self.watermark = -math.inf
        self.last_output = -math.inf
        self.last_mode = None
        self.late_events = 0
        self.unsynced_gps = 0
        self.log = Path(self.values['output_log']).open('w') if self.values['output_log'] else None
        self.input_log = (Path(self.values['output_log']).with_suffix('.inputs.jsonl').open('w')
                          if self.values['output_log'] else None)
        self.tf_broadcaster = None
        if self.values['publish_base_tf']:
            from tf2_ros import TransformBroadcaster
            self.tf_broadcaster = TransformBroadcaster(self)
        # 配信様式: stream (50 Hz, tick 周期)
        # QoS 例外: RELIABLE / depth 10。下流の制御入力であり取りこぼしを避ける。
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/localization/pose_enu', 10)
        self.health_pub = self.create_publisher(FusionState, '/fusion/status', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel/fusion_limited', 10)
        self.create_subscription(Odometry, '/lio/odometry', self.on_lio, 30)
        self.create_subscription(Odometry, '/ypspur_ros/odom', self.on_wheel, 30)
        self.gnss_dropout = GnssDropoutHold(
            self.declare_parameter('gnss_dropout.button_index', 5).value,
            self.declare_parameter('gnss_dropout.joy_timeout_s', .5).value)
        self.gnss_dropout_active = False
        # 配信様式: stream (10 Hz, dropout_tick 周期)
        self.dropout_pub = self.create_publisher(Bool, '/fusion/gnss_dropout_active', 1)
        self.create_subscription(Joy, '/joy', self.on_dropout_joy, qos_profile_sensor_data)
        self.dropout_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.1, self.dropout_tick, clock=self.dropout_clock)
        self.create_subscription(NavSatFix, '/rtk_gps/fix', self.on_fix, 30)
        self.create_subscription(RtkStatus, '/rtk_gps/rtk_status', self.on_status, 30)
        self.create_subscription(Twist, '/cmd_vel/autonomous', self.on_command, 10)
        self.create_timer(.02, self.tick)
        self.diagnostics = DiagnosticReporter(self)
        report_alive(self.diagnostics, '融合処理を実行中')
        self.get_logger().info('適応baseline・GNSS/LIO融合を開始する')

    def on_alignment(self, msg):
        self.alignment_received = time.monotonic()
        if self.values['require_gravity_alignment'] and not msg.data and self.alignment_ready:
            self.reset_alignment_state()
        self.alignment_ready = msg.data

    def reset_alignment_state(self):
        self.filter = FusionFilter(self.filter.config, self.filter.baseline.config)
        self.motion = MotionGuard()
        self.lios.clear()
        self.wheels.clear()
        self.events.clear()
        self.statuses.clear()
        self.watermark = self.last_output = -math.inf
        self.last_mode = None

    def alignment_available(self):
        if not self.values.get('require_gravity_alignment', False):
            return True
        if time.monotonic()-self.alignment_received > .7:
            if self.alignment_ready:
                self.reset_alignment_state()
                self.alignment_ready = False
            return False
        return self.alignment_ready

    def refresh_dropout(self):
        active = self.gnss_dropout.active(time.monotonic())
        if active != self.gnss_dropout_active:
            self.gnss_dropout_active = active
            self.events = [event for event in self.events if event[1] != 'gps']
            self.statuses.clear()
            self.get_logger().warning('GNSS途絶模擬: ' + ('開始（R1押下中）' if active else '解除'))
        return active

    def on_dropout_joy(self, msg):
        self.gnss_dropout.update(msg.buttons, time.monotonic())
        self.dropout_tick()

    def dropout_tick(self):
        self.dropout_pub.publish(Bool(data=self.refresh_dropout()))

    def on_status(self, msg: RtkStatus) -> None:
        if not self.alignment_available() or self.refresh_dropout():
            return
        self.statuses[seconds(msg.header.stamp)] = msg
        while len(self.statuses) > 100:
            del self.statuses[min(self.statuses)]

    def on_fix(self, msg: NavSatFix) -> None:
        if not self.alignment_available() or self.refresh_dropout():
            return
        self.events.append((seconds(msg.header.stamp), 'gps', msg))

    def on_wheel(self, msg: Odometry) -> None:
        """車輪の相対運動を保持する。Gazebo真値poseを代用しない."""
        t = seconds(msg.header.stamp)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        quaternion = np.array([q.x, q.y, q.z, q.w])
        norm = np.linalg.norm(quaternion)
        if not np.isfinite(quaternion).all() or not .5 <= norm <= 1.5:
            return
        x, y, z, w = quaternion/norm
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        pose = np.array([p.x, p.y, yaw])
        if not np.isfinite(pose).all() or (self.wheels and t <= self.wheels[-1][0]):
            return
        self.wheels.append((t, pose))
        while len(self.wheels) > 1 and t-self.wheels[0][0] > 5.:
            self.wheels.popleft()
        self.events.append((t, 'wheel', pose))
        self.record_input(dict(sim_s=t, kind='wheel', pose=pose.tolist()))

    def wheel_at(self, stamp: float):
        """車輪も測定時刻に内挿し、異なる時刻の運動を比較しない."""
        if not self.values['wheel_fallback_enabled']:
            return None
        for a, b in zip(self.wheels, list(self.wheels)[1:]):
            if a[0] <= stamp <= b[0] and b[0]-a[0] <= .2:
                ratio = (stamp-a[0])/(b[0]-a[0])
                pose = a[1]+ratio*(b[1]-a[1])
                pose[2] = wrap(a[1][2]+ratio*wrap(b[1][2]-a[1][2]))
                return pose
        return None

    def predict_motion(self, stamp: float, lio, wheel) -> None:
        """選択した相対運動と元のLIOを別に記録し、比較時に混同させない."""
        if not self.values['wheel_fallback_enabled']:
            if lio is not None:
                self.filter.advance(stamp, lio)
            return
        motion = self.motion.select(stamp, lio, wheel)
        if motion is not None:
            self.filter.advance(stamp, motion)
            if self.motion.source == 'WHEEL_FALLBACK':
                self.filter.heading_recovery = True
            self.record_input(dict(sim_s=stamp, kind='motion', pose=motion.tolist(),
                                   source=self.motion.source))

    def on_lio(self, msg: Odometry) -> None:
        if self.values.get('require_gravity_alignment', False) and (not self.alignment_available() or msg.header.frame_id != 'lio_level'):
            return
        t = seconds(msg.header.stamp)
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        quat = np.array([q.x, q.y, q.z, q.w])
        norm = np.linalg.norm(quat)
        if not np.isfinite(quat).all() or norm < .5 or norm > 1.5:
            self.filter.lio_ok = False
            self.filter.rejected_lio += 1
            return
        position, (roll, pitch, yaw) = base_from_sensor(
            [p.x, p.y, p.z], quat,
            [self.values['lio_forward_m'], self.values['lio_left_m'], self.values['lio_height_m']],
            [math.radians(self.values['lio_mount_'+axis+'_deg']) for axis in ['roll', 'pitch', 'yaw']])
        yaw = wrap(yaw+math.radians(self.values['lio_yaw_offset_deg']))
        body = np.array([position[0], position[1], yaw])
        if not np.isfinite(body).all() or (self.lios and t <= self.lios[-1][0]):
            return
        self.lios.append((t, body, roll, pitch))
        while len(self.lios) > 1 and t-self.lios[0][0] > 5.:
            self.lios.popleft()
        self.events.append((t, 'lio', body))

    def lio_at(self, stamp: float):
        """GNSS観測時刻のLIO姿勢を内挿する。外挿で遅延を隠さない."""
        for a, b in zip(self.lios, list(self.lios)[1:]):
            if a[0] <= stamp <= b[0] and b[0]-a[0] <= self.filter.config.max_lio_gap_s:
                ratio = (stamp-a[0])/(b[0]-a[0])
                pose = a[1]+ratio*(b[1]-a[1])
                pose[2] = wrap(a[1][2]+ratio*wrap(b[1][2]-a[1][2]))
                return pose, a[2]+ratio*wrap(b[2]-a[2]), a[3]+ratio*wrap(b[3]-a[3])
        return None

    def tick(self) -> None:
        now = self.get_clock().now().nanoseconds*1e-9
        if not self.alignment_available():
            self.events.clear()
            if now-getattr(self, 'last_alignment_health', -math.inf) >= .5:
                self.last_alignment_health = now
                health = self.filter.diagnostics(now)
                health.update(mode='WAIT_GRAVITY_ALIGNMENT', speed_limit_mps=0., sim_s=now,
                              reason='静止して水平基準の確定を待ってください')
                self.publish_fusion_state(health)
            return
        cutoff = now-self.values['buffer_s']
        self.events.sort(key=lambda event: (event[0], event[1]))
        remaining = []
        for t, kind, msg in self.events:
            if t > cutoff:
                remaining.append((t, kind, msg))
                continue
            if t < self.watermark:
                self.late_events += 1
                continue
            self.watermark = t
            if kind == 'wheel':
                if self.values['wheel_fallback_enabled'] and (not self.lios or t-self.lios[-1][0] > .4):
                    self.predict_motion(t, None, msg)
                continue
            if kind == 'lio':
                self.predict_motion(t, msg, self.wheel_at(t))
                self.record_input(dict(sim_s=t, kind='lio', pose=msg.tolist()))
                continue
            status = self.statuses.get(t)
            sample = self.lio_at(t)
            attitude_missing = sample is None
            if sample is None and self.lios and self.wheel_at(t) is not None:
                sample = (None, 0., 0.)
            if status is None or sample is None:
                self.unsynced_gps += 1
                continue
            if not all(math.isfinite(v) for v in [msg.latitude, msg.longitude, msg.altitude,
                                                  status.heading_deg]):
                self.filter.rejected_gps += 1
                continue
            raw, roll, pitch = sample
            self.predict_motion(t, raw, self.wheel_at(t))
            point = llh_to_enu(LlhPoint(msg.latitude, msg.longitude, msg.altitude), self.projection)
            yaw = wrap(math.radians(90-status.heading_deg+self.values['gnss_heading_offset_deg'])-self.projection.map_yaw_offset_rad)
            lever = [self.values['master_forward_m'], self.values['master_left_m'],
                     self.values['master_height_m']]
            offset = horizontal_lever(lever, roll, pitch, yaw)
            measured = np.array([point.x-offset[0], point.y-offset[1], yaw])
            position_variance = max(msg.position_covariance[0], msg.position_covariance[4])
            position_variance += float(np.dot(lever, lever)) if attitude_missing else 0.
            self.filter.observe_gps(t, measured, status.rtk_state, status.num_satellites,
                                    status.baseline_length_m, position_variance,
                                    status.heading_stddev_deg)
            self.record_input(dict(sim_s=t, kind='gps', pose=measured.tolist(),
                                   state=status.rtk_state, satellites=status.num_satellites,
                                   baseline_m=status.baseline_length_m,
                                   position_variance=position_variance,
                                   heading_stddev_deg=status.heading_stddev_deg))
        self.events = remaining
        if self.filter.x is None:
            if now-getattr(self, 'last_initial_health', -math.inf) >= 1.:
                self.last_initial_health = now
                health = self.filter.diagnostics(now)
                health.update(mode='WAIT_INITIAL_FIX', yaw=None, heading_sigma_deg=None,
                              speed_limit_mps=0., sim_s=now)
                self.publish_fusion_state(health)
            return
        if not self.lios:
            return
        # 遅延バッファ内の最新LIOまで予測して、古い位置を現在位置として配信しない。
        latest_t, latest, _, _ = self.lios[-1]
        newest = max(latest_t, self.wheels[-1][0] if self.wheels and self.values['wheel_fallback_enabled'] else latest_t)
        if newest <= self.last_output or now-newest > self.filter.config.max_lio_gap_s:
            return
        predicted = copy.copy(self.filter)
        predicted.x, predicted.p = self.filter.x.copy(), self.filter.p.copy()
        if self.values['wheel_fallback_enabled']:
            motion = copy.deepcopy(self.motion)
            if now-latest_t > .4 and self.wheels:
                latest_t, wheel = self.wheels[-1]
                selected = motion.select(latest_t, None, wheel)
            else:
                selected = motion.select(latest_t, latest, self.wheel_at(latest_t))
            if selected is None:
                if motion.stamp != latest_t:
                    return
                selected = motion.pose
            if selected is None:
                return
            predicted.advance(latest_t, selected)
        else:
            predicted.advance(latest_t, latest)
        if latest_t <= self.last_output:
            return
        self.last_output = latest_t
        output = PoseWithCovarianceStamped()
        output.header.frame_id = self.projection.map_frame_id
        output.header.stamp.sec = int(latest_t)
        output.header.stamp.nanosec = int((latest_t-int(latest_t))*1e9)
        output.pose.pose.position.x, output.pose.pose.position.y = map(float, predicted.x[:2])
        output.pose.pose.orientation.z = math.sin(predicted.x[2]/2)
        output.pose.pose.orientation.w = math.cos(predicted.x[2]/2)
        for i, a in enumerate([0, 1, 5]):
            for j, b in enumerate([0, 1, 5]):
                output.pose.covariance[a*6+b] = float(predicted.p[i, j])
        for axis in [2, 3, 4]:
            output.pose.covariance[axis*6+axis] = 1e6
        self.pose_pub.publish(output)
        if self.tf_broadcaster is not None:
            from geometry_msgs.msg import TransformStamped
            transform = TransformStamped()
            transform.header = output.header
            transform.child_frame_id = 'base_link'
            transform.transform.translation.x = output.pose.pose.position.x
            transform.transform.translation.y = output.pose.pose.position.y
            transform.transform.rotation = output.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)
        health = predicted.diagnostics(latest_t)
        health.update(motion_source=self.motion.source,
                      wheel_fallback_count=self.motion.fallback_count,
                      motion_rejected_count=self.motion.rejected_count,
                      motion_window_rejections=self.motion.window_rejections, sim_s=latest_t, x=float(predicted.x[0]), y=float(predicted.x[1]),
                      yaw=float(predicted.x[2]), late_events=self.late_events,
                      unsynced_gps=self.unsynced_gps)
        if self.motion.source == 'WHEEL_FALLBACK':
            health['mode'] = 'GPS_WHEEL' if health['mode'] == 'GPS_LIO' else 'WHEEL_PRIORITY'
            health['speed_limit_mps'] = min(health['speed_limit_mps'], .4)
        self.publish_fusion_state(health)
        if health['mode'] != self.last_mode:
            self.get_logger().info('融合状態: '+health['mode'])
            self.last_mode = health['mode']
        if self.log:
            # 出力ログは全診断量のダンプとして残す。配信する FusionState は
            # 購読側が判断に用いる値だけに絞るため、両者の項目数は一致しない。
            self.log.write(json.dumps(health, ensure_ascii=False, allow_nan=False)+'\n')
            self.log.flush()

    def publish_fusion_state(self, health: dict) -> None:
        """内部診断dictから `tc_geo_msgs/FusionState` を組み立てて配信する.

        `health` は全診断量を含むが、配信するのは購読側が判断に用いる値だけに
        絞る。未確定の姿勢（MODE_WAIT_*）は `has_estimate=False` で表し、
        数値フィールドには既定値を入れる。

        Args:
            health (dict): `FusionFilter.diagnostics()` に本ノードの値を加えたdict.
        """

        message = FusionState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.projection.map_frame_id
        message.mode = str(health['mode'])
        yaw = health.get('yaw')
        sigma = health.get('heading_sigma_deg')
        message.has_estimate = yaw is not None and sigma is not None
        if message.has_estimate:
            message.yaw_rad = float(yaw)
            message.heading_sigma_deg = float(sigma)
            message.position_sigma_m = float(health.get('position_sigma_m', 0.))
        baseline = health.get('baseline') or {}
        message.baseline_reference_m = float(baseline.get('reference_m', 0.))
        message.baseline_sigma_m = float(baseline.get('sigma_m', 0.))
        message.baseline_ready = bool(baseline.get('ready', False))
        message.speed_limit_mps = float(health.get('speed_limit_mps', 0.))
        message.estimate_stamp_s = float(health.get('sim_s', 0.))
        message.reason = str(health.get('reason', ''))
        self.health_pub.publish(message)
        self.report_quality(message)

    # 融合モードごとの診断レベルと説明。外形（/fusion/status の受信）だけでは
    # 「出てはいるが縮退している」ことが分からないため、自己申告で補う。
    QUALITY_BY_MODE = {
        'GPS_LIO': (OK, 'GNSS と LIO で推定中'),
        'WAIT_GRAVITY_ALIGNMENT': (WARN, '水平基準の確定待ち'),
        'WAIT_INITIAL_FIX': (WARN, '初期 fix の待機中'),
        'LIO_PRIORITY': (WARN, 'GNSS 途絶のため LIO のみで推定中'),
        'GPS_WHEEL': (WARN, 'LIO 途絶のため車輪オドメトリで補完中'),
        'WHEEL_PRIORITY': (WARN, '車輪オドメトリのみで推定中'),
        'LIO_FAULT': (ERROR, 'LIO が異常'),
    }

    def report_quality(self, message: FusionState) -> None:
        """融合モードを診断の `quality` 観点として自己申告する.

        Args:
            message (FusionState): 配信した融合状態.
        """

        level, text = self.QUALITY_BY_MODE.get(message.mode, (WARN, '不明なモード'))
        values = dict(mode=message.mode, speed_limit_mps=round(message.speed_limit_mps, 3))
        if message.has_estimate:
            values['heading_sigma_deg'] = round(message.heading_sigma_deg, 2)
        if message.reason:
            text = f'{text}: {message.reason}'
        self.diagnostics.report(ASPECT_QUALITY, level, text, values=values)

    def on_command(self, msg: Twist) -> None:
        """不確かさによる減速だけを加える。URGの停止指令は解除しない."""
        output = copy.deepcopy(msg)
        now = self.get_clock().now().nanoseconds*1e-9
        if not self.alignment_available() or self.filter.x is None or now-self.last_output > self.filter.config.max_lio_gap_s:
            output = Twist()
        else:
            limit = self.filter.diagnostics(now)['speed_limit_mps']
            if self.values['wheel_fallback_enabled'] and self.motion.source == 'WHEEL_FALLBACK':
                limit = min(limit, .4)
            # 曲率を維持し、減速で旋回だけが強くなることを防ぐ。
            scale = min(1., limit/max(abs(output.linear.x), 1e-9))
            output.linear.x *= scale
            output.angular.z *= scale
        self.cmd_pub.publish(output)

    def record_input(self, data: dict) -> None:
        """同じ走行の単独センサ比較に使う観測値を保存する."""
        if self.input_log:
            self.input_log.write(json.dumps(data, allow_nan=False)+'\n')
            self.input_log.flush()

    def close(self) -> None:
        if self.log:
            self.log.close()
        if self.input_log:
            self.input_log.close()
        if self.cache and self.filter.baseline.ready:
            self.cache.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.cache.with_suffix('.tmp')
            temporary.write_text(json.dumps(dict(reference_m=self.filter.baseline.reference)))
            temporary.replace(self.cache)
        self.get_logger().info('融合ノードを終了する')


def main() -> None:
    rclpy.init()
    node = FusionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
