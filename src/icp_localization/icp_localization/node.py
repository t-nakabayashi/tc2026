"""FAST-LIO点群を地図照合し、GNSS系と同じENU pose接口へ出力する。"""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import fields
import json
import math
import time

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, NavSatFix
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from rtk_gps_um982_msgs.msg import RtkStatus
from tc_diagnostics import DiagnosticReporter, report_alive, ASPECT_QUALITY, OK, WARN
from geo_pose_converter.geo_core import LlhPoint, ProjectionConfig, llh_to_enu

from .registration import Engine, load_manifest
from .tracker import QualityConfig, Tracker


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9


def stamp_key(msg):
    return msg.header.stamp.sec*1000000000 + msg.header.stamp.nanosec


def transform(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    a = np.array([q.x, q.y, q.z, q.w])
    if not np.isfinite(a).all() or not .99 < np.linalg.norm(a) < 1.01:
        raise ValueError('Invalid pose quaternion')
    t = np.eye(4)
    t[:3, :3] = Rotation.from_quat(a).as_matrix()
    t[:3, 3] = [p.x, p.y, p.z]
    if not np.isfinite(t).all():
        raise ValueError('Invalid pose position')
    return t


class Localizer(Node):
    def __init__(self, **kwargs):
        super().__init__('icp_localization', **kwargs)
        values = {k: self.declare_parameter(k, v).value for k, v in dict(
            map_manifest='', backend_python_path='', origin_latitude=0., origin_longitude=0.,
            origin_altitude=0., map_yaw_offset_rad=0., map_frame_id='map', datum='WGS84',
            base_imu_xyz=[0., 0., .6], base_imu_rpy_deg=[0., 0., 0.],
            antenna_xyz=[0., 0., .7], gnss_heading_offset_deg=180., publish_base_tf=True,
            initialize_from_gnss=True).items()}
        self.projection = ProjectionConfig(**{k: values[k] for k in (
            'origin_latitude', 'origin_longitude', 'origin_altitude', 'map_yaw_offset_rad',
            'map_frame_id', 'datum')})
        self.heading_offset = values['gnss_heading_offset_deg']
        self.initialize_from_gnss = values['initialize_from_gnss']
        self.core = Tracker(QualityConfig(**{
            f.name: self.declare_parameter('quality.'+f.name, getattr(QualityConfig(), f.name)).value
            for f in fields(QualityConfig)}))
        base_imu = np.eye(4)
        base_imu[:3, :3] = Rotation.from_euler('xyz', values['base_imu_rpy_deg'], degrees=True).as_matrix()
        base_imu[:3, 3] = values['base_imu_xyz']
        self.engine = Engine(load_manifest(values['map_manifest'], vars(self.projection)),
                             base_imu, values['antenna_xyz'], values['backend_python_path'])
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='icp')
        self.future = None
        self.odoms, self.clouds, self.fixes, self.statuses = {}, {}, {}, {}
        self.latest = None
        self.pending_seed = None
        self.pending_seed_deadline = math.inf
        self.last_lio_receive = -math.inf
        self.previous_state = None
        self.last_ros = self.ros_now()
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/localization/pose_enu', 10)
        # JSON診断のみ。走行許可や速度指令として購読させない。
        self.status_pub = self.create_publisher(String, '/localization/icp_status', 1)
        self.tf = TransformBroadcaster(self) if values['publish_base_tf'] else None
        self.create_subscription(Odometry, '/lio/odometry_raw', self.on_lio, 10)
        self.create_subscription(PointCloud2, '/cloud_registered_body', self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initialpose, 1)
        self.create_subscription(NavSatFix, '/rtk_gps/fix', self.on_fix, 10)
        self.create_subscription(RtkStatus, '/rtk_gps/rtk_status', self.on_status, 10)
        self.clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.02, self.tick, clock=self.clock)
        self.last_status = -math.inf
        self.diagnostics = DiagnosticReporter(self)
        report_alive(self.diagnostics, '固定地図ICP自己位置推定')

    def ros_now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def fresh(self, msg, maximum=.3):
        return -.05 <= self.ros_now()-stamp(msg) <= maximum

    @staticmethod
    def trim(buffer, maximum=20):
        while len(buffer) > maximum:
            del buffer[min(buffer)]

    def on_initialpose(self, msg):
        if msg.header.frame_id != self.projection.map_frame_id:
            self.get_logger().warning('initialpose rejected: map frame mismatch')
            return
        try:
            seed = transform(msg) @ self.engine.base_imu
        except ValueError as e:
            self.get_logger().warning(str(e))
            return
        self.core.clear('manual_initial_pose')
        self.pending_seed = seed
        self.pending_seed_deadline = math.inf
        self.clouds.clear()

    def invalidate_lio(self, reason):
        self.core.clear(reason)
        self.pending_seed = self.latest = None
        self.odoms.clear()
        self.clouds.clear()

    def on_lio(self, msg):
        if msg.child_frame_id != 'body' or not self.fresh(msg):
            return
        try:
            raw = transform(msg)
        except ValueError:
            self.invalidate_lio('invalid_lio_pose')
            return
        t = stamp(msg)
        if self.latest:
            old_msg, old_raw = self.latest
            dt = t-stamp(old_msg)
            if dt <= 0:
                return
            d = np.linalg.inv(old_raw) @ raw
            if (msg.header.frame_id != old_msg.header.frame_id or dt > .5
                    or np.linalg.norm(d[:3, 3]) > 3*dt+.05
                    or Rotation.from_matrix(d[:3, :3]).magnitude() > 2*dt+.05):
                self.invalidate_lio('lio_discontinuity_set_initialpose')
        self.latest = (msg, raw)
        self.last_lio_receive = time.monotonic()
        self.odoms[stamp_key(msg)] = (msg, raw)
        self.trim(self.odoms)
        self.submit()
        self.publish_pose()

    def on_cloud(self, msg):
        if msg.header.frame_id != 'body' or not self.fresh(msg):
            return
        self.clouds[stamp_key(msg)] = (time.monotonic(), msg)
        self.trim(self.clouds, 3)
        self.submit()

    def on_fix(self, msg):
        self.fixes[stamp_key(msg)] = msg
        self.trim(self.fixes)
        self.match_fix(stamp_key(msg))

    def on_status(self, msg):
        self.statuses[stamp_key(msg)] = msg
        self.trim(self.statuses)
        self.match_fix(stamp_key(msg))

    def match_fix(self, key):
        if key not in self.fixes or key not in self.statuses:
            return
        f, s = self.fixes.pop(key), self.statuses.pop(key)
        # 起動時の初期位置にだけ使う。追跡開始後は品質低下時も再利用しない。
        if (not self.initialize_from_gnss or self.core.initialized_once
                or self.core.pose is not None or self.pending_seed is not None
                or not self.fresh(f) or not self.fresh(s)):
            return
        if not (s.rtk_state == 4 and f.status.status >= 0
                and f.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN
                and -90 <= f.latitude <= 90 and -180 <= f.longitude <= 180
                and 0 <= s.heading_deg <= 360 and 0 < s.heading_stddev_deg <= 3.):
            return
        cov = np.array(f.position_covariance).reshape(3, 3)[:2, :2]
        if not np.isfinite(cov).all() or not np.allclose(cov, cov.T):
            return
        eig = np.linalg.eigvalsh(cov)
        if eig[0] <= 0 or eig[-1] > .3**2:
            return
        point = llh_to_enu(LlhPoint(f.latitude, f.longitude, self.projection.origin_altitude), self.projection)
        yaw = math.radians(90-s.heading_deg+self.heading_offset)-self.projection.map_yaw_offset_rad
        try:
            self.pending_seed = self.engine.seed(np.array([point.x, point.y, yaw]))
            self.pending_seed_deadline = time.monotonic()+.3
        except ValueError as e:
            self.core.reason = str(e)

    def submit(self):
        if self.future is not None:
            return
        common = self.clouds.keys() & self.odoms.keys()
        if not common:
            return
        key = max(common)
        received, cloud = self.clouds.pop(key)
        msg, raw = self.odoms[key]
        self.clouds = {k: v for k, v in self.clouds.items() if k > key}
        if not self.fresh(cloud):
            return
        if self.pending_seed is not None:
            if time.monotonic() > self.pending_seed_deadline:
                self.pending_seed = None
                return
            self.core.seed(self.pending_seed, raw)
            self.pending_seed = None
        seed = self.core.predict(raw)
        if seed is None:
            return
        initial = not self.core.localized
        def run():
            points = point_cloud2.read_points_numpy(cloud, field_names=('x', 'y', 'z'), skip_nans=True)
            return self.engine.register(points, seed, initial)
        self.job = (received, msg, raw, self.core.generation)
        self.future = self.pool.submit(run)

    def tick(self):
        now, ros = time.monotonic(), self.ros_now()
        if ros < self.last_ros-.05:
            self.invalidate_lio('ros_clock_reset_set_initialpose')
            self.fixes.clear(); self.statuses.clear()
        self.last_ros = ros
        if self.future is not None and self.future.done():
            received, msg, raw, generation = self.job
            try:
                pose, metrics = self.future.result()
                if self.core.result(now, received, stamp(msg), generation, pose, raw, metrics):
                    self.publish_pose()
            except Exception as e:
                if generation == self.core.generation:
                    self.core.reason = 'registration_error: '+str(e)
            self.future = None
            self.submit()
        if now-self.last_status < .1:
            return
        self.last_status = now
        state = self.core.state(now)
        lio_fresh = self.latest is not None and now-self.last_lio_receive <= .3 and self.fresh(self.latest[0])
        status = dict(state=state if lio_fresh else 'NO_LIO', reason=self.core.reason,
                      has_pose=bool(self.core.localized and lio_fresh),
                      estimate_stamp_s=stamp(self.latest[0]) if self.latest else None,
                      map_correction_age_s=now-self.core.last_accepted if self.core.localized else None,
                      metrics={k: v for k, v in self.core.metrics.items() if math.isfinite(v)})
        self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))
        self.diagnostics.report(ASPECT_QUALITY, OK if status['state'] == 'ICP_TRACKING' else WARN,
                                status['state']+': '+status['reason'], values=status['metrics'])
        current = (status['state'], status['reason'])
        if current != self.previous_state:
            self.get_logger().info(': '.join(current))
            self.previous_state = current

    def publish_pose(self):
        if not self.core.localized or self.latest is None or time.monotonic()-self.last_lio_receive > .3:
            return
        msg, raw = self.latest
        if not self.fresh(msg):
            return
        pose = self.engine.base_pose(self.core.predict(raw))
        out = PoseWithCovarianceStamped()
        out.header.stamp, out.header.frame_id = msg.header.stamp, self.projection.map_frame_id
        p, q = out.pose.pose.position, out.pose.pose.orientation
        p.x, p.y, p.z = map(float, pose[:3, 3])
        q.x, q.y, q.z, q.w = map(float, Rotation.from_matrix(pose[:3, :3]).as_quat())
        age = max(0., time.monotonic()-self.core.last_accepted)
        # 工学的な参考値。絶対位置誤差の推定共分散ではない。
        out.pose.covariance = np.diag([.3**2+.04*age]*3+[(math.radians(3)**2)+.001*age]*3).ravel().tolist()
        self.pub.publish(out)
        if self.tf:
            tf = TransformStamped()
            tf.header, tf.child_frame_id = out.header, 'base_link'
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = p.x, p.y, p.z
            tf.transform.rotation = q
            self.tf.sendTransform(tf)

    def destroy_node(self):
        self.pool.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = Localizer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
