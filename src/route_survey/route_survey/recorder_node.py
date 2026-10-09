"""共通自己位置、FAST-LIO body点群、Joyを購読して経路採取する。速度指令は出さない."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import json
import math
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy, PointCloud2, NavSatFix
from rtk_gps_um982_msgs.msg import RtkStatus
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tc_diagnostics import DiagnosticReporter, report_alive
from tc_geo_msgs.msg import FusionState
from geo_pose_converter.geo_core import load_projection_config_from_yaml, LlhPoint, llh_to_enu
from route_survey.trace_core import Traces, finite
from gnss_lio_fusion.mount_core import base_from_sensor, rotation, quaternion_rotation
from route_survey.surface_core import SurfaceWindow, unknown

from route_survey.survey_core import Pose, Survey, traversable_width
from route_survey.storage_core import save


def stamp(message) -> float:
    return message.header.stamp.sec+message.header.stamp.nanosec*1e-9


class Recorder(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__('route_survey', **kwargs)
        defaults = dict(output_directory='', projection_config='', spacing_m=5., turn_deg=25.,
                        localization_mode='gnss', icp_map_manifest='',
                        start_button=0, stop_button=1, signal_button=2, finish_button=3,
                        lidar_height_m=.6, lidar_forward_m=0., max_pose_variance=.25,
                        lidar_left_m=0., lidar_mount_roll_deg=0., lidar_mount_pitch_deg=0.,
                        lidar_mount_yaw_deg=0., cloud_frame='body', gnss_fix_topic='/rtk_gps/fix',
                        gnss_status_topic='/rtk_gps/rtk_status', surface_window_s=6.,
                        surface_margin_m=.35, surface_max_seed_intensity=30.)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        if self.p['localization_mode'] not in ('gnss', 'icp'):
            raise ValueError('localization_modeはicpまたはgnssが必要')
        if not self.p['output_directory'] or not self.p['projection_config']:
            raise ValueError('output_directoryとprojection_configが必要')
        self.directory = Path(self.p['output_directory'])
        self.directory.mkdir(parents=True, exist_ok=False)
        self.projection = load_projection_config_from_yaml(self.p['projection_config'])
        self.traces = Traces()
        self.fusion_state = None
        self.icp_state = {}
        self.pose_quality = {}
        self.survey = Survey(self.p['spacing_m'], self.p['turn_deg'])
        self.poses = deque(maxlen=100)
        self.attitudes = deque(maxlen=100)
        self.width = traversable_width(np.empty((0, 3)))
        self.cloud_stamp = -1e9
        self.voxels = {}
        self.surface_window = SurfaceWindow(self.p['surface_window_s'])
        self.last_surface_compute = -math.inf
        self.last_report = None
        self.save_error = ''
        self.save_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='survey-save')
        self.save_future = None
        self.save_again = False
        self.save_revision = 0
        self.saved_revision = -1
        self.last_pose_receive = -1e9
        self.status = self.create_publisher(String, '/route_survey/status', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/localization/pose_enu', self.pose, 10)
        if self.p['localization_mode'] == 'gnss':
            self.create_subscription(Odometry, '/lio/odometry', self.attitude, 10)
            self.create_subscription(FusionState, '/fusion/status', self.health, 10)
        else:
            self.create_subscription(String, '/localization/icp_status', self.icp_health, 10)
        self.create_subscription(PointCloud2, '/cloud_registered_body', self.cloud,
                                 qos_profile_sensor_data)
        self.create_subscription(Joy, '/joy', self.joy, qos_profile_sensor_data)
        self.create_subscription(String, '/route_survey/command', self.command, 10)
        self.create_subscription(NavSatFix, self.p['gnss_fix_topic'], self.gnss_fix,
                                 qos_profile_sensor_data)
        self.create_subscription(RtkStatus, self.p['gnss_status_topic'], self.gnss_status, 10)
        self.create_timer(1., lambda: self.flush())
        self.create_timer(.1, self.poll_save)
        self.diagnostics = DiagnosticReporter(self)
        report_alive(self.diagnostics, '経路採取ノード稼働中')
        self.get_logger().info('経路採取待機: Joy 0開始 / 1停止 / 2信号停止 / 3終了')

    def now(self) -> float:
        return self.get_clock().now().nanoseconds*1e-9

    def pose(self, message) -> None:
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        if message.header.frame_id != self.projection.map_frame_id:
            return
        variance = float(max(message.pose.covariance[0], message.pose.covariance[7]))
        if not math.isfinite(variance) or variance < 0:
            return
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            return
        try:
            base_rotation = quaternion_rotation([q.x, q.y, q.z, q.w])
        except ValueError:
            return
        yaw = math.atan2(base_rotation[1, 0], base_rotation[0, 0])
        state = self.fusion_state
        fresh = state is not None and abs(stamp(message)-state.estimate_stamp_s) <= .5
        self.pose_quality = dict(position_variance=variance,
                                 source=self.p['localization_mode'],
                                 mode=state.mode if fresh else 'UNKNOWN',
                                 uncertain=variance > self.p['max_pose_variance'])
        if self.p['localization_mode'] == 'icp':
            state = self.icp_state
            estimate_stamp = state.get('estimate_stamp_s')
            fresh = (isinstance(estimate_stamp, (int, float))
                     and abs(stamp(message)-estimate_stamp) <= .5)
            mode = state.get('state', 'UNKNOWN') if fresh else 'UNKNOWN'
            self.pose_quality.update(mode=mode, uncertain=(mode != 'ICP_TRACKING' or
                variance > self.p['max_pose_variance']),
                reason=state.get('reason') if fresh else 'status_unavailable',
                map_correction_age_s=state.get('map_correction_age_s') if fresh else None)
        self.traces.append('fused', dict(stamp=stamp(message), x=finite(p.x), y=finite(p.y),
                                       yaw=finite(yaw), **self.pose_quality))
        value = Pose(stamp(message), p.x, p.y, yaw)
        if self.survey.update(value, self.current_width()):
            if self.survey.active:
                self.save_revision += 1
            self.poses.append(value)
            self.last_pose_receive = self.now()
            if self.p['localization_mode'] == 'icp':
                # ICPのposeは取付補正済みbase_link。IMU/bodyへは取付変換を一度だけ戻す。
                base = np.array([p.x, p.y, p.z])
                lever = [self.p['lidar_forward_m'], self.p['lidar_left_m'], self.p['lidar_height_m']]
                imu_position = base + base_rotation @ lever
                imu_rotation = base_rotation @ rotation(*self.mount_rpy())
                self.attitudes.append((stamp(message), 0., 0., base, yaw, imu_position, imu_rotation))

    def icp_health(self, message: String) -> None:
        try:
            value = json.loads(message.data)
            if isinstance(value, dict):
                self.icp_state = value
        except (ValueError, TypeError):
            pass

    def health(self, message: FusionState) -> None:
        """融合状態を保持する。水平基準の再確定中は採取済みデータを破棄する。

        水平基準が未確定の間に積んだ姿勢・点群は基準がずれているため、
        そのまま経路として保存すると誤った軌跡になる。

        Args:
            message (FusionState): `/fusion/status` の受信メッセージ.
        """

        if message.mode == FusionState.MODE_WAIT_GRAVITY_ALIGNMENT:
            self.poses.clear()
            self.attitudes.clear()
            self.surface_window.clear()
            self.last_surface_compute = -math.inf
            self.cloud_stamp = -math.inf
            self.last_pose_receive = -math.inf
        self.fusion_state = message

    def gnss_status(self, message) -> None:
        self.traces.append('statuses', dict(stamp=stamp(message), state=int(message.rtk_state),
            satellites=int(message.num_satellites), hdop=finite(message.hdop),
            baseline_m=finite(message.baseline_length_m),
            correction_age_s=finite(message.correction_age_s),
            heading_stddev_deg=finite(message.heading_stddev_deg)))

    def gnss_fix(self, message) -> None:
        lat, lon = finite(message.latitude), finite(message.longitude)
        valid = (message.status.status >= 0 and lat is not None and lon is not None
                 and -90 <= lat <= 90 and -180 <= lon <= 180)
        point = (llh_to_enu(LlhPoint(lat, lon, self.projection.origin_altitude), self.projection)
                 if valid else None)
        self.traces.append('gnss', dict(stamp=stamp(message), latitude=lat, longitude=lon,
            x=point.x if point else None, y=point.y if point else None,
            navsat_status=int(message.status.status)))

    def attitude(self, message) -> None:
        q = message.pose.pose.orientation
        try:
            p = message.pose.pose.position
            imu_position = np.array([p.x, p.y, p.z])
            quat = [q.x, q.y, q.z, q.w]
            base, (roll, pitch, yaw) = base_from_sensor(
                imu_position, quat,
                [self.p['lidar_forward_m'], self.p['lidar_left_m'], self.p['lidar_height_m']],
                self.mount_rpy())
            imu_rotation = quaternion_rotation(quat)
        except ValueError:
            return
        self.attitudes.append((stamp(message), roll, pitch, base, yaw, imu_position, imu_rotation))

    def mount_rpy(self) -> list[float]:
        return [math.radians(self.p['lidar_mount_'+axis+'_deg'])
                for axis in ['roll', 'pitch', 'yaw']]

    def current_width(self) -> dict:
        value = (traversable_width(np.empty((0, 3))) if
                 abs(self.now()-self.cloud_stamp) > .5 or self.pose_quality.get('uncertain')
                 else dict(self.width))
        value['localization'] = dict(self.pose_quality)
        return value

    def cloud(self, message) -> None:
        if message.header.frame_id != self.p['cloud_frame'] or not self.poses or not self.attitudes:
            return
        t = stamp(message)
        pose = min(self.poses, key=lambda v: abs(v.stamp-t))
        attitude = min(self.attitudes, key=lambda v: abs(v[0]-t))
        if abs(pose.stamp-t) > .15 or abs(attitude[0]-t) > .15:
            return
        if t-self.last_surface_compute < .19:
            return
        self.last_surface_compute = t
        fields = ['x', 'y', 'z']
        if any(f.name == 'intensity' for f in message.fields):
            fields.append('intensity')
        values = point_cloud2.read_points(message, field_names=fields)
        points = np.column_stack([values[k].reshape(-1) for k in fields])
        points = points[np.isfinite(points[:, :3]).all(axis=1)]
        xyz = points[:, :3]
        roll, pitch = attitude[1:3]
        # GNSSモードは水平化LIO、ICPモードは地図座標で短時間蓄積する。
        local_world = points.copy()
        local_world[:, :3] = xyz @ attitude[6].T + attitude[5]
        accumulated = self.surface_window.update(t, local_world, attitude[3], attitude[4])
        self.width = traversable_width(accumulated, margin=self.p['surface_margin_m'],
            max_seed_intensity=self.p['surface_max_seed_intensity'])
        if not self.width.get('material_checked', False):
            self.width = unknown('material_unavailable')
        self.cloud_stamp = t
        if self.p.get('localization_mode') == 'icp':
            # full SE3で車体傾斜と地図の高さも保持する。
            world = local_world[:, :3]
            xyz = world-attitude[3]
        else:
            tilt = rotation(roll, pitch)
            # body点群はIMU座標。取付回転を除き、車体傾斜・位置を反映する。
            xyz = (xyz @ rotation(*self.mount_rpy()).T +
                   [self.p['lidar_forward_m'], self.p['lidar_left_m'], self.p['lidar_height_m']]) @ tilt.T
            c, s = math.cos(pose.yaw), math.sin(pose.yaw)
            world = xyz @ np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
            world += [pose.x, pose.y, 0]
        world = world[np.linalg.norm(xyz[:, :2], axis=1) < 15.]
        for point in world:
            self.voxels[tuple(np.floor(point/.15).astype(int))] = point
        while len(self.voxels) > 100000:
            self.voxels.pop(next(iter(self.voxels)))

    def command(self, message) -> None:
        fresh = bool(self.poses) and self.now()-self.last_pose_receive <= .5
        if message.data not in ('start', 'finish', 'line_stop', 'signal_stop'):
            return
        if not fresh and message.data != 'finish':
            self.get_logger().warning('自己位置が未受信または古いため採取操作を受け付けません')
            return
        self.survey.command(message.data, self.poses[-1] if fresh else None, self.current_width())
        self.save_revision += 1
        self.flush()

    def joy(self, message) -> None:
        pressed = {i for i, value in enumerate(message.buttons) if value}
        if not self.poses or self.now()-self.last_pose_receive > .5:
            if self.p['finish_button'] in pressed-self.survey.buttons:
                self.command(String(data='finish'))
            self.survey.buttons = pressed
            return
        changed = pressed != self.survey.buttons
        self.survey.joy(pressed, self.poses[-1], self.current_width(),
                        self.p['start_button'], self.p['stop_button'],
                        self.p['signal_button'], self.p['finish_button'])
        if changed:
            self.save_revision += 1
            self.flush()

    def poll_save(self) -> None:
        if self.save_future is None or not self.save_future.done():
            return
        self.finish_save()
        if self.save_again:
            self.flush()
        else:
            self.publish_status()

    def finish_save(self) -> None:
        """完了した保存だけをUIへ反映する。待機呼出しは終了処理に限定する。"""
        try:
            self.save_future.result()
            self.saved_revision = self.submitted_revision
            self.save_error = ''
        except Exception as exc:
            self.save_error = str(exc)
            self.get_logger().error(f'経路保存失敗: {exc}')
        self.save_future = None

    def flush(self, wait=False) -> None:
        report = (self.survey.active, len(self.survey.rows))
        if report != self.last_report:
            self.get_logger().info(f'経路採取: active={report[0]}, waypoint={report[1]}')
            self.last_report = report
        if self.save_future is not None:
            if wait or self.save_future.done():
                self.finish_save()
            else:
                # 保存要求を1件にまとめ、最新の受信データを次の保存へ渡す。
                self.save_again = True
                self.publish_status()
                return
        self.save_again = False
        if self.survey.rows:
            self.submitted_revision = self.save_revision
            self.save_future = self.save_pool.submit(
                save, self.directory, deepcopy(self.survey.rows), self.projection,
                np.array(list(self.voxels.values())).reshape(-1, 3), self.survey.active,
                traces=dict(self.traces.export(), source=self.p['localization_mode'],
                            icp_map_manifest=self.p['icp_map_manifest']))
            if wait:
                self.finish_save()
        self.publish_status()

    def publish_status(self) -> None:
        state = dict(active=self.survey.active, count=len(self.survey.rows),
                     localization_mode=self.p['localization_mode'],
                     pose_fresh=self.now()-self.last_pose_receive <= .5,
                     directory=str(self.directory), error=self.save_error,
                     saving=self.save_future is not None,
                     saved=bool(self.survey.rows) and not self.survey.active and not self.save_error
                           and self.saved_revision == self.save_revision,
                     width=self.current_width(), rejected_poses=self.survey.rejections)
        if self.context.ok():
            self.status.publish(String(data=json.dumps(state)))

    def destroy_node(self):
        self.flush(wait=True)
        self.save_pool.shutdown(wait=True)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
