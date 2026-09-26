"""UM982 RTK GNSS ドライバノード (rclpy)。"""

from dataclasses import replace
import json
import time

import rclpy
from rclpy.node import Node
from tc_diagnostics import ASPECT_DEVICE, ASPECT_QUALITY, ERROR, OK, WARN, DiagnosticReporter
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String

from rtk_gps_um982_msgs.msg import RtkStatus
from rtk_gps_um982.ntrip_client import CorrectedUM982Client as UM982Client

from rtk_gps_um982 import converters
from rtk_gps_um982.ntrip_status import NtripStatus
from rtk_gps_um982.time_sync_core import RmcClockRelay
from rtk_gps_um982.time_sync_client import ClockRelayClient


class Um982DriverNode(Node):

    def __init__(self) -> None:
        super().__init__('rtk_gps_um982_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('serial.port', '/dev/ttyUSB0'),
                ('serial.baud', 115200),
                ('output_rate_hz', 10),
                ('frame_id', 'gps_link'),
                ('stamp_source', 'gnss_utc'),
                ('transport_delay_ms', 0),
                ('time_sync.enabled', False),
                ('time_sync.chrony_socket', '/run/chrony/um982.sock'),
                ('ntrip.enabled', False),
                ('ntrip.station_id', ''),
                ('ntrip.station_label', ''),
                ('ntrip.site', ''),
                ('ntrip.host', ''),
                ('ntrip.port', 2101),
                ('ntrip.mountpoint', ''),
                ('ntrip.user', ''),
                ('ntrip.password', ''),
                ('publish.navsatfix', True),
                ('publish.imu_heading', True),
                ('publish.rtk_status', True),
                ('min_fix_for_publish', 'standalone'),
                ('hdop_sigma', 1.0),
            ],
        )

        p = self.get_parameter
        self._port = p('serial.port').value
        self._baud = p('serial.baud').value
        self._output_rate = p('output_rate_hz').value
        self._frame_id = p('frame_id').value
        self._stamp_source = p('stamp_source').value
        self._transport_delay_ms = p('transport_delay_ms').value
        self._min_fix = p('min_fix_for_publish').value
        self._hdop_sigma = float(p('hdop_sigma').value)

        self._pub_fix = (
            self.create_publisher(NavSatFix, '~/fix', 10)
            if p('publish.navsatfix').value else None
        )
        self._pub_imu = (
            self.create_publisher(Imu, '~/heading', 10)
            if p('publish.imu_heading').value else None
        )
        self._pub_status = (
            self.create_publisher(RtkStatus, '~/rtk_status', 10)
            if p('publish.rtk_status').value else None
        )

        self._relay = None
        client_type = UM982Client
        client_options = {}
        if p('time_sync.enabled').value:
            if (p('use_sim_time').value or self._stamp_source != 'gnss_utc'
                    or self._transport_delay_ms != 0):
                raise ValueError('時刻配信は実時刻・gnss_utc・transport_delay_ms=0が必要')
            self._relay = RmcClockRelay(p('time_sync.chrony_socket').value)
            client_type = ClockRelayClient
            client_options['clock_relay'] = self._relay
            self._pub_time = self.create_publisher(String, '~/time_sync', 10)
            self._time_timer = self.create_timer(1., self._publish_time_sync)

        self._client = client_type(
            port=self._port,
            baud=self._baud,
            output_rate=self._output_rate,
            **client_options,
        )
        self._client.set_position_callback(self._on_position)
        self._client.start()

        if p('ntrip.enabled').value:
            host = p('ntrip.host').value
            port = p('ntrip.port').value
            mountpoint = p('ntrip.mountpoint').value
            user = p('ntrip.user').value
            password = p('ntrip.password').value
            if not host or not mountpoint:
                self.get_logger().error(
                    'ntrip.enabled=true but ntrip.host / ntrip.mountpoint is empty'
                )
            else:
                self.get_logger().info(
                    f'Starting NTRIP: {host}:{port}/{mountpoint} as {user or "(anonymous)"}'
                )
                self._client.start_ntrip(
                    host=host, port=port, mountpoint=mountpoint,
                    user=user, password=password,
                )

        # 実際に起動した接続設定を保持。パラメータ変更だけで接続先表示を変えない。
        self._ntrip_fields = {key: p('ntrip.'+key).value for key in (
            'enabled', 'host', 'port', 'mountpoint', 'station_id', 'station_label', 'site')}
        self._ntrip_status = NtripStatus()
        self._pub_ntrip = self.create_publisher(String, '~/ntrip_status', 10)
        self._ntrip_timer = self.create_timer(1., self._publish_ntrip_status)

        # シリアル接続と補正受信は外形（topic の鮮度）に現れないため自己申告する。
        self._diagnostics = DiagnosticReporter(self)
        self._diagnostics.report(
            ASPECT_DEVICE, OK, f'受信機に接続済み ({self._port})', hardware_id=self._port)

        self.get_logger().info(
            f'rtk_gps_um982_node up (port={self._port} baud={self._baud} '
            f'rate={self._output_rate}Hz stamp={self._stamp_source})'
        )

    # NTRIP 状態ごとの診断レベルと説明。
    _NTRIP_QUALITY = {
        'RECEIVING': (OK, '補正を受信中'),
        'DISABLED': (OK, 'NTRIP 無効設定'),
        'WAITING': (WARN, '接続済み・RTCM 待ち'),
        'CONNECTING': (WARN, '接続待ち'),
        'RECONNECTING': (WARN, '再接続中'),
        'STALE': (WARN, '補正が途絶'),
        'ERROR': (ERROR, '設定エラー'),
    }

    def _publish_ntrip_status(self) -> None:
        # 配信様式: stream (1 Hz)
        data = self._ntrip_status.sample(
            self._client._ntrip_client, now=time.monotonic(), **self._ntrip_fields)
        self._pub_ntrip.publish(String(data=json.dumps(data, ensure_ascii=False, allow_nan=False)))
        level, text = self._NTRIP_QUALITY.get(data.get('state'), (WARN, 'NTRIP 状態不明'))
        self._diagnostics.report(
            ASPECT_QUALITY, level, text,
            values={'ntrip_state': data.get('state', ''),
                    'rtcm_bytes_total': data.get('rtcm_bytes_total', 0)})

    def _on_position(self, pos) -> None:
        if self._relay is not None:
            stamp = self._relay.date_gga(pos.timestamp, time.monotonic())
            if stamp is None:
                return
            pos = replace(pos, timestamp=stamp)
        if not pos.is_valid:
            return
        if not converters.passes_fix_filter(pos.rtk_state, self._min_fix):
            return

        stamp = converters.make_stamp(
            self._stamp_source,
            pos.timestamp,
            self._transport_delay_ms,
            self.get_clock(),
        )

        if self._pub_fix is not None:
            self._pub_fix.publish(
                converters.position_to_navsatfix(pos, stamp, self._frame_id, self._hdop_sigma)
            )
        if self._pub_imu is not None:
            self._pub_imu.publish(
                converters.position_to_imu(pos, stamp, self._frame_id)
            )
        if self._pub_status is not None:
            self._pub_status.publish(
                converters.position_to_rtk_status(
                    pos, stamp, self._frame_id, self._client.get_rtcm_bytes()
                )
            )

    def _publish_time_sync(self) -> None:
        anchor = self._relay.anchor
        age = time.monotonic()-anchor[1] if anchor else None
        msg = String()
        msg.data = json.dumps(dict(
            rmc_fresh=age is not None and age <= 2., rmc_age_s=age,
            samples_sent=self._relay.sent, rejected=self._relay.rejected,
            socket_error=self._relay.error, precision_verified=False))
        self._pub_time.publish(msg)

    def destroy_node(self) -> bool:
        try:
            self._client.stop()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'UM982Client.stop() raised: {exc}')
        if self._relay is not None:
            self._relay.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Um982DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
