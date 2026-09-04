#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""LLH自己位置をOpenStreetMap上に三角アイコンで表示するROS 2ノード.

route_geo_projector_node が publish する /localization/pose_llh
(tc_geo_msgs/GeoPoseWithQuality) を購読し、ブラウザ上のOpenStreetMapに
現在位置、active route、active targetを重畳表示する。

表示アイコン:
  - 底辺:高さ = 1:2 の二等辺三角形
  - 頂角から底辺への垂線方向が heading_deg を表す
  - heading_deg は真北基準・時計回り[deg]

注意:
  - 地図描画には Leaflet CDN と OpenStreetMap タイルを使う。
  - そのため、通常はブラウザ表示時にインターネット接続が必要。
"""

from __future__ import annotations

import json
import math
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tc_geo_msgs.msg import GeoPoseWithQuality
from tc_route_msgs.msg import ActiveTargetLlh, Route


class PoseStore:
    """HTTPサーバスレッドとROSスレッドの間でビューア状態を共有するクラス."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._pose: Optional[dict[str, Any]] = None
        self._route: Optional[dict[str, Any]] = None
        self._active_target: Optional[dict[str, Any]] = None

    def update(self, pose: dict[str, Any]) -> None:
        """最新姿勢を更新する."""
        with self._lock:
            self._pose = dict(pose)

    def update_route(self, route: dict[str, Any]) -> None:
        """最新route overlayを更新する."""
        with self._lock:
            self._route = dict(route)

    def update_active_target(self, active_target: dict[str, Any]) -> None:
        """最新active target overlayを更新する."""
        with self._lock:
            self._active_target = dict(active_target)

    def get(self) -> Optional[dict[str, Any]]:
        """最新姿勢を取得する."""
        with self._lock:
            if self._pose is None:
                return None
            return dict(self._pose)

    def get_state(self, stale_timeout_s: float, lost_timeout_s: float) -> dict[str, Any]:
        """ビューア全体の最新状態を取得する."""
        with self._lock:
            pose = dict(self._pose) if self._pose is not None else None
            route = dict(self._route) if self._route is not None else None
            active_target = (
                dict(self._active_target)
                if self._active_target is not None
                else None
            )

        now = time.time()
        pose_status = 'NO_DATA'
        if pose is not None:
            age_s = now - float(pose.get('received_wall_time', now))
            pose['age_s'] = age_s
            if age_s >= lost_timeout_s:
                pose_status = 'LOST'
            elif age_s >= stale_timeout_s:
                pose_status = 'STALE'
            else:
                pose_status = 'OK'

        return {
            'pose': pose,
            'route': route,
            'active_target': active_target,
            'pose_status': pose_status,
        }


def _parse_bool(value: Any) -> bool:
    """ROS parameter値をboolとして解釈する."""
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def _is_valid_lat_lon(latitude: float, longitude: float) -> bool:
    """緯度経度が地図表示可能な範囲か判定する."""
    return (
        math.isfinite(latitude)
        and math.isfinite(longitude)
        and -90.0 <= latitude <= 90.0
        and -180.0 <= longitude <= 180.0
    )


def _geo_pose_to_dict(geo_pose: Any) -> Optional[dict[str, Any]]:
    """GeoPose互換msgをviewer JSON用dictへ変換する."""
    latitude = float(geo_pose.point.latitude)
    longitude = float(geo_pose.point.longitude)
    if not _is_valid_lat_lon(latitude, longitude):
        return None

    has_heading = bool(geo_pose.has_heading)
    heading_deg = float(geo_pose.heading_deg) if has_heading else 0.0
    if not math.isfinite(heading_deg):
        heading_deg = 0.0
        has_heading = False

    return {
        'latitude': latitude,
        'longitude': longitude,
        'altitude': float(geo_pose.point.altitude),
        'has_altitude': bool(geo_pose.point.has_altitude),
        'heading_deg': heading_deg % 360.0,
        'has_heading': has_heading,
        'child_frame_id': str(geo_pose.child_frame_id),
    }


def _pose_llh_to_dict(msg: GeoPoseWithQuality) -> Optional[dict[str, Any]]:
    """GeoPoseWithQualityをviewer JSON用dictへ変換する."""
    pose = _geo_pose_to_dict(msg.pose)
    if pose is None:
        return None

    stamp_sec = (
        float(msg.header.stamp.sec)
        + float(msg.header.stamp.nanosec) * 1.0e-9
    )
    pose.update(
        {
            'status_text': str(msg.status_text),
            'fix_quality': int(msg.fix_quality),
            'fusion_status': int(msg.fusion_status),
            'source': int(msg.source),
            'stamp': stamp_sec,
            'frame_id': str(msg.header.frame_id),
            'received_wall_time': time.time(),
        }
    )
    return pose


def _route_to_dict(msg: Route, max_waypoints: int) -> dict[str, Any]:
    """Routeをviewer JSON用dictへ変換する."""
    waypoints: list[dict[str, Any]] = []
    skipped = 0
    for wp in msg.waypoints:
        if len(waypoints) >= max_waypoints:
            skipped += 1
            continue
        if not bool(wp.has_geo_pose):
            skipped += 1
            continue
        geo_pose = _geo_pose_to_dict(wp.geo_pose)
        if geo_pose is None:
            skipped += 1
            continue
        geo_pose.update(
            {
                'index': int(wp.index),
                'label': str(wp.label),
                'line_stop': bool(wp.line_stop),
                'signal_stop': bool(wp.signal_stop),
                'segment_is_fixed': bool(wp.segment_is_fixed),
            }
        )
        waypoints.append(geo_pose)

    return {
        'route_id': str(msg.route_id),
        'version': int(msg.version),
        'total_distance': float(msg.total_distance),
        'waypoints': waypoints,
        'skipped_waypoints': skipped,
        'projection_id': str(msg.projection.projection_id),
        'map_frame_id': str(msg.map_frame_id),
        'earth_frame_id': str(msg.earth_frame_id),
    }


def _active_target_to_dict(msg: ActiveTargetLlh) -> Optional[dict[str, Any]]:
    """ActiveTargetLlhをviewer JSON用dictへ変換する."""
    target_pose = _geo_pose_to_dict(msg.target_pose)
    if target_pose is None:
        return None
    target_pose.update(
        {
            'route_version': int(msg.route_version),
            'target_index': int(msg.target_index),
            'target_label': str(msg.target_label),
            'distance_m': float(msg.distance_m),
            'bearing_deg': float(msg.bearing_deg),
            'is_avoidance_subgoal': bool(msg.is_avoidance_subgoal),
            'target_kind': str(msg.target_kind),
        }
    )
    return target_pose


class LlhOsmViewerNode(Node):
    """GeoPoseWithQualityを購読してWeb地図ビューアへ表示するノード."""

    def __init__(self) -> None:
        super().__init__('llh_osm_viewer')

        self.declare_parameter('pose_llh_topic', '/localization/pose_llh')
        self.declare_parameter('active_route_topic', '/active_route')
        self.declare_parameter('active_target_llh_topic', '/route/active_target_llh')
        self.declare_parameter('http_host', '127.0.0.1')
        self.declare_parameter('http_port', 8765)
        self.declare_parameter('open_browser', True)
        self.declare_parameter('poll_interval_ms', 200)
        self.declare_parameter('initial_zoom', 19)
        self.declare_parameter('default_latitude', 36.082)
        self.declare_parameter('default_longitude', 140.111)
        self.declare_parameter('default_zoom', 17)
        self.declare_parameter('triangle_height_px', 48.0)
        self.declare_parameter('max_route_waypoints', 2000)
        self.declare_parameter('stale_timeout_s', 1.0)
        self.declare_parameter('lost_timeout_s', 3.0)

        self._pose_store = PoseStore()
        self._http_server: Optional[ThreadingHTTPServer] = None

        self._pose_llh_topic = str(self.get_parameter('pose_llh_topic').value)
        self._active_route_topic = str(self.get_parameter('active_route_topic').value)
        self._active_target_llh_topic = str(
            self.get_parameter('active_target_llh_topic').value
        )
        self._http_host = str(self.get_parameter('http_host').value)
        self._http_port = int(self.get_parameter('http_port').value)
        self._open_browser = _parse_bool(self.get_parameter('open_browser').value)
        self._poll_interval_ms = int(self.get_parameter('poll_interval_ms').value)
        self._initial_zoom = int(self.get_parameter('initial_zoom').value)
        self._default_latitude = float(self.get_parameter('default_latitude').value)
        self._default_longitude = float(self.get_parameter('default_longitude').value)
        self._default_zoom = int(self.get_parameter('default_zoom').value)
        self._triangle_height_px = float(self.get_parameter('triangle_height_px').value)
        self._max_route_waypoints = int(self.get_parameter('max_route_waypoints').value)
        self._stale_timeout_s = float(self.get_parameter('stale_timeout_s').value)
        self._lost_timeout_s = float(self.get_parameter('lost_timeout_s').value)

        qos_stream = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_route = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            GeoPoseWithQuality,
            self._pose_llh_topic,
            self._on_pose_llh,
            qos_stream,
        )
        self.create_subscription(
            Route,
            self._active_route_topic,
            self._on_active_route,
            qos_route,
        )
        self.create_subscription(
            ActiveTargetLlh,
            self._active_target_llh_topic,
            self._on_active_target_llh,
            qos_stream,
        )

        self._http_thread = threading.Thread(
            target=self._serve_http,
            daemon=True,
        )
        self._http_thread.start()

        viewer_url = f'http://{self._http_host}:{self._http_port}/'
        self.get_logger().info(
            'llh_osm_viewer node started: '
            f'pose={self._pose_llh_topic}, route={self._active_route_topic}, '
            f'target={self._active_target_llh_topic}, url={viewer_url}'
        )

        if self._open_browser:
            # HTTPサーバ起動直後にブラウザがアクセスすると失敗する場合があるため、
            # 1秒遅延してから開く。
            browser_timer = threading.Timer(
                1.0,
                lambda: webbrowser.open(viewer_url),
            )
            browser_timer.daemon = True
            browser_timer.start()

    def destroy_node(self) -> bool:
        """ノード破棄時にHTTPサーバも停止する."""
        if self._http_server is not None:
            self.get_logger().info('Shutting down llh_osm_viewer HTTP server.')
            self._http_server.shutdown()
            self._http_server.server_close()
            self._http_server = None
        return super().destroy_node()

    def _on_pose_llh(self, msg: GeoPoseWithQuality) -> None:
        """LLH姿勢メッセージ受信時の処理."""
        pose = _pose_llh_to_dict(msg)
        if pose is None:
            self.get_logger().warn(
                'Ignored invalid latitude/longitude in pose_llh message.'
            )
            return
        self._pose_store.update(pose)

    def _on_active_route(self, msg: Route) -> None:
        """active_route受信時の処理."""
        self._pose_store.update_route(_route_to_dict(msg, self._max_route_waypoints))

    def _on_active_target_llh(self, msg: ActiveTargetLlh) -> None:
        """active target LLH受信時の処理."""
        active_target = _active_target_to_dict(msg)
        if active_target is None:
            self.get_logger().warn(
                'Ignored invalid latitude/longitude in active_target_llh message.'
            )
            return
        self._pose_store.update_active_target(active_target)

    def _serve_http(self) -> None:
        """OpenStreetMapビューア用HTTPサーバを起動する."""
        node = self

        class Handler(BaseHTTPRequestHandler):
            """llh_osm_viewer用HTTPハンドラ."""

            def do_GET(self) -> None:  # noqa: N802
                """GETリクエストを処理する."""
                if self.path == '/' or self.path.startswith('/index.html'):
                    self._send_html()
                    return

                if self.path.startswith('/pose'):
                    self._send_pose()
                    return

                if self.path.startswith('/state'):
                    self._send_state()
                    return

                self.send_response(404)
                self.end_headers()

            def log_message(self, fmt: str, *args: Any) -> None:
                """HTTPアクセスログを抑制する."""
                return

            def _send_html(self) -> None:
                """ビューアHTMLを返す."""
                html = _make_html(
                    poll_interval_ms=node._poll_interval_ms,
                    initial_zoom=node._initial_zoom,
                    default_latitude=node._default_latitude,
                    default_longitude=node._default_longitude,
                    default_zoom=node._default_zoom,
                    triangle_height_px=node._triangle_height_px,
                )
                body = html.encode('utf-8')

                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

            def _send_pose(self) -> None:
                """最新姿勢をJSONで返す."""
                body = json.dumps(
                    {'pose': node._pose_store.get()},
                    ensure_ascii=False,
                ).encode('utf-8')

                self.send_response(200)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

            def _send_state(self) -> None:
                """最新ビューア状態をJSONで返す."""
                body = json.dumps(
                    node._pose_store.get_state(
                        node._stale_timeout_s,
                        node._lost_timeout_s,
                    ),
                    ensure_ascii=False,
                ).encode('utf-8')

                self.send_response(200)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

        try:
            self._http_server = ThreadingHTTPServer(
                (self._http_host, self._http_port),
                Handler,
            )
            self._http_server.serve_forever()
        except OSError as exc:
            self.get_logger().error(
                'Failed to start llh_osm_viewer HTTP server: '
                f'{exc}'
            )


def _make_html(
    poll_interval_ms: int,
    initial_zoom: int,
    default_latitude: float,
    default_longitude: float,
    default_zoom: int,
    triangle_height_px: float,
) -> str:
    """LeafletでOpenStreetMap表示を行うHTMLを生成する."""
    config_json = json.dumps(
        {
            'pollIntervalMs': poll_interval_ms,
            'initialZoom': initial_zoom,
            'defaultLatitude': default_latitude,
            'defaultLongitude': default_longitude,
            'defaultZoom': default_zoom,
            'triangleHeightPx': triangle_height_px,
        },
        ensure_ascii=False,
    )

    return f"""<!doctype html>
<html lang="ja">
<head>
  <meta charset="utf-8">
  <title>LLH OpenStreetMap Viewer</title>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">

  <link
    rel="stylesheet"
    href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"
    crossorigin=""
  />
  <script
    src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"
    crossorigin="">
  </script>

  <style>
    html, body {{
      height: 100%;
      margin: 0;
      font-family: sans-serif;
    }}

    #map {{
      height: 100%;
      width: 100%;
    }}

    #status {{
      position: absolute;
      z-index: 1000;
      left: 12px;
      bottom: 12px;
      min-width: 330px;
      padding: 10px 12px;
      background: rgba(255, 255, 255, 0.92);
      border-radius: 8px;
      box-shadow: 0 1px 8px rgba(0, 0, 0, 0.25);
      font-size: 13px;
      line-height: 1.45;
    }}
  </style>
</head>

<body>
<div id="map"></div>
<div id="status">Waiting for /localization/pose_llh ...</div>

<script>
'use strict';

const CONFIG = {config_json};

const map = L.map('map').setView(
  [CONFIG.defaultLatitude, CONFIG.defaultLongitude],
  CONFIG.defaultZoom
);

L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
  maxZoom: 22,
  attribution: '&copy; OpenStreetMap contributors'
}}).addTo(map);

const statusBox = document.getElementById('status');

let triangleLayer = null;
let centerMarker = null;
let routeLine = null;
let routeMarkers = [];
let activeTargetMarker = null;
let latestRouteKey = '';
let latestTargetKey = '';
let firstPoseReceived = false;
let latestPose = null;

function headingToPixelUnit(headingDeg) {{
  const rad = headingDeg * Math.PI / 180.0;
  return {{
    x: Math.sin(rad),
    y: -Math.cos(rad)
  }};
}}

function makeTriangleLatLngs(lat, lon, headingDeg) {{
  const center = L.latLng(lat, lon);
  const centerPt = map.latLngToLayerPoint(center);

  const heightPx = CONFIG.triangleHeightPx;
  const basePx = heightPx / 2.0;
  const halfBasePx = basePx / 2.0;

  const forward = headingToPixelUnit(headingDeg);
  const right = {{
    x: -forward.y,
    y: forward.x
  }};

  const apex = L.point(
    centerPt.x + forward.x * (heightPx * 2.0 / 3.0),
    centerPt.y + forward.y * (heightPx * 2.0 / 3.0)
  );

  const baseCenter = L.point(
    centerPt.x - forward.x * (heightPx / 3.0),
    centerPt.y - forward.y * (heightPx / 3.0)
  );

  const leftBase = L.point(
    baseCenter.x - right.x * halfBasePx,
    baseCenter.y - right.y * halfBasePx
  );

  const rightBase = L.point(
    baseCenter.x + right.x * halfBasePx,
    baseCenter.y + right.y * halfBasePx
  );

  return [
    map.layerPointToLatLng(apex),
    map.layerPointToLatLng(leftBase),
    map.layerPointToLatLng(rightBase)
  ];
}}

function redrawPose() {{
  if (latestPose === null) {{
    return;
  }}

  const lat = latestPose.latitude;
  const lon = latestPose.longitude;
  const headingDeg = latestPose.has_heading ? latestPose.heading_deg : 0.0;

  const latLngs = makeTriangleLatLngs(lat, lon, headingDeg);

  if (triangleLayer === null) {{
    triangleLayer = L.polygon(latLngs, {{
      color: '#d62728',
      weight: 2,
      fillColor: '#d62728',
      fillOpacity: 0.65
    }}).addTo(map);
  }} else {{
    triangleLayer.setLatLngs(latLngs);
  }}

  if (centerMarker === null) {{
    centerMarker = L.circleMarker([lat, lon], {{
      radius: 4,
      color: '#000000',
      weight: 1,
      fillColor: '#ffffff',
      fillOpacity: 1.0
    }}).addTo(map);
  }} else {{
    centerMarker.setLatLng([lat, lon]);
  }}

  if (!firstPoseReceived) {{
    map.setView([lat, lon], CONFIG.initialZoom);
    firstPoseReceived = true;
  }}
}}

function updateRoute(route) {{
  routeMarkers.forEach(marker => map.removeLayer(marker));
  routeMarkers = [];

  if (routeLine !== null) {{
    map.removeLayer(routeLine);
    routeLine = null;
  }}

  if (route === null || !Array.isArray(route.waypoints) || route.waypoints.length === 0) {{
    return;
  }}

  const latLngs = route.waypoints.map(wp => [wp.latitude, wp.longitude]);
  routeLine = L.polyline(latLngs, {{
    color: '#2563eb',
    weight: 4,
    opacity: 0.8
  }}).addTo(map);

  for (const wp of route.waypoints) {{
    const radius = wp.signal_stop || wp.line_stop ? 5 : 3;
    const marker = L.circleMarker([wp.latitude, wp.longitude], {{
      radius,
      color: wp.segment_is_fixed ? '#1d4ed8' : '#7c3aed',
      weight: 1,
      fillColor: '#ffffff',
      fillOpacity: 0.85
    }}).bindTooltip(`${{wp.index}} ${{wp.label || ''}}`);
    marker.addTo(map);
    routeMarkers.push(marker);
  }}
}}

function updateActiveTarget(target) {{
  if (activeTargetMarker !== null) {{
    map.removeLayer(activeTargetMarker);
    activeTargetMarker = null;
  }}

  if (target === null) {{
    return;
  }}

  activeTargetMarker = L.circleMarker([target.latitude, target.longitude], {{
    radius: 8,
    color: '#f97316',
    weight: 3,
    fillColor: '#fef3c7',
    fillOpacity: 0.9
  }}).bindTooltip(
    `target ${{target.target_index}} ${{target.target_label || ''}}`
  ).addTo(map);
}}

function updateStatus(state) {{
  const pose = state.pose;
  if (pose === null) {{
    statusBox.innerHTML = '<b>/localization/pose_llh</b><br>status: NO_DATA';
    return;
  }}

  const headingText = pose.has_heading
    ? `${{pose.heading_deg.toFixed(1)}} deg`
    : 'N/A';

  const altitudeText = pose.has_altitude
    ? `${{pose.altitude.toFixed(2)}} m`
    : 'N/A';

  const ageText = Number.isFinite(pose.age_s)
    ? `${{pose.age_s.toFixed(1)}} s`
    : 'N/A';

  const route = state.route;
  const routeText = route !== null
    ? `${{route.route_id || ''}} v${{route.version}} ${{route.waypoints.length}} pts`
    : 'N/A';

  const target = state.active_target;
  const targetText = target !== null
    ? `${{target.target_index}} ${{target.target_label || ''}} ` +
      `(${{target.distance_m.toFixed(1)}} m)`
    : 'N/A';

  statusBox.innerHTML =
    `<b>/localization/pose_llh</b><br>` +
    `pose status: ${{state.pose_status}} age: ${{ageText}}<br>` +
    `lat: ${{pose.latitude.toFixed(8)}}<br>` +
    `lon: ${{pose.longitude.toFixed(8)}}<br>` +
    `alt: ${{altitudeText}}<br>` +
    `heading: ${{headingText}}<br>` +
    `status: ${{pose.status_text || ''}}<br>` +
    `source: ${{pose.source}}, fix: ${{pose.fix_quality}}, fusion: ${{pose.fusion_status}}<br>` +
    `route: ${{routeText}}<br>` +
    `target: ${{targetText}}<br>` +
    `frame: ${{pose.frame_id || ''}} / ${{pose.child_frame_id || ''}}`;
}}

async function pollState() {{
  try {{
    const response = await fetch('/state', {{ cache: 'no-store' }});
    const state = await response.json();

    if (state.pose !== null) {{
      latestPose = state.pose;
      redrawPose();
    }}
    updateRoute(state.route);
    updateActiveTarget(state.active_target);
    updateStatus(state);
  }} catch (error) {{
    statusBox.textContent = `Failed to fetch state: ${{error}}`;
  }}
}}

setInterval(pollState, CONFIG.pollIntervalMs);

map.on('zoomend moveend', () => {{
  redrawPose();
}});

pollState();
</script>
</body>
</html>
"""


def main(args: Optional[list[str]] = None) -> None:
    """エントリポイント."""
    rclpy.init(args=args)
    node = LlhOsmViewerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
