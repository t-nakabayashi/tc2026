"""MapView（Leaflet+OSM地図Widget）の単体テスト（`QT_QPA_PLATFORM=offscreen` 前提）。"""

import json

import pytest
from PyQt5 import QtWidgets

from robot_console.core.snapshot_model import (
    LocalizationStateView,
    RouteView,
    RouteWaypointView,
    TargetView,
)
from robot_console.ui_qt.widgets.map_view import (
    DEFAULT_LATITUDE,
    DEFAULT_LONGITUDE,
    DEFAULT_ZOOM,
    MapView,
    build_map_html,
    build_update_markers_script,
    build_update_route_script,
)


@pytest.fixture(scope='module')
def qt_app():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    yield app


def test_build_map_html_embeds_leaflet_cdn_and_default_center():
    html = build_map_html()

    assert 'leaflet@1.9.4' in html
    assert str(DEFAULT_LATITUDE) in html
    assert str(DEFAULT_LONGITUDE) in html
    assert f'DEFAULT_ZOOM = {DEFAULT_ZOOM};' in html


def test_build_map_html_accepts_custom_center():
    html = build_map_html(default_latitude=1.0, default_longitude=2.0, default_zoom=10)

    assert 'setView([1.0, 2.0], DEFAULT_ZOOM)' in html
    assert 'DEFAULT_ZOOM = 10;' in html


def test_build_update_markers_script_serializes_current_and_target():
    localization = LocalizationStateView(latitude=36.083, longitude=140.113)
    target = TargetView(latitude=36.0832, longitude=140.1132)

    script = build_update_markers_script(localization, target)

    assert script.startswith('updateMarkers(')
    assert json.dumps({'latitude': 36.083, 'longitude': 140.113}) in script
    assert json.dumps({'latitude': 36.0832, 'longitude': 140.1132}) in script


def test_build_update_markers_script_handles_missing_position():
    localization = LocalizationStateView()
    target = TargetView()

    script = build_update_markers_script(localization, target)

    assert json.dumps({'latitude': None, 'longitude': None}) in script


def test_build_update_route_script_serializes_waypoints_and_current_index():
    route = RouteView(
        current_index=1,
        waypoints=[
            RouteWaypointView(index=0, latitude=36.083, longitude=140.113),
            RouteWaypointView(index=1, latitude=36.0831, longitude=140.1131),
        ],
    )

    script = build_update_route_script(route)

    assert script.startswith('updateRoute(')
    assert json.dumps({'index': 0, 'latitude': 36.083, 'longitude': 140.113}) in script
    assert script.endswith('1);')


def test_build_update_route_script_handles_empty_waypoints():
    route = RouteView()

    script = build_update_route_script(route)

    assert script == 'updateRoute([], 0);'


def test_map_view_update_map_does_not_raise(qt_app):
    view = MapView()
    localization = LocalizationStateView(latitude=36.083, longitude=140.113)
    target = TargetView(latitude=36.0832, longitude=140.1132)

    view.update_map(localization, target)


def test_map_view_update_route_does_not_raise(qt_app):
    view = MapView()
    route = RouteView(
        current_index=0,
        waypoints=[RouteWaypointView(index=0, latitude=36.083, longitude=140.113)],
    )

    view.update_route(route)


def test_map_html_fits_route_bounds_on_first_waypoints():
    """自己位置未受信でも初回waypoint受信で地図がroute範囲へ移動する（HTML観測UIと同じ挙動）。"""

    html = build_map_html()

    assert 'hasFitRouteBounds' in html
    assert 'map.fitBounds(' in html


def test_build_update_route_script_marks_all_waypoints_traveled_on_completion():
    """完走時は最終waypointまで走行済みとして色分けされる（PyQt5側もHTML側と同じ挙動）。"""

    route = RouteView(
        current_index=2,
        total_waypoints=3,
        is_completed=True,
        waypoints=[
            RouteWaypointView(index=0, latitude=36.083, longitude=140.113),
            RouteWaypointView(index=1, latitude=36.0831, longitude=140.1131),
            RouteWaypointView(index=2, latitude=36.0832, longitude=140.1132),
        ],
    )

    script = build_update_route_script(route)

    assert script.endswith('3);')


def test_build_update_route_script_uses_current_index_while_running():
    route = RouteView(
        current_index=1,
        total_waypoints=3,
        waypoints=[RouteWaypointView(index=0, latitude=36.083, longitude=140.113)],
    )

    script = build_update_route_script(route)

    assert script.endswith('1);')
