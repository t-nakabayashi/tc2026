"""Qt/Web共通の地図挙動をNodeで実行検証する（外部タイル通信なし）。"""
import ast
from pathlib import Path
import shutil
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[3] / 'robot_console'


@unittest.skipUnless(shutil.which('node'), 'Node.js is required')
class MapBehaviorTest(unittest.TestCase):
    def test_route_framing_and_zoom_in_both_frontends(self):
        tree = ast.parse((ROOT / 'ui_qt/widgets/map_view.py').read_text())
        template = next(
            ast.literal_eval(node.value) for node in tree.body
            if isinstance(node, ast.Assign)
            and any(isinstance(t, ast.Name) and t.id == '_MAP_HTML_TEMPLATE'
                    for t in node.targets)
        )
        qt_script = template.split('<script>')[1].split('</script>')[0]
        qt_script = (qt_script.replace('__DEFAULT_ZOOM__', '18')
                     .replace('__DEFAULT_LAT__', '36.083')
                     .replace('__DEFAULT_LON__', '140.113'))
        web_script = (ROOT / 'web/static/app.js').read_text().split('\ninitMap();')[0]
        mock = '''
const assert = require('node:assert/strict');
let fits = [], centers = [], tiles = [];
const fakeMap = {
  setView(p, z) { centers.push([p, z]); return this; },
  fitBounds(p, options) { fits.push([p, options]); return this; },
  removeLayer() {}, invalidateSize() {}
};
function layer() { return {
  addTo() { return this; }, bindTooltip() { return this; },
  setLatLng() {}, setStyle() {}, setLatLngs() {}
}; }
const L = {map() { return fakeMap; },
  tileLayer(url, opts) { tiles.push(opts); return layer(); },
  circleMarker: layer, polyline: layer};
const window = {addEventListener() {}};
const status = {textContent: ''};
const document = {getElementById() { return status; }};
const setTimeout = () => {};
'''
        checks = '''
assert.equal(tiles[0].maxNativeZoom, 19);
assert.equal(tiles[0].maxZoom, 22);
const points = [{index: 0, latitude: 35.65, longitude: 139.50},
                {index: 1, latitude: 35.651, longitude: 139.501}];
const pos = {latitude: 36.083, longitude: 140.113};
POSITION_FIRST
pushRoute(points);
assert.equal(fits.length, 1, 'route must fit even after current position arrives');
const centersBefore = centers.length;
pushPosition(pos);
assert.equal(centers.length, centersBefore, 'position must not override route framing');
pushRoute(points, 1);
assert.equal(fits.length, 1, 'progress must preserve user zoom');
const moved = points.map(p => ({...p, longitude: p.longitude + 0.01}));
pushRoute(moved);
assert.equal(fits.length, 2, 'same-size route replacement must fit');
assert.equal(fits[1][0][0][1], 139.51);
fitRouteBounds();
assert.equal(fits.length, 3, 'manual route overview');
pushRoute([]);
fitRouteBounds();
assert.equal(fits.length, 3, 'empty route must not move the map');
pushRoute(points.slice(0, 1));
assert.equal(fits.at(-1)[1].maxZoom, 18, 'single point overview must limit initial zoom');
'''
        adapters = {
            'qt': (qt_script, '''
const pushRoute = (points, traveled=0) => updateRoute(points, traveled);
const pushPosition = pos => updateMarkers(pos, null);
'''),
            'web': (web_script + '\ninitMap();', '''
const pushRoute = (points, traveled=0) => updateRouteOverlay({route: {
  waypoints: points, traveled_waypoint_count: traveled}});
const pushPosition = pos => updateMapMarkers({localization: pos,
  target: {latitude: null, longitude: null}});
'''),
        }
        for name, (script, adapter) in adapters.items():
            for position_first in (False, True):
                with self.subTest(frontend=name, position_first=position_first):
                    result = subprocess.run(
                        ['node', '-e', mock + script + adapter + checks.replace(
                            'POSITION_FIRST', 'pushPosition(pos);' if position_first else '')],
                        capture_output=True, text=True, timeout=10,
                    )
                    self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
