import json
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from icp_localization.registration import Engine, load_manifest, sha256
from icp_localization.tracker import quality_ok


PROJECTION = dict(origin_latitude=36., origin_longitude=140., origin_altitude=20.,
                  map_yaw_offset_rad=0., map_frame_id='map', datum='WGS84')


def fixture_map(tmp_path, plane_only=False):
    rng = np.random.default_rng(13)
    xy = rng.uniform(-8, 8, (6000, 2))
    ground = np.column_stack([xy, np.zeros(len(xy))])
    points = ground
    if not plane_only:
        walls = [np.column_stack([np.full(4000, 7.), rng.uniform(-8, 8, 4000), rng.uniform(0, 5, 4000)]),
                 np.column_stack([rng.uniform(-8, 8, 4000), np.full(4000, 6.), rng.uniform(0, 5, 4000)])]
        points = np.vstack([ground, *walls])
    np.save(tmp_path/'points.npy', points)
    anchor = np.eye(4); anchor[2, 3] = 1.
    np.save(tmp_path/'anchors.npy', anchor[None])
    manifest = dict(schema=1, projection=PROJECTION,
                    points=dict(path='points.npy', sha256=sha256(tmp_path/'points.npy')),
                    base_anchors=dict(path='anchors.npy', sha256=sha256(tmp_path/'anchors.npy')))
    path = tmp_path/'map.json'; path.write_text(json.dumps(manifest))
    return points, load_manifest(path, PROJECTION), path


def test_actual_plane_icp_recovers_3d_pose(tmp_path):
    points, manifest, _ = fixture_map(tmp_path)
    engine = Engine(manifest, np.eye(4), [0., 0., 0.])
    truth = np.eye(4)
    truth[:3, :3] = Rotation.from_euler('xyz', [.02, -.03, .2]).as_matrix()
    truth[:3, 3] = [1., 2., 1.]
    scan = (points-truth[:3, 3]) @ truth[:3, :3]
    seed = truth.copy(); seed[:3, 3] += [.2, -.15, .1]
    pose, metrics = engine.register(scan, seed)
    assert np.linalg.norm(pose[:3, 3]-truth[:3, 3]) < .04
    assert quality_ok(metrics)


def test_flat_ground_is_degenerate(tmp_path):
    points, manifest, _ = fixture_map(tmp_path, True)
    engine = Engine(manifest, np.eye(4), [0., 0., 0.])
    seed = np.eye(4); seed[2, 3] = .1
    _, metrics = engine.register(points, seed)
    assert metrics['information_ratio'] < .0001
    assert not quality_ok(metrics)


def test_projection_and_hash_mismatch_rejected(tmp_path):
    _, _, path = fixture_map(tmp_path)
    with pytest.raises(ValueError, match='projection mismatch'):
        load_manifest(path, dict(PROJECTION, origin_latitude=36.1))
    with (tmp_path/'points.npy').open('ab') as stream:
        stream.write(b'changed')
    with pytest.raises(ValueError, match='hash mismatch'):
        load_manifest(path, PROJECTION)


def test_seed_antenna_offset_yaw_and_map_height(tmp_path):
    _, manifest, _ = fixture_map(tmp_path)
    mount = np.eye(4); mount[2, 3] = .4
    engine = Engine(manifest, mount, [-.3, 0., .6])
    fix = np.array([0., 0., np.pi/2])
    seed = engine.seed(fix)
    assert np.allclose(engine.antenna_xy(seed), fix[:2])
    assert engine.base_pose(seed)[2, 3] == pytest.approx(1.)
    assert seed[2, 3] == pytest.approx(1.4)
    with pytest.raises(ValueError, match='outside'):
        engine.seed(np.array([100., 100., 0.]))
