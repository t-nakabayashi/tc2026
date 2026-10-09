"""隔離ROS環境でICP入力から経路・路面・品質・保存までを確認する。"""
import json
import numpy as np
import pytest
import rclpy
import yaml
from rclpy.parameter import Parameter
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header, String
from geo_pose_converter.geo_core import ProjectionConfig
from route_survey.recorder_node import Recorder


@pytest.fixture
def recorder(tmp_path):
    rclpy.init()
    projection = tmp_path/'projection.yaml'
    projection.write_text(yaml.safe_dump({'/**': {'ros__parameters': vars(ProjectionConfig(36., 140., 20.))}}))
    values = dict(output_directory=str(tmp_path/'survey'), projection_config=str(projection),
                  localization_mode='icp', icp_map_manifest='/maps/fix/map_manifest.json',
                  lidar_forward_m=-.04, lidar_left_m=.02, lidar_height_m=.6,
                  lidar_mount_pitch_deg=26.9, spacing_m=.5)
    node = Recorder(parameter_overrides=[Parameter(k, value=v) for k,v in values.items()])
    node.now = lambda: 10.
    yield node
    node.destroy_node()
    rclpy.shutdown()


def pose(t, x=100., y=200., z=3.):
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    ns = int(round(t*1e9)); msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 10**9)
    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = x,y,z
    q = Rotation.from_euler('xyz', [3,5,45], degrees=True).as_quat()
    o = msg.pose.pose.orientation
    o.x,o.y,o.z,o.w = map(float,q)
    msg.pose.covariance[0] = msg.pose.covariance[7] = .09
    return msg


def status(node, t, state='ICP_TRACKING'):
    node.icp_health(String(data=json.dumps(dict(state=state, estimate_stamp_s=t,
                      reason='test', map_correction_age_s=.1))))


def test_icp_route_uses_base_pose_without_gnss_or_fused_odometry(recorder):
    n = recorder
    topics = {s.topic_name for s in n.subscriptions}
    assert '/localization/icp_status' in topics
    assert '/lio/odometry' not in topics and '/fusion/status' not in topics
    n.command(String(data='start'))
    assert not n.survey.active
    status(n,10.); n.pose(pose(10.)); n.command(String(data='start'))
    fix = NavSatFix(); fix.header.stamp.sec=10
    fix.status.status=2; fix.latitude=36.01; fix.longitude=140.01
    n.gnss_fix(fix)
    for i in range(1,9):
        t=10.+i*.1; n.now=lambda t=t:t
        status(n,t); n.pose(pose(t,x=100.+i*.1))
    assert 100.4 < n.survey.rows[-1]['x'] <= 100.8
    assert n.survey.rows[0]['y'] == 200.
    assert not n.pose_quality['uncertain']
    status(n,10.9,'LIO_ONLY'); n.now=lambda:10.9; n.pose(pose(10.9,x=100.9))
    assert n.current_width()['left'] == n.current_width()['right'] == 0.
    assert n.pose_quality['uncertain']
    n.now=lambda:12.; n.command(String(data='finish')); n.flush(wait=True)
    data=json.loads((n.directory/'survey.json').read_text())
    assert data['active'] is False
    assert data['traces']['source']=='icp'
    assert data['traces']['icp_map_manifest']=='/maps/fix/map_manifest.json'
    assert data['traces']['gnss'][0]['x'] > 500.
    assert data['traces']['fused'][0]['x'] == 100.
    assert all(p['source']=='icp' for p in data['traces']['fused'])


def test_icp_cloud_preserves_map_height_and_mount_applied_once(recorder):
    n=recorder; status(n,10.); n.pose(pose(10.))
    bx,by=np.meshgrid(np.arange(-.59,.6,.025),np.arange(-2.99,3.,.025))
    ground=np.column_stack([bx.ravel(),by.ravel(),np.zeros(bx.size)])
    lever=np.array([-.04,.02,.6])
    mount=Rotation.from_euler('y',26.9,degrees=True)
    sensor=mount.inv().apply(ground-lever)
    fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1)
            for i,name in enumerate(('x','y','z','intensity'))]
    header=Header(frame_id='body');header.stamp.sec=10
    cloud=create_cloud(header,fields,np.column_stack([sensor,np.full(len(sensor),15.)]).astype(np.float32))
    n.cloud(cloud)
    assert n.width['material_checked']
    assert n.width['left'] > 2. and n.width['right'] > 2.
    world=np.array(list(n.voxels.values()))
    base=Rotation.from_euler('xyz',[3,5,45],degrees=True)
    recovered=base.inv().apply(world-[100.,200.,3.])
    assert abs(recovered[:,2]).max() < 1e-6
    assert len(n.surface_window.frames)==1


def test_slow_save_keeps_pose_input_live_and_reports_saved_only_after_finish(recorder,monkeypatch):
    import threading
    from types import SimpleNamespace
    from route_survey import recorder_node
    n=recorder; original=recorder_node.save
    entered,release=threading.Event(),threading.Event()
    def slow_save(*args,**kwargs):
        entered.set()
        assert release.wait(5.)
        return original(*args,**kwargs)
    monkeypatch.setattr(recorder_node,'save',slow_save)
    published=[]
    n.status=SimpleNamespace(publish=lambda msg:published.append(json.loads(msg.data)))
    try:
        status(n,10.);n.pose(pose(10.));n.command(String(data='start'))
        assert entered.wait(2.)
        status(n,10.1);n.now=lambda:10.1;n.pose(pose(10.1,x=100.1))
        assert n.poses[-1].x==100.1 and not n.save_future.done()
        n.command(String(data='finish'))
        assert published[-1]['saving'] and not published[-1]['saved']
    finally:
        release.set()
    n.flush(wait=True)
    assert published[-1]['saved'] and not published[-1]['saving']
    assert not json.loads((n.directory/'survey.json').read_text())['active']


def test_save_error_is_reported_and_finish_can_retry(recorder,monkeypatch):
    from types import SimpleNamespace
    from route_survey import recorder_node
    n=recorder;original=recorder_node.save
    def fail(*args,**kwargs):raise OSError('test disk failure')
    published=[]
    n.status=SimpleNamespace(publish=lambda msg:published.append(json.loads(msg.data)))
    monkeypatch.setattr(recorder_node,'save',fail)
    status(n,10.);n.pose(pose(10.));n.command(String(data='start'))
    n.command(String(data='finish'));n.flush(wait=True)
    assert 'test disk failure' in published[-1]['error'] and not published[-1]['saved']
    monkeypatch.setattr(recorder_node,'save',original)
    n.command(String(data='finish'));n.flush(wait=True)
    assert published[-1]['saved'] and not published[-1]['error']
