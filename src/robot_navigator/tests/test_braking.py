"""P0-1: asymmetric motion limits and braking with delay, no ROS graph needed."""
from pathlib import Path
import math
import sys
from types import SimpleNamespace
import pytest
import yaml
sys.path.insert(0, str(Path(__file__).parents[1]))
from robot_navigator.braking import (stopping_distance, speed_for_distance,
    limit_for_stop, validate_limits, driver_compatible)

@pytest.mark.parametrize('speed', [.1, .5, 1.0])
def test_braking_with_delay_and_inverse(speed):
    distance = stopping_distance(speed, 1.5, .25)
    assert distance == pytest.approx(speed*.25+speed**2/3)
    assert speed_for_distance(distance, 1.5, .25) == pytest.approx(speed)
    # Brake as soon as the measured velocity needs the available space.
    assert limit_for_stop(speed*.1, speed, speed, distance, 1.5, .25) == 0.

@pytest.mark.parametrize('speed', [.1, .5, 1.0])
def test_delayed_discrete_braking_does_not_cross_obstacle_margin(speed):
    # 0.25 s pipeline budget, then coordinator's 15 ms discrete slew.
    dt=.001
    clearance=stopping_distance(speed,1.5,.25)+.02
    assert limit_for_stop(speed, speed, speed, clearance-.02,1.5,.25)==0
    distance=0.; velocity=speed
    for i in range(2000):
        distance += velocity*dt
        if i*dt >= .25 and i % 15 == 0:
            velocity=max(0., velocity-1.5*.015)
    assert distance < clearance


def test_heading_reduction_and_previous_command_cannot_hide_momentum():
    assert limit_for_stop(.05,1.,0.,.5,1.5,.25)==0.
    assert limit_for_stop(.05,0.,1.,.5,1.5,.25)==0.
    assert 0 < limit_for_stop(1.,.1,.1,.4,1.5,.25) < 1.

@pytest.mark.parametrize('bad', [0., -1., math.nan, math.inf])
def test_invalid_deceleration_rejected(bad):
    with pytest.raises(ValueError):
        validate_limits(1.,1.,.7,.6,bad,.2,20.)


def test_icart_middle_calibration_and_three_layers_agree():
    root=Path(__file__).resolve().parents[3]
    cfg=yaml.safe_load((root/'src/ypspur_ros2/config/default.yaml').read_text())['/ypspur_node']['ros__parameters']
    nav=yaml.safe_load((root/'src/robot_navigator/params/default.yaml').read_text())['robot_navigator']['ros__parameters']
    raw=(root/'src/ypspur_ros2/config/icart-middle.param').read_text()
    hardware={r.split()[0]:float(r.split()[1]) for r in raw.splitlines() if r.strip() and not r.startswith('#')}
    assert cfg['acceleration_max']['linear']==nav['max_acc_v']==.7
    assert cfg['deceleration_max']['linear']==nav['max_decel_v']==hardware['MAX_ACC_V']==1.5
    assert cfg['velocity_max']['linear']==nav['max_vel']<=hardware['MAX_VEL']
    assert cfg['velocity_max']['angular']==nav['max_w']<=hardware['MAX_W']
    # navigator は要求能力の下限、driver は実際に使用する上限を設定する。
    assert nav['max_acc_w'] <= cfg['acceleration_max']['angular'] <= hardware['MAX_ACC_W']
    assert hardware['RADIUS[0]']==-.148248 and hardware['RADIUS[1]']==.148248
    assert hardware['TREAD']==.30737 and hardware['GEAR']==150
    actual=SimpleNamespace(max_linear_velocity=1.,max_angular_velocity=1.,
        linear_acceleration=.7,linear_deceleration=1.5,
        angular_acceleration=cfg['acceleration_max']['angular'])
    assert driver_compatible(actual,1.,1.,.7,.6,1.5)
    actual.angular_acceleration = nav['max_acc_w'] / 2
    assert not driver_compatible(actual,1.,1.,.7,nav['max_acc_w'],1.5)
    actual.angular_acceleration = cfg['acceleration_max']['angular']
    for bad in [.3, 1.0, math.nan]:
        actual.linear_deceleration=bad
        assert not driver_compatible(actual,1.,1.,.7,.6,1.5)
