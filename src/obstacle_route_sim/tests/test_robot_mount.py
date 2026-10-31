"""模擬ロボットのセンサ取付が実機の計測値と一致することを固定する。Gazebo を起動しない。

実機の取付は `icart_bringup/params/hardware.yaml` を正とする。模擬側がこれと食い違うと、
模擬で詰めた層の高さ・画像仕様・回避幅がそのままでは実機に通用しない。
"""

from pathlib import Path
import math
import xml.etree.ElementTree as ET

import pytest
import yaml

ROOT = Path(__file__).parents[3]
MODEL = Path(__file__).parents[1] / 'models/robot/model.sdf'
HARDWARE = ROOT / 'src/icart_bringup/params/hardware.yaml'

# 模擬モデルの車輪半径。床面から base_link 原点までの高さに等しい。
FLOOR_TO_BASE_LINK_M = 0.15


def _links() -> dict:
    tree = ET.parse(MODEL)
    return {link.get('name'): link for link in tree.getroot().iter('link')}


def _pose(name: str) -> list:
    return [float(value) for value in _links()[name].findtext('pose').split()]


def _hardware() -> dict:
    return yaml.safe_load(HARDWARE.read_text(encoding='utf-8'))


def test_mid360_height_matches_the_measured_mount() -> None:
    """MID-360 の床上高さが実機の計測値と一致する.

    hardware.yaml は base_link（車軸中心）からの値を持つ。模擬モデルは床面基準の
    絶対値で書くため、車輪半径を足して比較する。
    """
    expected = float(_hardware()['mid360']['xyz'][2]) + 0.148248

    assert _pose('mid360_frame')[2] == pytest.approx(expected, abs=0.01)


def test_mid360_is_pitched_down_as_mounted() -> None:
    """MID-360 が実機と同じ前下がり角・ロール角で取り付けられている.

    前下がり取付によって、停止判定が起きる距離でも低い層を観測できる。水平取付では
    床上 0.35 m の層が 1.7 m 以遠でしか観測できず、層の設計が成立しない。
    """
    roll, pitch = _pose('mid360_frame')[3], _pose('mid360_frame')[4]
    expected_roll, expected_pitch = _hardware()['mid360']['rpy_deg'][:2]

    assert math.degrees(roll) == pytest.approx(expected_roll, abs=0.1)
    assert math.degrees(pitch) == pytest.approx(expected_pitch, abs=0.1)


def test_top_urg_is_mounted_below_the_mid360() -> None:
    """topURG が MID-360 より低い位置にある.

    層は topURG と MID-360 設置高の間に置く。上下関係が逆だと層の高さを選べない。
    """
    assert _pose('laser')[2] < _pose('mid360_frame')[2]


def test_front_camera_is_present_for_advice_input() -> None:
    """AI 助言の入力に使うフロントカメラがあり、実機と同じ画素数で配信する."""
    camera = _links()['camera_link'].find('.//sensor[@name="front_camera"]')
    hardware_camera = _hardware()['camera']

    assert camera.findtext('topic') == '/usb_cam/image_raw'
    assert int(camera.findtext('.//image/width')) == int(hardware_camera['width'])
    assert int(camera.findtext('.//image/height')) == int(hardware_camera['height'])


def test_no_link_blocks_the_mid360_view_at_its_own_height() -> None:
    """MID-360 の高さに、視界を塞ぐ車体や支持台が無い.

    支持台や車体の上面がセンサ高さを超えると、前方の層に自機の構造が写り込む。
    """
    sensor_z = _pose('mid360_frame')[2]
    post_top = _pose('sensor_post')[2] + 0.112 / 2.0
    body_top = FLOOR_TO_BASE_LINK_M + 0.15 + 0.3 / 2.0

    assert post_top <= sensor_z + 1e-6
    assert body_top < sensor_z


def test_static_transforms_use_base_link_relative_values() -> None:
    """launch が配信する静的 TF が、base_link 相対の値になっている.

    SDF の link pose は model frame（床面）基準であり、base_link 相対の TF とは
    車輪半径の分だけ異なる。取り違えると層の切り出し高さが 0.15 m ずれる。
    """
    launch_text = (Path(__file__).parents[1]
                   / 'launch/sim_obstacle_route.launch.py').read_text(encoding='utf-8')
    laser_z = _pose('laser')[2] - FLOOR_TO_BASE_LINK_M
    mid360_z = _pose('mid360_frame')[2] - FLOOR_TO_BASE_LINK_M

    assert f'"{laser_z:.2f}"' in launch_text
    assert f'"{mid360_z:.3f}"' in launch_text
