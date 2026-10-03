"""起動設定で使う地域別RTK局一覧。接続設定は共通bringupのカタログを読む。"""
from pathlib import Path
import yaml


def station_choices(site):
    from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
    try:
        path = Path(get_package_share_directory('icart_bringup'))/'params/rtk_stations.yaml'
    except PackageNotFoundError:
        path = Path(__file__).resolve().parents[3]/'icart_bringup/params/rtk_stations.yaml'
    catalog = yaml.safe_load(path.read_text())
    region = catalog['sites'].get({'稲城': 'inagi', 'つくば': 'tsukuba'}.get(site))
    choices = [('地域の既定局', '地域の既定局')]
    if region:
        choices += [(catalog['stations'][key]['label'], key) for key in region['stations']
                    if key not in ('none', 'custom')]
    choices += [('NTRIPなし', 'NTRIPなし'), (catalog['stations']['custom']['label'], 'custom')]
    return choices
