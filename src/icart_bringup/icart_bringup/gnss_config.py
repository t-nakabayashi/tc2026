"""GNSS単独起動と実機共通起動の基地局選択・受信設定。"""

SITE_NAMES = {'稲城': 'inagi', 'つくば': 'tsukuba', 'inagi': 'inagi', 'tsukuba': 'tsukuba'}


def resolve_selection(catalog, site, station):
    site_id = SITE_NAMES.get(site)
    if site_id is None or site_id not in catalog['sites']:
        raise ValueError('起動・設定で場所（稲城／つくば）を選択してください')
    region = catalog['sites'][site_id]
    if station in ('地域の既定局', '', None):
        station_id = region['default_station']
    elif station == 'NTRIPなし':
        station_id = 'none'
    else:
        labels = {catalog['stations'][key]['label']: key for key in region['stations']}
        station_id = labels.get(station, station)
    if station_id not in region['stations']:
        raise ValueError('選択した場所で利用できるRTK補正局を選択してください')
    return site_id, station_id


def gnss_parameters(catalog, hardware, site, station=None, custom=None):
    # Connection validation and station catalog are shared with session generation.
    from .hardware_core import station_config
    station, ntrip = station_config(catalog, site, station, custom)
    gnss = hardware['gnss']
    params = {'serial.port': gnss['serial_port'], 'serial.baud': gnss['baud'],
              'ntrip.site': site, 'ntrip.station_id': station,
              'ntrip.station_label': catalog['stations'][station]['label'],
              'frame_id': 'gnss_main', 'stamp_source': 'gnss_utc',
              'transport_delay_ms': 0, 'use_sim_time': False,
              'time_sync.enabled': gnss['time_sync_enabled'],
              'time_sync.chrony_socket': '/run/chrony/um982.sock',
              **{'ntrip.'+key: value for key, value in ntrip.items()}}
    if not ntrip['enabled']:
        # A disabled source must not retain a station from an input config file.
        params.update({'ntrip.host': '', 'ntrip.port': 2101, 'ntrip.mountpoint': '',
                       'ntrip.user': '', 'ntrip.password': ''})
    return params
