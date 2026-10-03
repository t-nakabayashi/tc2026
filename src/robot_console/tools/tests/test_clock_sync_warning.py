import json
import pytest
from robot_console.core.clock_sync_status import clock_sync_status


@pytest.mark.parametrize('change,expected,absent', [
    ({'link_ready': False}, '有線LANが未接続', 'NTP同期の品質条件が未成立'),
    ({'clock_ready': False}, 'NTP同期の品質条件が未成立', '有線LANも未接続'),
    ({'clock_ready': False, 'link_ready': False}, '有線LANも未接続', 'PTP時刻配信を待って'),
    ({'ptp_running': False}, 'PTP時刻配信を待って', 'NTP同期の品質条件が未成立'),
    ({}, 'LiDAR・IMUの時刻同期を待って', 'NTP同期の品質条件が未成立'),
    ({'tools_ready': False}, '確認処理に異常', 'NTP同期の品質条件が未成立'),
    ({'check_error': 'failed'}, '確認処理に異常', 'NTP同期の品質条件が未成立'),
    ({'status_version': None}, 'サービスの更新が必要', 'NTP同期の品質条件が未成立'),
])
def test_warning_identifies_failed_stage(tmp_path, change, expected, absent):
    state = dict(ready=False, status_version=2, clock_source='ntp', boot_id='a',
                 checked_monotonic=9, clock_ready=True, link_ready=True,
                 tools_ready=True, ptp_running=True)
    state.update(change)
    path = tmp_path/'status.json'
    path.write_text(json.dumps(state))
    ready, reason = clock_sync_status(path, mono=10, boot_id='a')
    assert not ready and expected in reason and absent not in reason


def test_readiness_requires_live_ntp_status(tmp_path):
    p=tmp_path/'status.json'
    assert not clock_sync_status(p,mono=10,boot_id='a')[0]
    good=dict(ready=True,clock_source='ntp',boot_id='a',checked_monotonic=9)
    p.write_text(json.dumps(good))
    assert clock_sync_status(p,mono=10,boot_id='a')[0]
    for change in [dict(checked_monotonic=6),dict(boot_id='b'),dict(clock_source='gnss'),
                   dict(checked_monotonic=float('nan'))]:
        p.write_text(json.dumps(dict(good,**change)))
        assert not clock_sync_status(p,mono=10,boot_id='a')[0]
    for invalid in ['null','[]','{broken']:
        p.write_text(invalid)
        assert not clock_sync_status(p,mono=10,boot_id='a')[0]


def test_warning_wait_loss_recovery_and_simulation():
    from PyQt5 import QtWidgets
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    from robot_console.ui_qt.widgets.clock_sync_warning import ClockSyncWarning
    state=[False,'NTP未同期']
    widget=ClockSyncWarning(reader=lambda:tuple(state))
    widget.set_environment('実機')
    assert not widget.isHidden() and '未成立' in widget.text()
    state[0]=True;widget.refresh();assert widget.isHidden()
    state[0]=False;widget.refresh()
    assert not widget.isHidden() and '失われました' in widget.text()
    state[0]=True;widget.refresh();assert widget.isHidden()
    widget.set_environment('デジタルツイン')
    state[0]=False;widget.refresh();assert widget.isHidden()
    widget.close()
