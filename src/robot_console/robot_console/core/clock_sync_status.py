"""Read-only presentation of the host clock gate, independent of ROS startup."""
import json
import math
from pathlib import Path
import time


def clock_sync_status(path='/run/icart-clock/status.json', *, mono=None, boot_id=None):
    """Return (ready, reason); stale or unreadable state never implies readiness."""
    try:
        now = time.monotonic() if mono is None else mono
        boot = (Path('/proc/sys/kernel/random/boot_id').read_text().strip()
                if boot_id is None else boot_id)
        state = json.loads(Path(path).read_text())
        age = now - float(state['checked_monotonic'])
        if state.get('boot_id') != boot or not math.isfinite(age) or not 0 <= age <= 3:
            return False, '時刻同期の監視情報が更新されていません。'
        if state.get('clock_source') != 'ntp':
            return False, 'PCの時刻同期がNTP設定になっていません。'
        if state.get('ready') is True:
            return True, ''
        if state.get('status_version') != 2:
            return False, '時刻同期サービスの更新が必要です。NTP・有線LANの判定を分離したサービスを適用してください。'
        if state.get('check_error') or state.get('tools_ready') is not True:
            return False, '時刻同期の確認処理に異常があります。時刻同期サービスを確認してください。'
        if state.get('clock_ready') is not True:
            reason = 'PCのNTP同期の品質条件が未成立です。ネット接続・応答の鮮度・時刻誤差を確認してください。'
            if state.get('link_ready') is not True:
                reason += ' LiDAR用有線LANも未接続です。'
            return False, reason
        if state.get('link_ready') is not True:
            return False, 'LiDAR用有線LANが未接続です。LiDAR・スイッチの電源とLANケーブルを確認してください。'
        if state.get('ptp_running') is not True:
            return False, 'LiDARへのPTP時刻配信を待っています。'
        return False, 'LiDAR・IMUの時刻同期を待っています。センサの電源・接続を確認してください。'
    except (OSError, ValueError, KeyError, TypeError, AttributeError):
        return False, '時刻同期の状態を取得できません。時刻同期サービスを確認してください。'
