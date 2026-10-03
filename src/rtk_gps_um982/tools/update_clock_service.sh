#!/bin/bash
# Apply the clock monitor and polling configuration to an existing installation.
set -euo pipefail
if [[ $EUID -ne 0 ]]; then echo 'Run with sudo' >&2; exit 1; fi
if pgrep -f '(^|/)(fastlio_mapping|fusion_node|ypspur-coordinator|robot_navigator)( |$)' >/dev/null; then
  echo 'Stop localization and motion processes first' >&2; exit 1
fi
repo_dir=$(cd -- "$(dirname -- "$0")/../../.." && pwd)
test -f /etc/systemd/system/icart-clock.service
backup_dir=$(mktemp -d /var/backups/robot-clock-update-XXXXXXXX)
cp -a /usr/local/libexec/icart-clock "$backup_dir/"
python3 "$repo_dir/src/rtk_gps_um982/tools/configure_ntp_poll.py" --backup-directory "$backup_dir"
systemctl stop icart-clock.service
install -m 644 "$repo_dir/src/rtk_gps_um982/rtk_gps_um982/clock_service.py" /usr/local/libexec/icart-clock/
install -m 644 "$repo_dir/src/rtk_gps_um982/rtk_gps_um982/ptp_trial.py" /usr/local/libexec/icart-clock/
systemctl restart chrony.service
systemctl start icart-clock.service
systemctl is-active chrony icart-clock
printf 'Applied. Backup: %s\n' "$backup_dir"
