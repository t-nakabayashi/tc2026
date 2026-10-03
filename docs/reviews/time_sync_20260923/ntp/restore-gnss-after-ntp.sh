#!/bin/bash
set -euo pipefail
[[ $EUID -eq 0 ]] || exit 1
if pgrep -f '(^|/)(fastlio_mapping|fusion_node|ypspur-coordinator|robot_navigator)( |$)' >/dev/null; then
 echo 'Stop localization and motion first'; exit 1
fi
systemctl stop icart-clock
cp -a /var/backups/robot-clock-ntp-evaluation/um982.conf /etc/chrony/conf.d/um982.conf
rm -f /etc/systemd/system/icart-clock.service.d/ntp-evaluation.conf
systemctl daemon-reload
systemctl restart chrony
systemctl start icart-clock
