#!/bin/bash
set -euo pipefail
[[ $EUID -eq 0 ]] || { echo 'Run with sudo'; exit 1; }
if pgrep -f '(^|/)(fastlio_mapping|fusion_node|ypspur-coordinator|robot_navigator)( |$)' >/dev/null; then
 echo 'Stop localization and motion first'; exit 1
fi
backup=/var/backups/robot-clock-ntp-evaluation
mkdir -p "$backup"
if [[ ! -f "$backup/um982.conf" ]]; then
 cp -a /etc/chrony/conf.d/um982.conf "$backup/um982.conf"
fi
systemctl stop icart-clock
# Keep the SOCK for observing RMC, but never select it as a clock source.
cat > /etc/chrony/conf.d/um982.conf <<'EOF'
refclock SOCK /run/chrony/um982.sock refid UM98 poll 0 filter 4 precision 0.1 offset 0.0 noselect
EOF
install -m 644 /home/nkb/colcon_ws/src/rtk_gps_um982/rtk_gps_um982/ptp_trial.py /usr/local/libexec/icart-clock/
install -m 644 /home/nkb/colcon_ws/src/rtk_gps_um982/rtk_gps_um982/clock_service.py /usr/local/libexec/icart-clock/
mkdir -p /etc/systemd/system/icart-clock.service.d
cat > /etc/systemd/system/icart-clock.service.d/ntp-evaluation.conf <<'EOF'
[Service]
ExecStart=
ExecStart=/usr/bin/python3 /usr/local/libexec/icart-clock/clock_service.py --interface enp0s31f6 --lidar-ip 192.168.1.201 --clock-source ntp
EOF
systemctl daemon-reload
systemctl restart chrony
systemctl start icart-clock
systemctl is-active chrony icart-clock
chronyc -n sources
