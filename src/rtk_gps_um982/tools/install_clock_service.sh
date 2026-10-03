#!/bin/bash
set -euo pipefail
if [[ $EUID -ne 0 ]]; then echo 'Run with sudo' >&2; exit 1; fi
if pgrep -f '(^|/)(fastlio_mapping|fusion_node|ypspur-coordinator|robot_navigator)( |$)' >/dev/null; then
  echo 'Stop localization and motion processes first' >&2; exit 1
fi
repo_dir=$(cd -- "$(dirname -- "$0")/../../.." && pwd)
clock_user=${SUDO_USER:?Run through sudo as the ROS user}
clock_interface=${1:-enp0s31f6}
clock_lidar_ip=${2:-192.168.1.201}
if [[ ! $clock_user =~ ^[a-z_][a-z0-9_-]*$ || ! $clock_interface =~ ^[a-zA-Z0-9_.:-]+$ || ! $clock_lidar_ip =~ ^[0-9.]+$ ]]; then exit 1; fi
apt-get install -y linuxptp ethtool acl
backup_dir="/var/backups/robot-clock-$(date +%Y%m%d_%H%M%S)"
mkdir -p "$backup_dir"
cp -a /etc/systemd/system/chrony.service.d/um982-acl.conf "$backup_dir/"
cp -a /etc/chrony/conf.d/um982.conf "$backup_dir/"
if [[ -d /etc/systemd/system/icart-clock.service.d ]]; then
 cp -a /etc/systemd/system/icart-clock.service.d "$backup_dir/"
fi
python3 "$repo_dir/src/rtk_gps_um982/tools/configure_ntp_poll.py" --backup-directory "$backup_dir"
systemctl stop icart-clock.service 2>/dev/null || true
cat > /etc/chrony/conf.d/um982.conf <<'EOF'
# RMC retained for diagnostics and GNSS date anchoring; never discipline PC.
refclock SOCK /run/chrony/um982.sock refid UM98 poll 0 filter 4 precision 0.1 offset 0.0 noselect
EOF
# Retire the earlier evaluation override; the main unit now specifies NTP.
rm -f /etc/systemd/system/icart-clock.service.d/ntp-evaluation.conf
cat > /etc/systemd/system/chrony.service.d/um982-acl.conf <<EOF
[Service]
RuntimeDirectoryMode=0750
ExecStartPost=
ExecStartPost=!/usr/bin/setfacl -m u:${clock_user}:x /run/chrony
ExecStartPost=!/usr/bin/setfacl -m u:${clock_user}:rw /run/chrony/um982.sock
EOF
install -d -m 755 /usr/local/libexec/icart-clock
install -m 644 "$repo_dir"/src/rtk_gps_um982/rtk_gps_um982/ptp_trial.py /usr/local/libexec/icart-clock/
install -m 644 "$repo_dir"/src/rtk_gps_um982/rtk_gps_um982/clock_service.py /usr/local/libexec/icart-clock/
cat > /etc/systemd/system/icart-clock.service <<EOF
[Unit]
Description=i-Cart NTP-gated PTP and MID360 timestamp monitor
After=network-online.target chrony.service
Wants=network-online.target chrony.service
[Service]
Type=simple
ExecStart=/usr/bin/python3 /usr/local/libexec/icart-clock/clock_service.py --interface ${clock_interface} --lidar-ip ${clock_lidar_ip} --clock-source ntp
Restart=on-failure
RestartSec=5
RuntimeDirectory=icart-clock
RuntimeDirectoryMode=0755
UMask=0022
NoNewPrivileges=yes
ProtectSystem=strict
ProtectHome=yes
ReadWritePaths=/run/icart-clock
CapabilityBoundingSet=CAP_NET_RAW CAP_NET_BIND_SERVICE CAP_SYS_TIME
[Install]
WantedBy=multi-user.target
EOF
# Only the dedicated LiDAR can send PTP responses through the host firewall.
if command -v ufw >/dev/null && ufw status | grep -q '^Status: active'; then
  ufw allow in on "$clock_interface" proto udp from "$clock_lidar_ip" to any port 319,320 comment 'MID360 PTP only'
fi
systemctl daemon-reload
systemctl restart chrony
systemctl enable --now icart-clock.service
systemctl is-active chrony icart-clock
getfacl /run/chrony /run/chrony/um982.sock
