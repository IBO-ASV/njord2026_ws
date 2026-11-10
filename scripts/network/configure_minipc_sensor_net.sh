#!/usr/bin/env bash
# Keep the MID360S Ethernet static while using the internal Wi-Fi for Internet.
set -euo pipefail

SENSOR_CONNECTION=${SENSOR_CONNECTION:-"有線接続 1"}
SENSOR_ADDRESS=${SENSOR_ADDRESS:-192.168.1.2/24}
UPLINK_CONNECTION=${UPLINK_CONNECTION:-MankinNet}
UPLINK_INTERFACE=${UPLINK_INTERFACE:-wlp3s0}

sudo nmcli connection modify "$SENSOR_CONNECTION" \
  ipv4.method manual ipv4.addresses "$SENSOR_ADDRESS" \
  ipv4.gateway "" ipv4.dns "" ipv4.never-default yes
sudo nmcli connection up "$SENSOR_CONNECTION"
sudo nmcli connection modify "$UPLINK_CONNECTION" \
  ipv4.method auto ipv4.never-default no ipv4.route-metric 50 \
  ipv4.dns-priority -50 connection.autoconnect yes
sudo nmcli connection up "$UPLINK_CONNECTION" ifname "$UPLINK_INTERFACE"

ip -4 address show
ip -4 route get 8.8.8.8
