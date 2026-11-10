#!/usr/bin/env bash
# Share the Internet uplink with the BE25-connected LAN.
set -euo pipefail

UPLINK_CONNECTION=${UPLINK_CONNECTION:-"Wired connection 6"}
LAN_CONNECTION=${LAN_CONNECTION:-"Wired connection 5"}

sudo nmcli connection modify "$UPLINK_CONNECTION" ipv4.method auto ipv4.never-default no
sudo nmcli connection modify "$LAN_CONNECTION" ipv4.method shared ipv4.never-default yes
sudo nmcli connection up "$UPLINK_CONNECTION"
sudo nmcli connection up "$LAN_CONNECTION"

ip -4 route get 8.8.8.8
