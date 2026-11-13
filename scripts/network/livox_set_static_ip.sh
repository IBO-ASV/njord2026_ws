#!/bin/bash
# Configure enP8p1s0 with a static IPv4 address for talking to the Livox Mid360.
# The Mid360 host-side IP is fixed at 192.168.1.5 in
# src/driver/lidar/livox_ros_driver2/config/MID360_config.json, so this must match.
set -euo pipefail

IFACE="enP8p1s0"
STATIC_IP="192.168.1.5/24"
FALLBACK_CON_NAME="livox-mid360"

# Reuse whichever connection profile is already bound to the interface
# (e.g. the default "Wired connection 1"), so we don't fight NetworkManager
# with a second profile for the same device.
CON_NAME=$(nmcli -t -f NAME,DEVICE connection show | awk -F: -v d="$IFACE" '$2==d{print $1; exit}')

if [ -z "$CON_NAME" ]; then
    CON_NAME="$FALLBACK_CON_NAME"
    sudo nmcli connection add type ethernet ifname "$IFACE" con-name "$CON_NAME" \
        ipv4.method manual ipv4.addresses "$STATIC_IP" ipv4.never-default yes
else
    sudo nmcli connection modify "$CON_NAME" \
        ipv4.method manual ipv4.addresses "$STATIC_IP" \
        ipv4.gateway "" ipv4.dns "" ipv4.never-default yes
fi

sudo nmcli connection up "$CON_NAME"

echo "== $IFACE now (connection: $CON_NAME): =="
ip -4 addr show "$IFACE"
