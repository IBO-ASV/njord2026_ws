#!/bin/bash
# Revert enP8p1s0 back to DHCP after Livox Mid360 verification.
set -euo pipefail

IFACE="enP8p1s0"

CON_NAME=$(nmcli -t -f NAME,DEVICE connection show | awk -F: -v d="$IFACE" '$2==d{print $1; exit}')

if [ -z "$CON_NAME" ]; then
    echo "No connection profile is bound to $IFACE; nothing to revert." >&2
    exit 0
fi

sudo nmcli connection modify "$CON_NAME" ipv4.method auto
sudo nmcli connection up "$CON_NAME"

echo "== $IFACE now (connection: $CON_NAME): =="
ip -4 addr show "$IFACE"
