#!/bin/bash
# Keep the current Wi-Fi Internet gateway/DNS while making its IPv4 address static.
set -euo pipefail

if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <wifi-interface> <ipv4/prefix>" >&2
    exit 64
fi

IFACE="$1"
ADDRESS="$2"

command -v nmcli >/dev/null || { echo "NetworkManager (nmcli) is required" >&2; exit 69; }
ip link show dev "$IFACE" >/dev/null 2>&1 || { echo "Unknown interface: $IFACE" >&2; exit 64; }
nmcli -g GENERAL.TYPE device show "$IFACE" | grep -qx wifi || {
    echo "Not a Wi-Fi interface: $IFACE" >&2
    exit 64
}

CON_NAME=$(nmcli -t -f NAME,DEVICE connection show --active |
    awk -F: -v device="$IFACE" '$2 == device {print $1; exit}')
GATEWAY=$(ip -4 route show default dev "$IFACE" |
    awk '{for (i = 1; i <= NF; ++i) if ($i == "via") {print $(i + 1); exit}}')
DNS=$(nmcli -g IP4.DNS device show "$IFACE" | paste -sd, -)
if [ -z "$CON_NAME" ] || [ -z "$GATEWAY" ] || [ -z "$DNS" ]; then
    echo "Connect $IFACE to Wi-Fi with DHCP first so its profile, gateway, and DNS can be retained." >&2
    exit 69
fi

sudo nmcli connection modify "$CON_NAME" \
    ipv4.method manual ipv4.addresses "$ADDRESS" ipv4.gateway "$GATEWAY" \
    ipv4.dns "$DNS" ipv4.ignore-auto-dns yes ipv4.never-default no
sudo nmcli connection up "$CON_NAME"

echo "== $IFACE: $ADDRESS via $GATEWAY =="
ip -4 address show dev "$IFACE"
