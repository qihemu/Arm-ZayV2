#!/usr/bin/env bash
# 配置 Linux SocketCAN 为经典 1Mbps（J4310P/J4340 系列出厂默认）
set -euo pipefail

IFACE="${1:-can0}"

if ip link show "$IFACE" >/dev/null 2>&1; then
    sudo ip link set "$IFACE" down || true
fi

sudo ip link set "$IFACE" type can bitrate 1000000
sudo ip link set "$IFACE" up

echo "Configured $IFACE: classic CAN 1Mbps"
ip -details link show "$IFACE"
