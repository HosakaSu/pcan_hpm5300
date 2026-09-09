#!/bin/bash
# Phase 3b: listen-only + bus-off recovery acceptance
set -x
## ---- Part 1: listen-only (silent) ----
ip link set can1 down
ip link set can1 up type can bitrate 500000 listen-only on

timeout 6 candump can1 > /tmp/opencode/p3b_lo_can1.log 2>&1 &
sleep 0.3
# PCAN sends 20 frames -> can1 (listen-only) should receive all
cangen can0 -g 10 -n 20 -I 321 -L 8 -D 1122334455667788
sleep 1
# can1 tries to send 5 frames in listen-only -> must NOT appear on can0
timeout 4 candump can0 > /tmp/opencode/p3b_lo_can0.log 2>&1 &
sleep 0.3
timeout 3 cangen can1 -g 10 -n 5 -I 777 -L 8 -D 99aabbccddeeff00 && echo "UNEXPECTED: cangen succeeded" || echo "OK: cangen can1 blocked in listen-only"
wait
set +x
echo "== p3b: can1 listen-only RX (expect 20x 321) =="
grep -c " 321 " /tmp/opencode/p3b_lo_can1.log
echo "== p3b: can0 saw 777 from listen-only can1? (expect 0) =="
grep -c " 777 " /tmp/opencode/p3b_lo_can0.log || true
ip -details link show can1 | grep -E "state|bitrate"

## ---- Part 2: bus-off recovery (interactive) ----
set -x
ip link set can1 down
ip link set can1 up type can bitrate 500000
set +x
echo
echo ">>> 请用导线短暂短接 EVK J9 的 CAN_H(3) 和 CAN_L(1) 约 2-3 秒，然后移除短接"
read -p ">>> 短接完成并移除后按回车继续..."
set -x
ip -details link show can1 | grep -E "state|bus-off|restart"
# generate traffic while shorted window may still show errors; after removal should recover
timeout 6 candump can1 > /tmp/opencode/p3b_bo_can1.log 2>&1 &
sleep 0.3
cangen can0 -g 10 -n 20 -I 321 -L 8 -D 1122334455667788
sleep 0.5
cangen can1 -g 10 -n 10 -I 555 -L 8 -D 0011223344556677
wait
set +x
echo "== p3b: after bus-off removal, can1 RX (expect 20x 321) =="
grep -c " 321 " /tmp/opencode/p3b_bo_can1.log
echo "== p3b: can1 state (expect ERROR-ACTIVE again) =="
ip -details link show can1 | grep -E "state|bus-off"
ip -s link show can1 | tail -4
# the short may have driven can0 bus-off too - restart it if needed
CAN0_STATE=$(ip -details link show can0 | grep -o "CAN_STATE_[A-Z_]*" | head -1)
echo "== can0 state: $CAN0_STATE =="
if [ "$CAN0_STATE" = "CAN_STATE_BUS_OFF" ]; then
    ip link set can0 type can restart
    sleep 1
    ip -details link show can0 | grep -E "state"
fi
