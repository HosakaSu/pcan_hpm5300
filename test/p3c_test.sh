#!/bin/bash
# Phase 3c: bus-off + auto-recovery test (can1 in NORMAL mode this time)
set -x
## restore can1 to normal mode (listen-only flag is sticky!)
ip link set can1 down
ip link set can1 up type can bitrate 500000 listen-only off
ip -details link show can1 | grep -E "state|bitrate"

## heal can0 (ERROR-PASSIVE from the retransmit storm) with successful traffic
cangen can1 -g 20 -n 10 -I 555 -L 8 -D 0011223344556677
sleep 1
ip -details link show can0 | grep state
set +x

read -p ">>> 请短接 J9 CAN_H(3)-CAN_L(1) 并保持住，短接好后按回车..."
set -x
## transmit into the short -> can1 error counters explode -> bus-off
timeout 4 cangen can1 -g 5 -n 200 -I 556 -L 8 -D deadbeefcafebabe || true
sleep 1
echo "== can1 state while shorted (expect CAN_STATE_BUS_OFF) =="
ip -details link show can1 | grep -E "state|bus-off"
set +x

read -p ">>> 请移除短接线，移除后按回车..."
set -x
## firmware should auto-recover (M_CAN waits 128x11 recessive bits)
sleep 2
ip -details link show can1 | grep -E "state|bus-off"

## prove full recovery: PCAN -> EVK traffic again
timeout 5 candump can1 > /tmp/opencode/p3c_recover.log 2>&1 &
sleep 0.3
cangen can0 -g 20 -n 20 -I 321 -L 8 -D 1122334455667788
wait
set +x
echo "== recovery proof: can1 RX after bus-off (expect 20x 321) =="
grep -c " 321 " /tmp/opencode/p3c_recover.log
echo "== final can1 state (expect ERROR-ACTIVE) =="
ip -details link show can1 | grep -E "state|bus-off"
ip -s link show can1 | tail -4

## heal can0 if the short drove it bus-off too
CAN0_STATE=$(ip -details link show can0 | grep -o "CAN_STATE_[A-Z_]*" | head -1)
echo "== can0: $CAN0_STATE =="
if [ "$CAN0_STATE" = "CAN_STATE_BUS_OFF" ]; then
    ip link set can0 type can restart
    sleep 1
fi
