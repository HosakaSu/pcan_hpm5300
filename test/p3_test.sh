#!/bin/bash
# Phase 3 acceptance: real bus test can1(EVK MCAN3) <-> can0(PCAN-USB FD), 500k
set -x
ip link set can1 up type can bitrate 500000
ip -details link show can1 | grep -E "bitrate|can state"

# capture both directions
timeout 8 candump can1 > /tmp/opencode/p3_rx_can1.log 2>&1 &
timeout 8 candump can0 > /tmp/opencode/p3_rx_can0.log 2>&1 &
sleep 0.5

# EVK(can1) -> bus -> PCAN(can0): ID 0x123
cangen can1 -g 10 -n 30 -I 123 -L 8 -D 1122334455667788
sleep 0.5
# PCAN(can0) -> bus -> EVK(can1): ID 0x321
cangen can0 -g 10 -n 30 -I 321 -L 8 -D aabbccddeeff0011
wait

set +x
echo "==== candump can1 (expect 30x ID 321 from PCAN; own TX may show as [LO]) ===="
cat /tmp/opencode/p3_rx_can1.log
echo "==== candump can0 (expect 30x ID 123 from EVK) ===="
cat /tmp/opencode/p3_rx_can0.log
echo "==== can1 state/stats (expect: ERROR-ACTIVE, bus-off 0, TX 30, RX>0) ===="
ip -details link show can1 | grep -E "can state|bus-off|error" 
ip -s link show can1 | tail -4
