#!/bin/bash
# Phase 2 acceptance test for pcan_hpm5300 (can1 = new device, can0 = existing PCAN untouched)
set -x
ip link set can1 up type can bitrate 500000
ip -details -br link show can1

# listen on can1 while generating traffic; TX frames should come back as local echo [LO]
timeout 6 candump can1,123:7FF > /tmp/opencode/p2_candump.log 2>&1 &
sleep 0.5
cangen can1 -g 10 -n 30 -I 123 -L 8 -D 1122334455667788
wait
echo "---- candump can1 ----"
cat /tmp/opencode/p2_candump.log
echo "---- can1 stats ----"
ip -s link show can1
