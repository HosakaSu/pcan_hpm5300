#!/bin/bash
# Phase 4b: TRUE bus-saturation stress (burst-mode cangen, ENOBUFS-tolerant)
LOG=/tmp/opencode
DUR=30
set -x
ip link set can1 down
ip link set can1 up type can bitrate 500000 listen-only off
sleep 1
set +x

stats() { ip -s link show "$1" | awk '/RX:/{getline; rxp=$2; rxe=$3; rxd=$4} /TX:/{getline; txp=$2; txe=$3; txd=$4} END{print "RX pkts="rxp" err="rxe" drop="rxd"  TX pkts="txp" err="txe" drop="txd}'; }
delta() { echo $(( $3 - $1 )) ; }

echo "===== TEST1: RX saturation, can0 -> can1, ${DUR}s at bus max ====="
S1C1=($(ip -s link show can1 | awk '/RX:/{getline; print $2, $3, $4} /TX:/{getline; print $2, $3, $4}'))
S1C0=($(ip -s link show can0 | awk '/RX:/{getline; print $2, $3, $4} /TX:/{getline; print $2, $3, $4}'))
timeout $((DUR+8)) candump can1 > $LOG/p4b_rx_can1.log 2>&1 &
sleep 0.5
timeout $DUR bash -c 'while :; do cangen can0 -g 0 -n 15 -I 100 -L 8 -D aabbccddeeff0011 2>/dev/null || true; done'
sleep 6   # let the pipe drain
wait
E1C1=($(ip -s link show can1 | awk '/RX:/{getline; print $2, $3, $4} /TX:/{getline; print $2, $3, $4}'))
E1C0=($(ip -s link show can0 | awk '/RX:/{getline; print $2, $3, $4} /TX:/{getline; print $2, $3, $4}'))
echo "can0 TX delta (sent):      $(( E1C0[3] - S1C0[3] ))"
echo "can1 RX delta (received):  $(( E1C1[0] - S1C1[0] ))  err=$(( E1C1[1] - S1C1[1] )) drop=$(( E1C1[2] - S1C1[2] ))"
echo "candump lines: $(wc -l < $LOG/p4b_rx_can1.log)"

echo "===== TEST2: TX saturation, can1 -> can0, ${DUR}s at bus max ====="
S2C1=($(ip -s link show can1 | awk '/RX:/{getline; print $2, $3, $4} /TX:/{getline; print $2, $3, $4}'))
S2C0=($(ip -s link show can0 | awk '/RX:/{getline; print $2, $3, $4} /TX:/{getline; print $2, $3, $4}'))
timeout $((DUR+8)) candump can0 > $LOG/p4b_rx_can0.log 2>&1 &
sleep 0.5
timeout $DUR bash -c 'while :; do cangen can1 -g 0 -n 15 -I 200 -L 8 -D 1122334455667788 2>/dev/null || true; done'
sleep 6
wait
E2C1=($(ip -s link show can1 | awk '/RX:/{getline; print $2, $3, $4} /TX:/{getline; print $2, $3, $4}'))
E2C0=($(ip -s link show can0 | awk '/RX:/{getline; print $2, $3, $4} /TX:/{getline; print $2, $3, $4}'))
echo "can1 TX delta (sent):      $(( E2C1[3] - S2C1[3] ))  drop=$(( E2C1[5] - S2C1[5] ))"
echo "can0 RX delta (received):  $(( E2C0[0] - S2C0[0] ))  err=$(( E2C0[1] - S2C0[1] )) drop=$(( E2C0[2] - S2C0[2] ))"
echo "candump lines: $(wc -l < $LOG/p4b_rx_can0.log)"

echo "===== FINAL ====="
ip -details link show can1 | grep -E "state|bus-off"
stats can1
stats can0
set -x
ip link set can1 down; sleep 0.5; ip link set can1 up type can bitrate 500000 listen-only off
ip -details link show can1 | grep state
