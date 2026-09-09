# pcan_hpm5300

PCAN-USB (peak_usb) 兼容的 USB 转 CAN 固件，运行于 HPM5300EVK（HPM5361，RISC-V）。
移植自 [moonglow/pcan_cantact](https://github.com/moonglow/pcan_cantact)（STM32F042 版），
协议层原样复用，USB/CAN/时间戳/LED 层针对 HPM SDK 重写。

Linux 主机枚举为 SocketCAN 接口（`can?`），与真实 PCAN-USB 行为一致：
VID/PID `0c72:000c`，全速 USB，4 个 BULK 端点（CMDIN 0x81 / CMDOUT 0x01 mps16，MSGIN 0x82 / MSGOUT 0x02 mps64）。

## 目录结构

```
src/protocol/     平台无关 PCAN-USB 协议层（与参考固件逐字节一致，勿改）
src/pcan_usb.c    CherryUSB vendor 设备：描述符、EP 收发、m2h flush FSM
src/pcan_can.c    MCAN3 驱动适配：位时序映射、ISR 收发、bus-off 自恢复
src/pcan_timestamp.c  mchtmr(24MHz) → PCAN tick(42.666µs = 1024 counts，移位换算)
src/pcan_led.c    LED 状态机（板上单灯 PA23，TX/RX 复用）
test/             主机侧验收脚本（Linux + peak_usb + can-utils）
```

## 构建

```bash
cd pcan_hpm5300 && mkdir -p build && cd build
source ~/dev/hpm_sdk/env.sh
export GNURISCV_TOOLCHAIN_PATH=~/dev/hpm_sdk/toolchains/rv32imac_zicsr_zifencei_multilib_b_ext-linux
cmake -GNinja -DBOARD=hpm5300evk -DCMAKE_BUILD_TYPE=flash_xip ..
ninja
```

## 烧录（FT2232 板载调试器）

系统 openocd 缺 `hpm_xpi` flash 驱动，必须用 HPMicro 版 openocd：

```bash
# 终端1
cd $OPENOCD_SCRIPTS
~/.local/bin/openocd_hpm -c "set HPM_SDK_BASE $HOME/dev/hpm_sdk; set BOARD hpm5300evk; set PROBE ft2232;" \
    -f hpm5300_all_in_one.cfg

# 终端2
riscv32-unknown-elf-gdb build/output/demo.elf -batch \
    -ex "target remote :3333" -ex "monitor reset halt" -ex "load" \
    -ex "monitor reset" -ex "detach" -ex "quit"
```

串口控制台：`/dev/ttyUSB1` @115200（FT2232 重新枚举后需重设：`stty -F /dev/ttyUSB1 115200 raw -echo`）。

## 主机侧使用

```bash
lsusb | grep 0c72:000c                     # 应显示 PEAK System PCAN-USB
ip link set can1 up type can bitrate 500000  # peak_usb 自动绑定为 can1(can0 被占时)
candump can1 &
cangen can1 -g 10 -n 100 -I 123 -L 8 -D 1122334455667788
```

## 验收脚本（test/）

| 脚本 | 内容 |
|---|---|
| p2_test.sh  | probe + bitrate 下发 + TX 回声链路 |
| p3_test.sh  | 真实总线双向对测（需另一 CAN 节点，500k） |
| p3b_test.sh | listen-only 只听模式验证 |
| p3c_test.sh | bus-off 触发与自动恢复（需手动短接 CAN_H-CAN_L） |
| p4b_test.sh | 双向 30s 总线满载压测（突发式 cangen，零丢帧判据） |

## 关键实现要点

- **位时序**：peak_usb 按 SJA1000 @8MHz 有效时钟发 BTR0/BTR1；MCAN3 时钟 80MHz →
  `prescaler = brp×10, num_seg1 = tseg1, num_seg2 = tseg2`（MCAN seg1 不含同步段），
  `tseg2==1` 时从 seg1 借 1 tq（MCAN NTSEG2 最小 2），波特率保持精确
- **TX 回声**：不由 CAN 层产生；协议层在收到带 SRR 位的记录时立即回声（与参考固件一致），
  CAN 层硬件回声会导致 echo_skb 双重完成
- **背压**：软件 TX FIFO(100) 满时协议层在 USB ISR 内自旋等待并停发 MSGOUT read →
  主机 bulk OUT 被 NAK → 自然限流到总线速率
- **强制全速**：`-DCONFIG_USB_DEVICE_FS=1` 置 EHCI-device PFSC 位（HPM5361 USB0 带 HS PHY，
  不强制会以 480Mbps 枚举导致 bulk mps 非法）
- **bus-off**：ISR 上报 + poll 里 `mcan_recover_from_busoff()`，等效 bxCAN AutoBusOff

## 已验证

- peak_usb probe/attach、bitrate/listen-only/bus-off 控制路径
- 双向 30s 总线满载（~4300 帧/s）零丢帧零错误，ERROR-ACTIVE 保持
- listen-only 下 25 万帧 RX 洪峰仅 1 帧 socket drop
- bus-off 短接测试后自动恢复并恢复收发
