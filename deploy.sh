#!/bin/bash
set -e

BOARD_IP="192.168.88.10"
M33_FW="/home/alientek/STM32Cube_ATK_FW_MP2_V1.0.0/Projects/STM32MP257D-ATK/Applications/CM33_OpenAMP_DEMO/OpenAMP_TTY_echo/STM32CubeIDE/CM33/NonSecure/CA35TDCID_m33_ns_sign/OpenAMP_TTY_echo_CM33_NonSecure.elf"
M33_FW_NAME="OpenAMP_TTY_echo_CM33_NonSecure.elf"
RADAR_BIN="/home/alientek/radar_project/radar_bsd"
DTB_FILE="/home/alientek/linux/atk-mp257/kernel/linux/linux-6.6.48/arch/arm64/boot/dts/st/stm32mp257d-atk-ddr-1GB.dtb"

echo "===== Radar Project Full Deployment ====="
echo ""

echo "[1/5] Checking board connectivity..."
if ! ssh -o ConnectTimeout=3 root@${BOARD_IP} "echo OK" > /dev/null 2>&1; then
    echo "ERROR: Cannot connect to board at ${BOARD_IP}"
    echo "Please make sure the board is powered on and reachable."
    exit 1
fi
echo "Board is reachable."

echo ""
echo "[2/5] Deploying device tree..."
scp "${DTB_FILE}" root@${BOARD_IP}:/boot/stm32mp257d-atk-ddr-1GB.dtb
echo "Device tree deployed."

echo ""
echo "[3/5] Deploying M33 firmware (UART4 released)..."
scp "${M33_FW}" root@${BOARD_IP}:/lib/firmware/${M33_FW_NAME}
echo "M33 firmware deployed."

echo ""
echo "[4/5] Deploying radar_bsd program..."
scp "${RADAR_BIN}" root@${BOARD_IP}:/usr/local/bin/
ssh root@${BOARD_IP} "chmod +x /usr/local/bin/radar_bsd"
echo "radar_bsd deployed."

echo ""
echo "[5/5] Rebooting board to apply all changes..."
ssh root@${BOARD_IP} "sync && reboot"
echo "Board is rebooting. Wait ~30 seconds, then run:"
echo ""
echo "  ssh root@${BOARD_IP}"
echo "  # Check UART4 device"
echo "  ls -la /dev/ttySTM3"
echo "  # Check M33 core status"
echo "  cat /sys/class/remoteproc/remoteproc0/state"
echo "  # Start M33 core"
echo "  echo ${M33_FW_NAME} > /sys/class/remoteproc/remoteproc0/firmware"
echo "  echo start > /sys/class/remoteproc/remoteproc0/state"
echo "  # Run radar program"
echo "  /usr/local/bin/radar_bsd -d /dev/ttySTM3 -b 921600 -v -l /tmp/radar.log"
echo ""
echo "===== Deployment Complete ====="
