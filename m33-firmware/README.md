# M33固件修改说明

## 修改目的

原始M33核固件通过RIFSC资源管理器请求了UART4(PB6/PB7)作为调试串口，导致A35核的Linux系统无法访问UART4硬件。为使雷达模块能通过UART4与A35核通信，需要修改M33核固件释放UART4资源。

## 修改文件

**文件路径**: `/home/alientek/STM32Cube_ATK_FW_MP2_V1.0.0/Projects/STM32MP257D-ATK/Applications/CM33_OpenAMP_DEMO/OpenAMP_TTY_echo/CM33/NonSecure/Core/Src/main.c`

## 修改内容

在UART4资源请求代码块外添加 `#if 0` / `#endif` 条件编译：

```c
  /* Initialize Display destination */
#if defined(__VALID_OUTPUT_TERMINAL_IO__) && defined (__GNUC__)
  initialise_monitor_handles();
#else
#if 0   // <-- 添加: 禁用UART4资源请求，释放给A35核使用
  if(ResMgr_Request(RESMGR_RESOURCE_RIFSC, STM32MP25_RIFSC_UART4_ID) == RESMGR_STATUS_ACCESS_OK)
  {
    COM_Init.BaudRate                = 115200;
    COM_Init.WordLength              = UART_WORDLENGTH_8B;
    COM_Init.StopBits                = UART_STOPBITS_1;
    COM_Init.Parity                  = UART_PARITY_NONE;
    COM_Init.HwFlowCtl               = UART_HWCONTROL_NONE;
    BSP_COM_Init(COM_VCP_CM33, &COM_Init);
    BSP_COM_SelectLogPort(COM_VCP_CM33);
  }
#endif  // <-- 添加
#endif
```

## 编译命令

```bash
rm -rf /home/alientek/stm32cubeide_workspace
mkdir -p /home/alientek/stm32cubeide_workspace
/home/alientek/download/makeself_dir_RBGMxz/y/headless-build.sh \
  -data /home/alientek/stm32cubeide_workspace \
  -importAll "/home/alientek/STM32Cube_ATK_FW_MP2_V1.0.0/Projects/STM32MP257D-ATK/Applications/CM33_OpenAMP_DEMO/OpenAMP_TTY_echo/STM32CubeIDE" \
  -build all
```

## 编译输出

```
STM32CubeIDE/CM33/NonSecure/CA35TDCID_m33_ns_sign/OpenAMP_TTY_echo_CM33_NonSecure.elf
STM32CubeIDE/CM33/NonSecure/CA35TDCID_m33_ns_sign/OpenAMP_TTY_echo_CM33_NonSecure_sign.bin
```

## 部署到开发板

```bash
# 上传固件
scp OpenAMP_TTY_echo_CM33_NonSecure.elf root@192.168.88.10:/lib/firmware/

# 停止M33核（先卸载rpmsg模块）
ssh root@192.168.88.10 '
rmmod rpmsg_tty rpmsg_ctrl rpmsg_char 2>/dev/null
sleep 1
echo stop > /sys/class/remoteproc/remoteproc0/state
sleep 3
'

# 设置固件名并启动M33核
ssh root@192.168.88.10 '
echo OpenAMP_TTY_echo_CM33_NonSecure.elf > /sys/class/remoteproc/remoteproc0/firmware
echo start > /sys/class/remoteproc/remoteproc0/state
sleep 5
cat /sys/class/remoteproc/remoteproc0/state
'
```

## 注意事项

1. 修改后M33核不再有调试串口输出(loc_printf不会打印到UART4)
2. M33核的OpenAMP/RPMSG功能不受影响，仍然可以通过VIRT_UART与A35核通信
3. 如果需要恢复M33调试串口，将`#if 0`改回`#if 1`即可
