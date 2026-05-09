# 雷达BSD项目调试日志

## 2026-05-09 调试记录

### 问题1: UART4设备节点不出现

**现象**: 修改设备树后，`/dev/ttySTM*`中没有UART4对应的设备

**排查过程**:
1. 检查dmesg发现: `stm32mp257-pinctrl soc@0:pinctrl@46200000: invalid function 4 on pin 23`
2. 分析发现UART4的pinctrl配置被错误地放在了`&pinctrl_z`段
3. `&pinctrl_z`只管理GPIOZ引脚，PB6/PB7属于`&pinctrl`管理

**解决方案**: 将UART4 pinctrl配置从`&pinctrl_z`段移到`&pinctrl`段

**修改文件**: `stm32mp25-pinctrl-atk-ddr-1GB.dtsi`

---

### 问题2: UART4存在但收不到雷达数据

**现象**: `/dev/ttySTM3`设备存在，但`cat /dev/ttySTM3`或`stty -F /dev/ttySTM3`无法收到数据

**排查过程**:
1. 用示波器/逻辑分析仪确认雷达模块TX有数据输出
2. 确认PB7(UART4_RX)物理连接正确
3. 查看硬件参考手册发现: "UART4_TX和UART4_RX引脚被默认用作Cortex-M33核调试串口"
4. 分析M33核固件源码，确认M33通过`ResMgr_Request(RESMGR_RESOURCE_RIFSC, STM32MP25_RIFSC_UART4_ID)`占用了UART4

**根本原因**: M33核通过RIFSC资源管理器获取了UART4的独占访问权限，A35核虽然能创建设备节点，但无法真正访问UART4硬件寄存器

**解决方案**: 修改M33核固件，注释掉UART4资源请求代码

**修改文件**: `OpenAMP_TTY_echo/CM33/NonSecure/Core/Src/main.c`

---

### 问题3: M33核固件编译

**编译命令**:
```bash
rm -rf /home/alientek/stm32cubeide_workspace
mkdir -p /home/alientek/stm32cubeide_workspace
/home/alientek/download/makeself_dir_RBGMxz/y/headless-build.sh \
  -data /home/alientek/stm32cubeide_workspace \
  -importAll "/home/alientek/STM32Cube_ATK_FW_MP2_V1.0.0/Projects/STM32MP257D-ATK/Applications/CM33_OpenAMP_DEMO/OpenAMP_TTY_echo/STM32CubeIDE" \
  -build all
```

**编译结果**: 0 errors, 0 warnings (17.378s)

**输出文件**: `STM32CubeIDE/CM33/NonSecure/CA35TDCID_m33_ns_sign/OpenAMP_TTY_echo_CM33_NonSecure.elf`

---

### 其他UART方案分析（未采用）

| UART | 引脚 | 当前用途 | 可行性 |
|------|------|---------|--------|
| UART7 | PD0/PD3 | RS232(TPT3232E电平转换) | ❌ 有电平转换芯片，不能直接连TTL |
| USART1 | PG14/PG15 | RS485(TP8485E芯片) | ❌ 有RS485收发器，不能直接连TTL |
| UART4 | PB6/PB7 | M33调试串口 | ✅ 修改M33固件释放即可 |

---

## 待验证项（开发板上电后）

- [ ] 修改后的M33固件部署后，UART4是否可被A35核正常访问
- [ ] radar_bsd程序是否能正确接收雷达BSD数据
- [ ] BSD数据解析是否正确（距离、角度、速度）
- [ ] 预警功能是否正常触发
