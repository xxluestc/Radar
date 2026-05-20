# 雷达BSD项目调试日志

## 2026-05-19 PC端USB-TTL雷达调试

### 问题5: radar_debug.py 发送配置命令后雷达不上报数据

**现象**: 运行 `python radar_debug.py`，命令回复正常（版本/感测/检测/上报均返回OK），但无BSD上报帧输出。

**排查过程**:

1. **串口确认**: COM7 → COM9（CH343芯片），921600 bps
2. **命令回复分析**: 所有命令回复校验和正确，但发现关键问题：

```
发送: 开启感测功能(0xD1) [58 D1 00 29 01]   ← 参数长度=0，没带参数！
回复: G6.0x11 Len=1 OK | 59 D1 01 01        ← 回复参数=0x01（可能是错误码）
```

3. **协议核对**: AT6010 HCI协议要求开启命令必须带参数 `0x01`（使能），不带参数时雷达可能未真正开启功能

**根本原因**: `enable_bsd()`、`enable_bsd_detection()`、`enable_auto_report()` 三个函数调用 `send_cmd()` 时未传 `params` 参数，导致发送帧参数长度为0。

**修复**:
```python
# 修复前
send_cmd(ser, 6, 0x11, name="开启感测功能(0xD1)")

# 修复后
send_cmd(ser, 6, 0x11, bytes([0x01]), name="开启感测功能(0xD1)")
```

**修复后回复对比**:
```
修复前: [58 D1 00 ...] → 回复 59 D1 01 01   (参数=0x01，含义不明)
修复后: [58 D1 01 01 ...] → 回复 59 D1 01 00 (参数=0x00，表示成功)
```

**验证结果**: 修复后雷达立即开始上报BSD数据：
```
[06:50:21] BSD | 1目标 | OK
  ID+0 距离+3m 角度-6° 速度+0m/s
[06:52:46] BSD | 2目标 | OK
  ID+0 距离+6m 角度+30° 速度-1m/s
  ID+3 距离+10m 角度+11° 速度-2m/s
```

### 问题6: 速度字段始终为0

**现象**: 检测到目标（距离、角度正常变化），但速度始终为 0 m/s。

**原因分析**:
- FMCW雷达基于多普勒效应，只能测量**径向速度**（朝向/远离雷达的分量）
- 横向移动（在雷达面前左右走）时，径向速度分量≈0
- 手持雷达朝身体移动时，速度出现 -1~-2 m/s（3.6~7.2 km/h）

**结论**: 速度字段正常工作，需要目标有足够的径向运动分量。

### 雷达检测能力实测

| 测试方式 | 距离 | 角度 | 速度 | 结果 |
|----------|------|------|------|------|
| 人静立 | 3m | 0° | 0 m/s | ✅ 检测到（静止目标） |
| 人横向走动 | 3~4m | -31°~+26° | 0 m/s | ✅ 检测到，角度变化 |
| 手持雷达朝身体移 | 3~6m | 变化 | -1~-2 m/s | ✅ 速度有值 |
| 墙壁（静止） | 8~12m | -23°~-49° | 0 m/s | ✅ 检测到 |
| 多目标同时 | 6m+10m | 30°+11° | -1+-2 m/s | ✅ 同时2目标 |

**关键发现**:
- 雷达能检测人，不限于车辆
- 静止目标也能检测（与手册"6km/h最低速度"不完全一致，BSD模式下静态目标也上报）
- 多目标跟踪正常（最多8个）
- 速度测量基于多普勒效应，仅反映径向分量

### 固件版本信息

```
SW=0.5.0, Customer=0.0, CI=1.1, Algo=0
```

---

## 2026-05-16 调试记录

### 问题4: M33调试串口与雷达通信串口冲突（最终方案）

**需求**: M33调试串口(UART4)与雷达通信串口需要共存，但40pin排针上无其他空闲UART可用。

**方案**: 将M33调试串口从UART4(PB6/PB7)切换到LPUART1(PZ9/PZ4)，释放UART4给雷达。

**修改文件**:
1. `stm32mp257f_eval.h`: COM_CM33从UART4(PB6/PB7/AF3)改为LPUART1(PZ9/PZ4/AF6)
2. `main.c`: 移除`#if 0`屏蔽，启用BSP_COM_Init(COM_VCP_CM33)
3. `stm32mp257d-atk-ddr-1GB.dts`: 禁用I2C8（PZ4/PZ9引脚冲突）

**编译结果**: M33固件 0 errors, 0 warnings; DTB编译成功

**部署后验证**:
- M33状态: running
- RPMsg通道: rpmsg-tty addr 0x400 创建成功
- UART4: /dev/ttySTM3 可用，921600配置成功
- dmesg: 无UART4/LPUART1/I2C8相关错误

**最终资源分配**:
```
M33调试: LPUART1 (PZ9_TX/PZ4_RX) — 40pin pin7/pin11
雷达通信: UART4   (PB7_TX/PB6_RX) — 40pin pin8/pin10
```

---

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
