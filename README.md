# Radar BSD Detection System

基于STM32MP257开发板和AT6010 60GHz毫米波雷达的盲区检测(BSD)与后方危险预警系统，主要针对电动车场景，检测后方危险目标突然靠近。

## 项目架构

```
radar_project/
├── src/
│   ├── radar_bsd.c              # 雷达BSD主程序 (MP257 A35核运行)
│   ├── radar_test.c             # 雷达测试程序 (MP257版)
│   ├── radar_phytium_test.c     # 雷达测试程序 (飞腾派PE2204版)
│   └── radar_phytium_bsd.c      # 雷达BSD监控程序 (飞腾派PE2204版)
├── device-tree/
│   └── patches/                  # 设备树修改补丁
├── m33-firmware/
│   └── README.md                 # M33固件修改说明
├── deploy.sh                     # 一键部署脚本
├── Makefile                      # 交叉编译Makefile
├── knowledge_base.md             # 项目知识库
├── debug_log.md                  # 调试日志
├── radar_debug.py                # USB-TTL调试工具 (PC端Windows)
├── radar_debug_ttyACM0.py        # USB-TTL调试工具 (PC端Linux)
├── radar_phytium.py              # 飞腾派Python测试脚本 (依赖pyserial)
├── radar_phytium_nopyserial.py   # 飞腾派Python测试脚本 (纯termios)
└── README.md                     # 本文件
```

## 硬件连接

### 雷达模块 → STM32MP257开发板引脚映射

| 雷达模块引脚 | 开发板引脚 | 功能说明 |
|-------------|-----------|---------|
| TX | PB7 (UART4_TX) | 雷达数据发送 → 开发板接收 |
| RX | PB6 (UART4_RX) | 开发板命令发送 → 雷达接收 |
| OUT | PB2 (GPIO) | 雷达感应输出信号 |
| VCC | 3.3V | 电源 |
| GND | GND | 地 |

### 雷达模块 → 飞腾派PE2204开发板引脚映射

| 雷达模块引脚 | 开发板引脚 | 功能说明 |
|-------------|-----------|---------|
| TX | J1 Pin10 (UART2_RXD) | 雷达数据发送 → 开发板接收 |
| RX | J1 Pin8 (UART2_TXD) | 开发板命令发送 → 雷达接收 |
| VCC | 3.3V/5V | 电源 (根据雷达模块规格) |
| GND | GND | 地 |

**注意**: 飞腾派DEBUG_UART1 (J1 Pin7/Pin9) 对应 `/dev/ttyAMA1`，但被Linux console占用，不可直接使用。推荐使用UART2 (J1 Pin8/Pin10) 对应 `/dev/ttyAMA2`。

### UART配置
- 设备节点 (MP257): `/dev/ttySTM3`
- 设备节点 (飞腾派): `/dev/ttyAMA2`
- 默认波特率: 921600 bps
- 数据位: 8, 停止位: 1, 无校验

### 串口详细配置说明

雷达模块使用标准UART通信，配置参数如下：

| 参数 | 值 | 说明 |
|------|-----|------|
| 波特率 | 921600 | 雷达默认波特率 |
| 数据位 | 8 | CS8 |
| 停止位 | 1 | ~CSTOPB |
| 校验 | 无 | ~PARENB |
| 流控 | 无 | ~CRTSCTS |
| 模式 | RAW | cfmakeraw() |
| VMIN | 0 | 非阻塞读取 |
| VTIME | 3 (30ms) | 读取超时 |

**C代码配置示例 (termios)**:
```c
struct termios tty;
memset(&tty, 0, sizeof(tty));
tcgetattr(fd, &tty);

cfmakeraw(&tty);
cfsetospeed(&tty, B921600);
cfsetispeed(&tty, B921600);

tty.c_cflag |= CLOCAL | CREAD;     // 忽略调制解调器状态，启用接收
tty.c_cflag &= ~CRTSCTS;            // 禁用硬件流控
tty.c_iflag &= ~(IXON | IXOFF | IXANY); // 禁用软件流控
tty.c_cc[VMIN] = 0;                 // 非阻塞
tty.c_cc[VTIME] = 3;                // 300ms超时

tcsetattr(fd, TCSANOW, &tty);
tcflush(fd, TCIOFLUSH);             // 清空收发缓冲区
```

**重要注意事项**:
1. **发送前清空缓冲区**: 每次发送命令前必须执行 `tcflush(fd, TCIOFLUSH)`，清空旧数据
2. **发送后等待**: 发送命令后等待至少300ms再读取回复 (`usleep(300000)`)
3. **波特率必须正确**: 雷达默认921600，如果波特率不匹配会导致收到乱码或无回复
4. **校验和计算**: 命令帧使用16位校验和（低字节在前），上报帧使用8位校验和
5. **多进程冲突**: 多个进程同时打开同一串口会导致数据混乱，测试前必须确保串口未被占用

### 设备树配置要求

**飞腾派PE2204**:
- 飞腾派的UART设备树节点已在内核中默认启用，**无需修改设备树**。
- UART2对应设备树节点 `uart3@2800E000`，在系统中映射为 `/dev/ttyAMA2`。
- 确保内核启动参数中没有将ttyAMA2作为console使用。检查方法：`cat /proc/cmdline`，确认没有 `console=ttyAMA2`。
- 如果UART被console占用，需要修改bootargs，在设备树中移除该串口的console配置。

**STM32MP257**:
- MP257的UART4默认可能被M33核固件占用，需要修改M33固件释放UART4资源。
- 设备树中需要启用UART4节点并禁用DMA：
  ```dts
  &uart4 {
      pinctrl-names = "default";
      pinctrl-0 = <&uart4_pins_mx>;
      status = "okay";
      /delete-property/ dmas;
      /delete-property/ dma-names;
  };
  ```
- 需要在`&pinctrl`段添加UART4引脚配置：
  ```dts
  uart4_pins_mx: uart4_pins_mx {
      pins1 {
          pinmux = <STM32_PINMUX('B', 6, AF3)>, /* UART4_TX */
                   <STM32_PINMUX('B', 7, AF3)>; /* UART4_RX */
          bias-disable;
          drive-push-pull;
          slew-rate = <0>;
      };
  };
  ```

**通用检查清单**:
1. 确认设备树中对应UART节点 `status = "okay"`
2. 确认引脚复用配置正确（TX/RX引脚映射到正确的UART功能）
3. 确认没有其他驱动或固件占用该UART（如M33固件、Linux console）
4. 确认内核配置中启用了对应UART驱动（CONFIG_SERIAL_AMBA_PL011 for 飞腾派，CONFIG_SERIAL_STM32 for MP257）
5. 确认 `/dev/ttyXXX` 设备节点已生成

## 软件组件

### 1. USB-TTL 调试工具 (radar_debug.py / radar_debug_ttyACM0.py)

PC 端 Python 脚本，通过 USB-TTL 直连雷达模块，用于快速验证雷达硬件和协议通信。

**运行**:
```bash
python radar_debug.py              # 默认配置BSD + 监控 + CSV记录
python radar_debug.py -v           # 可视化模式 (鸟瞰图 + 距离/速度曲线)
python radar_debug.py -m           # 仅监控（不发送配置命令）
python radar_debug.py -s           # 波特率扫描 + 监控
python radar_debug.py -l FILE      # 指定CSV日志文件名
python radar_debug.py -r FILE      # 回放CSV日志
python radar_debug.py -rv FILE     # 回放CSV日志 + 可视化
python radar_debug.py 115200       # 指定波特率
```

**功能**:
- 发送配置命令（开启BSD检测、自动上报、获取版本）
- 帧解析（0x5A上报 / 0x59回复 / 0x58命令）
- 波特率扫描
- 校验和自动计算（命令帧16-bit，上报帧8-bit）
- **CSV数据记录**：每次运行自动保存到 `radar_log_YYYYMMDD_HHMMSS.csv`
- **实时可视化**（`-v`）：鸟瞰图 + 距离/速度时间曲线 + 报警区域
- **CSV回放**（`-r`）：离线分析历史数据，调整阈值后回放验证
- **三级报警**：WARN / DANGER / CRITICAL，基于距离+接近速度组合判断

**重要**: 雷达仅检测到有效目标才上报数据。首次建立检测需 **5 秒**。

**实测结论** (2026-05-19):
- 雷达能检测人，不限于车辆
- 静止目标（墙壁、人站立）也能检测并上报
- 速度字段基于多普勒效应，仅反映**径向速度**（朝向/远离雷达的分量）
- 横向移动时速度为0，朝向/远离雷达移动时速度有值
- 多目标跟踪正常（最多8个）
- ⚠ 开启命令必须带参数 `0x01`，否则雷达不真正开启功能

### 2. 飞腾派雷达测试程序 (radar_phytium_test.c)

交叉编译的aarch64 Linux程序，运行在飞腾派PE2204上。

**功能**:
- 通过UART2接收雷达BSD数据
- 解析AT6010协议帧 (Head=0x5A, TYPE=7)
- 发送配置命令并验证回复
- 实时监控BSD上报数据

**编译**:
```bash
export PATH="/home/alientek/Phytium_syscode/GCC编译器/gcc-arm-10.2-2020.11-x86_64-aarch64-none-linux-gnu/bin:$PATH"
cd /home/alientek/radar_project/src
aarch64-none-linux-gnu-gcc -Wall -O2 -o radar_phytium_test radar_phytium_test.c
```

**部署**:
```bash
scp radar_phytium_test user@192.168.88.11:/tmp/
```

**运行**:
```bash
# 基本运行 (默认 /dev/ttyAMA2 @ 921600)
/tmp/radar_phytium_test

# 指定设备和波特率
/tmp/radar_phytium_test /dev/ttyAMA2 921600
```

### 3. 雷达BSD程序 (radar_bsd)

交叉编译的aarch64 Linux程序，运行在Cortex-A35核上。

**功能**:
- 通过UART4接收雷达BSD(Bind Spot Detection)数据
- 解析AT6010协议帧 (Head=0x5A, TYPE=7)
- 实时检测后方目标距离、角度、速度
- 危险目标预警（距离≤阈值 且 速度>阈值）

**编译**:
```bash
export PATH="/home/alientek/Phytium_syscode/GCC编译器/gcc-arm-10.2-2020.11-x86_64-aarch64-none-linux-gnu/bin:$PATH"
make
```

**运行**:
```bash
# 基本运行
/usr/local/bin/radar_bsd -d /dev/ttySTM3 -b 921600 -v

# 带日志和自定义预警参数
/usr/local/bin/radar_bsd -d /dev/ttySTM3 -b 921600 -v -l /tmp/radar.log -W 5 -S 2

# 参数说明
# -d DEVICE   UART设备路径
# -b BAUD     波特率 (默认921600)
# -v          详细输出模式
# -l FILE     日志保存到文件
# -W DIST     预警距离阈值(米)，默认5
# -S SPEED    预警接近速度阈值(m/s)，默认2
# -n          禁用预警
```

### 4. 设备树修改 (MP257)

为使UART4可用于A35核，需要:
1. 在`&pinctrl`段添加UART4引脚配置(PB6/PB7, AF3)
2. 启用UART4节点并禁用DMA
3. 修改M33核固件，释放UART4资源

### 5. M33核固件修改 (MP257)

原始M33核固件通过RIFSC请求了UART4资源作为调试串口，导致A35核无法访问UART4。

**修改内容**: 注释掉`main.c`中的UART4资源请求代码:
```c
#if 0  // 原代码: 释放UART4给A35核使用
  if(ResMgr_Request(RESMGR_RESOURCE_RIFSC, STM32MP25_RIFSC_UART4_ID) == RESMGR_STATUS_ACCESS_OK)
  {
    COM_Init.BaudRate = 115200;
    ...
    BSP_COM_Init(COM_VCP_CM33, &COM_Init);
    BSP_COM_SelectLogPort(COM_VCP_CM33);
  }
#endif
```

## 报警阈值设计

BSD报警基于**距离 + 接近速度**的组合判断（速度为负 = 目标在靠近）：

| 级别 | 距离阈值 | 速度阈值 | 含义 |
|------|---------|---------|------|
| WARN | ≤ 15m | ≤ -2 m/s | 后方有目标接近，注意 |
| DANGER | ≤ 8m | ≤ -4 m/s | 后方目标快速接近，危险 |
| CRITICAL | ≤ 4m | ≤ -1 m/s | 目标极近，紧急避让 |

修改脚本顶部常量即可调整：
```python
ALARM_WARN_DIST = 15       # 预警距离(m)
ALARM_WARN_SPEED = -2      # 预警速度(m/s, 负=接近)
ALARM_DANGER_DIST = 8      # 危险距离(m)
ALARM_DANGER_SPEED = -4    # 危险速度(m/s)
ALARM_CRITICAL_DIST = 4    # 紧急距离(m)
ALARM_CRITICAL_SPEED = -1  # 紧急速度(m/s)
```

### 阈值确定方法

1. **采集数据**：在实际场景（骑电动车/站路边）运行 `python radar_debug.py`，记录多种情况
2. **回放分析**：`python radar_debug.py -rv radar_log_xxx.csv`，观察距离/速度曲线
3. **调整阈值**：修改脚本顶部常量，再次回放验证报警触发时机是否合理
4. **确认阈值**：将最终阈值写入 `radar_bsd.c` 的 `-W` 和 `-S` 参数

## 雷达通信协议 (AT6010 HCI)

### 帧格式

**命令帧** (上位机→雷达):
```
Head(0x58) + CMD_Group(3bit) + CMD(5bit) + Param_Len + Params... + Checksum
```

**回复帧** (雷达→上位机):
```
Head(0x59) + CMD_Group + CMD + Param_Len + Params... + Checksum
```

**上报帧** (雷达主动上报):
```
Head(0x5A) + LEN + TYPE + PAYLOAD + Checksum
```

### BSD上报数据 (TYPE=7)

```c
typedef struct {
    int8_t range_val;   // 距离(米)
    int8_t angle_val;   // 角度(度)
    int8_t velo_val;    // 速度(m/s)
    int8_t objId;       // 目标ID
} bsd_obj_info_t;

typedef struct {
    uint16_t obj_num;           // 目标数量(最大8)
    uint16_t reserved;
    bsd_obj_info_t obj[8];     // 目标数组
} bsd_det_info_t;
```

## 飞腾派测试记录 (2026-05-27)

### 测试环境
- **开发板**: 飞腾派 PE2204
- **雷达模块**: MS60-1211S80M-BSD (AT6010 60GHz)
- **串口**: UART2 (/dev/ttyAMA2, J1 Pin8=TXD, Pin10=RXD)
- **波特率**: 921600 bps
- **交叉编译器**: aarch64-none-linux-gnu-gcc (GCC 10.2)

### 测试过程

1. **串口确认**: 飞腾派DEBUG_UART1 (/dev/ttyAMA1) 被Linux console占用 (`console=ttyAMA1,115200`)，不可直接使用。改用UART2 (/dev/ttyAMA2)。

2. **Python脚本测试**: 最初尝试使用pyserial，但飞腾派无法联网安装。改用纯Python termios实现 (`radar_phytium_nopyserial.py`)。

3. **C程序开发**: 参考MP257版 `radar_test.c`，修改串口设备路径为 `/dev/ttyAMA2`，适配飞腾派。

4. **交叉编译**: 使用飞腾派SDK的aarch64-none-linux-gnu-gcc编译，生成ARM64可执行文件。

5. **问题排查**:
   - **问题1**: 多次测试无回复。原因：多个雷达测试进程同时占用ttyAMA2串口。
   - **解决**: 测试前执行 `killall -9 radar_phytium_test` 确保串口空闲。
   - **问题2**: 雷达反复复位，收到boot信息 (`at6010 boot v1.9`)。原因：雷达供电不稳定或接线接触不良。
   - **解决**: 重新插拔雷达电源线，确保供电稳定。
   - **问题3**: 配置有回复但无BSD数据。原因：雷达模块在虚拟机USB-TTL测试正常，但飞腾派串口连接有问题。
   - **解决**: 确认TX/RX接线正确（雷达TX→飞腾派RX，雷达RX→飞腾派TX），GND可靠连接。

6. **测试结果**: 雷达在飞腾派上工作正常，能够正确回复配置命令并上报BSD数据。

### 测试数据示例

```
=== Step 1: Read version ===
SEND [VERSION]: 58 FE 00 56 01 
RECV [VERSION] (13 bytes): 59 FE 08 00 05 00 00 00 01 01 00 66 01 
  -> G7.0x1E Len=8 CKSUM_OK

=== Step 3: Enable sensor ===
SEND [ENABLE_SENSE]: 58 D1 01 01 2B 01 
RECV [ENABLE_SENSE] (6 bytes): 59 D1 01 00 2B 01 
  -> G6.0x11 Len=1 CKSUM_OK

=== Step 4: Enable BSD detection ===
SEND [ENABLE_BSD]: 58 D0 01 01 2A 01 
RECV [ENABLE_BSD] (6 bytes): 59 D0 01 01 2B 01 
  -> G6.0x10 Len=1 CKSUM_OK

=== Step 5: Enable auto report ===
SEND [ENABLE_AUTO]: 58 D2 01 01 2C 01 
RECV [ENABLE_AUTO] (6 bytes): 59 D2 01 01 2D 01 
  -> G6.0x12 Len=1 CKSUM_OK

=== Step 6: Monitor BSD data ===
DATA (12 bytes): 5A 09 07 01 00 00 00 04 37 00 01 A7 
DATA (12 bytes): 5A 09 07 01 00 00 00 03 26 00 01 95 
DATA (12 bytes): 5A 09 07 01 00 00 00 06 22 00 00 93 
```

BSD上报帧解析：
- `5A 09 07 01 00 00 00 04 37 00 01 A7`: 目标ID 0，距离4米，角度55度，速度0m/s
- `5A 09 07 01 00 00 00 03 26 00 01 95`: 目标ID 0，距离3米，角度38度，速度0m/s
- `5A 09 07 01 00 00 00 06 22 00 00 93`: 目标ID 0，距离6米，角度34度，速度0m/s

### 关键结论

1. **雷达模块本身正常**: 通过USB-TTL在虚拟机上测试，雷达能够正常检测目标并上报BSD数据。
2. **飞腾派串口可用**: UART2 (/dev/ttyAMA2) 可直接作为通用串口使用，无需修改设备树。
3. **应用层代码一致**: 雷达配置命令和协议解析与开发板无关，应用层代码在MP257和飞腾派上完全一致，仅需修改串口设备路径。
4. **串口冲突问题**: 多个进程同时占用串口会导致通信失败，测试前必须确保串口空闲。
5. **供电稳定性**: 雷达模块对供电稳定性敏感，供电不稳会导致模块反复复位。

## 一键部署 (MP257)

```bash
./deploy.sh
```

部署脚本会自动:
1. 检查开发板连接
2. 部署设备树
3. 部署修改后的M33固件
4. 部署radar_bsd程序
5. 重启开发板

## 开发环境

- **开发板 (MP257)**: 正点原子 ATK-DLMP257B (STM32MP257D)
- **开发板 (飞腾派)**: 飞腾派 PE2204
- **雷达模块**: MS60-1211S80M-BSD (AT6010 60GHz)
- **交叉编译器**: aarch64-none-linux-gnu-gcc (GCC 10.2)
- **M33编译**: STM32CubeIDE headless build
- **MP257开发板IP**: 192.168.88.10
- **飞腾派开发板IP**: 192.168.88.11
- **虚拟机密码**: 123456
- **飞腾派密码**: user
