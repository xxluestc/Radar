# Radar BSD Detection System

基于STM32MP257开发板和AT6010 60GHz毫米波雷达的盲区检测(BSD)与后方危险预警系统，主要针对电动车场景，检测后方危险目标突然靠近。

## 项目架构

```
radar_project/
├── src/
│   └── radar_bsd.c          # 雷达BSD数据接收与解析程序
├── device-tree/
│   └── patches/              # 设备树修改补丁
├── m33-firmware/
│   └── README.md             # M33固件修改说明
├── deploy.sh                 # 一键部署脚本
├── Makefile                  # 交叉编译Makefile
├── knowledge_base.md         # 项目知识库
├── debug_log.md              # 调试日志
└── README.md                 # 本文件
```

## 硬件连接

### 雷达模块 → 开发板引脚映射

| 雷达模块引脚 | 开发板引脚 | 功能说明 |
|-------------|-----------|---------|
| TX | PB7 (UART4_TX) | 雷达数据发送 → 开发板接收 |
| RX | PB6 (UART4_RX) | 开发板命令发送 → 雷达接收 |
| OUT | PB2 (GPIO) | 雷达感应输出信号 |
| VCC | 3.3V | 电源 |
| GND | GND | 地 |

### UART配置
- 设备节点: `/dev/ttySTM3`
- 默认波特率: 921600 bps
- 数据位: 8, 停止位: 1, 无校验

## 软件组件

### 1. 雷达BSD程序 (radar_bsd)

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

### 2. 设备树修改

为使UART4可用于A35核，需要:
1. 在`&pinctrl`段添加UART4引脚配置(PB6/PB7, AF3)
2. 启用UART4节点并禁用DMA
3. 修改M33核固件，释放UART4资源

### 3. M33核固件修改

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

## 一键部署

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

- **开发板**: 正点原子 ATK-DLMP257B (STM32MP257D)
- **雷达模块**: MS60-1211S80M-BSD (AT6010 60GHz)
- **交叉编译器**: aarch64-none-linux-gnu-gcc (GCC 10.2)
- **M33编译**: STM32CubeIDE headless build
- **开发板IP**: 192.168.88.10
