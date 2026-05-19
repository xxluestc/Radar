# 雷达BSD项目知识库

## 一、硬件信息

### 1.1 开发板

- **型号**: 正点原子 ATK-DLMP257B
- **处理器**: STM32MP257D (双核Cortex-A35 + Cortex-M33)
- **A35核**: 运行Linux系统
- **M33核**: 运行FreeRTOS + OpenAMP
- **开发板IP**: 192.168.88.10
- **扩展排针**: 2x20PIN，引出28个GPIO + 5V/3.3V/GND
- **注意**: 排针引脚非独立使用，与板载功能电路共用

### 1.2 雷达模块

- **型号**: MS60-1211S80M-BSD
- **芯片**: AT6010 (隔空科技 60GHz雷达SOC)
- **频段**: 60GHz FMCW
- **接口**: UART (默认921600bps) + GPIO
- **功能**: BSD(盲区检测)，支持多目标检测(最大8个)
- **检测信息**: 距离(m)、角度(°)、速度(m/s)、目标ID

### 1.3 引脚连接

| 雷达引脚 | 开发板引脚 | STM32引脚 | 复用功能 |
|---------|-----------|----------|---------|
| TX | 排针PIN | PB7 | UART4_TX (AF3) |
| RX | 排针PIN | PB6 | UART4_RX (AF3) |
| OUT | 排针PIN | PB2 | GPIO |
| VCC | 3.3V | - | 电源 |
| GND | GND | - | 地 |

## 二、AT6010通信协议

### 2.1 帧格式

#### 命令帧 (上位机→雷达)
```
Head(0x58) + [CMD_Group(3bit)|CMD(5bit)] + Param_Len + Params... + Checksum
```

#### 回复帧 (雷达→上位机)
```
Head(0x59) + [CMD_Group|CMD] + Param_Len + Params... + Checksum
```

#### 上报帧 (雷达主动上报)
```
Head(0x5A) + LEN + TYPE + PAYLOAD + Checksum
```
- LEN: PAYLOAD部分字节数(不含Head/LEN/Checksum)
- CHECK: 8位校验和 = Head + LEN + PAYLOAD所有字节之和
- 所有字段: little-endian格式

### 2.2 关键命令

| 命令 | CMD_Group | CMD | 功能 |
|------|-----------|-----|------|
| 设置感应电平持续时间 | 0 | 0x04 | 设置OUT PIN有效电平时间 |
| 获取感应电平持续时间 | 0 | 0x05 | 读取当前有效电平时间 |
| 设置波特率 | 0 | 0x19 | 切换UART波特率 |
| 设置雷达感应功能 | 6 | 0xD1 | 开关雷达感应 |
| 设置BSD功能 | 6 | 0x10 | 开关BSD检测 |
| 设置自动上报 | 6 | 0x12 | 开关自动上报 |

### 2.3 BSD上报数据 (TYPE=7)

```c
typedef struct {
    int8_t range_val;   // 距离(米)
    int8_t angle_val;   // 角度(度)
    int8_t velo_val;    // 速度(m/s)
    int8_t objId;       // 目标ID
} bsd_obj_info_t;       // 每个目标4字节

typedef struct {
    uint16_t obj_num;           // 目标数量(最大8)
    uint16_t reserved;          // 保留
    bsd_obj_info_t obj[8];     // 目标数组
} bsd_det_info_t;
```

**帧示例** (2个目标):
```
5A [LEN] 07 [obj_num_lo obj_num_hi] [reserved_lo reserved_hi]
    [range0 angle0 velo0 id0]
    [range1 angle1 velo1 id1]
    [CHECKSUM]
```

### 2.4 其他上报类型

| TYPE | 名称 | 结构体 |
|------|------|--------|
| 0 | 完整检测信息 | fmcw_det_info_t |
| 1 | HTM高度信息 | htm_det_info_t |
| 2 | 电平探测信息 | level_probe_det_info_t |
| 3 | 运动存在检测 | motion_det_info_t |
| 4 | 呼吸心率检测 | bhr_det_info_t |
| 5 | 分区检测 | rgn_det_info_t |
| 6 | 简短呼吸心率 | bhr_det_info_short_t |
| 7 | BSD盲区检测 | bsd_det_info_t |

## 三、STM32MP257 UART资源分配

### 3.1 开发板UART使用情况

| UART | 引脚 | A35核设备 | 用途 | 备注 |
|------|------|----------|------|------|
| USART2 | PA4/PA8 | /dev/ttySTM0 | A35调试串口 | |
| UART4 | PB6/PB7 | /dev/ttySTM3 | **雷达通信** | M33已释放，921600bps |
| UART7 | PD0/PD3 | /dev/ttySTM1 | RS232 | 有TPT3232E电平转换 |
| USART1 | PG14/PG15 | /dev/ttySTM2 | RS485 | 有TP8485E芯片 |
| LPUART1 | PZ9/PZ4 | - | M33调试串口 | 替换UART4，40pin pin7/pin11 |

### 3.2 RIFSC资源隔离

STM32MP257使用RIFSC(Resource Isolation Framework)管理外设访问权限：
- M33核通过BSP_COM_Init自动请求LPUART1的RIFSC权限（COM_CM33 → LPUART1）
- M33同时通过RIF保护PZ4/PZ9引脚不被A35访问
- UART4完全释放给A35核，无RIFSC冲突
- A35设备树中I2C8已禁用（PZ4/PZ9与LPUART1引脚冲突）

### 3.3 40pin扩展排针(J9)引脚定义

| 左侧(奇数) | 引脚号 | 右侧(偶数) |
|-----------|--------|-----------|
| VCC3.3 | 1-2 | VCC5 |
| I2C7_SDA (PD14) | 3-4 | VCC5 |
| I2C7_SCL (PD15) | 5-6 | GND |
| I2C8_SDA / LPUART1_TX (PZ9) | 7-8 | UART4_TX (PB7) |
| GND | 9-10 | UART4_RX (PB6) |
| I2C8_SCL / LPUART1_RX (PZ4) | 11-12 | DSI_LCD_ID (PF10) |
| DSI_TP_INT (PB2) | 13-14 | GND |
| DSI_TP_RST (PB1) | 15-16 | DSI_BL (PB0) |
| VCC3.3 | 17-18 | FAN_PWM (PB10) |
| SPI8_MOSI (PZ0) | 19-20 | GND |
| SPI8_MISO (PZ1) | 21-22 | LVDS_TP_INT (PB5) |
| SPI8_CLK (PZ2) | 23-24 | SAI1_MCLK_B (PD7) |
| GND | 25-26 | SAI1_SCK_B (PD6) |
| I2C3_SDA (PG2) | 27-28 | I2C3_SCL (PG1) |
| SAI1_FS_B (PD5) | 29-30 | GND |
| SAI1_SD_A (PD9) | 31-32 | SAI1_SD_B (PD4) |
| I2C4_SCL (PD11) | 33-34 | GND |
| I2C4_SDA (PD10) | 35-36 | LVDS_TP_RST (PB4) |
| LVDS_BL (PB3) | 37-38 | LCD_BL (PD8) |
| GND | 39-40 | RGB_TP_RST (PF13) |

## 四、Pinctrl子系统

### 4.1 两个pinctrl控制器

| 控制器 | 设备树节点 | 管理引脚 | 基地址 |
|--------|-----------|---------|--------|
| pinctrl | &pinctrl | GPIOA-GPIOH, GPIOI-GPIOZ(除GPIOZ) | 0x46200000 |
| pinctrl_z | &pinctrl_z | 仅GPIOZ | 0x46020000 |

⚠️ **PB6/PB7属于`&pinctrl`管理，不是`&pinctrl_z`！**

### 4.2 引脚复用配置

PB6: UART4_RX → AF3
PB7: UART4_TX → AF3

设备树pinctrl配置:
```dts
uart4_pins_a: uart4-0 {
    pins1 {
        pinmux = <STM32_PINMUX('B', 7, AF3)>; /* UART4_TX */
        bias-disable;
        drive-push-pull;
        slew-rate = <0>;
    };
    pins2 {
        pinmux = <STM32_PINMUX('B', 6, AF3)>; /* UART4_RX */
        bias-pull-up;
    };
};
```

## 五、M33核固件

### 5.1 项目路径

```
/home/alientek/STM32Cube_ATK_FW_MP2_V1.0.0/Projects/STM32MP257D-ATK/Applications/CM33_OpenAMP_DEMO/OpenAMP_TTY_echo/
├── CM33/NonSecure/Core/Src/main.c          # 主程序源码
├── STM32CubeIDE/CM33/NonSecure/            # IDE项目
│   └── CA35TDCID_m33_ns_sign/              # 编译输出
│       └── OpenAMP_TTY_echo_CM33_NonSecure.elf
```

### 5.1a BSP修改：COM_CM33切至LPUART1

为释放UART4给雷达，将M33调试串口从UART4改为LPUART1。

**修改文件**: `Drivers/BSP/STM32MP257F-EV1/stm32mp257f_eval.h`
- `COM_CM33_UART`: UART4 → LPUART1
- TX引脚: PB7(AF3) → PZ9(AF6)
- RX引脚: PB6(AF3) → PZ4(AF6)
- RIFSC资源: `RESMGR_RIFSC_UART4_ID` → `RESMGR_RIFSC_LPUART1_ID`

**修改文件**: `CM33/NonSecure/Core/Src/main.c`
- 移除 `#if 0` 屏蔽，直接调用`BSP_COM_Init(COM_VCP_CM33, ...)`
- BSP_COM_Init内部自动处理LPUART1的RIFSC请求

### 5.2 编译命令

```bash
rm -rf /home/alientek/stm32cubeide_workspace
mkdir -p /home/alientek/stm32cubeide_workspace
/home/alientek/download/makeself_dir_RBGMxz/y/headless-build.sh \
  -data /home/alientek/stm32cubeide_workspace \
  -importAll "/home/alientek/STM32Cube_ATK_FW_MP2_V1.0.0/Projects/STM32MP257D-ATK/Applications/CM33_OpenAMP_DEMO/OpenAMP_TTY_echo/STM32CubeIDE" \
  -build all
```

### 5.3 部署命令

```bash
# 上传固件
scp <path>/OpenAMP_TTY_echo_CM33_NonSecure.elf root@192.168.88.10:/lib/firmware/

# 停止M33核
ssh root@192.168.88.10 '
rmmod rpmsg_tty rpmsg_ctrl rpmsg_char 2>/dev/null
sleep 1
echo stop > /sys/class/remoteproc/remoteproc0/state
sleep 3
'

# 启动M33核
ssh root@192.168.88.10 '
echo OpenAMP_TTY_echo_CM33_NonSecure.elf > /sys/class/remoteproc/remoteproc0/firmware
echo start > /sys/class/remoteproc/remoteproc0/state
sleep 5
cat /sys/class/remoteproc/remoteproc0/state
'
```

### 5.2a A35设备树修改

为释放PZ4/PZ9给M33 LPUART1使用，禁用I2C8节点（引脚冲突）。

**修改文件**: `stm32mp257d-atk-ddr-1GB.dts`
- `&i2c8`节点: `status = "okay"` → `status = "disabled"`

## 六、设备树编译

```bash
# 编译RGB LCD版本DTB
cd /home/alientek/linux/atk-mp257/kernel/linux/linux-6.6.48 && \
unset LD_LIBRARY_PATH && \
source /opt/st/stm32mp2/5.0.3-snapshot/environment-setup-cortexa35-ostl-linux 2>/dev/null && \
make st/stm32mp257d-atk-ddr-1GB-rgb.dtb -j$(nproc)

# 部署DTB
scp arch/arm64/boot/dts/st/stm32mp257d-atk-ddr-1GB-rgb.dtb root@192.168.88.10:/boot/
```

## 七、交叉编译

### 7.1 A35核程序 (aarch64)

```bash
export PATH="/home/alientek/Phytium_syscode/GCC编译器/gcc-arm-10.2-2020.11-x86_64-aarch64-none-linux-gnu/bin:$PATH"
aarch64-none-linux-gnu-gcc -Wall -O2 -static -o radar_bsd src/radar_bsd.c
```

### 7.2 M33核固件 (arm-none-eabi)

通过STM32CubeIDE headless build编译，见第五节。

## 八、开发板常用命令

```bash
# 检查UART设备
ls -la /dev/ttySTM*

# 查看UART配置
stty -F /dev/ttySTM3

# 设置UART波特率
stty -F /dev/ttySTM3 921600

# 读取雷达数据
cat /dev/ttySTM3 | hexdump -C

# 检查M33核状态
cat /sys/class/remoteproc/remoteproc0/state

# 查看内核日志
dmesg | grep -i uart | tail -20

# 检查pinctrl状态
cat /sys/kernel/debug/pinctrl/46200000.pinctrl/pinconf-pins | grep -E "pin 22|pin 23"
```

## 九、参考文档

- AT6010 HCI Protocol: `/home/alientek/radar/60GBSD汽车检测AT6010 SOC HCI Protocol_V1.4.pdf`
- 雷达模块手册: `/home/alientek/radar/MS60-1211S80M-BSD产品手册_v1.0_20250908.pdf`
- 开发板硬件手册: `/home/alientek/dvr_project/硬件参考手册V1.0.pdf`
- 引脚复用手册: `/home/alientek/dvr_project/核心板接口数据手册V1.2.xlsx`
- 设备树编译指南: `/home/alientek/dvr_project/设备树编译指南.md`
