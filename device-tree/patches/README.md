# 设备树修改补丁说明

## 修改文件1: stm32mp25-pinctrl-atk-ddr-1GB.dtsi

### 修改位置
文件末尾 `};` 之前，在 `&pinctrl` 段内添加UART4引脚配置。

### 添加内容

```dts
& pinctrl {
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

    uart4_idle_pins_a: uart4-idle-0 {
        pins1 {
            pinmux = <STM32_PINMUX('B', 7, ANALOG)>; /* UART4_TX */
        };
        pins2 {
            pinmux = <STM32_PINMUX('B', 6, AF3)>; /* UART4_RX */
            bias-pull-up;
        };
    };

    uart4_sleep_pins_a: uart4-sleep-0 {
        pins {
            pinmux = <STM32_PINMUX('B', 7, ANALOG)>, /* UART4_TX */
                     <STM32_PINMUX('B', 6, ANALOG)>; /* UART4_RX */
        };
    };
};
```

### 关键注意事项

⚠️ **UART4的pinctrl配置必须放在 `&pinctrl` 段，不能放在 `&pinctrl_z` 段！**

- `&pinctrl` 管理GPIOA-GPIOZ中除GPIOZ以外的引脚（包括PB6/PB7）
- `&pinctrl_z` 只管理GPIOZ引脚
- 如果错误地放在 `&pinctrl_z` 段，会出现 "invalid function 4 on pin 23" 错误

## 修改文件2: stm32mp257d-atk-ddr-1GB.dts

### 修改位置
`&uart4` 节点内。

### 添加内容

```dts
&uart4 {
    pinctrl-names = "default", "idle", "sleep";
    pinctrl-0 = <&uart4_pins_a>;
    pinctrl-1 = <&uart4_idle_pins_a>;
    pinctrl-2 = <&uart4_sleep_pins_a>;
    /delete-property/dmas;
    /delete-property/dma-names;
    status = "okay";
};
```

### 说明

- `/delete-property/dmas` 和 `/delete-property/dma-names` 用于禁用DMA模式，使用轮询模式通信
- DMA模式在某些情况下可能导致数据丢失，轮询模式更可靠

## 编译和部署

```bash
cd /home/alientek/linux/atk-mp257/kernel/linux/linux-6.6.48 && \
unset LD_LIBRARY_PATH && \
source /opt/st/stm32mp2/5.0.3-snapshot/environment-setup-cortexa35-ostl-linux 2>/dev/null && \
make st/stm32mp257d-atk-ddr-1GB.dtb -j$(nproc) && \
scp arch/arm64/boot/dts/st/stm32mp257d-atk-ddr-1GB.dtb root@192.168.88.10:/boot/stm32mp257d-atk-ddr-1GB.dtb && \
ssh root@192.168.88.10 "sync && reboot"
```
