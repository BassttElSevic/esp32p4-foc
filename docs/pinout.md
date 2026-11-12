# 引脚配置

本仓库的固件面向一块 **双路三相驱动板**（M1 / M2 两个通道）。
两个通道的驱动芯片、采样电路结构相同，只有引脚和 ADC 通道不同。

---

## 通道引脚对照

| 信号 | **M1** | **M2** | ADC 通道（M1 / M2） |
| --- | --- | --- | --- |
| PWM-1 (U 相) | **GPIO31** | **GPIO26** | – |
| PWM-2 (V 相) | **GPIO30** | **GPIO27** | – |
| PWM-3 (W 相) | **GPIO29** | **GPIO32** | – |
| EN（驱动使能） | **GPIO28** | **GPIO33** | – |
| ISENS-1（U 相电流） | **GPIO20** | **GPIO22** | ADC1_CH4 / **ADC1_CH6** |
| ISENS-2（V 相电流） | **GPIO21** | **GPIO23** | ADC1_CH5 / **ADC1_CH7** |

> ADC 通道号由 `components/soc/esp32p4/include/soc/adc_channel.h` 决定：
> `ADC1_GPIO20_CHANNEL=4`、`ADC1_GPIO21_CHANNEL=5`、
> `ADC1_GPIO22_CHANNEL=6`、`ADC1_GPIO23_CHANNEL=7`。
> **这两个通道号写错是常见错误**，会让 Clarke 变换拿到错误相电流。

在 `firmware/foc/main/main.c` 中切换通道，只需改这 8 个宏：

```c
/* ── M1 ── */
#define PIN_PWM_U   31
#define PIN_PWM_V   30
#define PIN_PWM_W   29
#define PIN_EN      28
#define PIN_ISENS_U 20
#define PIN_ISENS_V 21
#define ADC_CH_U    ADC_CHANNEL_4
#define ADC_CH_V    ADC_CHANNEL_5
```

```c
/* ── M2 ── */
#define PIN_PWM_U   26
#define PIN_PWM_V   27
#define PIN_PWM_W   32
#define PIN_EN      33
#define PIN_ISENS_U 22
#define PIN_ISENS_V 23
#define ADC_CH_U    ADC_CHANNEL_6
#define ADC_CH_V    ADC_CHANNEL_7
```

> **注意**：GPIO26/27 在 ESP32-P4 上默认是 USB 全速 OTG 的 D−/D+ 复用功能。
> 本工程不使用 USB FS，所以作为普通 GPIO 驱动 PWM 没有问题。

---

## 外设引脚

### AS5600 磁编码器（I2C）

| 信号 | 引脚 |
| --- | --- |
| SCL | **GPIO8** |
| SDA | **GPIO7** |
| 7bit 地址 | **0x36** |
| 速率 | **400 kHz** |
| 寄存器 | `0x0B` STATUS、`0x0C` RAW ANGLE（12bit） |

### WS2812 灯珠

| 信号 | 引脚 |
|---|---|
| DIN | **GPIO52** |
| 数量 | 2 颗（可改 `LED_COUNT`）|

### EC11 机械编码器（`firmware/encoder-ec11`）

| 信号 | 引脚 |
| --- | --- |
| A 相 | **GPIO2** |
| B 相 | **GPIO3** |
| 公共端 | **GPIO4**（代码里驱动为输出低电平，当作 GND 用） |

### ESP32-P4 原生 USB（`firmware/usb-serial-jtag`）

| 信号 | 引脚 | 说明 |
| --- | --- | --- |
| D− (DM) | **GPIO24** | USB Serial/JTAG |
| D+ (DP) | **GPIO25** | 复位后 `USB_PU` 自动使能 |

> P4 的 USB 有两套：
>
> - **USB Serial/JTAG**：GPIO24/25（全速，用于控制台与下载）
> - **USB 2.0 高速 OTG**：芯片 pin 49/50（`USB_DM`/`USB_DP`），需要 `VDD_USBPHY`(pin 51) 供电

### 控制台 UART

| 信号 | 引脚 | 说明 |
| --- | --- | --- |
| UART0 TXD | GPIO37 | 默认控制台，接 CH343 |
| UART0 RXD | GPIO38 | |

---

## 电流采样链路

```text
相线 ──[ R015 0.015Ω ]── 电机
          │     │
        IN+   IN−
          └──┬──┘
         INA199A1 (Gain 50)
              │  VREF = 1.65V
              ▼
            ADC (12dB atten)
```

| 参数 | 值 |
| --- | --- |
| 灵敏度 | 0.015Ω × 50 = **0.75 mV/mA** |
| 零点 | **≈1.68V**（实测，由 REF=1.65V 决定） |
| 量程 | **±1.93 A** |
| 采样极性 | `SENSE_SIGN = -1`（本板实测两路差分输入方向与约定相反） |

> **零点会自动标定**：上电时驱动器未使能，采 200 次取均值作为零点。
> 若读到的零点接近 0V，说明 INA199 的 REF 接在 GND —— 只能测单向电流，
> 负半周会被削平，此时 `Id/Iq` 会严重失真。
