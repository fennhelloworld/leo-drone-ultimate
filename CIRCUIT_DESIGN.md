# LeoDrone Ultimate — 电路设计

> ESP32-S3 传感器节点 + STM32H743 飞控 + 电源管理 + 通信接口

---

## 系统电路总览

```
┌─────────────────────────────────────────────────────────────┐
│                      4S LiPo (14.8V)                        │
│                    XT60 Connector                            │
└───────────┬────────────────────────────┬───────────────────┘
            │                            │
    ┌───────┴───────┐            ┌───────┴───────┐
    │ 5V/3A BEC    │            │ 4× ESC 30A    │
    │ (降压模块)    │            │ BLHeli_S      │
    └───────┬───────┘            └───────┬───────┘
            │ 5V                       │ PWM
    ┌───────┴───────────┐       ┌───────┴───────┐
    │ 3.3V LDO          │       │ 4× 2212 Motor │
    │ (AMS1117-3.3)     │       │ 920kV         │
    └───┬───────────┬───┘       └───────────────┘
        │ 3.3V      │ 3.3V
   ┌────┴────┐ ┌────┴────────────────────┐
   │ESP32-S3 │ │ STM32H743 (Pixhawk 6C) │
   │Sensor   │ │ Flight Controller      │
   │Node     │ │                         │
   └──┬──┬───┘ └──┬────┬────┬───────────┘
      │  │        │    │    │
   I2C│  │SPI  UART│ I2C│  UART│
      │  │        │    │    │
  ┌───┴──┴──┐  ┌──┴──┐ │ ┌──┴──────┐
  │BME280   │  │GPS  │ │ │Companion│
  │ICM-42688│  │M10N │ │ │Computer │
  └─────────┘  └─────┘ │ └─────────┘
                     I2C│
                    ┌───┴────┐
                    │QMC5883L│
                    │Compass │
                    └────────┘
```

---

## MCU 选型

### ESP32-S3 (传感器节点)

| 参数 | 规格 | 选择理由 |
|------|------|---------|
| 内核 | Xtensa LX7 双核 240MHz | 足够运行传感器采集+WiFi流 |
| SRAM | 512KB | 传感器缓冲+图像压缩 |
| Flash | 8MB (外挂) | 固件+校准数据 |
| WiFi | 802.11 b/g/n 2.4GHz | RTSP视频流+UDP传感器数据 |
| BLE | 5.0 | 远程配置/调试 |
| I2C | 2× | BME280 + 扩展 |
| SPI | 2× (40MHz) | ICM-42688-P + Flash |
| UART | 3× | 伴飞通信+调试 |
| GPIO | 45× | 充裕的IO |
| ADC | 2× 12-bit | 电池电压监测 |
| 价格 | ¥15-25 | 低成本 |

### STM32H743 (飞控 — Pixhawk 6C)

| 参数 | 规格 | 选择理由 |
|------|------|---------|
| 内核 | Cortex-M7 @480MHz | PX4飞控实时性要求 |
| SRAM | 1024KB | 飞控+日志 |
| Flash | 2MB | PX4固件 |
| 定时器 | 高级PWM ×多路 | 4×ESC + 云台PWM |
| UART | 8× | GPS+伴飞+数传+调试 |
| I2C | 4× | 罗盘+空速+扩展 |
| SPI | 4× | 双IMU+Flash |
| CAN | 2× | 扩展总线 |
| 价格 | ¥30-50 (飞控模组) | 成熟方案 |

---

## 电源管理

### 4S LiPo → 5V/3A BEC → 3.3V LDO

```
4S LiPo (14.8V nominal, 12.8-16.8V range)
    │
    ├──→ ESC×4 (直连, 14.8V → 电机)
    │
    └──→ 5V/3A BEC (MP1584EN 或类似)
         │
         ├──→ Pixhawk 6C POWER (5V input)
         │
         ├──→ ESP32-S3 VBUS (5V via USB)
         │
         └──→ AMS1117-3.3 LDO
              │
              ├──→ ESP32-S3 3.3V rail
              │    ├── BME280 (3.3V, <1mA)
              │    └── ICM-42688-P (3.3V, <4mA)
              │
              └──→ GPS M10N (3.3V, <30mA)
```

### 功耗预算

| 模块 | 电压 | 电流 | 功率 | 来源 |
|------|------|------|------|------|
| 4×ESC+电机 | 14.8V | 15A avg | 222W | 4S直连 |
| Pixhawk 6C | 5V | 300mA | 1.5W | BEC |
| ESP32-S3 | 3.3V | 200mA | 0.66W | LDO |
| BME280 | 3.3V | 1mA | 3.3mW | LDO |
| ICM-42688-P | 3.3V | 4mA | 13.2mW | LDO |
| GPS M10N | 3.3V | 30mA | 99mW | LDO |
| 4×IMX219 | 3.3V | 120mA×4 | 1.58W | RPi5 |
| RPi5 4GB | 5V | 3A | 15W | BEC |
| **总计** | | | **~241W** | |
| **电池续航** | 4S 3000mAh 30C | | **~11min** | 实际~15min(非满载) |

---

## 传感器接口

### I2C 总线 (ESP32-S3)

```
ESP32-S3 I2C0 (SDA=GPIO8, SCL=GPIO9, 400kHz Fast Mode)
    │
    ├── BME280 (地址: 0x76)
    │   ├── 温度: -40~+85°C, ±0.5°C
    │   ├── 湿度: 0~100%RH, ±2%RH
    │   ├── 气压: 300~1100hPa, ±1hPa
    │   └── 采样率: 100Hz (forced mode)
    │
    └── [扩展] BME680 (地址: 0x77) — 空气质量
```

### SPI 总线 (ESP32-S3)

```
ESP32-S3 SPI2 (MOSI=GPIO11, MISO=GPIO13, SCK=GPIO12, CS=GPIO10)
    │
    └── ICM-42688-P (地址: CS=GPIO10)
        ├── 加速度: ±2/4/8/16g, 16-bit
        ├── 陀螺仪: ±125/250/500/1000/2000°/s, 16-bit
        ├── 温度: -40~+85°C
        ├── 采样率: 200Hz (accel+gyro ODR)
        └── SPI时钟: 20MHz max
```

### MIPI-CSI (ESP32-S3 → 实际连接到 RPi5)

```
Raspberry Pi 5 CSI-2 接口 ×4 (通过适配板)
    │
    ├── CAM0: IMX219 鱼眼160° (前, 0°)
    ├── CAM1: IMX219 鱼眼160° (右, 90°)
    ├── CAM2: IMX219 鱼眼160° (后, 180°)
    └── CAM3: IMX219 鱼眼160° (左, 270°)

    每摄像头: 1640×1232 @30fps, H.264/MJPEG
    同步: 硬件触发 (GPIO脉冲同步)
```

---

## 通信接口

### UART: 飞控 ↔ 伴飞计算机

```
Pixhawk 6C TELEM2                Raspberry Pi 5 UART
┌─────────────────┐              ┌─────────────────┐
│ TX  ──────────────→ RX (GPIO15) │
│ RX ←────────────── TX (GPIO14) │
│ GND ──────────────── GND       │
└─────────────────┘              └─────────────────┘

波特率: 921600 bps (MAVLink 2.0)
协议: MAVLink 2.0 (消息ID + 签名)
用途: offboard控制指令 + 遥测数据
```

### UART: ESP32-S3 ↔ 伴飞计算机

```
ESP32-S3 UART0                    Raspberry Pi 5 USB
┌─────────────────┐              ┌─────────────────┐
│ TX  ──────────────→ USB-RX     │
│ RX ←────────────── USB-TX     │
│ GND ──────────────── GND       │
└─────────────────┘              └─────────────────┘

波特率: 460800 bps
协议: 自定义二进制帧 (header + payload + CRC)
用途: BME280温湿度 + ICM-42688-P IMU原始数据
```

### WiFi: ESP32-S3 ↔ 伴飞计算机 (RTSP视频流)

```
ESP32-S3 WiFi AP/STA              Raspberry Pi 5 WiFi
┌─────────────────┐              ┌─────────────────┐
│ RTSP Server     │ ──WiFi──→   │ RTSP Client     │
│ rtsp://esp32:   │   2.4GHz    │ GStreamer/FFmpeg│
│ 8554/stream     │   802.11n   │                 │
└─────────────────┘              └─────────────────┘

带宽: ~20Mbps (4×MJPEG @ 1080p)
延迟: < 50ms
```

### WiFi/4G: 伴飞 ↔ 地面站

```
Raspberry Pi 5                    Ground Station
┌─────────────────┐              ┌─────────────────┐
│ MAVLink UDP     │ ──WiFi──→   │ QGroundControl  │
│ :14550          │   或 4G     │ Web Dashboard   │
│ RTSP :8554      │ ──RTSP──→   │ 视频监控        │
└─────────────────┘              └─────────────────┘
```

---

## SKiDL 传感器节点电路

```python
"""
LeoDrone Ultimate — ESP32-S3 Sensor Node Circuit
SKiDL 2.2.3 代码

功能:
- ESP32-S3 主控
- BME280 温湿度 (I2C)
- ICM-42688-P IMU (SPI)
- 电源: 5V输入 → AMS1117-3.3V
- 调试: USB-C + UART
"""

from skidl import *

# ============================================================
# 电源部分
# ============================================================

# 5V 输入 (从BEC)
v5v = Net('+5V', netclass='Power')
gnd = Net('GND', netclass='Power')

# 3.3V LDO: AMS1117-3.3
ldo = Part('regulator', 'AMS1117-3.3', footprint='SOT-223')
ldo.IN += v5v
ldo.GND += gnd
v3v3 = Net('+3V3', netclass='Power')
ldo.OUT += v3v3

# LDO 输入/输出电容
c_in = Part('device', 'C', footprint='0805', value='10uF')
c_in[1] += v5v
c_in[2] += gnd

c_out = Part('device', 'C', footprint='0805', value='22uF')
c_out[1] += v3v3
c_out[2] += gnd

# 去耦电容 (每IC)
for i in range(3):
    c_dec = Part('device', 'C', footprint='0402', value='100nF')
    c_dec[1] += v3v3
    c_dec[2] += gnd

# ============================================================
# ESP32-S3 主控
# ============================================================

esp32 = Part('RF_Module', 'ESP32-S3-WROOM-1-N8R8',
             footprint='RF_Module:ESP32-S3-WROOM-1')

# 电源
esp32['3V3'] += v3v3
esp32['GND'] += gnd

# EN 上拉
r_en = Part('device', 'R', footprint='0402', value='10k')
r_en[1] += v3v3
r_en[2] += esp32['EN']

# BOOT 按钮
boot_btn = Part('Switch', 'SW_Push', footprint='B3U-1000P')
boot_btn[1] += gnd
boot_btn[2] += esp32['IO0']

# ============================================================
# I2C 总线: BME280 温湿度
# ============================================================

# I2C 信号
i2c_sda = Net('I2C_SDA')
i2c_scl = Net('I2C_SCL')

# ESP32-S3 I2C 引脚
esp32['IO8'] += i2c_sda   # SDA
esp32['IO9'] += i2c_scl   # SCL

# I2C 上拉电阻
for net in [i2c_sda, i2c_scl]:
    r_pullup = Part('device', 'R', footprint='0402', value='4.7k')
    r_pullup[1] += v3v3
    r_pullup[2] += net

# BME280 传感器
bme280 = Part('Sensor', 'BME280', footprint='Bosch_LGA-8')

bme280['VDD'] += v3v3
bme280['GND'] += gnd
bme280['SDI'] += i2c_sda
bme280['SCK'] += i2c_scl
bme280['SDO'] += gnd       # 地址: 0x76 (SDO=GND)
bme280['CSB'] += v3v3      # I2C模式 (CSB=HIGH)

# BME280 去耦
c_bme = Part('device', 'C', footprint='0402', value='100nF')
c_bme[1] += v3v3
c_bme[2] += gnd

# ============================================================
# SPI 总线: ICM-42688-P IMU
# ============================================================

# SPI 信号
spi_mosi = Net('SPI_MOSI')
spi_miso = Net('SPI_MISO')
spi_sck = Net('SPI_SCK')
spi_cs_imu = Net('SPI_CS_IMU')

# ESP32-S3 SPI 引脚 (SPI2)
esp32['IO11'] += spi_mosi   # MOSI
esp32['IO13'] += spi_miso   # MISO
esp32['IO12'] += spi_sck    # SCK
esp32['IO10'] += spi_cs_imu # CS (active low)

# ICM-42688-P IMU
icm = Part('Sensor_Motion', 'ICM-42688-P', footprint='LGA-14')

icm['VDD'] += v3v3
icm['VDDIO'] += v3v3
icm['GND'] += gnd
icm['SDI'] += spi_mosi
icm['SDO'] += spi_miso
icm['SCL'] += spi_sck
icm['CS'] += spi_cs_imu

# IMU 去耦
c_imu1 = Part('device', 'C', footprint='0402', value='100nF')
c_imu1[1] += v3v3
c_imu1[2] += gnd
c_imu2 = Part('device', 'C', footprint='0402', value='10uF')
c_imu2[1] += v3v3
c_imu2[2] += gnd

# ============================================================
# UART 调试/通信
# ============================================================

# UART0 (调试)
uart_tx = Net('UART0_TX')
uart_rx = Net('UART0_RX')
esp32['IO43'] += uart_tx
esp32['IO44'] += uart_rx

# USB-C 连接器 (调试+供电)
usbc = Part('Connector', 'USB_C_Receptacle', footprint='USB_C')
usbc['VBUS'] += v5v
usbc['GND'] += gnd
# USB D+/D- 连接 ESP32-S3 内置USB PHY
usbc['DP'] += esp32['IO20']
usbc['DN'] += esp32['IO19']

# ============================================================
# 状态指示
# ============================================================

# LED (电源+状态)
led_pwr = Part('device', 'LED', footprint='0805')
r_led = Part('device', 'R', footprint='0402', value='1k')
led_pwr['A'] += v3v3
led_pwr['K'] += r_led[1]
r_led[2] += gnd

led_status = Part('device', 'LED', footprint='0805')
r_led2 = Part('device', 'R', footprint='0402', value='1k')
led_status['A'] += v3v3
led_status['K'] += r_led2[1]
r_led2[2] += esp32['IO2']  # 状态LED

# ============================================================
# 生成网表
# ============================================================

generate_netlist()
```

---

## PCB 设计规范

| 参数 | 规格 |
|------|------|
| 板厚 | 1.6mm (标准) |
| 层数 | 4层 (Signal-GND-PWR-Signal) |
| 最小线宽/间距 | 0.15mm / 0.15mm |
| 过孔 | 0.3mm drill / 0.6mm pad |
| 阻抗控制 | USB: 90Ω差分, SPI: 无要求 |
| BME280 布局 | 远离热源, 底部通风孔 |
| ICM-42688-P 布局 | 中心位置, 远离振动, GND平面完整 |
| 尺寸 | 40mm × 35mm (适配无人机安装) |
| 安装孔 | 4×M2.5 (与S500机架兼容) |

---

## SPICE 仿真

### 电源纹波仿真

```
* LeoDrone Ultimate - Power Supply Simulation
* 4S LiPo → BEC → 3.3V LDO

Vin 1 0 DC 14.8 AC 0.5
XBEC 1 2 MP1584EN
XLDO 2 3 AMS1117-3.3

* 负载模型
RLoad 3 0 16.5  ; 3.3V/200mA = 16.5Ω
CLoad 3 0 22uF

* 瞬态分析
.tran 0.1m 10m
.ac dec 10 10 100k

.print ac v(3)
.end
```

### I2C 上拉仿真

```
* BME280 I2C Pull-up Simulation
VSCL 1 0 PULSE(3.3 0 0 100n 100n 2.5u 5u)
RPULL 1 2 4.7k
RSERIES 2 3 100
CBME 3 0 10pF

.tran 0.01u 20u
.print tran v(1) v(3)
.end
```
