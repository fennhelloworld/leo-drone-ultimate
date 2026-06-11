# LeoDrone Ultimate — 固件架构

> ESP32-S3 传感器节点固件: FreeRTOS + ESP-IDF

---

## 固件总览

```
┌─────────────────────────────────────────────────────────┐
│                    应用层 (Application)                   │
│   sensor_app · imu_app · stream_app · ota_app · cmd_app  │
├─────────────────────────────────────────────────────────┤
│                    中间件层 (Middleware)                  │
│   BME280 Driver · ICM-42688 Driver · RTSP Server        │
│   WiFi Manager · MQTT Client · UART Transport           │
├─────────────────────────────────────────────────────────┤
│                    RTOS层 (FreeRTOS)                     │
│   Tasks · Queues · Semaphores · Timers · Events         │
├─────────────────────────────────────────────────────────┤
│                    HAL层 (ESP-IDF)                       │
│   I2C Driver · SPI Driver · UART Driver · WiFi Stack    │
│   GPIO · ADC · Timer · NVS · Partition                   │
├─────────────────────────────────────────────────────────┤
│                    硬件层 (Hardware)                      │
│   ESP32-S3 · BME280 · ICM-42688-P · USB-C · LED        │
└─────────────────────────────────────────────────────────┘
```

---

## FreeRTOS 任务架构

### 任务列表

| 任务名 | 优先级 | 堆栈 | 频率 | 功能 |
|--------|--------|------|------|------|
| `sensor_task` | 5 (高) | 4KB | 100Hz | BME280 温湿度气压采集 |
| `imu_task` | 6 (最高) | 4KB | 200Hz | ICM-42688-P IMU采集 |
| `stream_task` | 3 (中) | 8KB | 30fps | RTSP 视频流管理 |
| `comm_task` | 4 (高) | 4KB | 事件驱动 | UART/WiFi 数据发送 |
| `cmd_task` | 2 (低) | 2KB | 事件驱动 | 接收/处理地面站命令 |
| `ota_task` | 1 (最低) | 4KB | 事件驱动 | OTA 固件更新 |
| `watchdog_task` | 7 (最高) | 1KB | 1Hz | 系统看门狗+心跳 |

### 任务间通信

```
┌────────────┐     Queue(50)     ┌────────────┐
│ sensor_task├──────────────────→│  comm_task │──→ WiFi UDP ──→ 伴飞
│ (100Hz)    │  BME280Data[3]    │            │
└────────────┘                   │            │
                                 │            │──→ UART ──→ Pixhawk
┌────────────┐     Queue(100)    │            │
│  imu_task  ├──────────────────→│            │
│ (200Hz)    │  IMUData[6]       │            │
└────────────┘                   └────────────┘
                                        ↑
┌────────────┐     Queue(10)     ┌──────┴─────┐
│stream_task ├──────────────────→│ RTSP Server│──→ WiFi RTSP ──→ 伴飞
│ (30fps)    │  VideoFrame       │            │
└────────────┘                   └────────────┘

┌────────────┐    EventGroup     ┌────────────┐
│  cmd_task  │←──────────────────│ WiFi/MQTT  │
│            │  CMD_FLAG         │  Listener  │
└──────┬─────┘                   └────────────┘
       │
       ├──→ sensor_task (改变采样率)
       ├──→ imu_task (改变量程)
       └──→ stream_task (改变分辨率/码率)
```

---

## BME280 驱动 (I2C)

### 初始化

```c
#include "driver/i2c.h"
#include "bme280.h"

#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_SDA_IO   8
#define I2C_MASTER_SCL_IO   9
#define I2C_MASTER_FREQ_HZ  400000  // Fast Mode
#define BME280_I2C_ADDR     0x76

typedef struct {
    float temperature;    // °C
    float humidity;       // %RH
    float pressure;       // hPa
    uint64_t timestamp;   // microseconds
} bme280_data_t;

static QueueHandle_t bme280_queue;

esp_err_t bme280_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));

    // 软复位
    bme280_write_reg(0xE0, 0xB6);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 配置: forced mode, oversampling ×1, filter off
    bme280_write_reg(0xF2, 0x01);  // ctrl_hum: oversampling ×1
    bme280_write_reg(0xF4, 0x27);  // ctrl_meas: temp×1, press×1, forced
    bme280_write_reg(0xF5, 0x00);  // config: filter off, standby 0.5ms

    bme280_queue = xQueueCreate(50, sizeof(bme280_data_t));
    return ESP_OK;
}
```

### 采集任务

```c
void sensor_task(void *pvParameters) {
    bme280_data_t data;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // 触发测量 (forced mode)
        bme280_write_reg(0xF4, 0x27);

        // 等待测量完成 (~6.4ms for ×1 oversampling)
        vTaskDelay(pdMS_TO_TICKS(7));

        // 读取原始数据
        uint8_t raw[8];
        bme280_read_regs(0xF7, raw, 8);

        // 补偿计算
        int32_t t_raw = (raw[3] << 12) | (raw[4] << 4) | (raw[5] >> 4);
        int32_t p_raw = (raw[0] << 12) | (raw[1] << 4) | (raw[2] >> 4);
        int32_t h_raw = (raw[6] << 8) | raw[7];

        data.temperature = bme280_compensate_temperature(t_raw);
        data.pressure = bme280_compensate_pressure(p_raw) / 100.0f;
        data.humidity = bme280_compensate_humidity(h_raw);
        data.timestamp = esp_timer_get_time();

        // 发送到队列
        xQueueSend(bme280_queue, &data, 0);
    }
}
```

---

## ICM-42688-P 驱动 (SPI)

### 初始化

```c
#include "driver/spi_master.h"

#define SPI2_HOST       SPI2_HOST
#define PIN_SPI_MOSI    11
#define PIN_SPI_MISO    13
#define PIN_SPI_CLK     12
#define PIN_SPI_CS      10
#define SPI_FREQ_HZ     20000000  // 20MHz

typedef struct {
    float accel[3];      // m/s² [x, y, z]
    float gyro[3];       // rad/s [x, y, z]
    float temperature;   // °C
    uint64_t timestamp;  // microseconds
} imu_data_t;

static QueueHandle_t imu_queue;
static spi_device_handle_t imu_spi;

esp_err_t icm42688_init(void) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_SPI_MOSI,
        .miso_io_num = PIN_SPI_MISO,
        .sclk_io_num = PIN_SPI_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_FREQ_HZ,
        .mode = 0,
        .spics_io_num = PIN_SPI_CS,
        .queue_size = 7,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &imu_spi));

    // 软复位
    icm42688_write_reg(0x11, 0x01);  // BANK_SEL = 0
    icm42688_write_reg(0x1F, 0x80);  // SOFT_RESET
    vTaskDelay(pdMS_TO_TICKS(10));

    // 配置加速度计: ±16g, ODR=200Hz
    icm42688_write_reg(0x1F, 0x06);  // ACCEL_CONFIG0: ±16g, ODR=200Hz

    // 配置陀螺仪: ±2000°/s, ODR=200Hz
    icm42688_write_reg(0x20, 0x06);  // GYRO_CONFIG0: ±2000°/s, ODR=200Hz

    // 使能加速度计+陀螺仪
    icm42688_write_reg(0x1D, 0x0F);  // PWR_MGMT0: ACCEL_LN_MODE, GYRO_LN_MODE

    vTaskDelay(pdMS_TO_TICKS(50));  // 等待稳定

    imu_queue = xQueueCreate(100, sizeof(imu_data_t));
    return ESP_OK;
}
```

### 采集任务

```c
void imu_task(void *pvParameters) {
    imu_data_t data;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // 200Hz

    // 量程常数
    const float accel_scale = 16.0f * 9.81f / 32768.0f;  // ±16g → m/s²
    const float gyro_scale = 2000.0f * M_PI / (180.0f * 32768.0f);  // ±2000°/s → rad/s

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // 读取数据 (0x1D~0x26: TEMP+ACCEL+GYRO, 14 bytes)
        uint8_t raw[14];
        icm42688_read_regs(0x1D, raw, 14);

        // 温度
        int16_t temp_raw = (raw[0] << 8) | raw[1];
        data.temperature = temp_raw / 132.48f + 25.0f;

        // 加速度 (big-endian)
        int16_t ax = (raw[2] << 8) | raw[3];
        int16_t ay = (raw[4] << 8) | raw[5];
        int16_t az = (raw[6] << 8) | raw[7];
        data.accel[0] = ax * accel_scale;
        data.accel[1] = ay * accel_scale;
        data.accel[2] = az * accel_scale;

        // 陀螺仪 (big-endian)
        int16_t gx = (raw[8] << 8) | raw[9];
        int16_t gy = (raw[10] << 8) | raw[11];
        int16_t gz = (raw[12] << 8) | raw[13];
        data.gyro[0] = gx * gyro_scale;
        data.gyro[1] = gy * gyro_scale;
        data.gyro[2] = gz * gyro_scale;

        data.timestamp = esp_timer_get_time();

        xQueueSend(imu_queue, &data, 0);
    }
}
```

---

## RTSP 流媒体管线

```
┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐
│ IMX219×4 │──→│ JPEG编码 │──→│ RTSP服务 │──→│ 伴飞/RPi │
│ MIPI-CSI │    │ (MJPEG)  │    │ (live555)│    │ GStreamer│
└──────────┘    └──────────┘    └──────────┘    └──────────┘
   30fps          硬件JPEG        端口8554        rtsp://
  1640×1232       压缩率80%       UDP/TCP         esp32:8554
```

### 流媒体任务

```c
void stream_task(void *pvParameters) {
    // 初始化摄像头
    camera_config_t camera_config = {
        .pin_pwdn = -1,
        .pin_reset = -1,
        .pin_xclk = 15,
        .pin_sccb_sda = 4,
        .pin_sccb_scl = 5,
        .pin_d7 = 16, .pin_d6 = 17, .pin_d5 = 18, .pin_d4 = 12,
        .pin_d3 = 10, .pin_d2 = 8,  .pin_d1 = 9,  .pin_d0 = 11,
        .pin_vsync = 6, .pin_href = 7, .pin_pclk = 13,
        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = FRAMESIZE_UXGA,   // 1600×1200
        .jpeg_quality = 80,
        .fb_count = 2,
        .grab_mode = CAMERA_GRAB_LATEST,
    };
    ESP_ERROR_CHECK(esp_camera_init(&camera_config));

    // 启动 RTSP 服务器
    rtsp_server_start(8554);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(33); // ~30fps

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // 获取 JPEG 帧
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            rtsp_server_send_frame(fb->buf, fb->len, fb->timestamp);
            esp_camera_fb_return(fb);
        }
    }
}
```

---

## 通信协议

### 传感器数据帧格式 (UART → 伴飞)

```
┌──────┬──────┬──────┬─────────────┬──────┐
│ 0xAA │ 0x55 │ TYPE │  PAYLOAD    │ CRC  │
│ 1B   │ 1B   │ 1B   │  N bytes    │ 2B   │
└──────┴──────┴──────┴─────────────┴──────┘

TYPE = 0x01: BME280 (12 bytes)
  [temp_f32][humid_f32][press_f32] = 12 bytes

TYPE = 0x02: ICM-42688-P (28 bytes)
  [accel_f32×3][gyro_f32×3][temp_f32] = 28 bytes

TYPE = 0x03: 组合帧 (40 bytes)
  [bme280_12B][icm_28B] = 40 bytes

CRC: CRC-16/MODBUS over TYPE+PAYLOAD
```

### WiFi UDP 数据流

```python
# 伴飞计算机接收端 (Python)
import socket
import struct

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 9876))

while True:
    data, addr = sock.recvfrom(1024)
    if data[0:2] == b'\xaa\x55':
        msg_type = data[2]
        payload = data[3:-2]
        crc = struct.unpack('<H', data[-2:])[0]

        if msg_type == 0x01:  # BME280
            temp, humid, press = struct.unpack('<fff', payload)
        elif msg_type == 0x02:  # IMU
            ax, ay, az, gx, gy, gz, temp = struct.unpack('<7f', payload)
```

---

## OTA 更新机制

```
┌──────────────┐         ┌──────────────┐
│ Ground       │  WiFi   │ ESP32-S3     │
│ Station      │ ──HTTP──│              │
│              │  POST   │ ┌──────────┐ │
│ firmware.bin │ ──────→ │ │ OTA      │ │
│ (新版本)     │         │ │ Task     │ │
└──────────────┘         │ └────┬─────┘ │
                         │      │        │
                         │ ┌────▼─────┐ │
                         │ │ A/B分区  │ │
                         │ │ OTA_0 ✓  │ │
                         │ │ OTA_1    │ │ ← 写入新固件
                         │ └──────────┘ │
                         │              │
                         │ 重启→验证→   │
                         │ 切换启动分区 │
                         └──────────────┘
```

### OTA 流程

1. **触发**: 地面站通过 MQTT/HTTP 发送 OTA 命令
2. **下载**: ESP32-S3 通过 HTTPS 下载固件到 OTA_1 分区
3. **验证**: 校验 SHA-256 + 签名
4. **切换**: 标记 OTA_1 为下次启动分区
5. **重启**: 软复位
6. **回滚**: 如果新固件启动失败，自动回滚到 OTA_0

### 分区表

```
# Name     Type  SubType  Offset   Size
nvs        data  nvs      0x9000   0x4000
phy_init   data  phy      0xd000   0x1000
ota_0      app   ota_0    0x10000  0x1E0000
ota_1      app   ota_1    0x1F0000 0x1E0000
ota_data   data  ota      0x3D0000 0x2000
storage    data  fat      0x3D2000 0x2E000
```

---

## 构建系统 (PlatformIO)

### platformio.ini

```ini
; LeoDrone Ultimate - ESP32-S3 Sensor Node
[env:esp32-s3]
platform = espressif32
board = esp32-s3-devkitc-1
framework = espidf

; 串口配置
monitor_speed = 460800
upload_speed = 921600
upload_port = /dev/ttyUSB0

; 编译选项
build_flags =
    -DCORE_DEBUG_LEVEL=3
    -DBOARD_ESP32S3_DEVKITC
    -DCONFIG_FREERTOS_HZ=1000

; 分区表 (支持OTA)
board_build.partitions = partitions.csv

; 库依赖
lib_deps =
    bme280@^1.0
    esp32-camera@^2.0
```

---

## 安全机制

| 机制 | 实现 | 说明 |
|------|------|------|
| 看门狗 | TaskWatchdog + HW WDT | 1s超时, 3次触发重启 |
| 传感器校验 | 范围检查+CRC | 温度[-40,85], 湿度[0,100] |
| OTA回滚 | A/B分区+bootloader | 新固件启动失败自动回滚 |
| WiFi断线重连 | 自动重连+AP模式回退 | 3次失败切换AP模式 |
| 数据完整性 | CRC-16/MODBUS | 每帧校验 |
| 堆栈溢出检测 | FreeRTOS Stack Overflow Hook | canary值检查 |
