# LeoDrone Ultimate — 7层全栈架构

> 从硅片到智能的全链路设计：L0 硬件 → L6 交互

---

## 架构总览

```
┌──────────────────────────────────────────────────────────────────────┐
│                        L6 交互层 Interaction                         │
│   EdgeVoiceBox · MCP Gateway · Web Dashboard · QGroundControl       │
│   ── 语音指令 → 智能决策可视化 → 远程操控 ──                          │
├──────────────────────────────────────────────────────────────────────┤
│                        L5 协同层 Coordination                        │
│   SwarmCoordinator · UAVPlanner(BSB-SSSP) · MAVSDK Offboard         │
│   ── 多机编队 → 路径规划 → 自主飞行控制 ──                           │
├──────────────────────────────────────────────────────────────────────┤
│                        L4 认知层 Cognition                            │
│   GOAT-Mamba Hybrid · MoE Router(160专家) · CausalEngine            │
│   ── 长序列推理 → 专家路由 → 因果安全预警 ──                         │
├──────────────────────────────────────────────────────────────────────┤
│                        L3 融合层 Fusion                               │
│   EKF SensorFusion(12维) · TemporalKG · DataIntegrator              │
│   ── IMU+GPS+温湿度融合 → 时序知识图谱 → 多源异构整编 ──            │
├──────────────────────────────────────────────────────────────────────┤
│                        L2 感知层 Perception                           │
│   VideoStabilizer(VQF) · SLAM(VINS) · ObjectTracker · Stitch360    │
│   ── 视频防抖 → 3D建图 → 目标检测跟踪 → 全景拼接 ──                 │
├──────────────────────────────────────────────────────────────────────┤
│                        L1 传感层 Sensing                              │
│   BME280(温湿度) · ICM-42688-P(IMU) · IMX219×4(视觉) · GPS M10N   │
│   ── 环境感知 → 姿态感知 → 视觉感知 → 定位感知 ──                   │
├──────────────────────────────────────────────────────────────────────┤
│                        L0 硬件层 Hardware                             │
│   STM32H743(飞控) · ESP32-S3(传感) · S500 Frame · 4S LiPo          │
│   ── 飞行控制 → 传感采集 → 机械结构 → 供电系统 ──                   │
└──────────────────────────────────────────────────────────────────────┘
```

---

## 各层详细映射

### L0 硬件层 (Hardware)

| 模块 | MCU | 接口 | 来源 | 功能 |
|------|-----|------|------|------|
| 飞控主控 | STM32H743 @480MHz | UART/SPI/I2C/CAN | drone-system | PX4飞控固件，4×PWM ESC输出 |
| 传感节点 | ESP32-S3 @240MHz | I2C/SPI/WiFi | 本项目 | BME280温湿度 + ICM-42688-P IMU + WiFi流 |
| GPS模块 | HGLRC M10N | UART | drone-system | 双频GPS + QMC5883L罗盘 |
| 摄像头×4 | Arducam IMX219 鱼眼160° | MIPI-CSI | drone-system | 360°全景采集阵列 |
| 伴飞计算机 | RPi5 / Jetson Orin NX | UART/CSI/Ethernet | drone-system | 视觉处理+AI推理 |
| 机架 | S500碳纤维 | — | drone-system | 450mm轴距四旋翼 |
| 电池 | 4S 3000mAh 30C LiPo | XT60 | drone-system | 14.8V标称, 44.4Wh |

### L1 传感层 (Sensing)

| 传感器 | 接口 | 采样率 | 数据类型 | 来源 |
|--------|------|--------|---------|------|
| BME280 温湿度 | I2C (0x76) | 100Hz | temperature, humidity, pressure | 本项目(Case A) |
| ICM-42688-P IMU | SPI (20MHz) | 200Hz | accel[3], gyro[3], temp | 本项目 |
| IMX219×4 鱼眼摄像头 | MIPI-CSI×4 | 30fps | H=1640, W=1232, FOV=160° | drone-system |
| GPS M10N | UART(460800) | 10Hz | lat, lon, alt, HDOP | drone-system |
| QMC5883L 罗盘 | I2C (0x0D) | 100Hz | mag[3] | drone-system |

### L2 感知层 (Perception)

| 模块 | 算法 | 输入 → 输出 | 来源 |
|------|------|------------|------|
| VideoStabilizer | VQF + EKF + 互补滤波 | IMU数据 → 稳定帧变换 | omni-perception-fusion (专利1) |
| Stitch360 | 鱼眼校正 + 球面投影 + 多频融合 | 4×鱼眼帧 → 等距柱状投影 | drone-system |
| SLAM | VINS-Fusion (立体VIO) | 同步帧+IMU → 6DoF位姿+3D点云 | drone-system |
| ObjectTracker | YOLOv8-nano + DeepSORT | 视频帧 → bbox[id,cls,conf] | drone-system |
| VelocityEstimator | 光流 + IMU积分 | 帧+IMU → 速度向量 | omni-perception-fusion |

### L3 融合层 (Fusion)

| 模块 | 算法 | 输入 → 输出 | 来源 |
|------|------|------------|------|
| EKFSensorFusion | 扩展卡尔曼滤波(12维) | IMU+GPS+Baro → 姿态/速度/位置 | omni-perception-fusion |
| CausalEngine | 因果图 + do-calculus | 传感器数据 → 因果推理结果 | omni-perception-fusion (专利3) |
| TemporalKG | 时序知识图谱 | 事件序列 → 时序关系图 | omni-perception-fusion (专利3) |
| DataIntegrator | 多源异构整编 | 温湿度+IMU+GPS+视觉 → 统一时空 | omni-perception-fusion |

### L4 认知层 (Cognition)

| 模块 | 算法 | 输入 → 输出 | 来源 |
|------|------|------------|------|
| GOAT-Mamba | Fourier先验+Sink+SSM | 长序列 → 上下文表征 | omni-perception-fusion (专利7) |
| MoERouter | DeepSeekMoE top-k | 任务向量 → 专家选择+权重 | omni-perception-fusion (专利8) |
| CausalGraph | LLM先验+传感器约束 | 查询 → 因果链+干预效应 | omni-perception-fusion (专利3) |

### L5 协同层 (Coordination)

| 模块 | 算法 | 输入 → 输出 | 来源 |
|------|------|------------|------|
| UAVPlanner | BSB-SSSP + HDP-TS RRT* + ESDF | UAV状态+障碍物 → 无碰撞路径 | omni-perception-fusion (专利2) |
| SwarmCoordinator | 领导-跟随 + 一致性协议 | 编队参数 → 协同控制指令 | omni-perception-fusion (专利5) |
| MAVSDK Offboard | 速度/位置/姿态控制 | 路径点 → MAVLink指令 | drone-system |

### L6 交互层 (Interaction)

| 模块 | 算法 | 输入 → 输出 | 来源 |
|------|------|------------|------|
| EdgeVoiceBox | Distil-Whisper + SmolLM-Int4 + Piper | 语音 → 文本 → 推理 → 语音 | omni-perception-fusion (专利4) |
| MCPGateway | MCP协议 + 工具注册 | 智能体请求 → 工具调用 | omni-perception-fusion (专利5) |
| WebDashboard | Flask + WebSocket + Three.js | 传感器数据 → 可视化 | drone-system |
| QGroundControl | MAVLink | 飞行状态 → GCS界面 | drone-system |

---

## 数据流图

```
                    ┌─────────────────────────────────┐
                    │        物理世界 (户外运动)         │
                    └──────────┬──────────────────────┘
                               │
          ┌────────────────────┼───────────────────────┐
          │                    │                        │
    ┌─────┴─────┐      ┌──────┴──────┐         ┌──────┴──────┐
    │  BME280   │      │ ICM-42688-P │         │  IMX219×4   │
    │ 温湿度气压 │      │  IMU 6轴    │         │  鱼眼摄像头  │
    └─────┬─────┘      └──────┬──────┘         └──────┬──────┘
          │ I2C              │ SPI                   │ MIPI-CSI
    ┌─────┴──────────────────┴───────────────────────┴──────┐
    │                    ESP32-S3 传感节点                    │
    │  sensor_task(100Hz)  imu_task(200Hz)  stream_task(30fps)│
    └─────┬──────────────────┬───────────────────────┬──────┘
          │ WiFi/UART        │ UART                  │ WiFi RTSP
    ┌─────┴──────────────────┴───────────────────────┴──────┐
    │               伴飞计算机 (RPi5/Jetson)                  │
    │                                                         │
    │  ┌──────────┐  ┌──────────┐  ┌──────────┐            │
    │  │Stitch360 │  │  VINS    │  │VideoStab │            │
    │  │4→1全景   │  │  SLAM    │  │ VQF+EKF  │            │
    │  └────┬─────┘  └────┬─────┘  └────┬─────┘            │
    │       │             │              │                    │
    │  ┌────┴─────────────┴──────────────┴─────┐            │
    │  │            EKF SensorFusion            │            │
    │  │   IMU + GPS + 温湿度 + Baro → 12维状态  │            │
    │  └────────────────┬───────────────────────┘            │
    │                   │                                     │
    │  ┌────────────────┴───────────────────────┐            │
    │  │            CausalEngine                │            │
    │  │  温湿度异常 → 飞行安全因果推理          │            │
    │  └────────────────┬───────────────────────┘            │
    │                   │                                     │
    │  ┌────────────────┴───────────────────────┐            │
    │  │         GOAT-Mamba + MoE               │            │
    │  │  长序列推理 → 专家路由 → 智能决策       │            │
    │  └────────────────┬───────────────────────┘            │
    │                   │                                     │
    │  ┌────────────────┴───────────────────────┐            │
    │  │         UAVPlanner + SwarmCoord        │            │
    │  │  多机路径规划 → 编队控制                │            │
    │  └────────────────┬───────────────────────┘            │
    └────────────────────┼────────────────────────────────────┘
                         │ UART (TELEM2, 921600)
                  ┌──────┴──────┐
                  │ Pixhawk 6C  │
                  │  PX4 飞控   │
                  └──────┬──────┘
                         │ MAVLink
            ┌────────────┼────────────┐
            │            │            │
     ┌──────┴─────┐ ┌───┴────┐ ┌────┴─────┐
     │ 4×ESC+Motor│ │ GPS    │ │ 地面站    │
     │ 飞行动力   │ │ M10N   │ │ QGC+Web  │
     └────────────┘ └────────┘ └──────────┘
```

---

## 集成点详解

### 集成点 1: ESP32-S3 ↔ 伴飞计算机

```
ESP32-S3 (传感节点)                    伴飞计算机 (RPi5/Jetson)
┌──────────────────┐                  ┌──────────────────────┐
│ sensor_task      │ ──WiFi/UART──→  │ EKF SensorFusion     │
│  BME280 → I2C    │   温湿度数据     │  IMU+温湿度+GPS融合  │
│ imu_task         │ ──UART────────→ │ VINS-Fusion          │
│  ICM-42688-P→SPI │   IMU原始数据    │  视觉惯性里程计      │
│ stream_task      │ ──RTSP/WiFi──→  │ Stitch360 + EIS      │
│  IMX219→JPEG/H264│   视频流        │  全景拼接+防抖       │
└──────────────────┘                  └──────────────────────┘
```

### 集成点 2: 伴飞计算机 ↔ Pixhawk 飞控

```
伴飞计算机 (RPi5/Jetson)              Pixhawk 6C (STM32H743)
┌──────────────────────┐              ┌──────────────────────┐
│ MAVSDK Offboard      │ ──UART──→   │ PX4 Flight Control   │
│  速度/位置/姿态指令   │  921600bps  │  姿态稳定+电机输出   │
│ SLAM位姿反馈         │ ←──UART──   │  原始IMU+GPS+状态    │
│ CausalEngine安全决策  │              │                      │
└──────────────────────┘              └──────────────────────┘
```

### 集成点 3: omni-perception-fusion AI ↔ drone-system 控制

```
omni-perception-fusion (AI决策)        drone-system (飞控执行)
┌──────────────────────┐              ┌──────────────────────┐
│ VideoStabilizer      │              │ 360° Stitch Pipeline │
│  VQF姿态 → 帧变换   │ ──补偿矩阵──→│  稳定全景输出        │
│ UAVPlanner           │              │ MAVSDK Offboard      │
│  BSB-SSSP路径        │ ──航点序列──→│  速度/位置控制       │
│ CausalEngine         │              │ 故障保护             │
│  温湿度→安全预警     │ ──安全指令──→│  返航/降高/悬停      │
│ EKFSensorFusion      │              │ VINS-Fusion          │
│  12维状态估计        │ ──位姿先验──→│  SLAM初始化+约束     │
│ SwarmCoordinator     │              │ MAVSDK Multi-Drone   │
│  编队参数            │ ──编队指令──→│  多机协同控制        │
│ EdgeVoiceBox         │              │ Web Dashboard        │
│  语音→文本→决策→语音 │ ──状态查询──→│  可视化显示          │
└──────────────────────┘              └──────────────────────┘
```

---

## 关键数据结构

### L1 → L2: 传感器原始数据

```python
# BME280 温湿度 (100Hz)
@dataclass
class BME280Sample:
    timestamp: float       # seconds
    temperature: float     # °C (±0.5°C)
    humidity: float        # %RH (±2%)
    pressure: float        # hPa (±1hPa)

# ICM-42688-P IMU (200Hz)
@dataclass
class IMUSample:
    timestamp: float       # seconds
    gyro: np.ndarray       # [wx, wy, wz] rad/s
    accel: np.ndarray      # [ax, ay, az] m/s²

# GPS M10N (10Hz)
@dataclass
class GPSSample:
    timestamp: float
    lat: float             # degrees
    lon: float             # degrees
    alt: float             # meters MSL
    hdop: float            # horizontal dilution
```

### L2 → L3: 感知结果

```python
# 稳定视频帧
@dataclass
class StabilizedFrame:
    timestamp: float
    frame: np.ndarray      # H×W×3
    transform: np.ndarray  # 3×3 homography

# SLAM位姿
@dataclass
class SLAMPose:
    timestamp: float
    position: np.ndarray   # [x, y, z] NED
    orientation: np.ndarray # [w, x, y, z] quaternion
    point_cloud: np.ndarray # N×3
```

### L3 → L4: 融合状态

```python
# EKF 12维状态
@dataclass
class FusedState:
    timestamp: float
    position: np.ndarray   # [x, y, z]
    velocity: np.ndarray   # [vx, vy, vz]
    orientation: np.ndarray # [roll, pitch, yaw]
    temperature: float     # °C (来自BME280)
    humidity: float        # %RH (来自BME280)
```

### L4 → L5: 认知决策

```python
# 因果推理安全预警
@dataclass
class SafetyAdvisory:
    timestamp: float
    risk_level: str        # LOW / MEDIUM / HIGH / CRITICAL
    causes: List[str]      # 因果链
    interventions: List[str] # do-intervention结果
    recommended_action: str  # CONTINUE / RETURN / LOWER_ALT / HOVER

# MoE专家选择
@dataclass
class ExpertDecision:
    task_type: str
    selected_experts: List[int]
    weights: np.ndarray
    confidence: float
```

---

## 性能预算

| 数据通路 | 延迟预算 | 带宽 | 优先级 |
|---------|---------|------|--------|
| IMU → EKF | <5ms | 4.8KB/s (200Hz) | 最高 |
| BME280 → Fusion | <10ms | 1.2KB/s (100Hz) | 高 |
| 摄像头 → Stitch | <33ms | 240MB/s (4×30fps) | 高 |
| Stitch → RTSP | <50ms | 50Mbps | 中 |
| SLAM位姿 → Planner | <20ms | 0.5KB/s (30Hz) | 高 |
| CausalEngine → Safety | <100ms | 事件触发 | 最高 |
| 语音 → 决策 | <2000ms | 16KB/s | 低 |
| 编队指令 → MAVLink | <50ms | 1KB/s (10Hz) | 高 |
