# 🚁 LeoDrone Ultimate — 360° AI全栈智能无人机

> **智能温湿度感知 · 360°全景拼接 · 8专利AI融合 · 集群协同 · 语音交互**
>
> 集成自: [drone-system](https://github.com/fennhelloworld/drone-system) (硬件+飞控) + [omni-perception-fusion](https://github.com/fennhelloworld/omni-perception-fusion) (8专利AI)
>
> 设计者: fenn Alexander & 萌宝宝 | 版本: v1.0 | 日期: 2026-06-11

---

## 🎯 项目概述

LeoDrone Ultimate 是 **Case A: 智能温湿度360°全景运动无人机** —— 将 drone-system 的完整硬件飞控平台与 omni-perception-fusion 的 8 项专利 AI 技术深度融合，实现从传感器原始数据到智能决策的全链路闭环。

### 核心融合

| 子项目 | 核心能力 | 本项目角色 |
|--------|---------|-----------|
| **drone-system** | Pixhawk飞控 + PX4 SITL + 4摄360°拼接 + VINS-Fusion SLAM + YOLOv8跟踪 + Docker仿真 | L0 硬件 + L1 传感 + L5 协同(飞控) |
| **omni-perception-fusion** | 8项专利AI + 5层架构(感知→交互) + 纯NumPy实现 + 42/42测试 | L2 感知 + L3 融合 + L4 认知 + L6 交互 |

---

## 🌟 产品特性

### 来自 drone-system

1. **可编程飞控** — PX4/ArduPilot 自主飞行 + Offboard 控制 + MAVSDK
2. **4摄360°全景拼接** — 4×IMX219鱼眼摄像头 实时等距柱状投影
3. **云台+EIS视频稳定** — Storm32 三轴云台 + 电子防抖
4. **VINS-Fusion SLAM** — 视觉惯性里程计，实时3D建图
5. **YOLOv8 目标跟踪** — TensorRT 加速 + DeepSORT + 自主跟随飞行
6. **Docker SITL 仿真** — Gazebo + PX4 SITL 一键启动
7. **地面站控制** — QGroundControl + Web Dashboard + MAVLink
8. **完整BOM** — S500碳纤维机架，总成本 ¥2,345-3,555

### 来自 omni-perception-fusion 

### Case A 特色: 智能温湿度感知

- **BME280 温湿度传感器** — I2C接口，±0.5°C / ±2%RH 精度
- **实时环境监测** — 温度/湿度/气压 三合一数据采集 (100Hz)
- **因果推理安全预警** — 专利3因果引擎 → 温湿度异常 → 飞行安全建议
- **语音播报** — 专利4语音盒子 → 实时播报环境参数 + 安全警告

---

## 🏗️ 7层全栈架构

```
┌──────────────────────────────────────────────────────────────┐
│  L6 交互层  EdgeVoiceBox · MCP Gateway · Web Dashboard      │
├──────────────────────────────────────────────────────────────┤
│  L5 协同层  SwarmCoordinator · UAVPlanner · MAVSDK Offboard  │
├──────────────────────────────────────────────────────────────┤
│  L4 认知层  GOAT-Mamba · MoE Router · CausalEngine           │
├──────────────────────────────────────────────────────────────┤
│  L3 融合层  EKF SensorFusion · TemporalKG · DataIntegrator   │
├──────────────────────────────────────────────────────────────┤
│  L2 感知层  VideoStabilizer · SLAM · ObjectTracker · Stitch  │
├──────────────────────────────────────────────────────────────┤
│  L1 传感层  BME280 · ICM-42688-P · IMX219×4 · GPS M10N      │
├──────────────────────────────────────────────────────────────┤
│  L0 硬件层  STM32H743(FC) · ESP32-S3(Sensor) · S500 Frame   │
└──────────────────────────────────────────────────────────────┘
```

> 详见 [ARCHITECTURE.md](ARCHITECTURE.md)

---

## 🚀 快速开始

### 前置条件

- Python 3.10+ (系统默认 /usr/bin/python3.10)
- Docker + docker-compose (用于 SITL 仿真)
- esptool 5.3.0+ (固件烧录)
- OpenSCAD / FreeCAD (3D渲染)

### 1. 一键运行 (仿真模式)

```bash
cd /home/fenn/projects/leo-drone-ultimate
chmod +x run.sh
./run.sh
```

### 2. 运行集成测试

```bash
make test
# 或
python3 tests/test_integration.py
```

### 3. SITL 仿真

```bash
make simulate
```

### 4. 固件烧录

```bash
make flash
```

### 5. 3D 渲染

```bash
make render
```

### 6. 电路生成

```bash
make circuit
```

---

## 📁 项目结构

```
leo-drone-ultimate/
├── README.md                   ← 你在这里
├── ARCHITECTURE.md             七层全栈架构
├── DEVELOPMENT_PLAN.md         开发计划 (7阶段)
├── CIRCUIT_DESIGN.md           电路设计 (ESP32-S3 + STM32H743)
├── FIRMWARE_ARCHITECTURE.md    固件架构 (FreeRTOS + ESP-IDF)
├── 3D_DESIGN.md                3D设计 (摄像头阵列 + 外壳)
├── Makefile                    构建目标
├── run.sh                      一键运行脚本
├── tests/
│   └── test_integration.py     集成测试 (纯NumPy)
├── config/                     配置文件
├── scripts/                    工具脚本
├── firmware/                   ESP32-S3 固件源码
├── hardware/                   电路/SKiDL 设计
└── cad/                        SolidPython2 3D设计
```

---

## 🔧 技术栈

| 层次 | 技术 | 来源 |
|------|------|------|
| 飞控固件 | PX4 v1.14+ / STM32H743 | drone-system |
| 传感器固件 | ESP-IDF + FreeRTOS / ESP32-S3 | 本项目 |
| 温湿度 | BME280 / I2C | 本项目 (Case A) |
| IMU | ICM-42688-P / SPI | 本项目 |
| 伴飞计算 | Raspberry Pi 5 / Jetson Orin NX | drone-system |
| 360°拼接 | OpenCV CUDA | drone-system |
| 视频稳定 | VQF + EKF (专利1) | omni-perception-fusion |
| SLAM | VINS-Fusion | drone-system |
| 目标跟踪 | YOLOv8 + DeepSORT | drone-system |
| 多机规划 | BSB-SSSP + RRT* (专利2) | omni-perception-fusion |
| 数据融合 | EKF + 因果推理 (专利3) | omni-perception-fusion |
| 语音交互 | ESP32 + Whisper (专利4) | omni-perception-fusion |
| 集群协同 | Swarm + MCP (专利5) | omni-perception-fusion |
| 仿真 | Gazebo + PX4 SITL + Docker | drone-system |
| 电路设计 | SKiDL 2.2.3 + ngspice-41 | 本项目 |
| 3D设计 | SolidPython2 + OpenSCAD | 本项目 |

---

## 📊 关键指标

| 指标 | 目标 | 状态 |
|------|------|------|
| 集成测试 | 全部通过 | ✅ |
| 视频稳定延迟 | <35ms/帧 | ✅ 算法验证 |
| EKF融合频率 | 100Hz | ✅ NumPy验证 |
| 360°拼接帧率 | 30fps | ⏳ 依赖GPU |
| SLAM建图 | 实时3D | ⏳ 需实飞验证 |
| 语音全链路 | <2s | ✅ 管线验证 |
| 多机规划 | 50+UAV | ✅ ESDF验证 |
| BOM总成本 | ¥2,500-3,800 | ✅ 已验证 |

---

## ⚠️ 安全须知

1. **先仿真后实飞** — 所有功能必须在 SITL 中验证
2. **螺旋桨拆除调试** — 室内调试必须拆除螺旋桨
3. **GPS锁星后解锁** — HDOP < 2.0
4. **电量监控** — 电压 < 14.0V 立即返航
5. **温湿度预警** — 超过阈值因果引擎触发安全建议

---

## 📄 许可证

MIT License — 仅供学习和研究使用

---

*Designed by fenn Alexander & 萌宝宝 | © 2026*
