# LeoDrone Ultimate — 3D设计

> 摄像头安装支架 · 传感器外壳 · 无人机集成

---

## 设计总览

```
┌────────────────────────────────────────────────────┐
│                 S500 碳纤维机架                      │
│                 (450mm轴距)                          │
│                                                     │
│    ┌─────┐                           ┌─────┐      │
│    │CAM0 │        ┌────────┐         │CAM3 │      │
│    │ 0°  │        │  RPi5  │         │270° │      │
│    └──┬──┘        │ 伴飞   │         └──┬──┘      │
│       │           │ 计算机 │            │          │
│  ┌────┴───────────┴────────┴───────────┴───┐      │
│  │         摄像头安装支架 (3D打印)          │      │
│  │         4×IMX219 90°间隔安装            │      │
│  └──────────────────┬──────────────────────┘      │
│                     │                              │
│              ┌──────┴──────┐                      │
│              │  Pixhawk 6C │                      │
│              │  飞控主板    │                      │
│              └─────────────┘                      │
│                                                     │
│    ┌─────┐                           ┌─────┐      │
│    │CAM1 │        ┌────────┐         │CAM2 │      │
│    │ 90° │        │ESP32-S3│         │180° │      │
│    └─────┘        │传感器板│         └─────┘      │
│                   │+BME280 │                       │
│                   │+IMU    │                       │
│                   └────────┘                       │
└────────────────────────────────────────────────────┘
```

---

## 摄像头安装支架 (4×IMX219 阵列)

### 设计参数

| 参数 | 规格 |
|------|------|
| 摄像头数量 | 4 |
| 摄像头型号 | Arducam IMX219 鱼眼160° |
| 安装间隔 | 90° (0°, 90°, 180°, 270°) |
| 安装半径 | 60mm (中心到摄像头光心) |
| 水平FOV | 160° (鱼眼) |
| 重叠区域 | ≥ 20° (相邻摄像头) |
| 360°覆盖 | 4 × 160° - 3 × 20° = 580° → 冗余覆盖 |
| 材质 | PETG (3D打印) |
| 重量 | < 50g |

### 安装布局

```
              前方 (0°)
                │
           ┌────┴────┐
           │  CAM0   │
           │ IMX219  │
           └────┬────┘
                │
    ┌────┐      │      ┌────┐
    │CAM3│──────┼──────│CAM1│
    │270°│      │      │ 90°│
    └────┘      │      └────┘
                │
           ┌────┴────┐
           │  CAM2   │
           │ IMX219  │
           └────┬────┘
                │
              后方 (180°)

    侧视图:
         ┌───┐
    ─────│CAM│─────  ← 向外倾斜5° (增加下方覆盖)
         └─┬─┘
           │
    ═══════╪═══════  ← 支架环
           │
         ┌─┴─┐
    ─────│CAM│─────
         └───┘
```

### SolidPython2 代码

```python
"""
LeoDrone Ultimate - Camera Mount Bracket
4×IMX219 鱼眼摄像头阵列, 90°间隔安装

依赖: SolidPython2, OpenSCAD
"""

from build123d import *
from ocp_vscode import *
import math

# ============================================================
# 参数定义
# ============================================================

# 支架参数
MOUNT_RADIUS = 60.0       # 安装半径 (中心到光心)
MOUNT_HEIGHT = 8.0        # 支架厚度
RING_INNER_R = 30.0       # 内环半径
RING_OUTER_R = 75.0       # 外环半径
NUM_CAMERAS = 4           # 摄像头数量
CAM_TILT_ANGLE = 5.0      # 向外倾斜角度 (度)

# IMX219 摄像头模块尺寸
CAM_WIDTH = 25.0          # 模块宽度
CAM_HEIGHT = 24.0         # 模块高度
CAM_DEPTH = 9.0           # 模块深度 (含镜头)
LENS_DIAMETER = 12.0      # 镜头直径
SCREW_HOLE_R = 1.0        # M2螺丝孔半径
SCREW_SPACING = 21.0      # 螺丝孔间距

# 无人机安装
FRAME_MOUNT_HOLE_R = 1.5  # M3螺丝孔半径
FRAME_MOUNT_SPACING = 45.0 # 安装孔间距

# ============================================================
# 摄像头安装座
# ============================================================

def camera_mount(angle_deg):
    """生成单个摄像头安装座"""
    angle = math.radians(angle_deg)
    tilt = math.radians(CAM_TILT_ANGLE)

    # 安装座位置
    x = MOUNT_RADIUS * math.cos(angle)
    y = MOUNT_RADIUS * math.sin(angle)

    # 基座 (固定到支架环上)
    base = Box(CAM_WIDTH + 4, CAM_DEPTH + 2, MOUNT_HEIGHT)
    base = Pos(x, y, MOUNT_HEIGHT / 2) * Rot(0, 0, angle_deg) * base

    # 摄像头槽位
    slot = Box(CAM_WIDTH, CAM_DEPTH, CAM_HEIGHT)
    slot = Pos(x, y, MOUNT_HEIGHT + CAM_HEIGHT / 2) * Rot(0, 0, angle_deg) * slot
    base = base - slot

    # 镜头孔
    lens_hole = Cylinder(radius=LENS_DIAMETER / 2 + 0.5, height=CAM_DEPTH + 4)
    lens_hole = Pos(x, y + CAM_DEPTH / 2, MOUNT_HEIGHT + CAM_HEIGHT / 2) * \
                Rot(0, 0, angle_deg) * lens_hole
    base = base - lens_hole

    # M2螺丝孔 ×2
    for dx in [-SCREW_SPACING / 2, SCREW_SPACING / 2]:
        screw = Cylinder(radius=SCREW_HOLE_R, height=MOUNT_HEIGHT + 2)
        screw = Pos(x + dx * math.cos(angle), y + dx * math.sin(angle), 0) * screw
        base = base - screw

    return base

# ============================================================
# 主体支架环
# ============================================================

def camera_array_mount():
    """生成4摄像头安装支架"""

    # 中心环
    ring = Cylinder(radius=RING_OUTER_R, height=MOUNT_HEIGHT) - \
           Cylinder(radius=RING_INNER_R, height=MOUNT_HEIGHT + 2)

    # 安装孔 (连接到机架)
    for i in range(4):
        angle = i * 90 + 45
        a_rad = math.radians(angle)
        x = (RING_INNER_R + 10) * math.cos(a_rad)
        y = (RING_INNER_R + 10) * math.sin(a_rad)
        hole = Cylinder(radius=FRAME_MOUNT_HOLE_R, height=MOUNT_HEIGHT + 2)
        ring = ring - Pos(x, y, 0) * hole

    # 线缆通道 (内环4个开口)
    for i in range(4):
        angle = i * 90
        a_rad = math.radians(angle)
        channel = Box(8, 15, MOUNT_HEIGHT + 2)
        channel = Pos(RING_INNER_R * math.cos(a_rad),
                      RING_INNER_R * math.sin(a_rad), 0) * \
                  Rot(0, 0, angle) * channel
        ring = ring - channel

    # 添加4个摄像头安装座
    for i in range(NUM_CAMERAS):
        angle = i * 360 / NUM_CAMERAS
        mount = camera_mount(angle)
        ring = ring + mount

    return ring

# ============================================================
# 导出
# ============================================================

if __name__ == "__main__":
    model = camera_array_mount()

    # 导出 STL
    export_stl(model, "cad/camera_array_mount.stl")

    # 导出 STEP (用于FreeCAD)
    export_step(model, "cad/camera_array_mount.step")

    print("✅ 摄像头支架模型已生成:")
    print("   cad/camera_array_mount.stl")
    print("   cad/camera_array_mount.step")
```

---

## 传感器外壳 (ESP32-S3 + BME280 + ICM-42688-P)

### 设计参数

| 参数 | 规格 |
|------|------|
| 外壳尺寸 | 50mm × 40mm × 20mm |
| PCB尺寸 | 40mm × 35mm (4层) |
| 材质 | PETG (3D打印) |
| 防护等级 | IP54 (通风设计) |
| 重量 | < 30g (不含PCB) |
| 安装 | M2.5螺丝 ×4 |

### SolidPython2 代码

```python
"""
LeoDrone Ultimate - Sensor Node Enclosure
ESP32-S3 + BME280 + ICM-42688-P 传感器外壳

依赖: SolidPython2, OpenSCAD
"""

from build123d import *
from ocp_vscode import *

# ============================================================
# 参数定义
# ============================================================

CASE_LENGTH = 50.0
CASE_WIDTH = 40.0
CASE_HEIGHT = 20.0
WALL_THICKNESS = 1.5
CORNER_RADIUS = 3.0

PCB_LENGTH = 40.0
PCB_WIDTH = 35.0
PCB_OFFSET_X = 5.0
PCB_OFFSET_Y = 2.5

# BME280 通风孔 (温度测量需要空气流通)
VENT_HOLE_R = 2.0
VENT_ROWS = 3
VENT_COLS = 4

# ============================================================
# 外壳底座
# ============================================================

def sensor_enclosure_bottom():
    """外壳底座"""

    # 主体
    outer = RoundedBox(CASE_LENGTH, CASE_WIDTH, CASE_HEIGHT / 2,
                       radius=CORNER_RADIUS)
    inner = RoundedBox(CASE_LENGTH - 2 * WALL_THICKNESS,
                       CASE_WIDTH - 2 * WALL_THICKNESS,
                       CASE_HEIGHT / 2 - WALL_THICKNESS,
                       radius=CORNER_RADIUS - WALL_THICKNESS)
    body = outer - Pos(WALL_THICKNESS, WALL_THICKNESS, WALL_THICKNESS) * inner

    # PCB 支撑柱 ×4
    pillar_h = 3.0
    for x in [6, CASE_LENGTH - 6]:
        for y in [6, CASE_WIDTH - 6]:
            pillar = Cylinder(radius=2.5, height=pillar_h)
            screw_hole = Cylinder(radius=1.25, height=pillar_h + 1)  # M2.5
            body = body + Pos(x, y, WALL_THICKNESS) * (pillar - screw_hole)

    # BME280 通风孔阵列 (底部)
    for row in range(VENT_ROWS):
        for col in range(VENT_COLS):
            x = 10 + col * 8
            y = 8 + row * 8
            vent = Cylinder(radius=VENT_HOLE_R, height=WALL_THICKNESS + 1)
            body = body - Pos(x, y, 0) * vent

    # USB-C 开口
    usb_cutout = Box(9, WALL_THICKNESS + 2, 3.5)
    body = body - Pos(CASE_LENGTH / 2 - 4.5, CASE_WIDTH - WALL_THICKNESS, 5) * usb_cutout

    # 安装耳 ×2
    for x in [-5, CASE_LENGTH + 5]:
        ear = Box(10, 8, 3)
        m3_hole = Cylinder(radius=1.5, height=4)
        body = body + Pos(x - 5, CASE_WIDTH / 2 - 4, 0) * (ear - Pos(5, 4, -0.5) * m3_hole)

    return body

# ============================================================
# 外壳顶盖
# ============================================================

def sensor_enclosure_top():
    """外壳顶盖"""

    top_h = CASE_HEIGHT / 2
    outer = RoundedBox(CASE_LENGTH, CASE_WIDTH, top_h, radius=CORNER_RADIUS)
    inner = RoundedBox(CASE_LENGTH - 2 * WALL_THICKNESS,
                       CASE_WIDTH - 2 * WALL_THICKNESS,
                       top_h - WALL_THICKNESS,
                       radius=CORNER_RADIUS - WALL_THICKNESS)
    body = outer - Pos(WALL_THICKNESS, WALL_THICKNESS, 0) * inner

    # ESP32-S3 天线区域 (顶部留薄壁)
    antenna_area = Box(20, CASE_WIDTH - 6, WALL_THICKNESS)
    body = body - Pos(CASE_LENGTH - 22, 3, top_h - WALL_THICKNESS - 0.3) * \
           Box(20, CASE_WIDTH - 6, WALL_THICKNESS - 0.5)

    # LED 窗口
    led_window = Cylinder(radius=1.5, height=WALL_THICKNESS + 1)
    body = body - Pos(8, CASE_WIDTH - 8, top_h - WALL_THICKNESS) * led_window

    return body

# ============================================================
# 导出
# ============================================================

if __name__ == "__main__":
    bottom = sensor_enclosure_bottom()
    top = sensor_enclosure_top()

    export_stl(bottom, "cad/sensor_enclosure_bottom.stl")
    export_stl(top, "cad/sensor_enclosure_top.stl")
    export_step(bottom, "cad/sensor_enclosure_bottom.step")
    export_step(top, "cad/sensor_enclosure_top.step")

    print("✅ 传感器外壳模型已生成:")
    print("   cad/sensor_enclosure_bottom.stl")
    print("   cad/sensor_enclosure_top.stl")
```

---

## 无人机机架集成

### S500 安装布局

```
顶视图:

    M1(前左)          M2(前右)
       ╲                ╱
        ╲   ┌──────┐  ╱
         ╲  │RPi5  │ ╱
    ──────CAM0──CAM1────── ← 摄像头支架
         ╱  │Pixhaw│ ╲
        ╱   │  FC  │  ╲
       ╱    └──────┘   ╲
    M3(后左)          M4(后右)

侧视图:

    ┌─顶层────────────────────┐
    │  摄像头支架 (CAM0~3)    │
    ├─第二层───────────────────┤
    │  RPi5 + 电池绑带        │
    ├─第三层───────────────────┤
    │  Pixhawk 6C + GPS支柱   │
    ├─底层─────────────────────┤
    │  ESP32-S3传感器板(底面)  │
    │  BME280朝下(通风)       │
    └─────────────────────────┘
```

### 重量分布

| 组件 | 重量 (g) | 层位 |
|------|---------|------|
| S500 机架 | 180 | 结构 |
| 4× 电机 2212 | 200 | 机臂 |
| 4× 电调 30A | 40 | 机臂 |
| Pixhawk 6C | 18 | 三层 |
| GPS M10N | 12 | 顶层支柱 |
| RPi5 4GB | 40 | 二层 |
| 4× IMX219 | 24 | 顶层支架 |
| ESP32-S3 传感器板 | 15 | 底层 |
| 4S 3000mAh 电池 | 280 | 二层(中心) |
| 摄像头支架(PETG) | 45 | 顶层 |
| 传感器外壳(PETG) | 25 | 底层 |
| 线材+连接器 | 30 | — |
| **总计** | **~909g** | |
| **最大起飞重量** | **~2000g** | |
| **负载余量** | **~1091g** | 54% |

---

## 渲染命令

```bash
# 使用 OpenSCAD 渲染
openscad -o cad/camera_array_mount.stl cad/camera_array_mount.scad
openscad -o cad/sensor_enclosure_bottom.stl cad/sensor_enclosure_bottom.scad
openscad -o cad/sensor_enclosure_top.stl cad/sensor_enclosure_top.scad

# 使用 FreeCAD 渲染 (Python脚本)
freecad-python cad/render_all.py

# 或使用 Makefile
make render
```

---

## 打印设置

| 参数 | 摄像头支架 | 传感器外壳 |
|------|-----------|-----------|
| 材质 | PETG | PETG |
| 喷嘴 | 0.4mm | 0.4mm |
| 层高 | 0.2mm | 0.2mm |
| 填充 | 30% Gyroid | 20% Gyroid |
| 壁厚 | 3 walls (1.2mm) | 2 walls (0.8mm) |
| 支撑 | 是 (摄像头座) | 是 (通风孔) |
| 温度 | 240°C / 80°C (床) | 240°C / 80°C |
| 打印时间 | ~3h | ~1.5h |
