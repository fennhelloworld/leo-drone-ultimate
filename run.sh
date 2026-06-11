#!/bin/bash
# LeoDrone Ultimate — 一键运行脚本
# 仿真模式: 完整数据流演示 (传感器→融合→认知→协同→交互)
#
# 用法:
#   ./run.sh              # 仿真模式 (默认)
#   ./run.sh --sim        # SITL仿真
#   ./run.sh --test       # 运行测试
#   ./run.sh --demo       # 功能演示
#   ./run.sh --help       # 帮助信息

set -euo pipefail

# ============================================================
# 配置
# ============================================================
PROJECT_ROOT="/home/fenn/projects/leo-drone-ultimate"
DRONE_SYSTEM="/home/fenn/projects/drone-system"
OMNI_PERCEPTION="/home/fenn/projects/omni-perception-fusion"
PYTHON="/usr/bin/python3.10"

# 颜色
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
RESET='\033[0m'

# ============================================================
# 函数
# ============================================================

print_banner() {
    echo ""
    echo -e "${CYAN}${BOLD}╔══════════════════════════════════════════════════════════╗${RESET}"
    echo -e "${CYAN}${BOLD}║     🚁 LeoDrone Ultimate — 360° AI全栈智能无人机        ║${RESET}"
    echo -e "${CYAN}${BOLD}║     智能温湿度 · 全景拼接 · 8专利AI · 集群协同          ║${RESET}"
    echo -e "${CYAN}${BOLD}╚══════════════════════════════════════════════════════════╝${RESET}"
    echo ""
}

print_status() {
    echo -e "${BLUE}[INFO]${RESET} $1"
}

print_success() {
    echo -e "${GREEN}[OK]${RESET} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${RESET} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${RESET} $1"
}

check_dependencies() {
    print_status "检查依赖..."
    local missing=0

    # Python
    if command -v $PYTHON &> /dev/null; then
        print_success "Python: $($PYTHON --version 2>&1 | head -1)"
    else
        print_error "Python3.10 未找到"
        missing=1
    fi

    # NumPy
    if $PYTHON -c "import numpy" 2>/dev/null; then
        np_ver=$($PYTHON -c "import numpy; print(numpy.__version__)")
        print_success "NumPy: $np_ver"
    else
        print_warning "NumPy 未安装 (pip install numpy)"
        missing=1
    fi

    # 子项目
    if [ -d "$DRONE_SYSTEM" ]; then
        print_success "drone-system: $DRONE_SYSTEM"
    else
        print_warning "drone-system 路径不存在: $DRONE_SYSTEM"
    fi

    if [ -d "$OMNI_PERCEPTION" ]; then
        print_success "omni-perception-fusion: $OMNI_PERCEPTION"
    else
        print_warning "omni-perception-fusion 路径不存在: $OMNI_PERCEPTION"
    fi

    # 工具
    for tool in esptool openscad ngspice docker; do
        if command -v $tool &> /dev/null; then
            print_success "$tool: $(command -v $tool)"
        else
            print_warning "$tool: 未安装 (可选)"
        fi
    done

    return $missing
}

run_simulation() {
    print_status "=========================================="
    print_status "  启动仿真模式 — 完整数据流演示"
    print_status "=========================================="
    echo ""

    if [ ! -d "$OMNI_PERCEPTION" ]; then
        print_error "omni-perception-fusion 未找到: $OMNI_PERCEPTION"
        exit 1
    fi

    # 完整仿真: 7层架构全链路
    $PYTHON << 'PYEOF'
import sys
sys.path.insert(0, "/home/fenn/projects/omni-perception-fusion")
import numpy as np

BLUE = "\033[0;34m"
GREEN = "\033[0;32m"
CYAN = "\033[0;36m"
BOLD = "\033[1m"
RESET = "\033[0m"

def info(msg):
    print(f"{BLUE}[INFO]{RESET} {msg}")

def ok(msg):
    print(f"{GREEN}[OK]{RESET} {msg}")

np.random.seed(42)

# ========== Phase 1: L1 传感器数据 ==========
info("Phase 1: 传感器数据生成 (BME280 + ICM-42688-P)")

# BME280 温湿度 (100Hz)
n = 100
t = np.arange(n) * 0.01
temp = 25.0 + np.sin(t * 0.1) * 2 + np.random.randn(n) * 0.1
humid = 60.0 + np.sin(t * 0.05) * 5 + np.random.randn(n) * 0.5
press = 1013.25 + np.sin(t * 0.01) * 2
print(f"  温度: {temp.mean():.1f}°C (±{temp.std():.2f})")
print(f"  湿度: {humid.mean():.1f}%RH (±{humid.std():.2f})")
print(f"  气压: {press.mean():.1f}hPa (±{press.std():.2f})")

# ICM-42688-P IMU (200Hz)
n2 = 200
accel_z = np.ones(n2) * 9.81 + np.random.randn(n2) * 0.1
gyro_mean = np.abs(np.random.randn(n2, 3) * 0.01).mean()
print(f"  加速度Z: {accel_z.mean():.3f} m/s²")
print(f"  陀螺仪: {gyro_mean:.4f} rad/s")
print()

# ========== Phase 2: L2 视频稳定 ==========
info("Phase 2: 视频稳定 (VQF + EKF)")

from src.perception.video_stabilizer.stabilizer import VideoStabilizer, IMUSample

stabilizer = VideoStabilizer()
samples = [
    IMUSample(timestamp=i*0.01, gyro=np.array([0.01, 0.005, 0.001]), accel=np.array([0, 0, 9.81]))
    for i in range(100)
]
results = stabilizer.process_sequence(samples)
print(f"  稳定帧数: {len(results)}/{len(samples)}")
print(f"  稳定算法: VQF (Versatile Quaternion Filter)")
print()

# ========== Phase 3: L3 EKF 传感器融合 ==========
info("Phase 3: EKF 传感器融合 (12维状态)")

from src.fusion.sensor_fusion.ekf_fusion import ExtendedKalmanFilter

ekf = ExtendedKalmanFilter()
for i in range(100):
    accel = np.array([0, 0, 9.81]) + np.random.randn(3) * 0.01
    gyro = np.random.randn(3) * 0.001
    ekf.predict(dt=0.01)
    ekf.update_imu(accel, gyro)
    if i % 10 == 0:
        gps_pos = np.array([0, 0, 0]) + np.random.randn(3) * 0.5
        ekf.update_gps(gps_pos)

state = ekf.get_state()
pos = np.array(state['position'])
print(f"  EKF状态: position={state['position']}")
print(f"  位置误差: {np.linalg.norm(pos):.3f}m")
print()

# ========== Phase 4: L4 因果推理 ==========
info("Phase 4: 因果推理安全预警 (温湿度→飞行安全)")

from src.fusion.causal_engine.causal_graph import CausalGraph

graph = CausalGraph()
priors = graph.llm_prior("outdoor_motion", "temperature humidity flight safety")
for p in priors:
    graph.add_edge(p)
print(f"  因果先验数量: {len(priors)}")
for p in priors[:3]:
    print(f"    - {p}")
if len(priors) > 3:
    print(f"    ... 共{len(priors)}条")
print()

# ========== Phase 5: L5 UAV路径规划 ==========
info("Phase 5: UAV多机路径规划 (BSB-SSSP + RRT*)")

from src.coordination.uav_planner.planner import UAVMultiAgentPlanner, UAVState, Obstacle

planner = UAVMultiAgentPlanner(num_uavs=3)
states = [
    UAVState(position=np.array([0, i*5, 5], dtype=float),
             velocity=np.array([0, 0, 0], dtype=float), heading=0, battery=0.9)
    for i in range(3)
]
goals = [np.array([30, 30-i*5, 5], dtype=float) for i in range(3)]
obstacles = [Obstacle(position=np.array([15, 15, 5], dtype=float), radius=3.0, obs_type="STATIC")]

result = planner.plan_mission(states, goals, obstacles)
print(f"  规划结果: {'成功' if result else '失败'}")
print(f"  UAV数量: 3")
print(f"  障碍物: 1 (静态)")
if result and 'paths' in result:
    for i, path in enumerate(result['paths']):
        print(f"  UAV{i} 路径点数: {len(path)}")
print()

# ========== Phase 6: L5 MoE路由 ==========
info("Phase 6: MoE专家路由 (DeepSeekMoE)")

from src.coordination.multi_agent_scheduler.moe_router import DeepSeekMoERouter

router = DeepSeekMoERouter(d_model=64, num_experts=32, top_k=4)
x = np.random.randn(4, 64)
selected, weights = router.route(x)
print(f"  输入数量: {x.shape[0]}")
print(f"  专家总数: 32")
print(f"  Top-K选择: 4")
print()

# ========== Phase 7: L6 GOAT-Mamba ==========
info("Phase 7: GOAT-Mamba混合注意力")

from src.coordination.goat_attention.goat_mamba import GOATMambaHybrid, GOATConfig

config = GOATConfig(head_dim=32, num_heads=4, pos_rank=4, abs_rank=2)
model = GOATMambaHybrid(d_model=64, num_mamba_layers=1, goat_config=config)
x = np.random.randn(1, 32, 64).astype(np.float32)
output = model.forward(x, use_goat=True)
print(f"  输入: {x.shape}")
print(f"  注意力: GOAT (Fourier+Sink)")
print(f"  序列: Mamba2 SSM (O(N))")
print()

# ========== Phase 8: L6 语音交互 ==========
info("Phase 8: 边缘语音交互")

from src.edge.voice_box.voice_pipeline import EdgeVoiceBox

voice = EdgeVoiceBox()
print(f"  语音管线: 已初始化")
print()

# ========== 总结 ==========
ok("==========================================")
ok("  仿真流程完成！")
ok("==========================================")
print()
print(f"  {CYAN}{BOLD}7层架构数据流:{RESET}")
print(f"  L0 硬件 → L1 传感 → L2 感知 → L3 融合 → L4 认知 → L5 协同 → L6 交互")
print()
print(f"  {CYAN}下一步:{RESET}")
print("  make simulate   # 启动PX4 SITL仿真")
print("  make test       # 运行集成测试")
print("  make render     # 渲染3D模型")
print("  make circuit    # 生成电路网表")
print()
PYEOF
}

run_tests() {
    print_status "运行集成测试..."
    echo ""
    $PYTHON "$PROJECT_ROOT/tests/test_integration.py"
}

run_demo() {
    print_status "运行功能演示..."
    echo ""

    if [ ! -d "$OMNI_PERCEPTION" ]; then
        print_error "omni-perception-fusion 未找到: $OMNI_PERCEPTION"
        exit 1
    fi

    $PYTHON << 'PYEOF'
import sys
sys.path.insert(0, "/home/fenn/projects/omni-perception-fusion")
import numpy as np

np.random.seed(42)

print('=' * 60)
print('LeoDrone Ultimate — 功能演示')
print('=' * 60)
print()

# 1. 温湿度安全预警
print('【Case A 特色】温湿度→飞行安全预警')
print('-' * 40)

from src.fusion.causal_engine.causal_graph import CausalGraph
graph = CausalGraph()
priors = graph.llm_prior('outdoor_motion', 'temperature humidity flight')
for p in priors[:5]:
    print(f'  因果先验: {p}')
print()

# 2. 视频稳定
print('【专利1】视频稳定 VQF+EKF')
print('-' * 40)
from src.perception.video_stabilizer.stabilizer import VideoStabilizer, IMUSample
stab = VideoStabilizer()
samples = [IMUSample(timestamp=i*0.01, gyro=np.array([0.01,0,0]), accel=np.array([0,0,9.81])) for i in range(50)]
res = stab.process_sequence(samples)
print(f'  处理帧数: {len(res)}/50')
print()

# 3. 多机规划
print('【专利2】UAV多机路径规划')
print('-' * 40)
from src.coordination.uav_planner.planner import UAVMultiAgentPlanner, UAVState, Obstacle
planner = UAVMultiAgentPlanner(num_uavs=3)
states = [UAVState(position=np.array([0,i*5,5], dtype=float), velocity=np.array([0,0,0], dtype=float), heading=0, battery=0.9) for i in range(3)]
goals = [np.array([30,30-i*5,5], dtype=float) for i in range(3)]
obs = [Obstacle(position=np.array([15,15,5], dtype=float), radius=3.0, obs_type='STATIC')]
result = planner.plan_mission(states, goals, obs)
print(f'  3机路径规划: {"✅ 成功" if result else "❌ 失败"}')
print()

# 4. MoE路由
print('【专利8】DeepSeekMoE专家路由')
print('-' * 40)
from src.coordination.multi_agent_scheduler.moe_router import DeepSeekMoERouter
router = DeepSeekMoERouter(d_model=64, num_experts=32, top_k=4)
x = np.random.randn(4, 64)
experts, weights = router.route(x)
print(f'  专家选择: top-4 from 32 experts')
print()

# 5. EKF融合
print('【核心】EKF传感器融合')
print('-' * 40)
from src.fusion.sensor_fusion.ekf_fusion import ExtendedKalmanFilter
ekf = ExtendedKalmanFilter()
for i in range(50):
    ekf.predict(dt=0.01)
    ekf.update_imu(np.array([0,0,9.81]), np.array([0,0,0]))
state = ekf.get_state()
print(f'  融合状态: position={state["position"]}')
print()

print('=' * 60)
print('✅ 演示完成！')
print('=' * 60)
PYEOF
}

run_sitl() {
    print_status "启动PX4 SITL仿真环境..."
    if [ -d "$DRONE_SYSTEM" ]; then
        cd "$DRONE_SYSTEM"
        docker compose up -d px4-sitl gazebo mavros 2>/dev/null || {
            print_error "Docker启动失败"
            print_warning "请确保Docker已安装且运行: sudo systemctl start docker"
            exit 1
        }
        print_success "SITL仿真已启动"
        echo "  Gazebo:    http://localhost:11345"
        echo "  MAVLink:   udp://localhost:14540"
        echo "  QGC:       连接 UDP 14550"
    else
        print_error "drone-system 未找到: $DRONE_SYSTEM"
        exit 1
    fi
}

# ============================================================
# 主逻辑
# ============================================================

print_banner

# 解析参数
MODE="${1:---sim}"

case "$MODE" in
    --sim|--simulation)
        check_dependencies
        echo ""
        run_simulation
        ;;
    --test)
        run_tests
        ;;
    --demo)
        check_dependencies
        echo ""
        run_demo
        ;;
    --sitl)
        run_sitl
        ;;
    --help|-h)
        echo "用法: ./run.sh [选项]"
        echo ""
        echo "选项:"
        echo "  --sim         仿真模式 (默认) — 完整数据流演示"
        echo "  --test        运行集成测试"
        echo "  --demo        功能演示"
        echo "  --sitl        启动PX4 SITL仿真"
        echo "  --help        显示帮助"
        echo ""
        echo "Makefile 目标:"
        echo "  make simulate    运行SITL仿真"
        echo "  make flash       烧录ESP32-S3固件"
        echo "  make render      渲染3D STL文件"
        echo "  make test        运行所有测试"
        echo "  make circuit     生成SPICE网表"
        echo "  make all         完整构建管线"
        echo "  make clean       清理"
        ;;
    *)
        print_error "未知选项: $MODE"
        echo "使用 --help 查看帮助"
        exit 1
        ;;
esac
