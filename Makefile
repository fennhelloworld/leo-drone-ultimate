# LeoDrone Ultimate — Makefile
# 一键构建/仿真/测试/烧录/渲染

.PHONY: all simulate flash render test circuit clean help setup \
        sim-sitl sim-all test-integration test-perception test-fusion \
        test-coordination ota render-camera render-enclosure render-all

# 默认目标
all: test circuit render ## 完整构建管线 (测试→电路→3D)

# ============================================================
# 颜色输出
# ============================================================
BLUE  := \033[0;34m
GREEN := \033[0;32m
YELLOW:= \033[0;33m
RED   := \033[0;31m
RESET := \033[0m

# ============================================================
# 路径配置
# ============================================================
DRONE_SYSTEM    := /home/fenn/projects/drone-system
OMNI_PERCEPTION := /home/fenn/projects/omni-perception-fusion
PROJECT_ROOT    := /home/fenn/projects/leo-drone-ultimate
CAD_DIR         := $(PROJECT_ROOT)/cad
HARDWARE_DIR    := $(PROJECT_ROOT)/hardware
FIRMWARE_DIR    := $(PROJECT_ROOT)/firmware
TEST_DIR        := $(PROJECT_ROOT)/tests

# 工具
PYTHON          := /usr/bin/python3.10
ESPTOOL         := esptool
OPENSCAD        := openscad
NGSPICE         := ngspice
PIO             := pio

# ============================================================
# 帮助信息
# ============================================================
help: ## 显示帮助信息
	@echo "$(BLUE)LeoDrone Ultimate — 360° AI全栈智能无人机$(RESET)"
	@echo ""
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  $(GREEN)%-20s$(RESET) %s\n", $$1, $$2}'

# ============================================================
# 安装依赖
# ============================================================
setup: ## 安装所有依赖
	@echo "$(BLUE)[SETUP] 安装Python依赖...$(RESET)"
	pip install numpy scipy
	@echo "$(BLUE)[SETUP] 检查工具链...$(RESET)"
	@which $(ESPTOOL) > /dev/null 2>&1 && echo "$(GREEN)  esptool: OK$(RESET)" || echo "$(YELLOW)  esptool: 未安装$(RESET)"
	@which $(OPENSCAD) > /dev/null 2>&1 && echo "$(GREEN)  OpenSCAD: OK$(RESET)" || echo "$(YELLOW)  OpenSCAD: 未安装$(RESET)"
	@which $(NGSPICE) > /dev/null 2>&1 && echo "$(GREEN)  ngspice: OK$(RESET)" || echo "$(YELLOW)  ngspice: 未安装$(RESET)"
	@which $(PIO) > /dev/null 2>&1 && echo "$(GREEN)  PlatformIO: OK$(RESET)" || echo "$(YELLOW)  PlatformIO: 未安装$(RESET)"
	@echo "$(GREEN)[SETUP] 完成$(RESET)"

# ============================================================
# 仿真 (SITL)
# ============================================================
simulate: sim-sitl ## 运行SITL仿真

sim-sitl: ## 启动PX4 SITL仿真环境
	@echo "$(BLUE)[SIM] 启动PX4 SITL仿真...$(RESET)"
	cd $(DRONE_SYSTEM) && docker compose up -d px4-sitl gazebo mavros
	@echo "$(GREEN)[SIM] 仿真已启动$(RESET)"
	@echo "  Gazebo:    http://localhost:11345"
	@echo "  MAVLink:   udp://localhost:14540"
	@echo "  QGC:       连接 UDP 14550"

sim-all: ## 启动完整仿真 (含AI节点)
	@echo "$(BLUE)[SIM-ALL] 启动完整仿真...$(RESET)"
	cd $(DRONE_SYSTEM) && docker compose up -d
	@echo "$(GREEN)[SIM-ALL] 全部服务已启动$(RESET)"
	@echo "  Web:       http://localhost:8080"
	@echo "  RTSP:      rtsp://localhost:8554/live"

# ============================================================
# 固件烧录
# ============================================================
flash: ## 烧录ESP32-S3固件
	@echo "$(BLUE)[FLASH] 检测ESP32-S3...$(RESET)"
	@$(ESPTOOL) chip_id 2>/dev/null || (echo "$(RED)未检测到ESP32-S3$(RESET)" && exit 1)
	@echo "$(BLUE)[FLASH] 烧录固件...$(RESET)"
	$(ESPTOOL) --chip esp32s3 --port /dev/ttyUSB0 --baud 921600 \
		write_flash -z 0x10000 $(FIRMWARE_DIR)/firmware.bin
	@echo "$(GREEN)[FLASH] 烧录完成$(RESET)"

flash-px4: ## 烧录PX4固件到Pixhawk
	@echo "$(BLUE)[FLASH-PX4] 烧录PX4固件...$(RESET)"
	cd $(DRONE_SYSTEM) && ./scripts/flash_px4.sh
	@echo "$(GREEN)[FLASH-PX4] 完成$(RESET)"

ota: ## OTA无线更新ESP32-S3
	@echo "$(BLUE)[OTA] 发送OTA更新...$(RESET)"
	$(PYTHON) scripts/ota_update.py --host 192.168.4.1 --firmware $(FIRMWARE_DIR)/firmware.bin
	@echo "$(GREEN)[OTA] 更新已发送$(RESET)"

# ============================================================
# 3D渲染
# ============================================================
render: render-all ## 渲染所有3D STL文件

render-camera: ## 渲染摄像头支架STL
	@echo "$(BLUE)[RENDER] 渲染摄像头支架...$(RESET)"
	@mkdir -p $(CAD_DIR)
	$(PYTHON) cad/camera_array_mount.py || \
	$(OPENSCAD) -o $(CAD_DIR)/camera_array_mount.stl cad/camera_array_mount.scad
	@echo "$(GREEN)[RENDER] cad/camera_array_mount.stl$(RESET)"

render-enclosure: ## 渲染传感器外壳STL
	@echo "$(BLUE)[RENDER] 渲染传感器外壳...$(RESET)"
	@mkdir -p $(CAD_DIR)
	$(PYTHON) cad/sensor_enclosure.py || \
	( $(OPENSCAD) -o $(CAD_DIR)/sensor_enclosure_bottom.stl cad/sensor_enclosure_bottom.scad && \
	  $(OPENSCAD) -o $(CAD_DIR)/sensor_enclosure_top.stl cad/sensor_enclosure_top.scad )
	@echo "$(GREEN)[RENDER] cad/sensor_enclosure_*.stl$(RESET)"

render-all: render-camera render-enclosure ## 渲染全部3D模型
	@echo "$(GREEN)[RENDER] 所有3D模型已生成$(RESET)"

# ============================================================
# 测试
# ============================================================
test: test-integration ## 运行所有测试

test-integration: ## 运行集成测试
	@echo "$(BLUE)[TEST] 运行集成测试...$(RESET)"
	$(PYTHON) $(TEST_DIR)/test_integration.py -v
	@echo "$(GREEN)[TEST] 集成测试完成$(RESET)"

test-perception: ## 测试感知模块
	@echo "$(BLUE)[TEST] 测试感知模块...$(RESET)"
	cd $(OMNI_PERCEPTION) && $(PYTHON) tests/test_all.py
	@echo "$(GREEN)[TEST] 感知测试完成$(RESET)"

test-fusion: ## 测试融合模块
	@echo "$(BLUE)[TEST] 测试EKF融合+因果推理...$(RESET)"
	$(PYTHON) -c "from tests.test_integration import *; test_ekf_sensor_fusion(); test_causal_safety()"
	@echo "$(GREEN)[TEST] 融合测试完成$(RESET)"

test-coordination: ## 测试协同模块
	@echo "$(BLUE)[TEST] 测试UAV规划+编队...$(RESET)"
	$(PYTHON) -c "from tests.test_integration import *; test_uav_path_planning(); test_moe_routing()"
	@echo "$(GREEN)[TEST] 协同测试完成$(RESET)"

# ============================================================
# 电路设计
# ============================================================
circuit: ## 生成SPICE网表
	@echo "$(BLUE)[CIRCUIT] 生成电路网表...$(RESET)"
	@mkdir -p $(HARDWARE_DIR)
	cd $(HARDWARE_DIR) && $(PYTHON) sensor_node_circuit.py
	@echo "$(BLUE)[CIRCUIT] 运行SPICE仿真...$(RESET)"
	$(NGSPICE) -b $(HARDWARE_DIR)/power_supply.sp 2>/dev/null || true
	$(NGSPICE) -b $(HARDWARE_DIR)/i2c_pullup.sp 2>/dev/null || true
	@echo "$(GREEN)[CIRCUIT] 网表和仿真完成$(RESET)"

# ============================================================
# 清理
# ============================================================
clean: ## 清理所有构建产物
	@echo "$(BLUE)[CLEAN] 清理...$(RESET)"
	rm -rf __pycache__ .pytest_cache *.pyc
	rm -rf $(CAD_DIR)/*.stl $(CAD_DIR)/*.step
	rm -rf $(HARDWARE_DIR)/*.net $(HARDWARE_DIR)/*.sp
	find . -name "*.pyc" -delete
	find . -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null || true
	@echo "$(GREEN)[CLEAN] 完成$(RESET)"

# ============================================================
# 状态检查
# ============================================================
status: ## 检查项目状态
	@echo "$(BLUE)[STATUS] LeoDrone Ultimate 状态检查$(RESET)"
	@echo ""
	@echo "项目路径:     $(PROJECT_ROOT)"
	@echo "drone-system: $(DRONE_SYSTEM)"
	@echo "omni-percept: $(OMNI_PERCEPTION)"
	@echo ""
	@echo "$(YELLOW)子项目测试状态:$(RESET)"
	@cd $(OMNI_PERCEPTION) && $(PYTHON) tests/test_all.py 2>&1 | tail -3 || echo "  ⚠️ 无法运行"
	@echo ""
	@echo "$(YELLOW)Docker服务:$(RESET)"
	@cd $(DRONE_SYSTEM) && docker compose ps 2>/dev/null || echo "  ⚠️ Docker未运行"
	@echo ""
	@echo "$(YELLOW)磁盘使用:$(RESET)"
	@du -sh $(PROJECT_ROOT) 2>/dev/null || echo "  ⚠️ 无法计算"
