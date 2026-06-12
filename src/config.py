#!/usr/bin/env python3
"""LeoDrone Ultimate — Central Configuration
Pin mappings, I2C/SPI addresses, camera params, PID gains
"""

# ============================================================
# Hardware Pin Mapping (RPi5 / RK3588)
# ============================================================
I2C_BUS = 1
SPI_BUS = 0
SPI_DEVICE = 0
UART_GPS = "/dev/ttyAMA0"
UART_GPS_BAUD = 9600

# ============================================================
# Sensor Addresses
# ============================================================
BME280_ADDR = 0x76        # I2C address (0x76 or 0x77)
ICM42688_SPI_CS = 0       # SPI chip select
GPS_UART_PORT = "/dev/ttyAMA0"

# ============================================================
# Camera Configuration (RK3588 Dual Fisheye)
# ============================================================
NUM_CAMERAS = 4            # 4× IMX219 for 360°, or 2 for stereo
CAMERA_FOV_DEG = 160       # Fisheye field of view
CAMERA_RESOLUTION = (640, 480)
CAMERA_FPS = 30
STEREO_BASELINE_M = 0.12   # 12cm baseline for stereo pair
FISHEYE_MODEL = "equidistant"  # fisheye distortion model

# ============================================================
# 360° Stitch Parameters
# ============================================================
STITCH_OUTPUT_WIDTH = 1920
STITCH_OUTPUT_HEIGHT = 960
STITCH_BLEND_WIDTH = 30    # Blending overlap in pixels
STITCH_EQUIRECT = True     # Equirectangular projection

# ============================================================
# Video Stabilization (EIS)
# ============================================================
EIS_CROP_RATIO = 0.9       # Crop 10% for stabilization margin
EIS_MAX_ROTATION_DEG = 5.0  # Max rotation to compensate
EIS_SMOOTH_FACTOR = 0.95    # Low-pass filter factor

# ============================================================
# Flight Control PID Gains
# ============================================================
PID_ROLL = {"kp": 1.2, "ki": 0.05, "kd": 0.3}
PID_PITCH = {"kp": 1.2, "ki": 0.05, "kd": 0.3}
PID_YAW = {"kp": 1.0, "ki": 0.02, "kd": 0.2}
PID_ALT = {"kp": 1.5, "ki": 0.1, "kd": 0.4}

# ============================================================
# Safety Limits
# ============================================================
MAX_ALTITUDE_M = 120.0     # Legal limit in most jurisdictions
MAX_SPEED_MS = 15.0        # m/s
MAX_DISTANCE_M = 500.0     # Geofence radius
MIN_BATTERY_PCT = 20.0     # Return home threshold
NO_FLY_ZONES = []          # Add GPS coordinates as needed

# ============================================================
# MAVLink / MAVSDK
# ============================================================
MAVSDK_URL = "udp://:14540"  # SITL default
MAVSDK_URL_REAL = "serial:///dev/ttyAMA1:921600"  # Real hardware

# ============================================================
# Audio Detection
# ============================================================
AUDIO_SAMPLE_RATE = 16000
AUDIO_CHANNELS = 1
NOISE_THRESHOLD_DB = 60.0     # Above this = noise event
ACTIVITY_THRESHOLD_DB = 45.0  # Above this = activity detected
VAD_AGGRESSIVENESS = 3        # 0-3, 3=most aggressive

# ============================================================
# RK3588 Specific
# ============================================================
RK3588_NPU_CORES = 6
RK3588_GPU_FREQ_MHZ = 1000
RK3588_CPU_CORES = 8          # 4×A76 + 4×A55

# ============================================================
# Simulation Defaults
# ============================================================
SIM_MODE = True               # Default to simulation
SIM_HOME_LAT = 39.9042
SIM_HOME_LON = 116.4074
SIM_HOME_ALT = 50.0
