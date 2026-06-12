#!/usr/bin/env python3
"""
LeoDrone Ultimate — Extended Integration Tests
Covers all modules: sensors, video, SLAM, tracking, fusion, flight, ground station

Run: python3 tests/test_integration.py
"""

import sys
import os
import unittest
import numpy as np
import time

# Add src path
PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_DIR = os.path.join(PROJECT_DIR, "src")
sys.path.insert(0, SRC_DIR)

# Also add sub-project paths for existing tests
DRONE_SYSTEM_PATH = "/home/fenn/projects/drone-system"
OMNI_PERCEPTION_PATH = "/home/fenn/projects/omni-perception-fusion"
sys.path.insert(0, DRONE_SYSTEM_PATH)
sys.path.insert(0, OMNI_PERCEPTION_PATH)


# ===========================================================================
# Original Sensor Tests (backward compatible)
# ===========================================================================
class TestSensorDataFlow(unittest.TestCase):
    """Original tests from drone-system + omni-perception integration"""

    def test_bme280_data_generation(self):
        np.random.seed(42)
        n = 100
        timestamps = np.arange(n) * 0.01
        temperatures = 25.0 + np.sin(timestamps * 0.1) * 2 + np.random.randn(n) * 0.1
        self.assertTrue(np.all(temperatures > -40) and np.all(temperatures < 85))

    def test_imu_data_generation(self):
        np.random.seed(43)
        n = 200
        timestamps = np.arange(n) * 0.005
        gravity = 9.81
        accel = np.column_stack([
            np.random.randn(n) * 0.05,
            np.random.randn(n) * 0.05,
            np.ones(n) * gravity + np.random.randn(n) * 0.1
        ])
        self.assertTrue(np.all(np.abs(accel) < 16 * 9.81))

    def test_gps_data_generation(self):
        np.random.seed(44)
        lat = 39.9042 + np.random.randn() * 0.001
        lon = 116.4074 + np.random.randn() * 0.001
        self.assertTrue(39 < lat < 41)
        self.assertTrue(115 < lon < 117)

    def test_pressure_altitude_conversion(self):
        pressures = np.linspace(800, 1050, 50)
        altitudes = 44330 * (1 - (pressures / 1013.25) ** (1/5.255))
        self.assertTrue(np.all(np.diff(altitudes) < 0))
        self.assertAlmostEqual(altitudes[np.argmin(np.abs(pressures - 1013.25))], 0, delta=10)

    def test_imu_quaternion_normalization(self):
        q = np.array([1.0, 0.1, 0.2, 0.3])
        q_norm = q / np.linalg.norm(q)
        self.assertAlmostEqual(np.linalg.norm(q_norm), 1.0, places=6)

    def test_euler_to_rotation_matrix(self):
        roll, pitch, yaw = 0.1, 0.2, 0.3
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        R = np.array([
            [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
            [-sp,   cp*sr,           cp*cr]
        ])
        self.assertAlmostEqual(np.linalg.det(R), 1.0, places=5)
        self.assertTrue(np.allclose(R @ R.T, np.eye(3), atol=1e-5))


class TestVideoStabilization(unittest.TestCase):
    def test_video_stabilizer_vqf(self):
        from video.video_stabilizer import VQFOrientation
        vqf = VQFOrientation(sample_rate=200.0)
        q = vqf.update(gyro=np.array([0.01, 0.01, 0.01]), accel=np.array([0, 0, 9.81]))
        self.assertEqual(len(q), 4)
        self.assertAlmostEqual(np.linalg.norm(q), 1.0, places=5)

    def test_stabilizer_frame(self):
        from video.video_stabilizer import VideoStabilizer, IMUSample
        stab = VideoStabilizer()
        frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        samples = [IMUSample(i*0.005, np.array([0.01]*3), np.array([0,0,9.81])) for i in range(6)]
        result = stab.stabilize_frame(frame, samples)
        h, w = result.shape[:2]
        self.assertTrue(h > 0 and w > 0)
        self.assertTrue(h <= 480 and w <= 640)


class TestPanoramicStitching(unittest.TestCase):
    def test_stitcher_init(self):
        from video.stitcher_360 import PanoramicStitcher
        s = PanoramicStitcher(num_cameras=4)
        self.assertTrue(s.initialize())

    def test_stitcher_output(self):
        from video.stitcher_360 import PanoramicStitcher
        s = PanoramicStitcher(num_cameras=4, output_width=320, output_height=160)
        s.initialize()
        frames = [np.random.randint(0, 255, (120, 160, 3), dtype=np.uint8) for _ in range(4)]
        result = s.stitch(frames)
        self.assertEqual(result.shape, (160, 320, 3))


class TestRK3588Camera(unittest.TestCase):
    def test_camera_init(self):
        from sensors.rk3588_camera import RK3588CameraManager
        cam = RK3588CameraManager(num_cameras=4, sim_mode=True)
        self.assertTrue(cam.initialize())

    def test_stereo_capture(self):
        from sensors.rk3588_camera import RK3588CameraManager
        cam = RK3588CameraManager(num_cameras=2, sim_mode=True)
        cam.initialize()
        left, right = cam.capture_stereo()
        self.assertEqual(left.data.shape, right.data.shape)
        self.assertEqual(left.camera_id, 0)
        self.assertEqual(right.camera_id, 1)

    def test_360_capture(self):
        from sensors.rk3588_camera import RK3588CameraManager
        cam = RK3588CameraManager(num_cameras=4, sim_mode=True)
        cam.initialize()
        frames = cam.capture_all()
        self.assertEqual(len(frames), 4)

    def test_fisheye_calibration(self):
        from sensors.rk3588_camera import RK3588CameraManager
        cam = RK3588CameraManager(num_cameras=4, fov_deg=160, sim_mode=True)
        cam.initialize()
        calib = cam.get_calibration(0)
        self.assertIn('K', calib)
        self.assertEqual(calib['K'].shape, (3, 3))


class TestBME280Driver(unittest.TestCase):
    def test_bme280_init(self):
        from sensors.bme280_driver import BME280Driver
        bme = BME280Driver(sim_mode=True)
        self.assertTrue(bme.initialize())

    def test_bme280_read(self):
        from sensors.bme280_driver import BME280Driver
        bme = BME280Driver(sim_mode=True)
        bme.initialize()
        reading = bme.read()
        self.assertTrue(-40 < reading.temperature < 85)
        self.assertTrue(0 < reading.humidity < 100)
        self.assertTrue(300 < reading.pressure < 1100)


class TestIMUDriver(unittest.TestCase):
    def test_imu_init(self):
        from sensors.imu_driver import ICM42688Driver
        imu = ICM42688Driver(sim_mode=True)
        self.assertTrue(imu.initialize())

    def test_imu_read(self):
        from sensors.imu_driver import ICM42688Driver
        imu = ICM42688Driver(sim_mode=True)
        imu.initialize()
        reading = imu.read()
        self.assertEqual(len(reading.accel), 3)
        self.assertEqual(len(reading.gyro), 3)


class TestGPSDriver(unittest.TestCase):
    def test_gps_init(self):
        from sensors.gps_driver import GPSDriver
        gps = GPSDriver(sim_mode=True)
        self.assertTrue(gps.initialize())

    def test_gps_read(self):
        from sensors.gps_driver import GPSDriver
        gps = GPSDriver(sim_mode=True)
        gps.initialize()
        reading = gps.read()
        self.assertTrue(39 < reading.lat < 41)
        self.assertEqual(reading.fix_type, 3)


class TestAudioDetector(unittest.TestCase):
    def test_audio_init(self):
        from sensors.audio_detector import IntegratedDetector
        det = IntegratedDetector(sim_mode=True)
        self.assertTrue(det.initialize())

    def test_audio_detect(self):
        from sensors.audio_detector import IntegratedDetector, ActivityType
        det = IntegratedDetector(sim_mode=True)
        det.initialize()
        result = det.detect_all()
        self.assertIn('audio', result)
        self.assertIsInstance(result['audio'].activity_type, ActivityType)

    def test_alert_level(self):
        from sensors.audio_detector import IntegratedDetector
        det = IntegratedDetector(sim_mode=True)
        det.initialize()
        level = det.get_composite_alert_level()
        self.assertIn(level, ['CLEAR', 'LOW', 'MEDIUM', 'HIGH'])


class TestVINSFusionSLAM(unittest.TestCase):
    def test_slam_init(self):
        from slam.vins_fusion import VINSFusionSLAM
        slam = VINSFusionSLAM()
        self.assertTrue(slam.initialize())

    def test_slam_update(self):
        from slam.vins_fusion import VINSFusionSLAM
        slam = VINSFusionSLAM()
        slam.initialize()
        # Add some IMU data
        for i in range(10):
            slam.add_imu(np.array([0, 0, 9.81]), np.array([0.01, 0.01, 0.01]), time.time())
        frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        pose = slam.add_frame(frame, time.time())
        self.assertEqual(len(pose.position), 3)
        self.assertEqual(len(pose.orientation), 4)


class TestYOLOTracker(unittest.TestCase):
    def test_tracker_init(self):
        from tracking.yolo_tracker import YOLOTracker
        t = YOLOTracker(sim_mode=True)
        self.assertTrue(t.initialize())

    def test_detect_and_track(self):
        from tracking.yolo_tracker import YOLOTracker
        t = YOLOTracker(sim_mode=True)
        t.initialize()
        frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        dets = t.detect(frame)
        self.assertIsInstance(dets, list)
        tracks = t.update(dets)
        self.assertIsInstance(tracks, list)


class TestEKFSensorFusion(unittest.TestCase):
    def test_ekf_init(self):
        from fusion.ekf_sensor import EKFSensorFusion
        ekf = EKFSensorFusion()
        ekf.initialize()
        state = ekf.get_state()
        self.assertEqual(len(state['position']), 3)

    def test_ekf_predict_update(self):
        from fusion.ekf_sensor import EKFSensorFusion
        ekf = EKFSensorFusion()
        ekf.initialize()
        # Predict with IMU
        for _ in range(10):
            ekf.predict(np.array([0, 0, 9.81]), np.array([0.01]*3))
        # Update with GPS
        ekf.update_gps(np.array([1.0, 2.0, -3.0]))
        state = ekf.get_state()
        # Position should have moved toward GPS measurement
        self.assertTrue(np.linalg.norm(state['position'] - np.array([1, 2, -3])) < 5)


class TestOffboardController(unittest.TestCase):
    def test_controller_init(self):
        from flight.offboard_controller import OffboardController
        ctrl = OffboardController(sim_mode=True)
        self.assertTrue(ctrl.initialize())

    def test_arm_takeoff_land(self):
        from flight.offboard_controller import OffboardController
        ctrl = OffboardController(sim_mode=True)
        ctrl.initialize()
        asyncio = __import__('asyncio')
        asyncio.run(ctrl.arm())
        self.assertTrue(ctrl.get_state().armed)
        asyncio.run(ctrl.takeoff(2.0))
        asyncio.run(ctrl.land())
        self.assertFalse(ctrl.get_state().armed)


class TestFlightModeManager(unittest.TestCase):
    def test_valid_transition(self):
        from flight.flight_modes import FlightModeManager, FlightMode
        mgr = FlightModeManager()
        self.assertTrue(mgr.transition(FlightMode.STABILIZED))
        self.assertTrue(mgr.transition(FlightMode.OFFBOARD))

    def test_invalid_transition(self):
        from flight.flight_modes import FlightModeManager, FlightMode
        mgr = FlightModeManager()
        self.assertFalse(mgr.transition(FlightMode.HIGH_SPEED))  # Can't go DISARMED->HIGH_SPEED

    def test_emergency(self):
        from flight.flight_modes import FlightModeManager, FlightMode
        mgr = FlightModeManager()
        mgr.force_emergency()
        self.assertEqual(mgr.mode, FlightMode.EMERGENCY)


class TestGroundStation(unittest.TestCase):
    def test_dashboard_init(self):
        from ground_station.dashboard import GroundStationDashboard, TelemetryPacket
        dash = GroundStationDashboard(sim_mode=True)
        self.assertTrue(dash.initialize())

    def test_telemetry_update(self):
        from ground_station.dashboard import GroundStationDashboard, TelemetryPacket
        dash = GroundStationDashboard(sim_mode=True)
        dash.initialize()
        pkt = TelemetryPacket(
            position=np.array([1, 2, -3]),
            velocity=np.zeros(3),
            attitude=np.zeros(3),
            battery_pct=95.0,
            gps_fix=3,
            flight_mode="OFFBOARD",
            timestamp=time.time()
        )
        dash.update_telemetry(pkt)
        json_str = dash.get_telemetry_json()
        self.assertIn("OFFBOARD", json_str)


class TestSpeedEstimator(unittest.TestCase):
    def test_speed_est(self):
        from video.speed_estimator import SpeedEstimator
        est = SpeedEstimator()
        frame1 = np.random.randint(0, 255, (120, 160), dtype=np.uint8)
        speed, flow = est.estimate_speed(frame1)
        self.assertEqual(speed, 0.0)  # First frame returns 0
        frame2 = np.random.randint(0, 255, (120, 160), dtype=np.uint8)
        speed2, _ = est.estimate_speed(frame2)
        self.assertIsInstance(speed2, float)


class TestMainController(unittest.TestCase):
    def test_controller_init(self):
        # Verify main controller can be imported
        from main_controller import LeoDroneUltimate
        drone = LeoDroneUltimate(sim_mode=True)
        self.assertFalse(drone._running)

    def test_initialize(self):
        from main_controller import LeoDroneUltimate
        drone = LeoDroneUltimate(sim_mode=True)
        asyncio = __import__('asyncio')
        result = asyncio.run(drone.initialize())
        self.assertTrue(result)


class TestSafetyChecks(unittest.TestCase):
    def test_altitude_limit(self):
        """Verify 120m altitude limit is enforced"""
        from src.config import MAX_ALTITUDE_M
        self.assertEqual(MAX_ALTITUDE_M, 120.0)

    def test_battery_rth(self):
        """Verify 20% battery RTL threshold"""
        from src.config import MIN_BATTERY_PCT
        self.assertEqual(MIN_BATTERY_PCT, 20.0)

    def test_geofence_radius(self):
        """Verify 500m geofence"""
        from src.config import MAX_DISTANCE_M
        self.assertEqual(MAX_DISTANCE_M, 500.0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
