#!/usr/bin/env python3
"""LeoDrone Ultimate — Main Orchestrator
Async event loop: sensor -> perception -> fusion -> cognition -> coordination -> flight
"""
import asyncio
import numpy as np
import time
import logging
import signal
import sys
from typing import Optional

# Local modules
sys.path.insert(0, '/home/fenn/projects/leo-drone-ultimate/src')

from sensors.bme280_driver import BME280Driver, BME280Reading
from sensors.imu_driver import ICM42688Driver, IMUReading
from sensors.gps_driver import GPSDriver, GPSReading
from sensors.rk3588_camera import RK3588CameraManager, CameraFrame
from sensors.audio_detector import IntegratedDetector, ActivityType
from video.stitcher_360 import PanoramicStitcher
from video.video_stabilizer import VideoStabilizer, IMUSample
from video.speed_estimator import SpeedEstimator
from slam.vins_fusion import VINSFusionSLAM
from tracking.yolo_tracker import YOLOTracker
from fusion.ekf_sensor import EKFSensorFusion
from flight.offboard_controller import OffboardController
from flight.flight_modes import FlightModeManager, FlightMode
from ground_station.dashboard import GroundStationDashboard, TelemetryPacket

logger = logging.getLogger("LeoDroneUltimate")

class LeoDroneUltimate:
    """Main orchestrator for the complete drone system
    
    Pipeline: Sensors → Perception → Fusion → Cognition → Coordination → Flight
    Safety layer monitors all stages.
    """
    
    def __init__(self, sim_mode: bool = True):
        self.sim_mode = sim_mode
        
        # L1: Sensors
        self.bme280 = BME280Driver(sim_mode=sim_mode)
        self.imu = ICM42688Driver(sim_mode=sim_mode)
        self.gps = GPSDriver(sim_mode=sim_mode)
        self.camera = RK3588CameraManager(num_cameras=4, sim_mode=sim_mode)
        self.audio = IntegratedDetector(sim_mode=sim_mode)
        
        # L2: Perception
        self.stitcher = PanoramicStitcher()
        self.stabilizer = VideoStabilizer()
        self.speed_est = SpeedEstimator()
        self.slam = VINSFusionSLAM()
        self.tracker = YOLOTracker(sim_mode=sim_mode)
        
        # L3: Fusion
        self.ekf = EKFSensorFusion()
        
        # L4-L5: Flight
        self.flight = OffboardController(sim_mode=sim_mode)
        self.mode_mgr = FlightModeManager()
        
        # L6: Ground Station
        self.dashboard = GroundStationDashboard(sim_mode=sim_mode)
        
        self._running = False
        self._cycle_count = 0
        
    async def initialize(self) -> bool:
        """Initialize all subsystems"""
        logger.info("Initializing LeoDrone Ultimate...")
        
        results = {
            'bme280': self.bme280.initialize(),
            'imu': self.imu.initialize(),
            'gps': self.gps.initialize(),
            'camera': self.camera.initialize(),
            'audio': self.audio.initialize(),
            'stitcher': self.stitcher.initialize(),
            'slam': self.slam.initialize(),
            'tracker': self.tracker.initialize(),
            'flight': self.flight.initialize(),
            'dashboard': self.dashboard.initialize(),
            'ekf': True,  # EKF init is trivial
        }
        
        self.ekf.initialize(position=np.zeros(3))
        
        failed = [k for k, v in results.items() if not v]
        if failed:
            logger.warning(f"Some subsystems failed: {failed}")
        
        logger.info(f"Initialization complete: {sum(results.values())}/{len(results)} OK")
        return all(results.values())
    
    async def run_cycle(self) -> dict:
        """Execute one complete sensor-to-flight cycle
        
        Returns dict with all current readings and states
        """
        self._cycle_count += 1
        
        # L1: Read sensors
        bme_reading = self.bme280.read()
        imu_reading = self.imu.read()
        gps_reading = self.gps.read()
        frames = self.camera.capture_all()
        audio_event = self.audio.detect_all()
        
        # L2: Perception
        panorama = self.stitcher.stitch([f.data for f in frames])
        imu_samples = [IMUSample(imu_reading.timestamp, imu_reading.gyro, imu_reading.accel)]
        slam_pose = self.slam.add_frame(frames[0].data, frames[0].timestamp)
        detections = self.tracker.detect(frames[0].data)
        tracks = self.tracker.update(detections)
        
        # L3: Fusion
        self.ekf.predict(imu_reading.accel, imu_reading.gyro)
        self.ekf.update_gps(np.array([gps_reading.lat, gps_reading.lon, gps_reading.alt]))
        self.ekf.update_baro(bme_reading.altitude)
        fused_state = self.ekf.get_state()
        
        # L4-L5: Decision & Flight
        follow_target = self.tracker.get_follow_target()
        if follow_target is not None:
            await self.flight.follow_target(follow_target[0])
        
        # L6: Dashboard update
        telemetry = TelemetryPacket(
            position=fused_state['position'],
            velocity=fused_state['velocity'],
            attitude=fused_state['attitude'],
            battery_pct=self.flight.get_state().battery_pct,
            gps_fix=gps_reading.fix_type,
            flight_mode=self.mode_mgr.mode.name,
            timestamp=time.time()
        )
        self.dashboard.update_telemetry(telemetry)
        
        return {
            'cycle': self._cycle_count,
            'bme280': bme_reading,
            'imu': imu_reading,
            'gps': gps_reading,
            'slam_pose': slam_pose,
            'tracks': len(tracks),
            'fused': fused_state,
            'audio_alert': audio_event.get('audio', None),
            'mode': self.mode_mgr.mode.name,
        }
    
    async def run(self, duration_s: float = 60.0):
        """Run the main loop for specified duration"""
        await self.initialize()
        self._running = True
        
        logger.info(f"Running for {duration_s}s...")
        start = time.time()
        
        while self._running and (time.time() - start) < duration_s:
            result = await self.run_cycle()
            if self._cycle_count % 10 == 0:
                logger.info(f"Cycle {self._cycle_count}: "
                           f"T={result['bme280'].temperature:.1f}C "
                           f"H={result['bme280'].humidity:.1f}% "
                           f"Tracks={result['tracks']} "
                           f"Mode={result['mode']}")
            await asyncio.sleep(0.05)  # 20Hz main loop
        
        logger.info(f"Completed {self._cycle_count} cycles")
    
    def stop(self):
        self._running = False
        self.bme280.shutdown()
        self.imu.shutdown()
        self.gps.shutdown()
        self.camera.shutdown()
        logger.info("All subsystems shut down")


async def main():
    drone = LeoDroneUltimate(sim_mode=True)
    
    def signal_handler(sig, frame):
        drone.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    await drone.run(duration_s=10.0)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    asyncio.run(main())
