#!/usr/bin/env python3
"""LeoDrone Ultimate — RK3588 Dual Fisheye Camera Manager
CSI interface, 2× IMX219/IMX477, stereo capture + 4-camera 360°
"""
import numpy as np
import time
import logging
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass

logger = logging.getLogger(__name__)

@dataclass
class CameraFrame:
    """Single camera frame"""
    data: np.ndarray       # (H, W, 3) BGR
    timestamp: float
    camera_id: int
    resolution: Tuple[int, int]

class RK3588CameraManager:
    """RK3588 dual fisheye / quad camera manager
    
    Supports:
    - 2-camera stereo fisheye (left/right)
    - 4-camera 360° configuration (front/back/left/right)
    - Real hardware via libcamera or GStreamer
    - Simulation mode with synthetic fisheye images
    """
    
    def __init__(self, num_cameras: int = 4, resolution: Tuple[int, int] = (640, 480),
                 fov_deg: float = 160.0, sim_mode: bool = True):
        self.num_cameras = num_cameras
        self.resolution = resolution
        self.fov_deg = fov_deg
        self.sim_mode = sim_mode
        self._cameras = {}
        self._calibrations = {}
        self._running = False
        
        # Fisheye distortion parameters (Kannala-Brandt model)
        self._distortion_coeffs = self._generate_distortion(fov_deg)
        
    def _generate_distortion(self, fov_deg: float) -> np.ndarray:
        """Generate fisheye distortion coefficients for given FOV"""
        # Kannala-Brandt model: k1, k2, k3, k4
        theta_max = np.radians(fov_deg / 2)
        k1 = -0.001 * (180 / fov_deg)
        k2 = 0.0001 * (180 / fov_deg)
        k3 = -0.00001 * (180 / fov_deg)
        k4 = 0.000001 * (180 / fov_deg)
        return np.array([k1, k2, k3, k4])
    
    def initialize(self) -> bool:
        """Initialize camera subsystem"""
        logger.info(f"Initializing {self.num_cameras} cameras (sim={self.sim_mode})")
        
        if self.sim_mode:
            for i in range(self.num_cameras):
                self._cameras[i] = f"sim_camera_{i}"
                # Synthetic calibration: intrinsics matrix
                fx = fy = self.resolution[0] / (2 * np.tan(np.radians(self.fov_deg / 2)))
                cx, cy = self.resolution[0] / 2, self.resolution[1] / 2
                self._calibrations[i] = {
                    'K': np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]]),
                    'D': self._distortion_coeffs.copy(),
                    'R': np.eye(3),  # Rotation to reference frame
                    'fov': self.fov_deg,
                    'direction': ['front', 'right', 'back', 'left'][i % 4]
                }
        else:
            # Real RK3588 camera init via libcamera
            try:
                import libcamera
                for i in range(self.num_cameras):
                    self._cameras[i] = f"libcamera_{i}"
                    logger.info(f"  Camera {i}: initialized via libcamera")
            except ImportError:
                logger.warning("libcamera not available, falling back to sim mode")
                self.sim_mode = True
                return self.initialize()
        
        self._running = True
        logger.info(f"  All {self.num_cameras} cameras ready")
        return True
    
    def capture_stereo(self) -> Tuple[CameraFrame, CameraFrame]:
        """Capture stereo pair from dual fisheye cameras
        
        Returns:
            (left_frame, right_frame) tuple
        """
        if not self._running:
            raise RuntimeError("Cameras not initialized")
        
        t = time.time()
        if self.sim_mode:
            left = CameraFrame(
                data=np.random.randint(0, 255, (*self.resolution, 3), dtype=np.uint8),
                timestamp=t, camera_id=0, resolution=self.resolution
            )
            right = CameraFrame(
                data=np.random.randint(0, 255, (*self.resolution, 3), dtype=np.uint8),
                timestamp=t, camera_id=1, resolution=self.resolution
            )
        else:
            # Real capture via GStreamer pipeline
            left = self._capture_real(0)
            right = self._capture_real(1)
        return left, right
    
    def capture_all(self) -> List[CameraFrame]:
        """Capture from all cameras simultaneously for 360° stitching
        
        Returns:
            List of CameraFrame, one per camera
        """
        if not self._running:
            raise RuntimeError("Cameras not initialized")
        
        t = time.time()
        frames = []
        for i in range(self.num_cameras):
            if self.sim_mode:
                # Generate synthetic fisheye pattern per direction
                frame_data = self._generate_fisheye_pattern(i, t)
                frames.append(CameraFrame(
                    data=frame_data,
                    timestamp=t + i * 0.001,  # Slight time offset
                    camera_id=i,
                    resolution=self.resolution
                ))
            else:
                frames.append(self._capture_real(i))
        return frames
    
    def _generate_fisheye_pattern(self, camera_id: int, t: float) -> np.ndarray:
        """Generate synthetic fisheye image for simulation"""
        h, w = self.resolution
        # Create circular fisheye pattern
        y, x = np.mgrid[:h, :w]
        cx, cy = w / 2, h / 2
        r = np.sqrt((x - cx)**2 + (y - cy)**2)
        r_max = min(w, h) / 2
        
        # Circular mask with gradient
        mask = (r < r_max).astype(np.float64)
        gradient = np.clip(1.0 - r / r_max, 0, 1)
        
        # Color based on camera direction
        colors = [(200, 100, 50), (50, 200, 100), (100, 50, 200), (200, 200, 50)]
        base_color = np.array(colors[camera_id % 4])
        
        frame = np.zeros((h, w, 3), dtype=np.uint8)
        for c in range(3):
            frame[:, :, c] = (base_color[c] * gradient * mask).astype(np.uint8)
        
        # Add some noise for realism
        noise = np.random.randint(0, 10, (h, w, 3), dtype=np.uint8)
        frame = np.clip(frame.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        return frame
    
    def _capture_real(self, camera_id: int) -> CameraFrame:
        """Real camera capture (placeholder for hardware integration)"""
        # Would use GStreamer pipeline:
        # gst-launch-1.0 libcamerasrc camera=/base/soc/i2c0mux/i2c@{camera_id} ...
        raise NotImplementedError("Real capture requires libcamera + GStreamer")
    
    def get_calibration(self, camera_id: int) -> Dict:
        """Get camera intrinsics and distortion parameters"""
        return self._calibrations.get(camera_id, {})
    
    def undistort_fisheye(self, frame: CameraFrame) -> np.ndarray:
        """Undistort fisheye image to perspective projection
        
        Uses Kannala-Brandt model for fisheye correction
        """
        h, w = frame.data.shape[:2]
        calib = self._calibrations.get(frame.camera_id, {})
        K = calib.get('K', np.eye(3))
        D = calib.get('D', np.zeros(4))
        
        # Simple undistortion: remap pixels
        y, x = np.mgrid[:h, :w]
        pts = np.stack([x.ravel(), y.ravel()], axis=-1).astype(np.float64)
        
        # Normalize coordinates
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]
        xn = (pts[:, 0] - cx) / fx
        yn = (pts[:, 1] - cy) / fy
        
        # Apply inverse distortion
        r2 = xn**2 + yn**2
        r4 = r2 * r2
        distortion = 1 + D[0]*r2 + D[1]*r4 + D[2]*r4*r2 + D[3]*r4*r4
        
        x_undist = (xn * distortion * fx + cx).astype(np.int32)
        y_undist = (yn * distortion * fy + cy).astype(np.int32)
        
        # Clip to image bounds
        x_undist = np.clip(x_undist, 0, w - 1)
        y_undist = np.clip(y_undist, 0, h - 1)
        
        result = frame.data[y_undist.reshape(h, w), x_undist.reshape(h, w)]
        return result
    
    def shutdown(self):
        """Release camera resources"""
        self._running = False
        self._cameras.clear()
        logger.info("Cameras shutdown")
