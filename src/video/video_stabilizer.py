#!/usr/bin/env python3
"""LeoDrone Ultimate - EIS + Gimbal Video Stabilizer
VQF quaternion-based orientation estimation + IMU compensation + crop-transform
"""
import numpy as np
import time
import logging
from dataclasses import dataclass
from typing import List, Optional

logger = logging.getLogger(__name__)

@dataclass
class IMUSample:
    timestamp: float
    gyro: np.ndarray     # (3,) rad/s
    accel: np.ndarray    # (3,) m/s^2

class VQFOrientation:
    """Versatile Quaternion-based Filter for orientation estimation
    Fuses gyro (angular rate) and accel (gravity) to estimate 3D orientation.
    """
    def __init__(self, sample_rate: float = 200.0):
        self.dt = 1.0 / sample_rate
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # quaternion [w, x, y, z]
        self.gyro_bias = np.zeros(3)
        self._initialized = False
        
    def update(self, gyro: np.ndarray, accel: np.ndarray) -> np.ndarray:
        """Update orientation with new IMU sample
        
        Returns quaternion [w, x, y, z]
        """
        # Compensate gyro bias
        gyro_unbiased = gyro - self.gyro_bias
        
        # Gyro integration (quaternion derivative)
        omega = np.array([0, *gyro_unbiased])
        q_dot = 0.5 * self._quat_multiply(self.q, omega)
        self.q = self.q + q_dot * self.dt
        
        # Normalize quaternion
        self.q = self.q / (np.linalg.norm(self.q) + 1e-10)
        
        # Gravity correction (accelerometer)
        if np.linalg.norm(accel) > 1.0:
            gravity_world = np.array([0, 0, 0, 9.81])
            gravity_body = self._quat_conjugate_multiply(self.q, gravity_world)
            g_est = gravity_body[1:]  # Expected gravity direction in body frame
            g_meas = accel / (np.linalg.norm(accel) + 1e-10) * 9.81
            
            # Cross product for correction
            error = np.cross(g_est, g_meas)
            correction = error * 0.01  # Small gain
            
            # Apply correction to bias estimate
            self.gyro_bias += correction * 0.001
            
        self._initialized = True
        return self.q.copy()
    
    def get_rotation_matrix(self) -> np.ndarray:
        """Get current rotation as 3x3 matrix"""
        w, x, y, z = self.q
        return np.array([
            [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]
        ])
    
    @staticmethod
    def _quat_multiply(q1, q2):
        w1,x1,y1,z1 = q1; w2,x2,y2,z2 = q2
        return np.array([
            w1*w2-x1*x2-y1*y2-z1*z2,
            w1*x2+x1*w2+y1*z2-z1*y2,
            w1*y2-x1*z2+y1*w2+z1*x2,
            w1*z2+x1*y2-y1*x2+z1*w2
        ])
    
    @staticmethod
    def _quat_conjugate_multiply(q, v):
        q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
        return VQFOrientation._quat_multiply(
            VQFOrientation._quat_multiply(q_conj, v), q)


class VideoStabilizer:
    """EIS + gimbal video stabilization
    
    Uses VQF orientation estimation to compensate camera shake:
    1. Estimate orientation from IMU (VQF filter)
    2. Compute inverse rotation (stabilization transform)
    3. Apply transform + crop to remove black borders
    """
    
    def __init__(self, crop_ratio: float = 0.9, smooth_factor: float = 0.95,
                 max_rotation_deg: float = 5.0, imu_rate: float = 200.0):
        self.crop_ratio = crop_ratio
        self.smooth_factor = smooth_factor
        self.max_rotation = np.radians(max_rotation_deg)
        self.vqf = VQFOrientation(sample_rate=imu_rate)
        self._smoothed_rotation = np.eye(3)
        self._frame_count = 0
        
    def stabilize_frame(self, frame: np.ndarray, imu_samples: List[IMUSample]) -> np.ndarray:
        """Stabilize a single video frame using IMU data
        
        Args:
            frame: (H, W, 3) BGR image
            imu_samples: list of IMU samples covering this frame interval
        Returns:
            (H', W', 3) stabilized (cropped) image
        """
        h, w = frame.shape[:2]
        
        # Update orientation from all IMU samples
        for sample in imu_samples:
            self.vqf.update(sample.gyro, sample.accel)
        
        # Get current and target rotations
        current_rot = self.vqf.get_rotation_matrix()
        
        # Smooth the rotation (low-pass filter to avoid jitter)
        self._smoothed_rotation = (
            self.smooth_factor * self._smoothed_rotation +
            (1 - self.smooth_factor) * current_rot
        )
        
        # Compute stabilization transform
        delta_rot = current_rot @ self._smoothed_rotation.T
        
        # Apply affine transform
        center = np.array([w / 2, h / 2])
        
        # Build 2D affine from 3D rotation (project to image plane)
        r21 = delta_rot[:2, :2]  # 2x2 submatrix
        r31 = delta_rot[2, :2]   # perspective terms
        
        # Apply rotation with perspective correction
        y_coords, x_coords = np.mgrid[:h, :w]
        x_norm = (x_coords - center[0]) / center[0]
        y_norm = (y_coords - center[1]) / center[1]
        
        # Inverse warp coordinates
        x_new = r21[0, 0] * x_norm + r21[0, 1] * y_norm
        y_new = r21[1, 0] * x_norm + r21[1, 1] * y_norm
        
        # Perspective division
        denom = r31[0] * x_norm + r31[1] * y_norm + 1.0
        x_new = x_new / (denom + 1e-10)
        y_new = y_new / (denom + 1e-10)
        
        # Back to pixel coords
        x_pixel = (x_new * center[0] + center[0]).astype(np.int32)
        y_pixel = (y_new * center[1] + center[1]).astype(np.int32)
        
        # Clip to image bounds
        x_pixel = np.clip(x_pixel, 0, w - 1)
        y_pixel = np.clip(y_pixel, 0, h - 1)
        
        # Apply warp
        result = frame[y_pixel, x_pixel]
        
        # Crop to remove black borders
        if self.crop_ratio < 1.0:
            crop_h = int(h * self.crop_ratio)
            crop_w = int(w * self.crop_ratio)
            y_off = (h - crop_h) // 2
            x_off = (w - crop_w) // 2
            result = result[y_off:y_off+crop_h, x_off:x_off+crop_w]
        
        self._frame_count += 1
        return result
    
    def process_sequence(self, imu_sequence: List[IMUSample], 
                         frame_interval: int = 6) -> List[np.ndarray]:
        """Process entire IMU sequence and return stabilization transforms
        
        Args:
            imu_sequence: all IMU samples
            frame_interval: IMU samples per video frame
        Returns:
            List of rotation matrices (one per frame)
        """
        rotations = []
        for i in range(0, len(imu_sequence), frame_interval):
            samples = imu_sequence[i:i+frame_interval]
            for s in samples:
                self.vqf.update(s.gyro, s.accel)
            rotations.append(self.vqf.get_rotation_matrix().copy())
        return rotations
