#!/usr/bin/env python3
"""LeoDrone Ultimate - Video Speed Estimator
Optical flow + frame difference velocity estimation
"""
import numpy as np
import logging
from typing import Tuple, Optional

logger = logging.getLogger(__name__)

class SpeedEstimator:
    """Video-based speed estimation using optical flow
    
    Pipeline: frame pair -> feature detection -> flow -> velocity
    """
    
    def __init__(self, focal_length_px: float = 300.0, 
                 camera_height_m: float = 2.0,
                 method: str = "lk"):
        self.focal_length = focal_length_px
        self.camera_height = camera_height_m
        self.method = method
        self._prev_frame = None
        self._prev_features = None
        
    def estimate_speed(self, frame: np.ndarray, dt: float = 0.033) -> Tuple[float, np.ndarray]:
        """Estimate ground speed from consecutive frames
        
        Args:
            frame: (H, W) grayscale image
            dt: time between frames
        Returns:
            (speed_ms, flow_vectors) - speed in m/s and optical flow field
        """
        h, w = frame.shape[:2]
        
        if self._prev_frame is None:
            self._prev_frame = frame
            return 0.0, np.zeros((h//8, w//8, 2))
        
        # Compute optical flow (simple block matching)
        flow = self._compute_flow(self._prev_frame, frame)
        
        # Average flow magnitude
        flow_mag = np.sqrt(flow[:, :, 0]**2 + flow[:, :, 1]**2)
        avg_flow = np.median(flow_mag)
        
        # Convert pixel flow to speed
        # v = (flow_px / focal_length) * (camera_height / dt)
        speed = (avg_flow / self.focal_length) * (self.camera_height / max(dt, 1e-6))
        
        self._prev_frame = frame
        return float(speed), flow
    
    def _compute_flow(self, prev: np.ndarray, curr: np.ndarray, 
                      block_size: int = 8, search_range: int = 4) -> np.ndarray:
        """Simple block-matching optical flow"""
        h, w = prev.shape[:2]
        bh, bw = h // block_size, w // block_size
        flow = np.zeros((bh, bw, 2))
        
        # Downsample
        prev_d = prev[::block_size, ::block_size][:bh, :bw].astype(np.float64)
        curr_d = curr[::block_size, ::block_size][:bh, :bw].astype(np.float64)
        
        # Phase correlation for global motion
        f_prev = np.fft.fft2(prev_d)
        f_curr = np.fft.fft2(curr_d)
        cross = f_curr * np.conj(f_prev)
        cross = cross / (np.abs(cross) + 1e-10)
        correlation = np.real(np.fft.ifft2(cross))
        
        # Find peak
        peak = np.unravel_index(np.argmax(correlation), correlation.shape)
        dy = peak[0] if peak[0] < bh/2 else peak[0] - bh
        dx = peak[1] if peak[1] < bw/2 else peak[1] - bw
        
        flow[:, :, 0] = dx * block_size
        flow[:, :, 1] = dy * block_size
        
        return flow
    
    def estimate_from_imu(self, accel: np.ndarray, dt: float = 0.005) -> float:
        """Estimate speed from IMU integration (fallback method)"""
        return float(np.linalg.norm(accel) * dt)
