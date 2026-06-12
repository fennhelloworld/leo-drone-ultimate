#!/usr/bin/env python3
"""LeoDrone Ultimate - 360 Panoramic Stitcher
4 fisheye cameras: undistort -> equirectangular -> blend
"""
import numpy as np
import logging
from typing import List, Tuple

logger = logging.getLogger(__name__)

class PanoramicStitcher:
    """4-camera 360-degree panoramic image stitcher
    
    Pipeline per camera: fisheye undistort -> equirectangular projection
    Then: seam finding -> multi-band blending -> output equirect
    """
    
    def __init__(self, num_cameras: int = 4, camera_fov_deg: float = 160.0,
                 output_width: int = 1920, output_height: int = 960,
                 blend_width: int = 30):
        self.num_cameras = num_cameras
        self.fov = camera_fov_deg
        self.out_w = output_width
        self.out_h = output_height
        self.blend_width = blend_width
        self._warp_maps = {}
        self._masks = {}
        self._initialized = False
        
    def initialize(self) -> bool:
        """Pre-compute warp maps and blending masks"""
        for cam_id in range(self.num_cameras):
            # Each camera covers ~90 degrees of the 360 sphere
            theta_start = cam_id * (2 * np.pi / self.num_cameras)
            theta_end = (cam_id + 1) * (2 * np.pi / self.num_cameras)
            
            # Equirectangular coordinate grid for this camera's sector
            theta = np.linspace(theta_start, theta_end, self.out_w // self.num_cameras)
            phi = np.linspace(-np.pi/2, np.pi/2, self.out_h)
            theta_grid, phi_grid = np.meshgrid(theta, phi)
            
            # Spherical to Cartesian (for fisheye projection lookup)
            x = np.cos(phi_grid) * np.sin(theta_grid - (theta_start + theta_end) / 2)
            y = np.sin(phi_grid)
            z = np.cos(phi_grid) * np.cos(theta_grid - (theta_start + theta_end) / 2)
            
            # Fisheye projection: r = f * theta
            r = np.sqrt(x**2 + y**2)
            theta_fish = np.arctan2(r, z)
            theta_fish = np.clip(theta_fish, 0, np.radians(self.fov / 2))
            
            # Normalize to camera pixel coordinates
            px = (x / (r + 1e-10) * theta_fish / np.radians(self.fov / 2))
            py = (y / (r + 1e-10) * theta_fish / np.radians(self.fov / 2))
            
            self._warp_maps[cam_id] = {
                'px': px, 'py': py,
                'theta_start': theta_start,
                'sector_w': self.out_w // self.num_cameras
            }
            
            # Blending mask with linear ramp at edges
            sector_w = self.out_w // self.num_cameras
            mask = np.ones((self.out_h, sector_w))
            if self.blend_width > 0:
                ramp_l = np.linspace(0, 1, self.blend_width)
                ramp_r = np.linspace(1, 0, self.blend_width)
                mask[:, :self.blend_width] *= ramp_l
                mask[:, -self.blend_width:] *= ramp_r
            self._masks[cam_id] = mask
            
        self._initialized = True
        logger.info(f"360 stitcher: {self.num_cameras} cameras, {self.out_w}x{self.out_h} output")
        return True
    
    def stitch(self, frames: List[np.ndarray]) -> np.ndarray:
        """Stitch multiple camera frames into 360 panorama
        
        Args:
            frames: list of (H, W, 3) BGR numpy arrays, one per camera
        Returns:
            (out_h, out_w, 3) equirectangular panoramic image
        """
        if not self._initialized:
            raise RuntimeError("Stitcher not initialized")
        if len(frames) != self.num_cameras:
            raise ValueError(f"Expected {self.num_cameras} frames, got {len(frames)}")
        
        panorama = np.zeros((self.out_h, self.out_w, 3), dtype=np.float64)
        weight = np.zeros((self.out_h, self.out_w, 1), dtype=np.float64)
        
        for cam_id, frame in enumerate(frames):
            warp = self._warp_maps[cam_id]
            mask = self._masks[cam_id]
            h, w = frame.shape[:2]
            sector_w = warp['sector_w']
            theta_s = warp['theta_start']
            
            # Map equirect coords to camera pixel coords
            px = warp['px']  # Normalized [-1, 1]
            py = warp['py']
            
            # Convert to pixel indices
            ix = ((px + 1) / 2 * (w - 1)).astype(np.int32)
            iy = ((py + 1) / 2 * (h - 1)).astype(np.int32)
            ix = np.clip(ix, 0, w - 1)
            iy = np.clip(iy, 0, h - 1)
            
            # Warp frame
            warped = frame[iy, ix]  # (out_h, sector_w, 3)
            
            # Place in panorama
            x_start = int(theta_s / (2 * np.pi) * self.out_w) % self.out_w
            x_end = x_start + sector_w
            if x_end <= self.out_w:
                panorama[:, x_start:x_end] += warped * mask[:, :, np.newaxis]
                weight[:, x_start:x_end] += mask[:, :, np.newaxis]
            else:
                # Wrap around
                w1 = self.out_w - x_start
                panorama[:, x_start:] += warped[:, :w1] * mask[:, :w1, np.newaxis]
                weight[:, x_start:] += mask[:, :w1, np.newaxis]
                panorama[:, :sector_w-w1] += warped[:, w1:] * mask[:, w1:, np.newaxis]
                weight[:, :sector_w-w1] += mask[:, w1:, np.newaxis]
        
        # Normalize by weight
        valid = weight > 0
        for c in range(3):
            panorama[:, :, c][valid.squeeze()] /= weight[valid].flatten()
        
        return np.clip(panorama, 0, 255).astype(np.uint8)
