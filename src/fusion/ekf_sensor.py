#!/usr/bin/env python3
"""LeoDrone Ultimate - Extended Kalman Filter 12-state Sensor Fusion
State: [pos(3), vel(3), att(3), gyro_bias(3)]
Observations: IMU(6) + GPS(3) + Baro(1)
"""
import numpy as np
import logging
from dataclasses import dataclass

logger = logging.getLogger(__name__)

class EKFSensorFusion:
    """12-state Extended Kalman Filter for IMU+GPS+Baro fusion
    
    State vector: [px, py, pz, vx, vy, vz, roll, pitch, yaw, bg_x, bg_y, bg_z]
    - Position (3): NED frame
    - Velocity (3): m/s
    - Attitude (3): Euler angles (rad)
    - Gyro bias (3): rad/s
    
    Observation models:
    - IMU: provides accel (3) and gyro (3) as control inputs
    - GPS: provides position (3) at 5-10Hz
    - Baro: provides altitude (1) at 25Hz
    """
    
    def __init__(self, gravity: float = 9.81, imu_rate: float = 200.0,
                 gps_rate: float = 5.0):
        self.gravity = gravity
        self.imu_dt = 1.0 / imu_rate
        self.gps_dt = 1.0 / gps_rate
        
        # State and covariance
        self.x = np.zeros(12)
        self.P = np.eye(12) * 0.1
        
        # Process noise
        self.Q = np.eye(12) * 0.001
        self.Q[0:3, 0:3] *= 0.01   # position noise
        self.Q[3:6, 3:6] *= 0.1    # velocity noise
        self.Q[6:9, 6:9] *= 0.01   # attitude noise
        self.Q[9:12, 9:12] *= 0.001 # bias noise
        
        # Observation noise
        self.R_gps = np.eye(3) * 2.0       # GPS position noise (m^2)
        self.R_baro = np.eye(1) * 1.0       # Baro altitude noise (m^2)
        
        self._initialized = False
        
    def initialize(self, position: np.ndarray = None, attitude: np.ndarray = None):
        """Initialize EKF with known state"""
        if position is not None:
            self.x[0:3] = position
        if attitude is not None:
            self.x[6:9] = attitude
        self._initialized = True
        
    def predict(self, accel: np.ndarray, gyro: np.ndarray):
        """EKF predict step using IMU measurements
        
        Args:
            accel: (3,) m/s^2 in body frame
            gyro: (3,) rad/s in body frame
        """
        if not self._initialized:
            return
            
        dt = self.imu_dt
        
        # Current state
        pos = self.x[0:3]
        vel = self.x[3:6]
        att = self.x[6:9]  # roll, pitch, yaw
        bg = self.x[9:12]  # gyro bias
        
        # Corrected gyro
        gyro_corrected = gyro - bg
        
        # Rotation matrix (body to world)
        R = self._euler_to_rotation(att)
        
        # Acceleration in world frame
        accel_world = R @ accel - np.array([0, 0, self.gravity])
        
        # State transition
        new_pos = pos + vel * dt + 0.5 * accel_world * dt**2
        new_vel = vel + accel_world * dt
        new_att = att + gyro_corrected * dt
        new_bg = bg  # Bias is constant (random walk)
        
        self.x = np.concatenate([new_pos, new_vel, new_att, new_bg])
        
        # Jacobian of state transition
        F = np.eye(12)
        F[0:3, 3:6] = np.eye(3) * dt
        F[3:6, 6:9] = -R @ self._skew(accel) * dt  # attitude affects velocity
        F[3:6, 0:3] = np.zeros(3)  # pos doesn't affect vel directly
        F[6:9, 9:12] = -np.eye(3) * dt  # bias affects attitude
        
        # Covariance prediction
        self.P = F @ self.P @ F.T + self.Q
        
    def update_gps(self, gps_pos: np.ndarray):
        """EKF update with GPS position measurement
        
        Args:
            gps_pos: (3,) NED position in meters
        """
        if not self._initialized:
            return
            
        H = np.zeros((3, 12))
        H[0:3, 0:3] = np.eye(3)
        
        # Innovation
        y = gps_pos - self.x[0:3]
        S = H @ self.P @ H.T + self.R_gps
        K = self.P @ H.T @ np.linalg.inv(S)
        
        self.x = self.x + K @ y
        self.P = (np.eye(12) - K @ H) @ self.P
        
    def update_baro(self, altitude: float):
        """EKF update with barometric altitude
        
        Args:
            altitude: altitude in meters (positive up)
        """
        if not self._initialized:
            return
            
        H = np.zeros((1, 12))
        H[0, 2] = -1.0  # NED: down is positive
        
        y = np.array([-altitude]) - self.x[2:3]
        S = H @ self.P @ H.T + self.R_baro
        K = self.P @ H.T @ np.linalg.inv(S)
        
        self.x = self.x + (K @ y).flatten()
        self.P = (np.eye(12) - K @ H) @ self.P
    
    def get_state(self) -> dict:
        """Get current state estimate"""
        return {
            'position': self.x[0:3].copy(),
            'velocity': self.x[3:6].copy(),
            'attitude': self.x[6:9].copy(),
            'gyro_bias': self.x[9:12].copy(),
            'covariance': self.P.copy()
        }
    
    @staticmethod
    def _euler_to_rotation(euler):
        """Convert Euler angles (roll, pitch, yaw) to rotation matrix"""
        r, p, y = euler
        cr, sr = np.cos(r), np.sin(r)
        cp, sp = np.cos(p), np.sin(p)
        cy, sy = np.cos(y), np.sin(y)
        return np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr]
        ])
    
    @staticmethod
    def _skew(v):
        return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
