#!/usr/bin/env python3
"""LeoDrone Ultimate - ICM-42688-P IMU Driver
SPI interface, 200Hz, accel 16g, gyro 2000dps
"""
import numpy as np
import time
import logging
from dataclasses import dataclass

logger = logging.getLogger(__name__)

@dataclass
class IMUReading:
    accel: np.ndarray    # (3,) m/s^2
    gyro: np.ndarray     # (3,) rad/s
    temperature: float   # C
    timestamp: float

class ICM42688Driver:
    """ICM-42688-P 6-axis IMU driver"""
    def __init__(self, spi_bus=0, spi_cs=0, accel_range=16, gyro_range=2000, sim_mode=True):
        self.accel_range = accel_range
        self.gyro_range = gyro_range
        self.sim_mode = sim_mode
        self._initialized = False
        self._accel_scale = accel_range * 9.81 / 32768.0
        self._gyro_scale = np.radians(gyro_range) / 32768.0
        
    def initialize(self) -> bool:
        if self.sim_mode:
            self._initialized = True
            return True
        try:
            import spidev
            self._spi = spidev.SpiDev(spi_bus, spi_cs)
            self._spi.mode = 3
            self._initialized = True
            return True
        except ImportError:
            self.sim_mode = True
            return self.initialize()
    
    def read(self) -> IMUReading:
        if not self._initialized:
            raise RuntimeError("ICM-42688-P not initialized")
        t = time.time()
        if self.sim_mode:
            accel = np.array([0.0, 0.0, 9.81]) + np.random.randn(3) * 0.05
            gyro = np.random.randn(3) * 0.01
            temp = 25.0 + np.random.randn() * 0.5
        else:
            accel = np.zeros(3)  # placeholder for real read
            gyro = np.zeros(3)
            temp = 25.0
        return IMUReading(accel=accel, gyro=gyro, temperature=temp, timestamp=t)
    
    def shutdown(self):
        self._initialized = False
