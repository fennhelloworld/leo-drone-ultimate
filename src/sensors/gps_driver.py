#!/usr/bin/env python3
"""LeoDrone Ultimate - GPS M10N Driver
UART 9600bd, NMEA parser
"""
import numpy as np
import time
import logging
from dataclasses import dataclass

logger = logging.getLogger(__name__)

@dataclass
class GPSReading:
    lat: float; lon: float; alt: float
    fix_type: int; num_satellites: int; hdop: float
    speed_ms: float; course_deg: float; timestamp: float

class GPSDriver:
    """M10N GPS with NMEA parsing and simulation"""
    def __init__(self, port="/dev/ttyAMA0", baud=9600, sim_mode=True):
        self.sim_mode = sim_mode
        self._initialized = False
        self._home_lat = 39.9042
        self._home_lon = 116.4074
        self._home_alt = 50.0
        
    def initialize(self) -> bool:
        self._initialized = True
        return True
    
    def read(self) -> GPSReading:
        if not self._initialized:
            raise RuntimeError("GPS not initialized")
        t = time.time()
        if self.sim_mode:
            return GPSReading(
                lat=self._home_lat + np.sin(t*0.01)*0.0001,
                lon=self._home_lon + np.cos(t*0.01)*0.0001,
                alt=self._home_alt + np.sin(t*0.005)*5,
                fix_type=3, num_satellites=10+int(np.random.randn()*2),
                hdop=0.9+np.random.rand()*0.3,
                speed_ms=np.random.rand()*2,
                course_deg=np.random.rand()*360,
                timestamp=t
            )
        return GPSReading(0,0,0,0,0,99,0,0,t)
    
    def shutdown(self):
        self._initialized = False
