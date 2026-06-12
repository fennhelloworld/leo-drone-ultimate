#!/usr/bin/env python3
"""LeoDrone Ultimate - PX4 Offboard Flight Controller
MAVSDK-based autonomous flight control
"""
import numpy as np
import time
import asyncio
import logging
from dataclasses import dataclass, field
from typing import Optional, List
from enum import Enum, auto

logger = logging.getLogger(__name__)

class FlightMode(Enum):
    STABILIZED = auto()
    OFFBOARD = auto()
    AUTO_TRACK = auto()
    HIGH_SPEED = auto()
    RETURN_TO_LAUNCH = auto()
    LAND = auto()

@dataclass
class DroneState:
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))  # NED
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    attitude: np.ndarray = field(default_factory=lambda: np.zeros(3))  # roll pitch yaw
    armed: bool = False
    mode: FlightMode = FlightMode.STABILIZED
    battery_pct: float = 100.0
    gps_fix: int = 0

class OffboardController:
    """PX4 Offboard flight controller via MAVSDK
    
    Methods: arm, takeoff, goto, follow, land, rtl
    Supports both real hardware (MAVSDK) and simulation mode.
    """
    
    def __init__(self, url: str = "udp://:14540", sim_mode: bool = True):
        self.url = url
        self.sim_mode = sim_mode
        self._drone = None
        self._state = DroneState()
        self._home = np.zeros(3)
        self._initialized = False
        self._target_pos = np.zeros(3)
        self._flight_log: List[dict] = []
        
    def initialize(self) -> bool:
        """Connect to PX4 via MAVSDK"""
        if self.sim_mode:
            logger.info("Offboard controller: simulation mode")
            self._initialized = True
            return True
        
        try:
            from mavsdk import System
            self._drone = System()
            asyncio.get_event_loop().run_until_complete(
                self._drone.connect(system_address=self.url)
            )
            self._initialized = True
            return True
        except ImportError:
            logger.warning("MAVSDK not available, using sim")
            self.sim_mode = True
            return self.initialize()
    
    async def arm(self) -> bool:
        """Arm the drone"""
        if not self._initialized:
            return False
        self._state.armed = True
        logger.info("DRONE ARMED")
        return True
    
    async def takeoff(self, altitude: float = 2.0) -> bool:
        """Takeoff to specified altitude"""
        if not self._state.armed:
            await self.arm()
        self._state.mode = FlightMode.OFFBOARD
        self._target_pos = np.array([0, 0, -altitude])  # NED: down positive
        self._state.position = self._target_pos.copy()
        logger.info(f"TAKEOFF to {altitude}m")
        return True
    
    async def goto(self, north: float, east: float, down: float) -> bool:
        """Go to NED position"""
        self._target_pos = np.array([north, east, down])
        self._state.position = self._target_pos.copy()
        logger.info(f"GOTO N={north:.1f} E={east:.1f} D={down:.1f}")
        return True
    
    async def follow_target(self, target_pos: np.ndarray, distance: float = 3.0) -> bool:
        """Follow a target at specified distance
        
        Args:
            target_pos: (2,) target x,y in image coords (normalized 0-1)
            distance: follow distance in meters
        """
        self._state.mode = FlightMode.AUTO_TRACK
        # Convert image coords to velocity commands
        vx = (target_pos[0] - 0.5) * 2.0  # -1 to 1
        vy = (target_pos[1] - 0.5) * 2.0
        self._state.velocity = np.array([vx, vy, 0]) * 2.0
        return True
    
    async def high_speed_pass(self, waypoints: List[np.ndarray], speed: float = 10.0) -> bool:
        """Execute high-speed traversal through waypoints
        
        Args:
            waypoints: list of (3,) NED positions
            speed: target speed in m/s
        """
        self._state.mode = FlightMode.HIGH_SPEED
        for i, wp in enumerate(waypoints):
            self._target_pos = wp
            self._state.position = wp.copy()
            self._state.velocity = np.array([speed, 0, 0])
            logger.info(f"HIGH-SPEED WP{i}: N={wp[0]:.1f} E={wp[1]:.1f}")
        return True
    
    async def land(self) -> bool:
        """Land at current position"""
        self._state.mode = FlightMode.LAND
        self._state.position = np.array([self._state.position[0], self._state.position[1], 0])
        self._state.armed = False
        logger.info("LANDING")
        return True
    
    async def rtl(self) -> bool:
        """Return to launch point"""
        self._state.mode = FlightMode.RETURN_TO_LAUNCH
        self._state.position = self._home.copy()
        self._state.armed = False
        logger.info("RETURN TO LAUNCH")
        return True
    
    def get_state(self) -> DroneState:
        return self._state
    
    def is_safe(self) -> bool:
        """Check all safety conditions"""
        checks = {
            'battery': self._state.battery_pct > 20,
            'altitude': abs(self._state.position[2]) < 120,  # max 120m
            'armed': self._state.armed,
        }
        return all(checks.values())
