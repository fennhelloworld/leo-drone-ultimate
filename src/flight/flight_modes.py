#!/usr/bin/env python3
"""LeoDrone Ultimate - Flight Mode Manager
State machine for flight mode transitions
"""
import logging
from enum import Enum, auto
from typing import Optional

logger = logging.getLogger(__name__)

class FlightMode(Enum):
    DISARMED = auto()
    STABILIZED = auto()
    OFFBOARD = auto()
    AUTO_TRACK = auto()
    HIGH_SPEED = auto()
    RETURN_TO_LAUNCH = auto()
    LAND = auto()
    EMERGENCY = auto()

# Valid transitions
TRANSITIONS = {
    FlightMode.DISARMED: [FlightMode.STABILIZED],
    FlightMode.STABILIZED: [FlightMode.OFFBOARD, FlightMode.RETURN_TO_LAUNCH, FlightMode.DISARMED],
    FlightMode.OFFBOARD: [FlightMode.AUTO_TRACK, FlightMode.HIGH_SPEED, FlightMode.LAND, FlightMode.RETURN_TO_LAUNCH, FlightMode.STABILIZED],
    FlightMode.AUTO_TRACK: [FlightMode.OFFBOARD, FlightMode.LAND, FlightMode.RETURN_TO_LAUNCH],
    FlightMode.HIGH_SPEED: [FlightMode.OFFBOARD, FlightMode.LAND, FlightMode.RETURN_TO_LAUNCH],
    FlightMode.RETURN_TO_LAUNCH: [FlightMode.LAND, FlightMode.OFFBOARD],
    FlightMode.LAND: [FlightMode.DISARMED],
    FlightMode.EMERGENCY: [FlightMode.LAND],  # Emergency can only land
}

class FlightModeManager:
    """Flight mode state machine with safety guards"""
    
    def __init__(self):
        self._mode = FlightMode.DISARMED
        self._mode_history = []
        
    def transition(self, target: FlightMode) -> bool:
        """Request mode transition"""
        if target not in TRANSITIONS.get(self._mode, []):
            logger.warning(f"Invalid transition: {self._mode.name} -> {target.name}")
            return False
        old = self._mode
        self._mode = target
        self._mode_history.append((old, target))
        logger.info(f"Mode: {old.name} -> {target.name}")
        return True
    
    def force_emergency(self):
        """Force emergency mode"""
        self._mode = FlightMode.EMERGENCY
        logger.critical("EMERGENCY MODE ACTIVATED")
    
    @property
    def mode(self) -> FlightMode:
        return self._mode
    
    @property
    def can_fly(self) -> bool:
        return self._mode in (FlightMode.OFFBOARD, FlightMode.AUTO_TRACK, 
                              FlightMode.HIGH_SPEED, FlightMode.STABILIZED)
