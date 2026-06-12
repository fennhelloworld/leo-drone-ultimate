#!/usr/bin/env python3
"""LeoDrone Ultimate — BME280 Thermo-Hygro-Baro Driver
I2C interface, calibration coefficients, compensated readings
"""
import numpy as np
import time
import logging
from typing import Dict, Optional
from dataclasses import dataclass

logger = logging.getLogger(__name__)

@dataclass
class BME280Reading:
    temperature: float   # °C
    humidity: float      # %RH
    pressure: float      # hPa
    altitude: float      # m (estimated from pressure)
    timestamp: float

class BME280Driver:
    """BME280 temperature/humidity/pressure sensor driver
    
    Supports real I2C hardware and simulation mode.
    Real mode requires smbus2 package.
    """
    
    # BME280 register addresses
    REG_CTRL_HUM = 0xF2
    REG_CTRL_MEAS = 0xF4
    REG_CONFIG = 0xF5
    REG_PRESSURE = 0xF7
    REG_TEMP = 0xFA
    REG_HUMIDITY = 0xFD
    REG_CHIP_ID = 0xD0
    CHIP_ID = 0x60
    
    # Oversampling settings
    OVERSAMPLING_1X = 1
    OVERSAMPLING_2X = 2
    OVERSAMPLING_4X = 3
    OVERSAMPLING_8X = 4
    OVERSAMPLING_16X = 5
    
    # Typical calibration coefficients (Bosch default)
    CALIB_DEFAULT = {
        'dig_T1': 28198, 'dig_T2': 26346, 'dig_T3': 50,
        'dig_P1': 37491, 'dig_P2': -10642, 'dig_P3': 3024,
        'dig_P4': 6821, 'dig_P5': 18, 'dig_P6': -82,
        'dig_P7': 89, 'dig_P8': 4916, 'dig_P9': -4170,
        'dig_H1': 75, 'dig_H2': 352, 'dig_H3': 0,
        'dig_H4': 329, 'dig_H5': 50, 'dig_H6': 30,
    }
    
    def __init__(self, address: int = 0x76, bus: int = 1, sim_mode: bool = True):
        self.address = address
        self.bus = bus
        self.sim_mode = sim_mode
        self._device = None
        self._calib = self.CALIB_DEFAULT.copy()
        self._t_fine = 0
        self._sea_level_hpa = 1013.25
        self._initialized = False
        
    def initialize(self) -> bool:
        """Initialize BME280 sensor"""
        if self.sim_mode:
            logger.info(f"BME280: simulation mode (addr=0x{self.address:02X})")
            self._initialized = True
            return True
            
        try:
            import smbus2
            self._device = smbus2.SMBus(self.bus)
            chip_id = self._device.read_byte_data(self.address, self.REG_CHIP_ID)
            if chip_id != self.CHIP_ID:
                logger.error(f"BME280: unexpected chip ID 0x{chip_id:02X}")
                return False
            self._read_calibration()
            self._configure()
            self._initialized = True
            logger.info(f"BME280: initialized on bus {self.bus}")
            return True
        except ImportError:
            logger.warning("smbus2 not available, using sim mode")
            self.sim_mode = True
            return self.initialize()
        except Exception as e:
            logger.error(f"BME280 init error: {e}")
            self.sim_mode = True
            return self.initialize()
    
    def _read_calibration(self):
        """Read calibration data from sensor (real hardware only)"""
        # Read T1-T3
        data = self._device.read_i2c_block_data(self.address, 0x88, 6)
        self._calib['dig_T1'] = data[0] | (data[1] << 8)
        self._calib['dig_T2'] = self._to_signed(data[2] | (data[3] << 8))
        self._calib['dig_T3'] = self._to_signed(data[4] | (data[5] << 8))
        
    def _configure(self):
        """Configure oversampling and filter settings"""
        if self._device is None:
            return
        # Humidity oversampling x1
        self._device.write_byte_data(self.address, self.REG_CTRL_HUM, self.OVERSAMPLING_1X)
        # Temp+Pressure oversampling x4, normal mode
        ctrl_meas = (self.OVERSAMPLING_4X << 5) | (self.OVERSAMPLING_4X << 2) | 3
        self._device.write_byte_data(self.address, self.REG_CTRL_MEAS, ctrl_meas)
        # Filter coefficient 4, standby 125ms
        config = (4 << 2) | (2 << 5)
        self._device.write_byte_data(self.address, self.REG_CONFIG, config)
    
    @staticmethod
    def _to_signed(val: int) -> int:
        return val - 65536 if val >= 32768 else val
    
    def read(self) -> BME280Reading:
        """Read compensated temperature, humidity, pressure
        
        Returns:
            BME280Reading with all values compensated
        """
        if not self._initialized:
            raise RuntimeError("BME280 not initialized")
        
        t = time.time()
        
        if self.sim_mode:
            # Simulate realistic BME280 readings with drift
            temp = 25.0 + np.sin(t * 0.01) * 2 + np.random.randn() * 0.1
            hum = 55.0 + np.sin(t * 0.005) * 5 + np.random.randn() * 0.5
            press = 1013.25 + np.sin(t * 0.001) * 2 + np.random.randn() * 0.1
            alt = 44330 * (1 - (press / self._sea_level_hpa) ** (1/5.255))
            return BME280Reading(
                temperature=round(temp, 2),
                humidity=round(hum, 2),
                pressure=round(press, 2),
                altitude=round(alt, 1),
                timestamp=t
            )
        
        # Real hardware reading with compensation
        data = self._device.read_i2c_block_data(self.address, self.REG_PRESSURE, 8)
        adc_p = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        adc_t = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        adc_h = (data[6] << 8) | data[7]
        
        temp = self._compensate_temp(adc_t)
        press = self._compensate_pressure(adc_p) / 100.0
        hum = self._compensate_humidity(adc_h)
        alt = 44330 * (1 - (press / self._sea_level_hpa) ** (1/5.255))
        
        return BME280Reading(
            temperature=round(temp, 2), humidity=round(hum, 2),
            pressure=round(press, 2), altitude=round(alt, 1),
            timestamp=t
        )
    
    def _compensate_temp(self, adc_t: int) -> float:
        """Compensate temperature reading using calibration data"""
        c = self._calib
        var1 = ((adc_t / 16384.0) - (c['dig_T1'] / 1024.0)) * c['dig_T2']
        var2 = ((adc_t / 131072.0) - (c['dig_T1'] / 8192.0)) ** 2 * c['dig_T3']
        self._t_fine = var1 + var2
        return (self._t_fine * 5 + 128) / 25600.0
    
    def _compensate_pressure(self, adc_p: int) -> float:
        """Compensate pressure reading"""
        c = self._calib
        var1 = self._t_fine / 2.0 - 64000.0
        var2 = var1 * var1 * c['dig_P6'] / 32768.0
        var2 = var2 + var1 * c['dig_P5'] * 2.0
        var2 = var2 / 4.0 + c['dig_P4'] * 65536.0
        var1 = (c['dig_P3'] * var1 * var1 / 524288.0 + c['dig_P2'] * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * c['dig_P1']
        if var1 == 0:
            return 0
        p = 1048576.0 - adc_p
        p = (p - var2 / 4096.0) * 6250.0 / var1
        var1 = c['dig_P9'] * p * p / 2147483648.0
        var2 = p * c['dig_P8'] / 32768.0
        return p + (var1 + var2 + c['dig_P7']) / 16.0
    
    def _compensate_humidity(self, adc_h: int) -> float:
        """Compensate humidity reading"""
        c = self._calib
        var = self._t_fine - 76800.0
        var = (adc_h - (c['dig_H4'] * 64.0 + c['dig_H5'] / 16384.0 * var))
        var = var * (c['dig_H2'] / 65536.0 * (1.0 + c['dig_H6'] / 67108864.0 * var *
               (1.0 + c['dig_H3'] / 67108864.0 * var)))
        return max(0, min(100, var))
    
    def shutdown(self):
        """Release sensor resources"""
        if self._device is not None:
            self._device.close()
        self._initialized = False
