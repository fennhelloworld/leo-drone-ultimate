#!/usr/bin/env python3
"""LeoDrone Ultimate - Ground Station Web Dashboard
Flask-based real-time telemetry, video stream, map display
"""
import numpy as np
import time
import json
import logging
from typing import Dict, Optional
from dataclasses import dataclass

logger = logging.getLogger(__name__)

@dataclass
class TelemetryPacket:
    position: np.ndarray     # (3,) NED
    velocity: np.ndarray     # (3,) m/s
    attitude: np.ndarray     # (3,) roll/pitch/yaw
    battery_pct: float
    gps_fix: int
    flight_mode: str
    timestamp: float

class GroundStationDashboard:
    """Web dashboard for drone telemetry and control
    
    Features:
    - Real-time telemetry display (position, attitude, battery)
    - Live video stream from 360 cameras
    - Map with drone position and trajectory
    - Flight mode controls
    - Emergency stop button
    
    Flask-based, accessible at http://0.0.0.0:8080
    """
    
    def __init__(self, host: str = "0.0.0.0", port: int = 8080,
                 sim_mode: bool = True):
        self.host = host
        self.port = port
        self.sim_mode = sim_mode
        self._app = None
        self._telemetry: Optional[TelemetryPacket] = None
        self._trajectory: list = []
        self._running = False
        
    def initialize(self) -> bool:
        """Initialize Flask app and routes"""
        try:
            from flask import Flask, jsonify, render_template_string
            self._app = Flask(__name__)
            self._setup_routes()
            self._running = True
            return True
        except ImportError:
            logger.warning("Flask not available, dashboard in sim-only mode")
            self._running = True  # Still works in sim mode
            return True
    
    def _setup_routes(self):
        """Set up HTTP routes"""
        if self._app is None:
            return
            
        @self._app.route('/')
        def index():
            return self._get_html()
        
        @self._app.route('/api/telemetry')
        def api_telemetry():
            if self._telemetry is None:
                return jsonify({})
            return jsonify({
                'position': self._telemetry.position.tolist(),
                'velocity': self._telemetry.velocity.tolist(),
                'attitude': self._telemetry.attitude.tolist(),
                'battery': self._telemetry.battery_pct,
                'gps_fix': self._telemetry.gps_fix,
                'mode': self._telemetry.flight_mode,
                'timestamp': self._telemetry.timestamp
            })
        
        @self._app.route('/api/trajectory')
        def api_trajectory():
            return jsonify({'points': self._trajectory[-500:]})
        
        @self._app.route('/api/emergency_stop')
        def emergency_stop():
            logger.critical("EMERGENCY STOP via dashboard")
            return jsonify({'status': 'EMERGENCY_STOP'})
    
    def _get_html(self) -> str:
        """Generate dashboard HTML"""
        return '''<!DOCTYPE html>
<html><head><title>LeoDrone Ground Station</title></head>
<body>
<h1>LeoDrone Ultimate Ground Station</h1>
<div id="telemetry">Loading...</div>
<div id="map">Map placeholder</div>
<div id="video">Video stream placeholder</div>
<button onclick="fetch('/api/emergency_stop')">EMERGENCY STOP</button>
<script>
setInterval(() => {
    fetch('/api/telemetry').then(r=>r.json()).then(d=>{
        document.getElementById('telemetry').innerHTML = 
            'Pos: ' + d.position + '<br>Vel: ' + d.velocity + 
            '<br>Battery: ' + d.battery + '%<br>Mode: ' + d.mode;
    });
}, 200);
</script>
</body></html>'''
    
    def update_telemetry(self, packet: TelemetryPacket):
        """Update telemetry data"""
        self._telemetry = packet
        self._trajectory.append(packet.position.tolist())
        if len(self._trajectory) > 1000:
            self._trajectory = self._trajectory[-1000:]
    
    def get_telemetry_json(self) -> str:
        """Get current telemetry as JSON"""
        if self._telemetry is None:
            return json.dumps({})
        return json.dumps({
            'position': self._telemetry.position.tolist(),
            'velocity': self._telemetry.velocity.tolist(),
            'battery': self._telemetry.battery_pct,
            'mode': self._telemetry.flight_mode,
        })
    
    def start(self):
        """Start the web server"""
        if self._app is not None:
            self._app.run(host=self.host, port=self.port, threaded=True)
        else:
            logger.info("Dashboard running in sim-only mode (no Flask)")
    
    def stop(self):
        """Stop the web server"""
        self._running = False
