#!/usr/bin/env python3
"""LeoDrone Ultimate — WiFi + MIC Activity & Noise Detector
Audio-based activity detection, noise level monitoring, WiFi CSI motion detection
"""
import numpy as np
import time
import logging
from typing import Optional, Dict, List, Tuple
from dataclasses import dataclass
from enum import Enum, auto

logger = logging.getLogger(__name__)

class ActivityType(Enum):
    SILENCE = auto()
    AMBIENT = auto()
    VOICE = auto()
    NOISE = auto()
    MOTION = auto()

@dataclass
class AudioEvent:
    """Detected audio/motion event"""
    timestamp: float
    activity_type: ActivityType
    level_db: float
    duration_s: float
    confidence: float
    metadata: Dict = None

class AudioDetector:
    """Microphone-based activity and noise detector
    
    Pipeline: MIC → VAD → Sound Level (dB) → Classification
    """
    
    def __init__(self, sample_rate: int = 16000, channels: int = 1,
                 noise_threshold_db: float = 60.0,
                 activity_threshold_db: float = 45.0,
                 sim_mode: bool = True):
        self.sample_rate = sample_rate
        self.channels = channels
        self.noise_threshold_db = noise_threshold_db
        self.activity_threshold_db = activity_threshold_db
        self.sim_mode = sim_mode
        self._stream = None
        self._running = False
        self._event_history: List[AudioEvent] = []
        
    def initialize(self) -> bool:
        """Initialize audio capture"""
        if self.sim_mode:
            logger.info("Audio detector: simulation mode")
            self._running = True
            return True
            
        try:
            import pyaudio
            self._pa = pyaudio.PyAudio()
            self._stream = self._pa.open(
                format=pyaudio.paInt16,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=1024
            )
            self._running = True
            logger.info("Audio detector: real hardware mode")
            return True
        except ImportError:
            logger.warning("pyaudio not available, using sim mode")
            self.sim_mode = True
            return self.initialize()
    
    def read_level_db(self) -> float:
        """Read current sound level in dB"""
        if self.sim_mode:
            # Simulate ambient noise with occasional spikes
            base = 35.0 + np.random.randn() * 5
            if np.random.random() < 0.05:  # 5% chance of noise event
                base += np.random.uniform(20, 40)
            return max(0, base)
        
        if not self._running or self._stream is None:
            return 0.0
            
        try:
            data = self._stream.read(1024, exception_on_overflow=False)
            samples = np.frombuffer(data, dtype=np.int16).astype(np.float64)
            rms = np.sqrt(np.mean(samples**2))
            if rms < 1e-10:
                return 0.0
            return 20 * np.log10(rms / 32767.0) + 94  # dB SPL approximation
        except Exception as e:
            logger.error(f"Audio read error: {e}")
            return 0.0
    
    def detect_activity(self) -> AudioEvent:
        """Detect current audio activity type"""
        level = self.read_level_db()
        t = time.time()
        
        if level > self.noise_threshold_db:
            activity = ActivityType.NOISE
            confidence = min(1.0, (level - self.noise_threshold_db) / 20.0)
        elif level > self.activity_threshold_db:
            # Could be voice or general activity
            activity = ActivityType.VOICE if level > 55 else ActivityType.AMBIENT
            confidence = 0.7
        elif level > 30:
            activity = ActivityType.AMBIENT
            confidence = 0.5
        else:
            activity = ActivityType.SILENCE
            confidence = 0.9
            
        event = AudioEvent(
            timestamp=t, activity_type=activity,
            level_db=level, duration_s=0.1,
            confidence=confidence
        )
        self._event_history.append(event)
        return event
    
    def get_noise_history(self, last_n: int = 100) -> List[AudioEvent]:
        """Get recent noise detection history"""
        return self._event_history[-last_n:]

class WiFiMotionDetector:
    """WiFi CSI (Channel State Information) based motion detection
    
    Uses WiFi signal fluctuations to detect movement without cameras.
    Works with any WiFi adapter that supports CSI extraction.
    """
    
    def __init__(self, interface: str = "wlan0", threshold: float = 0.3,
                 sim_mode: bool = True):
        self.interface = interface
        self.threshold = threshold
        self.sim_mode = sim_mode
        self._baseline_csi = None
        self._running = False
        self._motion_history: List[AudioEvent] = []
        
    def initialize(self) -> bool:
        """Initialize WiFi CSI monitoring"""
        if self.sim_mode:
            # Generate synthetic baseline CSI
            self._baseline_csi = np.random.randn(3, 56) * 0.1 + 1.0
            self._running = True
            logger.info("WiFi motion detector: simulation mode")
            return True
        
        # Real CSI extraction would require linux-80211n-csitool
        logger.warning("WiFi CSI requires linux-80211n-csitool, using sim")
        self.sim_mode = True
        return self.initialize()
    
    def detect_motion(self) -> Optional[AudioEvent]:
        """Detect motion via WiFi CSI perturbation
        
        Returns AudioEvent with MOTION type if movement detected
        """
        if not self._running:
            return None
            
        if self.sim_mode:
            # Simulate CSI perturbation
            current_csi = self._baseline_csi + np.random.randn(3, 56) * 0.05
            # Random motion event with 3% probability
            if np.random.random() < 0.03:
                current_csi += np.random.randn(3, 56) * 0.3
            
            perturbation = np.mean(np.abs(current_csi - self._baseline_csi))
            detected = perturbation > self.threshold
            
            if detected:
                event = AudioEvent(
                    timestamp=time.time(),
                    activity_type=ActivityType.MOTION,
                    level_db=0, duration_s=0.5,
                    confidence=min(1.0, perturbation / self.threshold),
                    metadata={'perturbation': float(perturbation)}
                )
                self._motion_history.append(event)
                return event
        
        return None
    
    def get_motion_history(self, last_n: int = 50) -> List[AudioEvent]:
        return self._motion_history[-last_n:]

class IntegratedDetector:
    """Combined audio + WiFi motion detection"""
    
    def __init__(self, sim_mode: bool = True):
        self.sim_mode = sim_mode
        self.audio = AudioDetector(sim_mode=sim_mode)
        self.wifi = WiFiMotionDetector(sim_mode=sim_mode)
        
    def initialize(self) -> bool:
        a = self.audio.initialize()
        w = self.wifi.initialize()
        return a and w
    
    def detect_all(self) -> Dict:
        """Run all detection modalities
        
        Returns dict with 'audio' event and optional 'motion' event
        """
        audio_event = self.audio.detect_activity()
        motion_event = self.wifi.detect_motion()
        
        result = {'audio': audio_event, 'motion': None}
        if motion_event is not None:
            result['motion'] = motion_event
            
        return result
    
    def get_composite_alert_level(self) -> str:
        """Compute composite alert level from all sensors"""
        audio = self.audio.detect_activity()
        motion = self.wifi.detect_motion()
        
        if audio.activity_type == ActivityType.NOISE:
            return "HIGH"
        if motion is not None and motion.confidence > 0.7:
            return "MEDIUM"
        if audio.activity_type == ActivityType.VOICE:
            return "LOW"
        return "CLEAR"
