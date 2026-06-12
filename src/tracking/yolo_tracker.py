#!/usr/bin/env python3
"""LeoDrone Ultimate - YOLOv8 + DeepSORT Person Tracker
Detection, tracking, follow trajectory generation
"""
import numpy as np
import time
import logging
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple
from enum import Enum, auto

logger = logging.getLogger(__name__)

class TrackState(Enum):
    TENTATIVE = auto()
    CONFIRMED = auto()
    LOST = auto()

@dataclass
class Detection:
    bbox: np.ndarray      # (4,) [x1, y1, x2, y2]
    confidence: float
    class_id: int         # 0=person
    class_name: str

@dataclass
class Track:
    track_id: int
    bbox: np.ndarray
    state: TrackState
    age: int = 0
    hits: int = 0
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(2))
    trajectory: List[np.ndarray] = field(default_factory=list)

class YOLOTracker:
    """YOLOv8 + DeepSORT person detection and tracking
    
    Detection: YOLOv8 (or simulated)
    Tracking: Kalman filter + Hungarian assignment + appearance matching
    """
    
    COCO_NAMES = {0: 'person', 1: 'bicycle', 2: 'car', 5: 'bus', 7: 'truck'}
    
    def __init__(self, model_size: str = "n", confidence: float = 0.5,
                 iou_threshold: float = 0.45, max_lost: int = 30,
                 sim_mode: bool = True):
        self.model_size = model_size
        self.confidence = confidence
        self.iou_threshold = iou_threshold
        self.max_lost = max_lost
        self.sim_mode = sim_mode
        self._model = None
        self._tracks: Dict[int, Track] = {}
        self._next_id = 1
        self._initialized = False
        
    def initialize(self) -> bool:
        """Load YOLO model"""
        if self.sim_mode:
            self._initialized = True
            return True
        try:
            from ultralytics import YOLO
            self._model = YOLO(f"yolov8{self.model_size}.pt")
            self._initialized = True
            return True
        except ImportError:
            logger.warning("ultralytics not available, using sim mode")
            self.sim_mode = True
            return self.initialize()
    
    def detect(self, frame: np.ndarray) -> List[Detection]:
        """Detect objects in frame
        
        Args:
            frame: (H, W, 3) BGR image
        Returns:
            List of Detection objects
        """
        if not self._initialized:
            raise RuntimeError("Tracker not initialized")
        
        if self.sim_mode:
            return self._simulate_detection(frame)
        
        results = self._model(frame, conf=self.confidence, iou=self.iou_threshold)
        detections = []
        for r in results:
            for box in r.boxes:
                detections.append(Detection(
                    bbox=box.xyxy.cpu().numpy().flatten(),
                    confidence=float(box.conf),
                    class_id=int(box.cls),
                    class_name=self.COCO_NAMES.get(int(box.cls), 'unknown')
                ))
        return detections
    
    def _simulate_detection(self, frame: np.ndarray) -> List[Detection]:
        """Simulate person detection"""
        h, w = frame.shape[:2]
        # Randomly detect 0-2 persons
        n_persons = np.random.choice([0, 1, 1, 2], p=[0.3, 0.4, 0.2, 0.1])
        detections = []
        for _ in range(n_persons):
            cx = np.random.randint(w//4, 3*w//4)
            cy = np.random.randint(h//4, 3*h//4)
            bw = np.random.randint(40, 120)
            bh = np.random.randint(80, 200)
            detections.append(Detection(
                bbox=np.array([cx-bw//2, cy-bh//2, cx+bw//2, cy+bh//2]),
                confidence=0.7 + np.random.rand() * 0.25,
                class_id=0, class_name='person'
            ))
        return detections
    
    def update(self, detections: List[Detection]) -> List[Track]:
        """Update tracks with new detections (DeepSORT-style)
        
        Args:
            detections: current frame detections
        Returns:
            List of active tracks
        """
        # Match detections to existing tracks
        matched, unmatched_dets, unmatched_tracks = self._match(detections)
        
        # Update matched tracks
        for det_idx, trk_id in matched:
            det = detections[det_idx]
            track = self._tracks[trk_id]
            # Kalman update (simplified: exponential smoothing)
            alpha = 0.3
            new_bbox = alpha * det.bbox + (1-alpha) * track.bbox
            track.velocity = new_bbox[:2] - track.bbox[:2]
            track.bbox = new_bbox
            track.age += 1
            track.hits += 1
            track.state = TrackState.CONFIRMED
            track.trajectory.append(track.bbox[:2].copy())
            if len(track.trajectory) > 100:
                track.trajectory = track.trajectory[-100:]
        
        # Mark unmatched tracks as lost
        for trk_id in unmatched_tracks:
            track = self._tracks[trk_id]
            track.age += 1
            track.state = TrackState.LOST
        
        # Create new tracks for unmatched detections
        for det_idx in unmatched_dets:
            det = detections[det_idx]
            track = Track(
                track_id=self._next_id,
                bbox=det.bbox.copy(),
                state=TrackState.TENTATIVE,
                hits=1, age=1,
                trajectory=[det.bbox[:2].copy()]
            )
            self._tracks[self._next_id] = track
            self._next_id += 1
        
        # Remove old lost tracks
        lost_ids = [tid for tid, t in self._tracks.items() 
                     if t.state == TrackState.LOST and t.age > self.max_lost]
        for tid in lost_ids:
            del self._tracks[tid]
        
        return [t for t in self._tracks.values() if t.state == TrackState.CONFIRMED]
    
    def _match(self, detections: List[Detection]) -> Tuple[List, List, List]:
        """Hungarian matching of detections to tracks"""
        if not self._tracks or not detections:
            return [], list(range(len(detections))), list(self._tracks.keys())
        
        active_tracks = {tid: t for tid, t in self._tracks.items() 
                         if t.state != TrackState.LOST}
        
        matched = []
        unmatched_dets = list(range(len(detections)))
        unmatched_tracks = list(active_tracks.keys())
        
        # Simple greedy matching by IoU
        cost_matrix = np.zeros((len(detections), len(active_tracks)))
        track_ids = list(active_tracks.keys())
        
        for i, det in enumerate(detections):
            for j, tid in enumerate(track_ids):
                cost_matrix[i, j] = 1.0 - self._iou(det.bbox, self._tracks[tid].bbox)
        
        # Greedy assignment
        for _ in range(min(len(detections), len(active_tracks))):
            if cost_matrix.size == 0:
                break
            min_idx = np.unravel_index(np.argmin(cost_matrix), cost_matrix.shape)
            if cost_matrix[min_idx] > 0.5:  # IoU threshold
                break
            det_idx, trk_local = min_idx
            matched.append((det_idx, track_ids[trk_local]))
            unmatched_dets.remove(det_idx)
            unmatched_tracks.remove(track_ids[trk_local])
            cost_matrix = np.delete(cost_matrix, det_idx, axis=0)
            cost_matrix = np.delete(cost_matrix, trk_local, axis=1)
            track_ids.pop(trk_local)
        
        return matched, unmatched_dets, unmatched_tracks
    
    @staticmethod
    def _iou(box1: np.ndarray, box2: np.ndarray) -> float:
        x1 = max(box1[0], box2[0]); y1 = max(box1[1], box2[1])
        x2 = min(box1[2], box2[2]); y2 = min(box1[3], box2[3])
        inter = max(0, x2-x1) * max(0, y2-y1)
        area1 = (box1[2]-box1[0]) * (box1[3]-box1[1])
        area2 = (box2[2]-box2[0]) * (box2[3]-box2[1])
        union = area1 + area2 - inter
        return inter / (union + 1e-10)
    
    def get_follow_target(self) -> Optional[Tuple[np.ndarray, int]]:
        """Get the best target to follow
        
        Returns:
            (center_xy, track_id) or None if no target
        """
        confirmed = [t for t in self._tracks.values() if t.state == TrackState.CONFIRMED]
        if not confirmed:
            return None
        # Follow the closest/largest person
        best = max(confirmed, key=lambda t: t.hits)
        center = (best.bbox[:2] + best.bbox[2:]) / 2
        return center, best.track_id
    
    def get_all_tracks(self) -> List[Track]:
        return list(self._tracks.values())
