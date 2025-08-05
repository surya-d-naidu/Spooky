"""
Camera sensor module for visual input
"""

import cv2
import base64
import threading
import time
from typing import Dict, Any, Optional
import numpy as np

try:
    from ..core.interfaces import ISensorInterface, SensorData, DetectedObject
except ImportError:
    import sys
    import os
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from core.interfaces import ISensorInterface, SensorData, DetectedObject

class CameraSensor(ISensorInterface):
    """Camera sensor for visual input"""
    
    def __init__(self, camera_id: int = 0, width: int = 640, height: int = 480):
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.cap = None
        self.current_frame = None
        self.running = False
        self.thread = None
        self.frame_lock = threading.Lock()
        
        # Object detection (simple)
        self.face_cascade = None
        self.init_cv_detectors()
        
    def init_cv_detectors(self):
        """Initialize OpenCV detectors"""
        try:
            self.face_cascade = cv2.CascadeClassifier(
                cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
            )
        except:
            print("Warning: Could not load face detector")
    
    def initialize(self) -> bool:
        """Initialize camera"""
        try:
            self.cap = cv2.VideoCapture(self.camera_id)
            if not self.cap.isOpened():
                print(f"Failed to open camera {self.camera_id}")
                return False
                
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            
            # Start capture thread
            self.running = True
            self.thread = threading.Thread(target=self._capture_loop)
            self.thread.daemon = True
            self.thread.start()
            
            print(f"Camera {self.camera_id} initialized ({self.width}x{self.height})")
            return True
            
        except Exception as e:
            print(f"Camera initialization failed: {e}")
            return False
    
    def _capture_loop(self):
        """Continuous frame capture loop"""
        while self.running and self.cap:
            ret, frame = self.cap.read()
            if ret:
                with self.frame_lock:
                    self.current_frame = frame
            time.sleep(0.033)  # ~30 FPS
    
    def read(self) -> SensorData:
        """Read current frame and detect objects"""
        with self.frame_lock:
            if self.current_frame is None:
                return SensorData(
                    timestamp=time.time(),
                    sensor_type="camera",
                    data={"status": "no_frame"},
                    confidence=0.0
                )
            
            frame = self.current_frame.copy()
        
        # Encode frame for AI processing
        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        encoded_frame = base64.b64encode(buffer).decode('utf-8')
        
        # Simple object detection
        detected_objects = self._detect_objects(frame)
        
        # Analyze frame properties
        brightness = np.mean(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        motion_level = self._estimate_motion(frame)
        
        return SensorData(
            timestamp=time.time(),
            sensor_type="camera",
            data={
                "image": encoded_frame,
                "objects": detected_objects,
                "brightness": float(brightness),
                "motion_level": motion_level,
                "resolution": (self.width, self.height)
            },
            confidence=1.0 if len(detected_objects) > 0 else 0.8
        )
    
    def _detect_objects(self, frame) -> list:
        """Simple object detection"""
        objects = []
        
        if self.face_cascade is not None:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)
            
            for (x, y, w, h) in faces:
                objects.append({
                    'label': 'person',
                    'confidence': 0.8,
                    'bbox': (int(x), int(y), int(w), int(h)),
                    'center': (int(x + w/2), int(y + h/2))
                })
        
        return objects
    
    def _estimate_motion(self, frame) -> float:
        """Estimate motion in frame (placeholder)"""
        # Simple motion estimation based on frame changes
        # In a real implementation, you'd compare with previous frame
        return 0.5  # Placeholder
    
    def is_available(self) -> bool:
        """Check if camera is available"""
        return self.running and self.cap is not None and self.cap.isOpened()
    
    def cleanup(self):
        """Cleanup camera resources"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)
        if self.cap:
            self.cap.release()
        print("Camera sensor cleaned up")
    
    def get_latest_frame(self) -> Optional[np.ndarray]:
        """Get latest frame for display/debugging"""
        with self.frame_lock:
            return self.current_frame.copy() if self.current_frame is not None else None
