import cv2
import threading
import time
import base64
import io
from PIL import Image

class CameraStream:
    def __init__(self, camera_index=0):
        self.camera_index = camera_index
        self.cap = None
        self.frame = None
        self.is_running = False
        self.thread = None
        self.frame_lock = threading.Lock()
        
    def start(self):
        if self.is_running:
            return
            
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera {self.camera_index}")
            
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 15)
        
        self.is_running = True
        self.thread = threading.Thread(target=self._update_frame)
        self.thread.daemon = True
        self.thread.start()
        
    def _update_frame(self):
        while self.is_running:
            ret, frame = self.cap.read()
            if ret:
                with self.frame_lock:
                    self.frame = frame
            time.sleep(0.033)
            
    def get_frame(self):
        with self.frame_lock:
            return self.frame.copy() if self.frame is not None else None
            
    def get_base64_frame(self, quality=85):
        frame = self.get_frame()
        if frame is None:
            return None
            
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pil_image = Image.fromarray(frame_rgb)
        
        buffer = io.BytesIO()
        pil_image.save(buffer, format='JPEG', quality=quality)
        img_str = base64.b64encode(buffer.getvalue()).decode('utf-8')
        return img_str
        
    def stop(self):
        self.is_running = False
        if self.thread:
            self.thread.join()
        if self.cap:
            self.cap.release()
