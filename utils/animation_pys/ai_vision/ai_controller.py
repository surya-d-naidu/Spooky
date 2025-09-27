import time
import threading
import logging
from typing import Optional, Dict, Any, Callable
from .camera_stream import CameraStream
from .ollama_client import OllamaAI
from .dog_personality import DogPersonality

logging.basicConfig(level=logging.INFO)

class AIVisionController:
    def __init__(self, animation_controller, ollama_host="localhost", camera_index=0):
        self.animation_controller = animation_controller
        self.camera = CameraStream(camera_index)
        self.ai_client = OllamaAI(host=ollama_host)
        self.personality = DogPersonality()
        
        self.is_running = False
        self.analysis_thread = None
        self.current_action_thread = None
        self.logger = logging.getLogger(__name__)
        
        self.analysis_interval = 3.0
        self.last_analysis_time = 0
        self.last_action_start_time = 0
        self.current_action_duration = 0
        
        self.action_map = {
            "walk": self.animation_controller.start_animation,
            "sit": lambda: self.animation_controller.start_animation("sit"),
            "stand": self.animation_controller.set_idle_position,
            "turn_left": lambda: self.animation_controller.start_animation("rotate_left"),
            "turn_right": lambda: self.animation_controller.start_reverse_animation("rotate_left"),
            "walk_backward": lambda: self.animation_controller.start_reverse_animation("walk_forward"),
            "say_hi": lambda: self.animation_controller.start_animation("hi"),
            "end_walk": self._end_walk_action
        }
    
    def start(self):
        if self.is_running:
            return
            
        try:
            self.camera.start()
            self.is_running = True
            self.analysis_thread = threading.Thread(target=self._analysis_loop)
            self.analysis_thread.daemon = True
            self.analysis_thread.start()
            self.logger.info("AI Vision Controller started")
        except Exception as e:
            self.logger.error(f"Failed to start AI Vision Controller: {str(e)}")
            self.stop()
    
    def stop(self):
        self.is_running = False
        if self.analysis_thread:
            self.analysis_thread.join(timeout=5)
        if self.current_action_thread:
            self.current_action_thread.join(timeout=2)
        self.camera.stop()
        self.logger.info("AI Vision Controller stopped")
    
    def _analysis_loop(self):
        while self.is_running:
            try:
                current_time = time.time()
                
                if current_time - self.last_analysis_time >= self.analysis_interval:
                    self._perform_analysis()
                    self.last_analysis_time = current_time
                
                if self._should_stop_current_action():
                    self._handle_action_completion()
                
                time.sleep(0.5)
                
            except Exception as e:
                self.logger.error(f"Error in analysis loop: {str(e)}")
                time.sleep(1)
    
    def _perform_analysis(self):
        frame_b64 = self.camera.get_base64_frame()
        if not frame_b64:
            self.logger.warning("No camera frame available")
            self._handle_no_input()
            return
        
        self.logger.info("Analyzing camera frame...")
        ai_response = self.ai_client.analyze_image(frame_b64)
        
        if ai_response:
            processed_response = self.personality.process_ai_response(ai_response)
            self._execute_response(processed_response)
        else:
            self.logger.warning("No AI response received")
            self._handle_no_input()
    
    def _execute_response(self, response: Dict[str, Any]):
        emotion = response.get("emotion", "calm")
        action = response.get("action", "stand")
        duration = response.get("duration", 3)
        confidence = response.get("confidence", 0.5)
        
        self.logger.info(f"Executing: {emotion} emotion -> {action} action (duration: {duration}s, confidence: {confidence:.2f})")
        
        if action in self.action_map:
            self._stop_current_action()
            
            if action == "walk":
                self.action_map[action]("walk_forward")
            else:
                self.action_map[action]()
                
            self.current_action_duration = duration
            self.last_action_start_time = time.time()
            
            if action in ["walk", "turn_left", "turn_right", "walk_backward"]:
                self.current_action_thread = threading.Thread(target=self._monitor_action_duration)
                self.current_action_thread.daemon = True
                self.current_action_thread.start()
        else:
            self.logger.warning(f"Unknown action: {action}")
    
    def _monitor_action_duration(self):
        time.sleep(self.current_action_duration)
        if self.is_running:
            self._stop_current_action()
            self.animation_controller.set_idle_position()
    
    def _should_stop_current_action(self) -> bool:
        if self.current_action_duration <= 0:
            return False
        return time.time() - self.last_action_start_time >= self.current_action_duration
    
    def _handle_action_completion(self):
        self._stop_current_action()
        self.animation_controller.set_idle_position()
        self.current_action_duration = 0
    
    def _stop_current_action(self):
        if self.current_action_thread and self.current_action_thread.is_alive():
            self.current_action_thread.join(timeout=0.1)
        self.animation_controller.stop_current_animation()
    
    def _end_walk_action(self):
        self.animation_controller.stop_current_animation()
        self.animation_controller.set_idle_position()
    
    def _handle_no_input(self):
        idle_response = self.personality._get_idle_behavior()
        self._execute_response(idle_response)
    
    def get_status(self) -> Dict[str, Any]:
        return {
            "is_running": self.is_running,
            "camera_active": self.camera.is_running if hasattr(self.camera, 'is_running') else False,
            "last_analysis": self.last_analysis_time,
            "personality_state": self.personality.get_current_state(),
            "current_action_duration": self.current_action_duration,
            "time_since_action_start": time.time() - self.last_action_start_time if self.last_action_start_time > 0 else 0
        }
    
    def set_analysis_interval(self, interval: float):
        self.analysis_interval = max(1.0, interval)
        self.logger.info(f"Analysis interval set to {self.analysis_interval}s")
