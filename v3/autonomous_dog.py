#!/usr/bin/env python3
"""
Autonomous Dog Robot System
Acts like an intelligent dog, making decisions and interacting with environment
"""

import cv2
import numpy as np
import threading
import time
import random
import json
from enum import Enum
from dataclasses import dataclass
from typing import Optional, List, Dict, Any
import requests
import base64

# Import existing modules
from trot import crawl_gait_loop, rotate_in_place, legs, HIP_FIXED
from servo import set_servo_angle as set_servo

class BehaviorState(Enum):
    IDLE = "idle"
    EXPLORING = "exploring"
    FOLLOWING_PERSON = "following_person"
    PLAYING = "playing"
    INVESTIGATING = "investigating"
    GREETING = "greeting"
    AVOIDING_OBSTACLE = "avoiding_obstacle"

@dataclass
class DetectedObject:
    name: str
    confidence: float
    bbox: tuple  # (x, y, w, h)
    distance: Optional[float] = None

@dataclass
class RobotState:
    behavior: BehaviorState = BehaviorState.IDLE
    energy_level: float = 1.0
    curiosity_level: float = 0.5
    social_mode: bool = True
    last_interaction_time: float = 0.0

class CameraStream:
    """Handles camera input and basic computer vision"""
    def __init__(self, camera_id=0):
        self.cap = cv2.VideoCapture(camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.current_frame = None
        self.running = False
        
    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._capture_loop)
        self.thread.daemon = True
        self.thread.start()
        
    def _capture_loop(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                self.current_frame = frame
            time.sleep(0.033)  # ~30 FPS
            
    def get_frame(self):
        return self.current_frame
        
    def stop(self):
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join()
        self.cap.release()

class ObjectDetector:
    """Simple object detection using OpenCV (can be enhanced with YOLO/other models)"""
    def __init__(self):
        # Load pre-trained face detector
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        self.body_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_fullbody.xml')
        
    def detect_objects(self, frame) -> List[DetectedObject]:
        if frame is None:
            return []
            
        objects = []
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect faces (people)
        faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)
        for (x, y, w, h) in faces:
            objects.append(DetectedObject("person", 0.8, (x, y, w, h)))
            
        # Simple motion detection for moving objects
        # This is a placeholder - can be enhanced with proper object detection
        
        return objects
    
    def estimate_distance(self, bbox, known_width=0.2):
        """Estimate distance based on bounding box size (rough approximation)"""
        _, _, w, h = bbox
        # Simple distance estimation - larger objects are closer
        focal_length = 500  # Approximate focal length
        distance = (known_width * focal_length) / w if w > 0 else float('inf')
        return min(distance, 5.0)  # Cap at 5 meters

class AINode:
    """Communicates with Ollama for decision making"""
    def __init__(self, base_url="http://localhost:11434", model="llava:7b"):
        self.base_url = base_url
        self.model = model
        self.api_url = f"{self.base_url}/api/generate"
        
    def analyze_scene(self, frame, detected_objects: List[DetectedObject], robot_state: RobotState) -> Dict[str, Any]:
        """Analyze the current scene and make decisions"""
        if frame is None:
            return {"action": "idle", "reason": "no_visual_input"}
            
        # Encode frame to base64
        frame_encoded = self._encode_frame(frame)
        if not frame_encoded:
            return {"action": "idle", "reason": "encoding_failed"}
            
        # Create context prompt
        context = self._create_context_prompt(detected_objects, robot_state)
        
        try:
            payload = {
                "model": self.model,
                "prompt": context,
                "images": [frame_encoded],
                "stream": False
            }
            
            response = requests.post(self.api_url, json=payload, timeout=10)
            response.raise_for_status()
            
            ai_response = response.json().get('response', '')
            return self._parse_ai_response(ai_response)
            
        except Exception as e:
            print(f"AI Node error: {e}")
            return {"action": "idle", "reason": "ai_error"}
    
    def _encode_frame(self, frame):
        """Encode frame to base64"""
        try:
            _, buffer = cv2.imencode('.jpg', frame)
            return base64.b64encode(buffer).decode('utf-8')
        except:
            return None
            
    def _create_context_prompt(self, objects: List[DetectedObject], state: RobotState) -> str:
        """Create prompt for AI decision making"""
        prompt = f"""You are an intelligent dog robot. Analyze this image and decide what action to take.

Current state: {state.behavior.value}
Energy level: {state.energy_level:.2f}
Curiosity level: {state.curiosity_level:.2f}

Detected objects: {[obj.name for obj in objects]}

You can choose from these actions:
- explore: Move around and investigate
- follow_person: Follow detected person
- play: Playful behavior if person is close
- investigate: Check out interesting objects
- greet: Friendly greeting behavior
- avoid: Avoid obstacles
- idle: Stay still

Respond with a JSON object containing:
{{"action": "action_name", "reason": "brief_explanation", "confidence": 0.8}}

Be dog-like: curious, friendly, playful, but also cautious of obstacles."""
        
        return prompt
    
    def _parse_ai_response(self, response: str) -> Dict[str, Any]:
        """Parse AI response into actionable commands"""
        try:
            # Try to extract JSON from response
            import re
            json_match = re.search(r'\{.*\}', response, re.DOTALL)
            if json_match:
                return json.loads(json_match.group())
        except:
            pass
            
        # Fallback parsing
        response_lower = response.lower()
        if "follow" in response_lower:
            return {"action": "follow_person", "reason": "person_detected", "confidence": 0.7}
        elif "play" in response_lower:
            return {"action": "play", "reason": "playful_mood", "confidence": 0.6}
        elif "explore" in response_lower:
            return {"action": "explore", "reason": "curious", "confidence": 0.5}
        else:
            return {"action": "idle", "reason": "uncertain", "confidence": 0.3}

class MovementController:
    """Controls robot movement based on decisions"""
    def __init__(self):
        self.current_action = None
        self.action_start_time = 0
        self.movement_thread = None
        self.stop_movement = False
        
    def execute_action(self, action: str, duration: float = 3.0):
        """Execute movement action"""
        if self.movement_thread and self.movement_thread.is_alive():
            self.stop_current_action()
            
        self.current_action = action
        self.action_start_time = time.time()
        self.stop_movement = False
        
        self.movement_thread = threading.Thread(
            target=self._movement_worker, 
            args=(action, duration)
        )
        self.movement_thread.daemon = True
        self.movement_thread.start()
        
    def _movement_worker(self, action: str, duration: float):
        """Worker thread for movement execution"""
        start_time = time.time()
        
        try:
            if action == "explore":
                self._explore_movement(duration)
            elif action == "follow_person":
                self._follow_movement(duration)
            elif action == "play":
                self._play_movement(duration)
            elif action == "greet":
                self._greet_movement(duration)
            elif action == "avoid":
                self._avoid_movement(duration)
            else:
                self._idle_movement(duration)
                
        except Exception as e:
            print(f"Movement error: {e}")
            
    def _explore_movement(self, duration):
        """Random exploration movement"""
        end_time = time.time() + duration
        while time.time() < end_time and not self.stop_movement:
            # Random movement pattern
            if random.random() < 0.3:
                rotate_in_place("left" if random.random() < 0.5 else "right", 1)
            else:
                crawl_gait_loop()  # This needs to be modified to run for limited time
            time.sleep(0.5)
            
    def _follow_movement(self, duration):
        """Follow person movement"""
        # Simple forward movement - in real implementation, 
        # this would use object tracking
        crawl_gait_loop()  # Needs modification for timed execution
        
    def _play_movement(self, duration):
        """Playful movement patterns"""
        end_time = time.time() + duration
        while time.time() < end_time and not self.stop_movement:
            # Playful spinning and movement
            rotate_in_place("left", 0.5)
            time.sleep(0.2)
            rotate_in_place("right", 0.5)
            time.sleep(0.2)
            
    def _greet_movement(self, duration):
        """Greeting behavior - tail wag simulation"""
        # Simple greeting gesture
        for _ in range(3):
            if self.stop_movement:
                break
            # Simulate excited movement
            time.sleep(0.3)
            
    def _avoid_movement(self, duration):
        """Obstacle avoidance movement"""
        rotate_in_place("right", 1)
        
    def _idle_movement(self, duration):
        """Idle behavior"""
        time.sleep(duration)
        
    def stop_current_action(self):
        """Stop current movement"""
        self.stop_movement = True
        if self.movement_thread:
            self.movement_thread.join(timeout=1)

class AutonomousDog:
    """Main autonomous dog robot controller"""
    def __init__(self):
        self.state = RobotState()
        self.camera = CameraStream()
        self.detector = ObjectDetector()
        self.ai_node = AINode()
        self.movement = MovementController()
        self.running = False
        
        # Behavior parameters
        self.decision_interval = 2.0  # Make decisions every 2 seconds
        self.last_decision_time = 0
        
    def start(self):
        """Start the autonomous system"""
        print("Starting Autonomous Dog Robot...")
        self.running = True
        self.camera.start()
        
        # Main control loop
        try:
            self._main_loop()
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            self.stop()
            
    def _main_loop(self):
        """Main decision and control loop"""
        while self.running:
            current_time = time.time()
            
            # Make decisions at regular intervals
            if current_time - self.last_decision_time > self.decision_interval:
                self._make_decision()
                self.last_decision_time = current_time
                
            # Update robot state
            self._update_state()
            
            time.sleep(0.1)  # 10Hz main loop
            
    def _make_decision(self):
        """Make behavioral decisions based on current situation"""
        frame = self.camera.get_frame()
        if frame is None:
            return
            
        # Detect objects in scene
        detected_objects = self.detector.detect_objects(frame)
        
        # Get AI decision
        ai_decision = self.ai_node.analyze_scene(frame, detected_objects, self.state)
        
        action = ai_decision.get("action", "idle")
        reason = ai_decision.get("reason", "no_reason")
        confidence = ai_decision.get("confidence", 0.5)
        
        print(f"Decision: {action} (reason: {reason}, confidence: {confidence:.2f})")
        
        # Update behavior state
        try:
            self.state.behavior = BehaviorState(action)
        except ValueError:
            self.state.behavior = BehaviorState.IDLE
            
        # Execute movement
        if confidence > 0.4:  # Only act if confident enough
            self.movement.execute_action(action, self.decision_interval * 1.5)
            
    def _update_state(self):
        """Update internal robot state"""
        current_time = time.time()
        
        # Simulate energy decay
        self.state.energy_level = max(0.1, self.state.energy_level - 0.001)
        
        # Update curiosity based on time since last interaction
        time_since_interaction = current_time - self.state.last_interaction_time
        if time_since_interaction > 30:  # 30 seconds
            self.state.curiosity_level = min(1.0, self.state.curiosity_level + 0.01)
        
    def stop(self):
        """Stop the autonomous system"""
        self.running = False
        self.movement.stop_current_action()
        self.camera.stop()
        print("Autonomous Dog Robot stopped.")

def main():
    """Main entry point"""
    print("=== Autonomous Dog Robot ===")
    print("Starting intelligent dog behavior system...")
    
    try:
        dog = AutonomousDog()
        dog.start()
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
