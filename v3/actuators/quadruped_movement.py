"""
Quadruped movement actuator for dog-like robots
"""

import threading
import time
import random
from typing import Dict, Any

from ..core.interfaces import IActuatorInterface, RobotAction

class QuadrupedMovement(IActuatorInterface):
    """Movement actuator for quadruped robots"""
    
    def __init__(self, movement_interface=None):
        self.movement_interface = movement_interface
        self.current_action = None
        self.action_thread = None
        self.stop_flag = False
        self.busy = False
        self.movement_lock = threading.Lock()
        
        # Try to import movement functions
        self._init_movement_functions()
        
    def _init_movement_functions(self):
        """Initialize movement functions"""
        try:
            if self.movement_interface:
                # Use provided interface
                self.crawl_gait_loop = self.movement_interface.crawl_gait_loop
                self.rotate_in_place = self.movement_interface.rotate_in_place
                self.available = True
            else:
                # Try to import from trot module
                from ..trot import crawl_gait_loop, rotate_in_place
                self.crawl_gait_loop = crawl_gait_loop
                self.rotate_in_place = rotate_in_place
                self.available = True
        except ImportError:
            print("Warning: Movement functions not available. Using mock functions.")
            self.available = False
            self._setup_mock_functions()
    
    def _setup_mock_functions(self):
        """Setup mock movement functions for testing"""
        def mock_crawl(duration=None):
            print(f"[MOCK] Walking for {duration} seconds")
            time.sleep(duration if duration else 1)
            
        def mock_rotate(direction, duration=1):
            print(f"[MOCK] Rotating {direction} for {duration} seconds")
            time.sleep(duration)
            
        self.crawl_gait_loop = mock_crawl
        self.rotate_in_place = mock_rotate
    
    def initialize(self) -> bool:
        """Initialize movement system"""
        print(f"Quadruped movement initialized (Available: {self.available})")
        return True
    
    def execute_action(self, action: RobotAction) -> bool:
        """Execute movement action"""
        with self.movement_lock:
            # Stop current action
            self.stop()
            
            # Start new action
            self.stop_flag = False
            self.busy = True
            self.current_action = action
            
            self.action_thread = threading.Thread(
                target=self._movement_worker,
                args=(action,)
            )
            self.action_thread.daemon = True
            self.action_thread.start()
            
        return True
    
    def _movement_worker(self, action: RobotAction):
        """Worker thread for movement execution"""
        action_type = action.action_type
        duration = action.duration
        parameters = action.parameters
        
        print(f"Executing movement: {action_type} for {duration:.1f}s")
        
        try:
            if action_type == "explore":
                self._explore_movement(duration, parameters)
            elif action_type == "investigate":
                self._investigate_movement(duration, parameters)
            elif action_type == "social_approach":
                self._social_approach_movement(duration, parameters)
            elif action_type == "play":
                self._play_movement(duration, parameters)
            elif action_type == "follow":
                self._follow_movement(duration, parameters)
            elif action_type == "avoid":
                self._avoid_movement(duration, parameters)
            elif action_type == "alert":
                self._alert_movement(duration, parameters)
            elif action_type == "rest":
                self._rest_movement(duration, parameters)
            else:
                print(f"Unknown action type: {action_type}")
                self._rest_movement(duration, parameters)
                
        except Exception as e:
            print(f"Movement execution error: {e}")
        finally:
            self.busy = False
            self.current_action = None
    
    def _explore_movement(self, duration: float, params: Dict[str, Any]):
        """Exploration movement pattern"""
        end_time = time.time() + duration
        
        while time.time() < end_time and not self.stop_flag:
            # Random exploration pattern
            remaining_time = end_time - time.time()
            
            if random.random() < 0.4:
                # Random rotation
                direction = params.get('direction', 'left' if random.random() < 0.5 else 'right')
                rotate_duration = min(random.uniform(0.5, 1.5), remaining_time)
                if rotate_duration > 0.2:
                    self.rotate_in_place(direction, rotate_duration)
            else:
                # Forward movement
                walk_duration = min(random.uniform(1.0, 3.0), remaining_time)
                if walk_duration > 0.5:
                    self.crawl_gait_loop(walk_duration)
            
            if not self.stop_flag and time.time() < end_time:
                time.sleep(random.uniform(0.3, 0.8))
    
    def _investigate_movement(self, duration: float, params: Dict[str, Any]):
        """Investigation movement - slow, careful approach"""
        target_direction = params.get('target_direction', 'forward')
        
        if target_direction != 'forward':
            # Turn towards target first
            turn_duration = min(1.0, duration * 0.3)
            self.rotate_in_place(target_direction, turn_duration)
            if self.stop_flag:
                return
        
        # Slow approach
        approach_duration = duration * 0.7
        if approach_duration > 0.5:
            self.crawl_gait_loop(approach_duration)
    
    def _social_approach_movement(self, duration: float, params: Dict[str, Any]):
        """Social approach - friendly movement towards detected person"""
        # Quick friendly approach
        approach_duration = duration * 0.8
        if approach_duration > 0.5:
            self.crawl_gait_loop(approach_duration)
        
        if not self.stop_flag:
            # Friendly greeting wiggle
            for _ in range(2):
                if self.stop_flag:
                    break
                self.rotate_in_place("left", 0.2)
                self.rotate_in_place("right", 0.2)
    
    def _play_movement(self, duration: float, params: Dict[str, Any]):
        """Playful movement pattern"""
        end_time = time.time() + duration
        play_intensity = params.get('intensity', 'normal')
        
        if play_intensity == 'high':
            spin_duration = 0.3
            pause_duration = 0.1
        else:
            spin_duration = 0.5
            pause_duration = 0.3
        
        while time.time() < end_time and not self.stop_flag:
            # Playful spins
            self.rotate_in_place("left", spin_duration)
            if self.stop_flag:
                break
            time.sleep(pause_duration)
            
            self.rotate_in_place("right", spin_duration)
            if self.stop_flag:
                break
            time.sleep(pause_duration)
            
            # Quick forward movement
            remaining_time = end_time - time.time()
            if remaining_time > 1.0:
                self.crawl_gait_loop(min(0.8, remaining_time))
            
            if not self.stop_flag:
                time.sleep(pause_duration)
    
    def _follow_movement(self, duration: float, params: Dict[str, Any]):
        """Following movement - continuous forward with occasional corrections"""
        end_time = time.time() + duration
        follow_speed = params.get('speed', 'normal')
        
        if follow_speed == 'fast':
            walk_segments = 2.0
            pause_time = 0.2
        else:
            walk_segments = 1.5
            pause_time = 0.5
        
        while time.time() < end_time and not self.stop_flag:
            remaining_time = end_time - time.time()
            segment_duration = min(walk_segments, remaining_time)
            
            if segment_duration > 0.5:
                self.crawl_gait_loop(segment_duration)
            
            if not self.stop_flag and time.time() < end_time:
                time.sleep(pause_time)
    
    def _avoid_movement(self, duration: float, params: Dict[str, Any]):
        """Avoidance movement - back away and turn"""
        avoid_direction = params.get('avoid_direction', 'right')
        
        # Quick turn away
        turn_duration = min(1.0, duration * 0.4)
        self.rotate_in_place(avoid_direction, turn_duration)
        
        if not self.stop_flag:
            # Move forward in new direction
            escape_duration = duration * 0.6
            if escape_duration > 0.5:
                self.crawl_gait_loop(escape_duration)
    
    def _alert_movement(self, duration: float, params: Dict[str, Any]):
        """Alert movement - scanning behavior"""
        end_time = time.time() + duration
        scan_intensity = params.get('intensity', 'normal')
        
        if scan_intensity == 'high':
            scan_duration = 0.2
            pause_duration = 0.1
        else:
            scan_duration = 0.4
            pause_duration = 0.6
        
        while time.time() < end_time and not self.stop_flag:
            # Scan left and right
            direction = 'left' if random.random() < 0.5 else 'right'
            self.rotate_in_place(direction, scan_duration)
            
            if not self.stop_flag:
                time.sleep(pause_duration)
    
    def _rest_movement(self, duration: float, params: Dict[str, Any]):
        """Rest movement - minimal activity"""
        rest_type = params.get('type', 'still')
        
        if rest_type == 'fidget':
            # Occasional small movements
            end_time = time.time() + duration
            while time.time() < end_time and not self.stop_flag:
                if random.random() < 0.2:  # 20% chance of small movement
                    self.rotate_in_place('left' if random.random() < 0.5 else 'right', 0.2)
                time.sleep(1.0)
        else:
            # Just wait
            time.sleep(duration)
    
    def stop(self):
        """Stop current action"""
        self.stop_flag = True
        if self.action_thread and self.action_thread.is_alive():
            self.action_thread.join(timeout=2)
    
    def is_busy(self) -> bool:
        """Check if actuator is busy"""
        return self.busy
    
    def cleanup(self):
        """Cleanup movement resources"""
        self.stop()
        print("Quadruped movement cleaned up")
