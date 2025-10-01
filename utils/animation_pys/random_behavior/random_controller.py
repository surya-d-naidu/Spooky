import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import main
import random
import time
import threading
import logging

logging.basicConfig(level=logging.INFO)

class RandomBehaviorController:
    def __init__(self):
        self.animation_controller = main.animation_controller
        self.is_running = False
        self.behavior_thread = None
        self.logger = logging.getLogger(__name__)
        
        self.robot_state = {
            'current_action': 'stopped',
            'is_moving': False,
            'last_command': None,
            'mode': 'random_basic'
        }
        
        self.available_actions = [
            "walk_forward",
            "walk_backward", 
            "turn_left",
            "turn_right",
            "sit",
            "stand",
            "hi",
            "rotate_left"
        ]
        
        self.action_weights = {
            "walk_forward": 0.25,
            "turn_left": 0.15,
            "turn_right": 0.15,
            "sit": 0.1,
            "stand": 0.1,
            "hi": 0.05,
            "walk_backward": 0.1,
            "rotate_left": 0.1
        }
        
        self.min_action_duration = 6
        self.max_action_duration = 15
        self.min_pause_duration = 2
        self.max_pause_duration = 8
        
    def start(self):
        if self.is_running:
            return
            
        self.is_running = True
        self.behavior_thread = threading.Thread(target=self._behavior_loop)
        self.behavior_thread.daemon = True
        self.behavior_thread.start()
        self.robot_state['mode'] = 'random_basic'
        self.logger.info("Random behavior controller started")
        
    def stop(self):
        self.is_running = False
        if self.behavior_thread:
            self.behavior_thread.join(timeout=5)
        self.animation_controller.stop_current_animation()
        self.animation_controller.set_idle_position()
        self.robot_state['current_action'] = 'stopped'
        self.robot_state['is_moving'] = False
        self.robot_state['mode'] = 'stopped'
        self.logger.info("Random behavior controller stopped")
        
    def get_status(self):
        """Returns current robot state like Flask server"""
        return self.robot_state
        
    def _behavior_loop(self):
        consecutive_errors = 0
        max_consecutive_errors = 5
        
        while self.is_running:
            try:
                action = self._choose_random_action()
                duration = random.uniform(self.min_action_duration, self.max_action_duration)
                
                self.logger.info(f"Starting random action: {action} for {duration:.1f}s")
                self._execute_action(action)
                
                # Reset error counter on successful action
                consecutive_errors = 0
                
                time.sleep(duration)
                
                if self.is_running:
                    self.animation_controller.stop_current_animation()
                    self.animation_controller.set_idle_position()
                    self.robot_state['current_action'] = 'idle'
                    self.robot_state['is_moving'] = False
                    
                    pause_duration = random.uniform(self.min_pause_duration, self.max_pause_duration)
                    self.logger.info(f"Pausing for {pause_duration:.1f}s")
                    time.sleep(pause_duration)
                    
            except Exception as e:
                consecutive_errors += 1
                self.logger.error(f"Error in behavior loop (attempt {consecutive_errors}): {str(e)}")
                self.robot_state['current_action'] = 'error'
                self.robot_state['is_moving'] = False
                
                if consecutive_errors >= max_consecutive_errors:
                    self.logger.error(f"Too many consecutive errors ({consecutive_errors}), stopping behavior loop")
                    self.is_running = False
                    break
                
                # Exponential backoff for errors
                error_sleep = min(2 ** consecutive_errors, 30)
                self.logger.info(f"Waiting {error_sleep}s before retry...")
                time.sleep(error_sleep)
                
    def _choose_random_action(self):
        actions = list(self.action_weights.keys())
        weights = list(self.action_weights.values())
        return random.choices(actions, weights=weights)[0]
        
    def _execute_action(self, action):
        """Execute action using same pattern as Flask server"""
        try:
            if action == "walk_forward":
                self.animation_controller.start_animation("walk_forward")
                self.robot_state['current_action'] = 'walking_forward'
                self.robot_state['is_moving'] = True
                
            elif action == "walk_backward":
                self.animation_controller.start_reverse_animation("walk_forward")
                self.robot_state['current_action'] = 'walking_backward'
                self.robot_state['is_moving'] = True
                
            elif action == "turn_left":
                self.animation_controller.start_animation("rotate_left")
                self.robot_state['current_action'] = 'rotating_left'
                self.robot_state['is_moving'] = True
                
            elif action == "turn_right":
                self.animation_controller.start_reverse_animation("rotate_left")
                self.robot_state['current_action'] = 'rotating_right'
                self.robot_state['is_moving'] = True
                
            elif action == "sit":
                self.animation_controller.start_animation("sit")
                self.robot_state['current_action'] = 'sitting'
                self.robot_state['is_moving'] = False
                
            elif action == "stand":
                self.animation_controller.set_idle_position()
                self.robot_state['current_action'] = 'standing'
                self.robot_state['is_moving'] = False
                
            elif action == "hi":
                self.animation_controller.start_animation("hi")
                self.robot_state['current_action'] = 'waving'
                self.robot_state['is_moving'] = True
                
            elif action == "rotate_left":
                self.animation_controller.start_animation("rotate_left")
                self.robot_state['current_action'] = 'rotating_left'
                self.robot_state['is_moving'] = True
                
            self.robot_state['last_command'] = action
            
        except Exception as e:
            self.logger.error(f"Error executing action {action}: {str(e)}")
            self.robot_state['current_action'] = 'error'
            self.robot_state['is_moving'] = False
            
    def set_action_duration_range(self, min_duration, max_duration):
        self.min_action_duration = max(1, min_duration)
        self.max_action_duration = max(self.min_action_duration, max_duration)
        
    def set_pause_duration_range(self, min_pause, max_pause):
        self.min_pause_duration = max(0.5, min_pause)
        self.max_pause_duration = max(self.min_pause_duration, max_pause)
        
    def set_action_weights(self, new_weights):
        for action in new_weights:
            if action in self.action_weights:
                self.action_weights[action] = new_weights[action]
