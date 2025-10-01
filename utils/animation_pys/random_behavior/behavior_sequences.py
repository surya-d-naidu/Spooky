import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import main
import random
import time
import threading
import logging

class BehaviorSequences:
    def __init__(self):
        self.animation_controller = main.animation_controller
        self.logger = logging.getLogger(__name__)
        self.robot_state = {
            'current_action': 'stopped',
            'is_moving': False,
            'last_command': None,
            'sequence': None
        }
        
    def playful_sequence(self):
        sequence = [
            ("hi", 2),
            ("walk_forward", 3),
            ("turn_left", 2),
            ("hi", 2),
            ("turn_right", 2),
            ("sit", 3)
        ]
        self._execute_sequence(sequence, "Playful")
        
    def patrol_sequence(self):
        sequence = [
            ("walk_forward", 4),  # Done
            ("turn_right", 3),    # Failure
            ("walk_forward", 4),  
            ("turn_right", 3),
            ("walk_forward", 4),
            ("turn_right", 3),
            ("walk_forward", 4),
            ("turn_right", 3),
            ("stand", 2)
        ]
        self._execute_sequence(sequence, "Patrol")
        
    def exploration_sequence(self):
        directions = ["turn_left", "turn_right", "rotate_left"]
        sequence = [
            ("stand", 1),
            (random.choice(directions), 2),
            ("walk_forward", random.randint(3, 6)),
            (random.choice(directions), 2),
            ("walk_forward", random.randint(2, 4)),
            ("sit", 3),
            ("stand", 1)
        ]
        self._execute_sequence(sequence, "Exploration")
        
    def dance_sequence(self):
        sequence = [
            ("hi", 1.5),
            ("turn_left", 1),
            ("turn_right", 1),
            ("turn_left", 1),
            ("turn_right", 1),
            ("hi", 2),
            ("rotate_left", 3),
            ("hi", 2)
        ]
        self._execute_sequence(sequence, "Dance")
        
    def exercise_sequence(self):
        sequence = [
            ("stand", 1),
            ("walk_forward", 5),
            ("walk_backward", 3),
            ("walk_forward", 5),
            ("turn_left", 2),
            ("walk_forward", 4),
            ("turn_right", 2),
            ("walk_forward", 4),
            ("sit", 4)
        ]
        self._execute_sequence(sequence, "Exercise")
        
    def greeting_sequence(self):
        sequence = [
            ("stand", 1),
            ("hi", 2),
            ("turn_left", 1.5),
            ("hi", 2),
            ("turn_right", 3),
            ("hi", 2),
            ("sit", 2)
        ]
        self._execute_sequence(sequence, "Greeting")
        
    def _execute_sequence(self, sequence, sequence_name):
        self.logger.info(f"Starting {sequence_name} sequence")
        self.robot_state['sequence'] = sequence_name
        
        for action, duration in sequence:
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
                time.sleep(duration)
                self.animation_controller.stop_current_animation()
                time.sleep(0.5)
                
            except Exception as e:
                self.logger.error(f"Error in sequence {sequence_name} action {action}: {str(e)}")
                self.robot_state['current_action'] = 'error'
                break
            
        self.animation_controller.set_idle_position()
        self.robot_state['current_action'] = 'standing'
        self.robot_state['is_moving'] = False
        self.robot_state['sequence'] = None
        self.logger.info(f"{sequence_name} sequence completed")
        
    def get_status(self):
        """Returns current robot state"""
        return self.robot_state

class AdvancedRandomBehavior:
    def __init__(self):
        self.animation_controller = main.animation_controller
        self.sequences = BehaviorSequences()
        self.is_running = False
        self.behavior_thread = None
        self.logger = logging.getLogger(__name__)
        
        self.robot_state = {
            'current_action': 'stopped',
            'is_moving': False,
            'last_command': None,
            'mode': 'advanced_sequences'
        }
        
        self.sequence_list = [
            self.sequences.playful_sequence,
            self.sequences.patrol_sequence,
            self.sequences.exploration_sequence,
            self.sequences.dance_sequence,
            self.sequences.exercise_sequence,
            self.sequences.greeting_sequence
        ]
        
        self.sequence_weights = [0.2, 0.15, 0.25, 0.1, 0.2, 0.1]
        
    def start(self):
        if self.is_running:
            return
            
        self.is_running = True
        self.behavior_thread = threading.Thread(target=self._advanced_behavior_loop)
        self.behavior_thread.daemon = True
        self.behavior_thread.start()
        self.robot_state['mode'] = 'advanced_sequences'
        self.logger.info("Advanced random behavior controller started")
        
    def stop(self):
        self.is_running = False
        if self.behavior_thread:
            self.behavior_thread.join(timeout=10)
        self.animation_controller.stop_current_animation()
        self.animation_controller.set_idle_position()
        self.robot_state['current_action'] = 'stopped'
        self.robot_state['is_moving'] = False
        self.robot_state['mode'] = 'stopped'
        self.logger.info("Advanced random behavior controller stopped")
        
    def get_status(self):
        """Returns current robot state"""
        sequence_status = self.sequences.get_status()
        self.robot_state.update(sequence_status)
        return self.robot_state
        
    def _advanced_behavior_loop(self):
        while self.is_running:
            try:
                sequence_func = random.choices(self.sequence_list, weights=self.sequence_weights)[0]
                self.robot_state['current_action'] = 'executing_sequence'
                self.robot_state['is_moving'] = True
                sequence_func()
                
                if self.is_running:
                    rest_time = random.uniform(5, 15)
                    self.robot_state['current_action'] = 'resting'
                    self.robot_state['is_moving'] = False
                    self.logger.info(f"Resting for {rest_time:.1f}s")
                    time.sleep(rest_time)
                    
            except Exception as e:
                self.logger.error(f"Error in advanced behavior loop: {str(e)}")
                self.robot_state['current_action'] = 'error'
                self.robot_state['is_moving'] = False
                time.sleep(3)
