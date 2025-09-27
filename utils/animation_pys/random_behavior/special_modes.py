import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import main
import random
import time
import threading
import logging

class ChaosMode:
    def __init__(self):
        self.animation_controller = main.animation_controller
        self.is_running = False
        self.chaos_thread = None
        self.logger = logging.getLogger(__name__)
        
        self.robot_state = {
            'current_action': 'stopped',
            'is_moving': False,
            'last_command': None,
            'mode': 'chaos'
        }
        
        self.actions = [
            "walk_forward", "walk_backward", "turn_left", "turn_right",
            "sit", "stand", "hi", "rotate_left"
        ]
        
    def start(self):
        if self.is_running:
            return
            
        self.is_running = True
        self.chaos_thread = threading.Thread(target=self._chaos_loop)
        self.chaos_thread.daemon = True
        self.chaos_thread.start()
        self.robot_state['mode'] = 'chaos'
        self.logger.info("CHAOS MODE ACTIVATED!")
        
    def stop(self):
        self.is_running = False
        if self.chaos_thread:
            self.chaos_thread.join(timeout=3)
        self.animation_controller.stop_current_animation()
        self.animation_controller.set_idle_position()
        self.robot_state['current_action'] = 'stopped'
        self.robot_state['is_moving'] = False
        self.robot_state['mode'] = 'stopped'
        self.logger.info("Chaos mode deactivated")
        
    def get_status(self):
        """Returns current robot state"""
        return self.robot_state
        
    def _chaos_loop(self):
        while self.is_running:
            try:
                action = random.choice(self.actions)
                duration = random.uniform(0.5, 3)
                
                self._execute_action(action)
                time.sleep(duration)
                
                if random.random() < 0.3:
                    self.animation_controller.stop_current_animation()
                    time.sleep(random.uniform(0.1, 1))
                    
            except Exception as e:
                self.logger.error(f"Error in chaos mode: {str(e)}")
                time.sleep(0.5)
                
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

class ZenMode:
    def __init__(self):
        self.animation_controller = main.animation_controller
        self.is_running = False
        self.zen_thread = None
        self.logger = logging.getLogger(__name__)
        
        self.robot_state = {
            'current_action': 'stopped',
            'is_moving': False,
            'last_command': None,
            'mode': 'zen'
        }
        
    def start(self):
        if self.is_running:
            return
            
        self.is_running = True
        self.zen_thread = threading.Thread(target=self._zen_loop)
        self.zen_thread.daemon = True
        self.zen_thread.start()
        self.robot_state['mode'] = 'zen'
        self.logger.info("Zen mode activated - peaceful movements")
        
    def stop(self):
        self.is_running = False
        if self.zen_thread:
            self.zen_thread.join(timeout=5)
        self.animation_controller.stop_current_animation()
        self.animation_controller.set_idle_position()
        self.robot_state['current_action'] = 'stopped'
        self.robot_state['is_moving'] = False
        self.robot_state['mode'] = 'stopped'
        self.logger.info("Zen mode deactivated")
        
    def get_status(self):
        """Returns current robot state"""
        return self.robot_state
        
    def _zen_loop(self):
        while self.is_running:
            try:
                peaceful_actions = ["sit", "stand", "turn_left", "turn_right"]
                action = random.choice(peaceful_actions)
                duration = random.uniform(5, 12)
                
                self.logger.info(f"Zen action: {action} for {duration:.1f}s")
                self._execute_action(action)
                time.sleep(duration)
                
                self.animation_controller.stop_current_animation()
                self.animation_controller.set_idle_position()
                
                meditation_time = random.uniform(8, 20)
                self.logger.info(f"Meditating for {meditation_time:.1f}s")
                time.sleep(meditation_time)
                
            except Exception as e:
                self.logger.error(f"Error in zen mode: {str(e)}")
                time.sleep(3)
                
    def _execute_action(self, action):
        """Execute peaceful actions with state tracking"""
        try:
            if action == "turn_left":
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
                
            self.robot_state['last_command'] = action
            
        except Exception as e:
            self.logger.error(f"Error executing zen action {action}: {str(e)}")
            self.robot_state['current_action'] = 'error'
            self.robot_state['is_moving'] = False

class PartyMode:
    def __init__(self):
        self.animation_controller = main.animation_controller
        self.is_running = False
        self.party_thread = None
        self.logger = logging.getLogger(__name__)
        
        self.robot_state = {
            'current_action': 'stopped',
            'is_moving': False,
            'last_command': None,
            'mode': 'party',
            'dance_move': None
        }
        
    def start(self):
        if self.is_running:
            return
            
        self.is_running = True
        self.party_thread = threading.Thread(target=self._party_loop)
        self.party_thread.daemon = True
        self.party_thread.start()
        self.robot_state['mode'] = 'party'
        self.logger.info("PARTY MODE! Let's dance!")
        
    def stop(self):
        self.is_running = False
        if self.party_thread:
            self.party_thread.join(timeout=5)
        self.animation_controller.stop_current_animation()
        self.animation_controller.set_idle_position()
        self.robot_state['current_action'] = 'stopped'
        self.robot_state['is_moving'] = False
        self.robot_state['mode'] = 'stopped'
        self.robot_state['dance_move'] = None
        self.logger.info("Party's over")
        
    def get_status(self):
        """Returns current robot state"""
        return self.robot_state
        
    def _party_loop(self):
        while self.is_running:
            try:
                dance_moves = [
                    self._spin_dance,
                    self._wave_dance,
                    self._bounce_dance,
                    self._twist_dance
                ]
                
                dance = random.choice(dance_moves)
                self.robot_state['current_action'] = 'dancing'
                self.robot_state['is_moving'] = True
                dance()
                
                if self.is_running:
                    break_time = random.uniform(2, 5)
                    self.robot_state['current_action'] = 'taking_break'
                    self.robot_state['is_moving'] = False
                    self.robot_state['dance_move'] = None
                    self.logger.info(f"Taking a break for {break_time:.1f}s")
                    self.animation_controller.set_idle_position()
                    time.sleep(break_time)
                    
            except Exception as e:
                self.logger.error(f"Error in party mode: {str(e)}")
                self.robot_state['current_action'] = 'error'
                self.robot_state['is_moving'] = False
                time.sleep(2)
                
    def _spin_dance(self):
        self.logger.info("Spin dance!")
        self.robot_state['dance_move'] = 'spinning'
        for _ in range(random.randint(3, 6)):
            self.animation_controller.start_animation("rotate_left")
            time.sleep(1.5)
            if not self.is_running:
                break
                
    def _wave_dance(self):
        self.logger.info("Wave dance!")
        self.robot_state['dance_move'] = 'waving'
        for _ in range(random.randint(4, 8)):
            self.animation_controller.start_animation("hi")
            time.sleep(1)
            self.animation_controller.set_idle_position()
            time.sleep(0.5)
            if not self.is_running:
                break
                
    def _bounce_dance(self):
        self.logger.info("Bounce dance!")
        self.robot_state['dance_move'] = 'bouncing'
        for _ in range(random.randint(5, 10)):
            self.animation_controller.set_idle_position()
            time.sleep(0.3)
            self.animation_controller.start_animation("sit")
            time.sleep(0.5)
            if not self.is_running:
                break
                
    def _twist_dance(self):
        self.logger.info("Twist dance!")
        self.robot_state['dance_move'] = 'twisting'
        for _ in range(random.randint(6, 12)):
            actions = ["rotate_left", "turn_left", "turn_right"]
            action = random.choice(actions)
            reverse = random.random() < 0.5
            
            if reverse and action == "rotate_left":
                self.animation_controller.start_reverse_animation(action)
            elif action == "turn_left":
                self.animation_controller.start_animation("rotate_left")
            elif action == "turn_right":
                self.animation_controller.start_reverse_animation("rotate_left")
            else:
                self.animation_controller.start_animation(action)
            time.sleep(0.8)
            if not self.is_running:
                break
