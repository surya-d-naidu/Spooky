from servo import setFromDict as s
import json
import time
import threading

class AnimationController:
    def __init__(self):
        self.current_animation = None
        self.is_playing = False
        self.animation_thread = None
        self.stop_animation = False
        
    def load_animation(self, animation_name):
        """Load animation frames from JSON file"""
        try:
            import os
            base_dir = os.path.dirname(os.path.abspath(__file__))
            animation_path = os.path.join(base_dir, 'animation_j', f'{animation_name}.json')
            with open(animation_path) as f:
                return json.load(f)
        except FileNotFoundError:
            print(f"Animation file {animation_name}.json not found")
            return None
    
    def play_animation_once(self, animation_name):
        """Play animation once - skips last frame to avoid duplicate"""
        frames = self.load_animation(animation_name)
        if not frames:
            return
            
        for i in range(len(frames) - 1):
            if self.stop_animation:
                break
            frame = frames[i]
            positions = frame.get('positions', frame)
            s(positions)
            duration = frame.get('duration', 100)
            time.sleep(duration / 1000)
    
    def play_animation_loop(self, animation_name, reverse=False):
        """Play animation in a loop - handles first/last frame duplication"""
        frames = self.load_animation(animation_name)
        if not frames:
            return
            
        self.is_playing = True
        first_loop = True
        
        while self.is_playing and not self.stop_animation:
            if reverse:
                start_idx = len(frames) - 2 if first_loop else len(frames) - 2
                end_idx = 0
                
                for i in range(start_idx, end_idx, -1):
                    if not self.is_playing or self.stop_animation:
                        break
                    frame = frames[i]
                    positions = frame.get('positions', frame)
                    s(positions)
                    duration = frame.get('duration', 100)
                    time.sleep(duration / 1000)
                    
                if self.is_playing and not self.stop_animation:
                    frame = frames[0]
                    positions = frame.get('positions', frame)
                    s(positions)
                    duration = frame.get('duration', 100)
                    time.sleep(duration / 1000)
            else:
                start_idx = 0 if first_loop else 1
                end_idx = len(frames) - 1
                
                for i in range(start_idx, end_idx):
                    if not self.is_playing or self.stop_animation:
                        break
                    frame = frames[i]
                    positions = frame.get('positions', frame)
                    s(positions)
                    duration = frame.get('duration', 100)
                    time.sleep(duration / 1000)
            
            first_loop = False
    
    def start_animation(self, animation_name, reverse=False):
        """Start playing animation in a separate thread"""
        self.stop_current_animation()
        self.stop_animation = False
        self.current_animation = animation_name
        self.animation_thread = threading.Thread(target=self.play_animation_loop, args=(animation_name, reverse))
        self.animation_thread.daemon = True
        self.animation_thread.start()
    
    def start_reverse_animation(self, animation_name):
        """Start playing animation in reverse"""
        self.start_animation(animation_name, reverse=True)
    
    def stop_current_animation(self):
        """Stop current animation"""
        self.is_playing = False
        self.stop_animation = True
        if self.animation_thread and self.animation_thread.is_alive():
            self.animation_thread.join(timeout=1)
    
    def set_idle_position(self):
        """Set robot to idle/standing position"""
        self.stop_current_animation()
        frames = self.load_animation('stand')
        if frames:
            frame = frames[0]
            positions = frame.get('positions', frame)
            s(positions)

animation_controller = AnimationController()

def walk():
    animation_controller.start_animation('walk_forward')

def end_walk():
    animation_controller.stop_current_animation()
    animation_controller.set_idle_position()

def turn_left():
    animation_controller.start_animation('rotate_left')

def turn_right():
    animation_controller.start_reverse_animation('rotate_left')

def walk_backward():
    animation_controller.start_reverse_animation('walk_forward')

def stand():
    animation_controller.set_idle_position()

def say_hi():
    animation_controller.start_animation("hi")

def sit():
    animation_controller.start_animation("sit")
