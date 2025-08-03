#!/usr/bin/env python3
"""
Simple Autonomous Dog Robot System
Acts like an intelligent dog with basic behaviors
"""

import time
import random
import threading
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Dict, Any

# Import existing modules (these should work if the files are in the same directory)
try:
    from trot import crawl_gait_loop, rotate_in_place
    from servo import set_servo_angle as set_servo
    MOVEMENT_AVAILABLE = True
except ImportError:
    print("Warning: Movement modules not available. Running in simulation mode.")
    MOVEMENT_AVAILABLE = False
    
    # Mock functions for testing
    def crawl_gait_loop(duration=None):
        print(f"[MOCK] Walking for {duration} seconds")
        time.sleep(duration if duration else 1)
        
    def rotate_in_place(direction, duration=1):
        print(f"[MOCK] Rotating {direction} for {duration} seconds")
        time.sleep(duration)
        
    def set_servo(channel, angle):
        print(f"[MOCK] Servo {channel} -> {angle}°")

class BehaviorState(Enum):
    IDLE = "idle"
    EXPLORING = "exploring"
    ALERT = "alert"
    PLAYING = "playing"
    SOCIAL = "social"
    CURIOUS = "curious"

@dataclass
class RobotPersonality:
    energy_level: float = 0.8
    curiosity: float = 0.6
    playfulness: float = 0.7
    social_drive: float = 0.5
    alertness: float = 0.4
    
    def update(self):
        """Update personality over time"""
        # Simulate natural personality changes
        self.energy_level = max(0.1, min(1.0, self.energy_level + random.uniform(-0.05, 0.02)))
        self.curiosity = max(0.1, min(1.0, self.curiosity + random.uniform(-0.02, 0.03)))
        self.playfulness = max(0.1, min(1.0, self.playfulness + random.uniform(-0.02, 0.02)))

class SimpleBehaviorEngine:
    """Simple rule-based behavior engine"""
    def __init__(self):
        self.current_state = BehaviorState.IDLE
        self.state_start_time = time.time()
        self.last_decision_time = 0
        self.decision_interval = 3.0  # Make decisions every 3 seconds
        
    def decide_action(self, personality: RobotPersonality, environment_factors: Dict) -> Dict[str, Any]:
        """Make behavioral decisions based on personality and environment"""
        current_time = time.time()
        time_in_state = current_time - self.state_start_time
        
        # Decision weights based on personality
        decisions = []
        
        # High energy -> more likely to explore or play
        if personality.energy_level > 0.6:
            decisions.append(("explore", personality.energy_level * 0.4))
            decisions.append(("play", personality.playfulness * 0.3))
            
        # High curiosity -> investigate environment
        if personality.curiosity > 0.5:
            decisions.append(("curious", personality.curiosity * 0.5))
            
        # Social behavior
        if personality.social_drive > 0.4:
            decisions.append(("social", personality.social_drive * 0.3))
            
        # Stay in current state if comfortable
        if time_in_state < 5:  # Stay at least 5 seconds in each state
            decisions.append((self.current_state.value, 0.6))
        else:
            decisions.append(("idle", 0.2))  # Default to idle
            
        # Add some randomness for unpredictability
        decisions.append((random.choice(list(BehaviorState)).value, 0.1))
        
        # Choose action with highest weight
        if decisions:
            action, weight = max(decisions, key=lambda x: x[1] + random.uniform(0, 0.2))
            
            # Update state if changed
            if action != self.current_state.value:
                try:
                    self.current_state = BehaviorState(action)
                    self.state_start_time = current_time
                except ValueError:
                    self.current_state = BehaviorState.IDLE
                    
            return {
                "action": action,
                "weight": weight,
                "duration": random.uniform(2, 8),  # Action duration
                "reason": f"personality_driven_{action}"
            }
        
        return {"action": "idle", "weight": 0.5, "duration": 3, "reason": "default"}

class MovementController:
    """Controls robot movement with thread safety"""
    def __init__(self):
        self.current_action = None
        self.action_thread = None
        self.stop_flag = False
        self.movement_lock = threading.Lock()
        
    def execute_action(self, action: str, duration: float = 3.0):
        """Execute movement action in a separate thread"""
        with self.movement_lock:
            # Stop current action
            self.stop_current_action()
            
            # Start new action
            self.stop_flag = False
            self.current_action = action
            self.action_thread = threading.Thread(
                target=self._movement_worker, 
                args=(action, duration)
            )
            self.action_thread.daemon = True
            self.action_thread.start()
            
    def _movement_worker(self, action: str, duration: float):
        """Worker thread for movement execution"""
        print(f"Executing: {action} for {duration:.1f}s")
        
        try:
            if action == "explore":
                self._explore_behavior(duration)
            elif action == "play":
                self._play_behavior(duration)
            elif action == "curious":
                self._curious_behavior(duration)
            elif action == "social":
                self._social_behavior(duration)
            elif action == "alert":
                self._alert_behavior(duration)
            else:
                self._idle_behavior(duration)
                
        except Exception as e:
            print(f"Movement error: {e}")
            
        finally:
            self.current_action = None
            
    def _explore_behavior(self, duration):
        """Exploration movement pattern"""
        end_time = time.time() + duration
        
        while time.time() < end_time and not self.stop_flag:
            # Random exploration pattern
            if random.random() < 0.4:
                direction = "left" if random.random() < 0.5 else "right"
                rotate_duration = random.uniform(0.5, 1.5)
                rotate_in_place(direction, rotate_duration)
            else:
                walk_duration = min(2.0, end_time - time.time())
                if walk_duration > 0.5:
                    crawl_gait_loop(walk_duration)
            
            if not self.stop_flag:
                time.sleep(random.uniform(0.5, 1.0))
                
    def _play_behavior(self, duration):
        """Playful movement pattern"""
        end_time = time.time() + duration
        
        while time.time() < end_time and not self.stop_flag:
            # Playful spins and movements
            rotate_in_place("left", 0.5)
            if self.stop_flag:
                break
            time.sleep(0.2)
            
            rotate_in_place("right", 0.5)
            if self.stop_flag:
                break
            time.sleep(0.3)
            
            # Quick forward movement
            crawl_gait_loop(0.8)
            if self.stop_flag:
                break
            time.sleep(0.5)
            
    def _curious_behavior(self, duration):
        """Curious investigation behavior"""
        # Look around curiously
        for _ in range(int(duration)):
            if self.stop_flag:
                break
            direction = "left" if random.random() < 0.5 else "right"
            rotate_in_place(direction, 0.3)
            time.sleep(0.7)
            
    def _social_behavior(self, duration):
        """Social/greeting behavior"""
        # Friendly approach pattern
        crawl_gait_loop(duration * 0.6)
        if not self.stop_flag:
            # Excited greeting
            for _ in range(2):
                if self.stop_flag:
                    break
                rotate_in_place("left", 0.2)
                rotate_in_place("right", 0.2)
                
    def _alert_behavior(self, duration):
        """Alert/watchful behavior"""
        # Stay still and scan
        for _ in range(int(duration * 2)):
            if self.stop_flag:
                break
            direction = "left" if random.random() < 0.5 else "right"
            rotate_in_place(direction, 0.2)
            time.sleep(0.8)
            
    def _idle_behavior(self, duration):
        """Idle/resting behavior"""
        print("Resting...")
        time.sleep(duration)
        
    def stop_current_action(self):
        """Stop current movement action"""
        self.stop_flag = True
        if self.action_thread and self.action_thread.is_alive():
            self.action_thread.join(timeout=2)

class SimpleAutonomousDog:
    """Main autonomous dog robot controller"""
    def __init__(self):
        self.personality = RobotPersonality()
        self.behavior_engine = SimpleBehaviorEngine()
        self.movement = MovementController()
        self.running = False
        
        # Status tracking
        self.start_time = time.time()
        self.total_actions = 0
        
    def start(self):
        """Start the autonomous system"""
        print("=== Simple Autonomous Dog Robot ===")
        print("Personality:")
        print(f"  Energy: {self.personality.energy_level:.2f}")
        print(f"  Curiosity: {self.personality.curiosity:.2f}")
        print(f"  Playfulness: {self.personality.playfulness:.2f}")
        print(f"  Sociability: {self.personality.social_drive:.2f}")
        print("\nStarting autonomous behavior...")
        
        self.running = True
        
        try:
            self._main_loop()
        except KeyboardInterrupt:
            print("\nReceived stop signal...")
        finally:
            self.stop()
            
    def _main_loop(self):
        """Main decision and control loop"""
        last_decision_time = 0
        
        while self.running:
            current_time = time.time()
            
            # Make behavioral decisions periodically
            if current_time - last_decision_time > self.behavior_engine.decision_interval:
                self._make_decision()
                last_decision_time = current_time
                self.total_actions += 1
                
            # Update personality gradually
            if current_time % 10 < 0.1:  # Every ~10 seconds
                self.personality.update()
                
            # Status update
            if current_time % 30 < 0.1:  # Every ~30 seconds
                self._print_status()
                
            time.sleep(0.1)  # 10Hz main loop
            
    def _make_decision(self):
        """Make behavioral decision and execute action"""
        # Simple environment factors (can be expanded)
        environment = {
            "time_of_day": time.time() % (24 * 3600),
            "activity_level": random.uniform(0, 1)
        }
        
        # Get decision from behavior engine
        decision = self.behavior_engine.decide_action(self.personality, environment)
        
        action = decision["action"]
        duration = decision["duration"]
        reason = decision["reason"]
        
        print(f"\nDecision #{self.total_actions + 1}: {action.upper()}")
        print(f"  Duration: {duration:.1f}s")
        print(f"  Reason: {reason}")
        print(f"  Current state: {self.behavior_engine.current_state.value}")
        
        # Execute the action
        self.movement.execute_action(action, duration)
        
    def _print_status(self):
        """Print current status"""
        uptime = time.time() - self.start_time
        print(f"\n--- Status (Uptime: {uptime:.0f}s) ---")
        print(f"State: {self.behavior_engine.current_state.value}")
        print(f"Actions executed: {self.total_actions}")
        print(f"Energy: {self.personality.energy_level:.2f}")
        print(f"Curiosity: {self.personality.curiosity:.2f}")
        
    def stop(self):
        """Stop the autonomous system"""
        print("\nStopping autonomous dog...")
        self.running = False
        self.movement.stop_current_action()
        
        uptime = time.time() - self.start_time
        print(f"Final Statistics:")
        print(f"  Total uptime: {uptime:.1f} seconds")
        print(f"  Actions executed: {self.total_actions}")
        print("Autonomous dog stopped.")

def main():
    """Main entry point"""
    try:
        dog = SimpleAutonomousDog()
        dog.start()
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
