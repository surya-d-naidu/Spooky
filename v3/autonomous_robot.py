"""
Main autonomous robot system - modular and extensible
"""

import time
import threading
from typing import Dict, Any, List, Optional

try:
    from .core.interfaces import (
        ModuleManager, RobotConfig, RobotState, 
        ISensorInterface, IActuatorInterface, IAIInterface
    )
    # Import specific modules
    from .ai.ollama_ai import OllamaAI
    from .sensors.camera import CameraSensor
    from .actuators.quadruped_movement import QuadrupedMovement
    from .personality.dog_personality import DogPersonality
except ImportError:
    # Fallback for direct execution
    import sys
    import os
    script_dir = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, script_dir)
    
    from core.interfaces import (
        ModuleManager, RobotConfig, RobotState, 
        ISensorInterface, IActuatorInterface, IAIInterface
    )
    # Import specific modules
    from ai.ollama_ai import OllamaAI
    from sensors.camera import CameraSensor
    from actuators.quadruped_movement import QuadrupedMovement
    from personality.dog_personality import DogPersonality

class AutonomousRobot:
    """Main autonomous robot controller - modular and extensible"""
    
    def __init__(self, config: Dict[str, Any] = None):
        self.config = RobotConfig(config or {})
        self.module_manager = ModuleManager()
        self.current_state = RobotState.IDLE
        self.running = False
        
        # Core control loop parameters
        self.decision_interval = self.config.get('decision_interval', 3.0)
        self.status_interval = self.config.get('status_interval', 30.0)
        self.personality_update_interval = self.config.get('personality_update_interval', 10.0)
        
        # Statistics
        self.start_time = 0
        self.total_decisions = 0
        self.last_decision_time = 0
        self.last_status_time = 0
        self.last_personality_update = 0
        
        # Thread safety
        self.state_lock = threading.Lock()
        
    def setup_default_modules(self):
        """Setup default modules for quadruped dog robot"""
        
        # AI Module
        ai_config = self.config.get('ai', {})
        ollama_ai = OllamaAI(
            base_url=ai_config.get('ollama_url', 'http://localhost:11434'),
            model=ai_config.get('model', 'llava:7b'),
            timeout=ai_config.get('timeout', 30)
        )
        self.module_manager.register_ai_module('ollama', ollama_ai)
        
        # Camera Sensor
        camera_config = self.config.get('camera', {})
        camera = CameraSensor(
            camera_id=camera_config.get('camera_id', 0),
            width=camera_config.get('width', 640),
            height=camera_config.get('height', 480)
        )
        self.module_manager.register_sensor('camera', camera)
        
        # Movement Actuator
        movement_config = self.config.get('movement', {})
        movement = QuadrupedMovement(
            movement_interface=movement_config.get('interface', None)
        )
        self.module_manager.register_actuator('movement', movement)
        
        # Personality System
        personality_config = self.config.get('personality', {})
        personality = DogPersonality(
            base_traits=personality_config.get('base_traits', None)
        )
        self.module_manager.set_personality(personality)
        
        print("Default modules configured")
    
    def add_custom_sensor(self, name: str, sensor: ISensorInterface):
        """Add custom sensor module"""
        self.module_manager.register_sensor(name, sensor)
        
    def add_custom_actuator(self, name: str, actuator: IActuatorInterface):
        """Add custom actuator module"""
        self.module_manager.register_actuator(name, actuator)
        
    def add_custom_ai(self, name: str, ai_module: IAIInterface):
        """Add custom AI module"""
        self.module_manager.register_ai_module(name, ai_module)
    
    def initialize(self) -> bool:
        """Initialize all modules"""
        print("Initializing Autonomous Robot System...")
        
        # Initialize all modules
        success = self.module_manager.initialize_all()
        
        if success:
            print("All modules initialized successfully")
            self.current_state = RobotState.IDLE
        else:
            print("Some modules failed to initialize")
            
        return success
    
    def start(self):
        """Start the autonomous robot system"""
        if not self.initialize():
            print("Failed to initialize. Aborting start.")
            return
            
        print("=== Autonomous Robot System Starting ===")
        
        # Print configuration
        self._print_startup_info()
        
        self.running = True
        self.start_time = time.time()
        
        try:
            self._main_control_loop()
        except KeyboardInterrupt:
            print("\nReceived shutdown signal...")
        except Exception as e:
            print(f"Unexpected error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.stop()
    
    def _main_control_loop(self):
        """Main control loop"""
        print("Starting main control loop...")
        
        while self.running:
            current_time = time.time()
            
            # Decision making
            if current_time - self.last_decision_time >= self.decision_interval:
                self._make_decision()
                self.last_decision_time = current_time
                self.total_decisions += 1
            
            # Personality updates
            if (current_time - self.last_personality_update >= 
                self.personality_update_interval):
                self._update_personality()
                self.last_personality_update = current_time
            
            # Status reporting
            if current_time - self.last_status_time >= self.status_interval:
                self._print_status()
                self.last_status_time = current_time
            
            # Sleep to prevent busy waiting
            time.sleep(0.1)  # 10Hz main loop
    
    def _make_decision(self):
        """Make autonomous decision"""
        # Collect sensor data
        sensor_data = self.module_manager.get_sensor_data()
        
        # Create decision context
        context = {
            'robot_type': self.config.get('robot_type', 'quadruped_dog'),
            'personality': (self.module_manager.personality.get_personality_traits() 
                          if self.module_manager.personality else {}),
            'current_state': self.current_state,
            'uptime': time.time() - self.start_time,
            'total_decisions': self.total_decisions
        }
        
        # Get AI decision
        ai_module = self._get_primary_ai_module()
        if ai_module and ai_module.is_available():
            try:
                base_action = ai_module.make_decision(sensor_data, self.current_state, context)
                
                # Apply personality influence
                if self.module_manager.personality:
                    final_action = self.module_manager.personality.influence_decision(base_action)
                else:
                    final_action = base_action
                
                # Execute action
                self._execute_action(final_action)
                
                # Update state based on action
                self._update_state_from_action(final_action)
                
                # Log decision
                self._log_decision(final_action, sensor_data)
                
            except Exception as e:
                print(f"Decision making error: {e}")
                self._execute_fallback_action()
        else:
            print("AI not available, using fallback behavior")
            self._execute_fallback_action()
    
    def _execute_action(self, action):
        """Execute robot action"""
        # Find appropriate actuator
        if action.action_type in ['explore', 'investigate', 'social_approach', 
                                 'play', 'follow', 'avoid', 'alert', 'rest']:
            # Movement action
            movement = self.module_manager.actuators.get('movement')
            if movement:
                movement.execute_action(action)
            else:
                print("No movement actuator available")
        else:
            print(f"Unknown action type: {action.action_type}")
    
    def _execute_fallback_action(self):
        """Execute simple fallback action when AI is unavailable"""
        from .core.interfaces import RobotAction
        
        # Simple state-based behavior
        if self.current_state == RobotState.IDLE:
            action_type = 'explore'
        else:
            action_type = 'rest'
            
        fallback_action = RobotAction(
            action_type=action_type,
            parameters={},
            duration=3.0,
            priority=1,
            metadata={'source': 'fallback'}
        )
        
        self._execute_action(fallback_action)
        self._update_state_from_action(fallback_action)
    
    def _update_state_from_action(self, action):
        """Update robot state based on executed action"""
        with self.state_lock:
            if action.action_type in ['explore', 'investigate', 'follow']:
                self.current_state = RobotState.EXPLORING
            elif action.action_type in ['social_approach', 'play']:
                self.current_state = RobotState.INTERACTING
            elif action.action_type in ['alert', 'avoid']:
                self.current_state = RobotState.ALERT
            elif action.action_type == 'rest':
                self.current_state = RobotState.IDLE
            else:
                self.current_state = RobotState.MOVING
    
    def _update_personality(self):
        """Update personality based on recent experiences"""
        if not self.module_manager.personality:
            return
            
        # Create experience summary
        experience = {
            'type': 'general_activity',
            'outcome': 'neutral',
            'intensity': 0.5,
            'context': {
                'state': self.current_state.value,
                'recent_decisions': self.total_decisions
            }
        }
        
        self.module_manager.personality.update_personality(experience)
    
    def _log_decision(self, action, sensor_data):
        """Log decision for debugging"""
        confidence = action.metadata.get('confidence', 0.0) if action.metadata else 0.0
        reasoning = action.metadata.get('reasoning', 'unknown') if action.metadata else 'unknown'
        
        print(f"\nDecision #{self.total_decisions}:")
        print(f"  Action: {action.action_type}")
        print(f"  Duration: {action.duration:.1f}s")
        print(f"  Confidence: {confidence:.2f}")
        print(f"  Reasoning: {reasoning}")
        print(f"  Sensors: {len(sensor_data)} active")
        print(f"  State: {self.current_state.value}")
        
        # Personality summary
        if self.module_manager.personality:
            personality_summary = self.module_manager.personality.get_personality_summary()
            print(f"  {personality_summary}")
    
    def _get_primary_ai_module(self):
        """Get primary AI module"""
        return self.module_manager.ai_modules.get('ollama')
    
    def _print_startup_info(self):
        """Print startup information"""
        print(f"Robot Type: {self.config.get('robot_type', 'quadruped_dog')}")
        print(f"Decision Interval: {self.decision_interval}s")
        print(f"Modules: {len(self.module_manager.sensors)} sensors, "
              f"{len(self.module_manager.actuators)} actuators, "
              f"{len(self.module_manager.ai_modules)} AI modules")
        print(f"Personality: {'enabled' if self.module_manager.personality else 'disabled'}")
        print()
    
    def _print_status(self):
        """Print current status"""
        uptime = time.time() - self.start_time
        
        print(f"\n=== Status (Uptime: {uptime:.0f}s) ===")
        print(f"State: {self.current_state.value}")
        print(f"Decisions: {self.total_decisions}")
        print(f"Active Sensors: {sum(1 for s in self.module_manager.sensors.values() if s.is_available())}")
        print(f"Busy Actuators: {sum(1 for a in self.module_manager.actuators.values() if a.is_busy())}")
        
        if self.module_manager.personality:
            traits = self.module_manager.personality.get_personality_traits()
            print(f"Energy: {traits.get('energy', 0):.2f}, "
                  f"Curiosity: {traits.get('curiosity', 0):.2f}, "
                  f"Playfulness: {traits.get('playfulness', 0):.2f}")
    
    def stop(self):
        """Stop the autonomous robot system"""
        print("\nStopping Autonomous Robot System...")
        
        self.running = False
        
        # Stop all actuators
        for actuator in self.module_manager.actuators.values():
            actuator.stop()
        
        # Cleanup all modules
        self.module_manager.cleanup_all()
        
        # Final statistics
        uptime = time.time() - self.start_time if self.start_time > 0 else 0
        print(f"\nFinal Statistics:")
        print(f"  Total uptime: {uptime:.1f} seconds")
        print(f"  Decisions made: {self.total_decisions}")
        print(f"  Average decision rate: {self.total_decisions/max(uptime, 1):.2f} decisions/sec")
        print("Autonomous Robot System stopped.")

# Factory function for easy setup
def create_dog_robot(config: Dict[str, Any] = None) -> AutonomousRobot:
    """Create a quadruped dog robot with default configuration"""
    
    default_config = {
        'robot_type': 'quadruped_dog',
        'decision_interval': 3.0,
        'ai': {
            'ollama_url': 'http://localhost:11434',
            'model': 'llava:7b',
            'timeout': 30
        },
        'camera': {
            'camera_id': 0,
            'width': 640,
            'height': 480
        },
        'personality': {
            'base_traits': {
                'energy': 0.8,
                'curiosity': 0.7,
                'playfulness': 0.6,
                'sociability': 0.8,
                'courage': 0.5,
                'independence': 0.4,
                'loyalty': 0.9,
                'alertness': 0.6
            }
        }
    }
    
    if config:
        default_config.update(config)
    
    robot = AutonomousRobot(default_config)
    robot.setup_default_modules()
    
    return robot

def main():
    """Main entry point for autonomous dog robot"""
    
    print("=== Modular Autonomous Dog Robot ===")
    print("Powered by Ollama + llava:7b")
    print()
    
    try:
        # Create and start robot
        robot = create_dog_robot()
        robot.start()
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
