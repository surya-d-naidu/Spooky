#!/usr/bin/env python3
"""
Example usage of the modular autonomous robot system
"""

from autonomous_robot import create_dog_robot, AutonomousRobot
from core.interfaces import ISensorInterface, SensorData
import time
import random

class MockIMUSensor(ISensorInterface):
    """Example custom sensor - IMU/accelerometer"""
    
    def __init__(self):
        self.available = False
        
    def initialize(self) -> bool:
        print("Mock IMU sensor initialized")
        self.available = True
        return True
        
    def read(self) -> SensorData:
        # Simulate IMU readings
        return SensorData(
            timestamp=time.time(),
            sensor_type="imu",
            data={
                "pitch": random.uniform(-10, 10),
                "roll": random.uniform(-5, 5),
                "yaw": random.uniform(0, 360),
                "acceleration": {
                    "x": random.uniform(-1, 1),
                    "y": random.uniform(-1, 1), 
                    "z": random.uniform(9, 11)  # gravity
                }
            },
            confidence=0.9
        )
    
    def is_available(self) -> bool:
        return self.available
        
    def cleanup(self):
        print("Mock IMU sensor cleaned up")

def example_basic_usage():
    """Basic usage example"""
    print("=== Basic Usage Example ===")
    
    # Create robot with default configuration
    robot = create_dog_robot()
    
    # Start the robot (this will run until Ctrl+C)
    robot.start()

def example_custom_config():
    """Example with custom configuration"""
    print("=== Custom Configuration Example ===")
    
    custom_config = {
        'robot_type': 'custom_quadruped',
        'decision_interval': 2.0,  # Faster decisions
        'ai': {
            'model': 'llava:7b',
            'timeout': 20
        },
        'personality': {
            'base_traits': {
                'energy': 0.9,      # Very energetic
                'curiosity': 0.8,   # Very curious
                'playfulness': 0.9, # Very playful
                'sociability': 0.6, # Moderately social
                'courage': 0.7,     # Quite brave
                'independence': 0.3,# Low independence (follows more)
                'loyalty': 1.0,     # Maximum loyalty
                'alertness': 0.8    # Very alert
            }
        }
    }
    
    robot = create_dog_robot(custom_config)
    robot.start()

def example_custom_modules():
    """Example with custom modules"""
    print("=== Custom Modules Example ===")
    
    # Create robot with default setup
    robot = create_dog_robot()
    
    # Add custom sensor
    imu_sensor = MockIMUSensor()
    robot.add_custom_sensor('imu', imu_sensor)
    
    print("Added custom IMU sensor")
    
    # Start robot
    robot.start()

def example_modular_robot():
    """Example of building robot from scratch"""
    print("=== Modular Robot Example ===")
    
    # Create empty robot
    config = {
        'robot_type': 'experimental_bot',
        'decision_interval': 1.5
    }
    
    robot = AutonomousRobot(config)
    
    # Add only specific modules you want
    from ai.ollama_ai import OllamaAI
    from sensors.camera import CameraSensor
    from actuators.quadruped_movement import QuadrupedMovement
    from personality.dog_personality import DogPersonality
    
    # Setup AI
    ai = OllamaAI(model='llava:7b')
    robot.module_manager.register_ai_module('main_ai', ai)
    
    # Setup camera
    camera = CameraSensor()
    robot.module_manager.register_sensor('vision', camera)
    
    # Setup movement
    movement = QuadrupedMovement()
    robot.module_manager.register_actuator('legs', movement)
    
    # Setup personality
    personality = DogPersonality({
        'energy': 0.5,
        'curiosity': 1.0,  # Maximum curiosity
        'playfulness': 0.3,
        'sociability': 0.7
    })
    robot.module_manager.set_personality(personality)
    
    print("Custom modular robot configured")
    robot.start()

def run_example():
    """Run the examples"""
    print("Modular Autonomous Robot Examples")
    print("=================================")
    print()
    print("Choose an example:")
    print("1. Basic Usage (default configuration)")
    print("2. Custom Configuration")
    print("3. Add Custom Modules")
    print("4. Build Modular Robot from Scratch")
    print()
    
    choice = input("Enter choice (1-4): ").strip()
    
    try:
        if choice == '1':
            example_basic_usage()
        elif choice == '2':
            example_custom_config()
        elif choice == '3':
            example_custom_modules()
        elif choice == '4':
            example_modular_robot()
        else:
            print("Invalid choice. Running basic example.")
            example_basic_usage()
    except KeyboardInterrupt:
        print("\nExample stopped.")
    except Exception as e:
        print(f"Example error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    run_example()
