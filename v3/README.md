# Modular Autonomous Robot System

A completely modular, AI-powered autonomous robot system using Ollama with llava:7b vision-language model. Designed for quadruped robots but easily adaptable to other robot types.

## 🎯 Features

- **🧠 AI-Powered Decisions**: Uses Ollama with llava:7b for intelligent decision making
- **📷 Computer Vision**: Real-time visual scene analysis and object detection
- **🎭 Dynamic Personality**: Evolving personality traits that influence behavior
- **🔧 Fully Modular**: Easy to extend with custom sensors, actuators, and AI modules
- **🐕 Dog-like Behavior**: Naturally behaves like an intelligent dog
- **⚡ Real-time Operation**: Responsive decision making and smooth movement

## 🏗️ Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Sensors       │    │   AI Module     │    │   Actuators     │
│  - Camera       │───▶│  - Ollama+llava │───▶│  - Movement     │
│  - IMU (opt)    │    │  - Decision     │    │  - Servos       │
│  - Custom...    │    │    Engine       │    │  - Custom...    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         └───────────────────────┼───────────────────────┘
                                 │
                    ┌─────────────────┐
                    │  Personality    │
                    │   System        │
                    │ - Traits        │
                    │ - Learning      │
                    └─────────────────┘
```

## 🚀 Quick Start

### Prerequisites

1. **Install Ollama**:
   ```bash
   # Install Ollama (see https://ollama.ai)
   ollama pull llava:7b
   ```

2. **Install Python Dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

### Basic Usage

```python
from autonomous_robot import create_dog_robot

# Create robot with default configuration
robot = create_dog_robot()

# Start autonomous behavior
robot.start()
```

### Run Examples

```bash
python examples.py
```

## 📁 Project Structure

```
v3/
├── core/
│   ├── __init__.py
│   └── interfaces.py          # Core interfaces and base classes
├── ai/
│   ├── __init__.py
│   └── ollama_ai.py          # Ollama AI integration
├── sensors/
│   ├── __init__.py
│   └── camera.py             # Camera sensor module
├── actuators/
│   ├── __init__.py
│   └── quadruped_movement.py # Quadruped movement controller
├── personality/
│   ├── __init__.py
│   └── dog_personality.py    # Dynamic personality system
├── autonomous_robot.py       # Main robot controller
├── examples.py              # Usage examples
├── trot.py                  # Original movement functions (modified)
├── servo.py                 # Servo control
└── requirements.txt
```

## 🎛️ Configuration

```python
config = {
    'robot_type': 'quadruped_dog',
    'decision_interval': 3.0,  # Seconds between decisions
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

robot = create_dog_robot(config)
```

## 🧩 Adding Custom Modules

### Custom Sensor

```python
from core.interfaces import ISensorInterface, SensorData

class MySensor(ISensorInterface):
    def initialize(self) -> bool:
        # Initialize your sensor
        return True
        
    def read(self) -> SensorData:
        # Read sensor data
        return SensorData(
            timestamp=time.time(),
            sensor_type="my_sensor",
            data={"value": 42},
            confidence=1.0
        )
    
    def is_available(self) -> bool:
        return True
        
    def cleanup(self):
        pass

# Add to robot
robot.add_custom_sensor('my_sensor', MySensor())
```

### Custom Actuator

```python
from core.interfaces import IActuatorInterface, RobotAction

class MyActuator(IActuatorInterface):
    def initialize(self) -> bool:
        return True
        
    def execute_action(self, action: RobotAction) -> bool:
        # Execute the action
        print(f"Executing: {action.action_type}")
        return True
    
    def stop(self):
        pass
        
    def is_busy(self) -> bool:
        return False
        
    def cleanup(self):
        pass

# Add to robot
robot.add_custom_actuator('my_actuator', MyActuator())
```

## 🐕 Behavior System

The robot exhibits natural dog-like behaviors:

- **🔍 Exploring**: Random movement and environmental investigation
- **🔬 Investigating**: Focused examination of interesting objects
- **👋 Social Approach**: Friendly approach to detected people
- **🎾 Playing**: Playful spins, jumps, and energetic movement
- **👥 Following**: Following detected entities
- **⚠️ Avoiding**: Moving away from obstacles or threats
- **👁️ Alert**: Heightened awareness and scanning
- **😴 Resting**: Calm observation and energy recovery

## 🎭 Personality System

Dynamic personality traits that evolve over time:

- **Energy**: Activity level and movement intensity
- **Curiosity**: Interest in exploring and investigating
- **Playfulness**: Tendency toward playful behaviors
- **Sociability**: Interest in social interaction
- **Courage**: Willingness to approach unknown situations
- **Independence**: Tendency to act autonomously
- **Loyalty**: Attachment and following behavior
- **Alertness**: Environmental awareness level

## 🤖 Adapting to Other Robots

The system is designed to be robot-agnostic:

1. **Replace Movement Module**:
   ```python
   class MyRobotMovement(IActuatorInterface):
       # Implement your robot's movement
   
   robot.add_custom_actuator('movement', MyRobotMovement())
   ```

2. **Add Robot-Specific Sensors**:
   ```python
   robot.add_custom_sensor('lidar', LidarSensor())
   robot.add_custom_sensor('ultrasonic', UltrasonicSensor())
   ```

3. **Customize Personality**:
   ```python
   # For a more aggressive robot
   personality_traits = {
       'energy': 0.9,
       'courage': 0.8,
       'independence': 0.7
   }
   ```

## 🎮 Available Actions

The AI can choose from these actions:

- `explore`: Move around and investigate environment
- `investigate`: Focus on specific object/area
- `social_approach`: Approach detected person/animal
- `play`: Playful behavior (spins, jumps)
- `rest`: Stay still and observe
- `avoid`: Move away from obstacles/threats  
- `follow`: Follow detected entity
- `alert`: Heightened awareness mode

## 🛠️ Troubleshooting

### Ollama Not Available
```bash
# Make sure Ollama is running
ollama serve

# Pull the model if not available
ollama pull llava:7b
```

### Camera Issues
```python
# Try different camera IDs
config['camera']['camera_id'] = 1  # or 2, 3, etc.
```

### Movement Not Working
The system will fall back to mock functions if movement modules aren't available. Check your `trot.py` and `servo.py` implementations.

## 📊 Performance

- **Decision Rate**: ~0.33 Hz (every 3 seconds, configurable)
- **Camera FPS**: ~30 FPS
- **AI Response Time**: 2-10 seconds (depending on hardware)
- **Memory Usage**: ~200-500 MB (depending on model)

## 🔮 Future Extensions

- **Memory System**: Long-term experience storage
- **Voice Interaction**: Speech recognition and synthesis
- **Multi-Robot Coordination**: Swarm behavior
- **Learning**: Reinforcement learning integration
- **Mobile App**: Remote monitoring and control

## 📝 License

This project is open source. Feel free to adapt for your robot platform!

## 🤝 Contributing

1. Fork the repository
2. Create your feature branch
3. Add your custom modules
4. Test with your robot platform
5. Submit a pull request

---

**Built with ❤️ for autonomous robotics**
