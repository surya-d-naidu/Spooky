"""
Core interfaces and base classes for modular autonomous robot system
"""

from abc import ABC, abstractmethod
from typing import Dict, Any, List, Optional, Tuple
from dataclasses import dataclass
from enum import Enum
import time

class RobotState(Enum):
    IDLE = "idle"
    MOVING = "moving"
    EXPLORING = "exploring"
    INVESTIGATING = "investigating"
    INTERACTING = "interacting"
    ALERT = "alert"
    ERROR = "error"

@dataclass
class SensorData:
    """Generic sensor data container"""
    timestamp: float
    sensor_type: str
    data: Dict[str, Any]
    confidence: float = 1.0

@dataclass
class DetectedObject:
    """Object detection result"""
    label: str
    confidence: float
    bbox: Tuple[int, int, int, int]  # x, y, width, height
    distance: Optional[float] = None
    properties: Dict[str, Any] = None

@dataclass
class RobotAction:
    """Action to be executed by the robot"""
    action_type: str
    parameters: Dict[str, Any]
    duration: float
    priority: int = 1
    metadata: Dict[str, Any] = None

class ISensorInterface(ABC):
    """Interface for all sensor modules"""
    
    @abstractmethod
    def initialize(self) -> bool:
        """Initialize the sensor"""
        pass
    
    @abstractmethod
    def read(self) -> SensorData:
        """Read data from sensor"""
        pass
    
    @abstractmethod
    def is_available(self) -> bool:
        """Check if sensor is available"""
        pass
    
    @abstractmethod
    def cleanup(self):
        """Cleanup sensor resources"""
        pass

class IActuatorInterface(ABC):
    """Interface for all actuator modules"""
    
    @abstractmethod
    def initialize(self) -> bool:
        """Initialize the actuator"""
        pass
    
    @abstractmethod
    def execute_action(self, action: RobotAction) -> bool:
        """Execute an action"""
        pass
    
    @abstractmethod
    def stop(self):
        """Stop current action"""
        pass
    
    @abstractmethod
    def is_busy(self) -> bool:
        """Check if actuator is busy"""
        pass
    
    @abstractmethod
    def cleanup(self):
        """Cleanup actuator resources"""
        pass

class IAIInterface(ABC):
    """Interface for AI decision making modules"""
    
    @abstractmethod
    def initialize(self) -> bool:
        """Initialize the AI module"""
        pass
    
    @abstractmethod
    def make_decision(self, 
                     sensor_data: List[SensorData], 
                     current_state: RobotState,
                     context: Dict[str, Any]) -> RobotAction:
        """Make a decision based on inputs"""
        pass
    
    @abstractmethod
    def analyze_scene(self, image_data: bytes, prompt: str) -> Dict[str, Any]:
        """Analyze visual scene"""
        pass
    
    @abstractmethod
    def is_available(self) -> bool:
        """Check if AI service is available"""
        pass

class IPersonalityInterface(ABC):
    """Interface for personality/behavior modules"""
    
    @abstractmethod
    def get_personality_traits(self) -> Dict[str, float]:
        """Get current personality traits"""
        pass
    
    @abstractmethod
    def update_personality(self, experience: Dict[str, Any]):
        """Update personality based on experience"""
        pass
    
    @abstractmethod
    def influence_decision(self, base_action: RobotAction) -> RobotAction:
        """Modify action based on personality"""
        pass

class RobotConfig:
    """Configuration container for robot parameters"""
    
    def __init__(self, config_dict: Dict[str, Any] = None):
        self.config = config_dict or {}
        
    def get(self, key: str, default=None):
        """Get configuration value"""
        return self.config.get(key, default)
    
    def set(self, key: str, value: Any):
        """Set configuration value"""
        self.config[key] = value
    
    def update(self, new_config: Dict[str, Any]):
        """Update configuration"""
        self.config.update(new_config)

class ModuleManager:
    """Manages all robot modules"""
    
    def __init__(self):
        self.sensors: Dict[str, ISensorInterface] = {}
        self.actuators: Dict[str, IActuatorInterface] = {}
        self.ai_modules: Dict[str, IAIInterface] = {}
        self.personality: Optional[IPersonalityInterface] = None
        
    def register_sensor(self, name: str, sensor: ISensorInterface):
        """Register a sensor module"""
        self.sensors[name] = sensor
        
    def register_actuator(self, name: str, actuator: IActuatorInterface):
        """Register an actuator module"""
        self.actuators[name] = actuator
        
    def register_ai_module(self, name: str, ai_module: IAIInterface):
        """Register an AI module"""
        self.ai_modules[name] = ai_module
        
    def set_personality(self, personality: IPersonalityInterface):
        """Set personality module"""
        self.personality = personality
        
    def initialize_all(self) -> bool:
        """Initialize all modules"""
        success = True
        
        # Initialize sensors
        for name, sensor in self.sensors.items():
            if not sensor.initialize():
                print(f"Failed to initialize sensor: {name}")
                success = False
                
        # Initialize actuators
        for name, actuator in self.actuators.items():
            if not actuator.initialize():
                print(f"Failed to initialize actuator: {name}")
                success = False
                
        # Initialize AI modules
        for name, ai_module in self.ai_modules.items():
            if not ai_module.initialize():
                print(f"Failed to initialize AI module: {name}")
                success = False
                
        return success
    
    def cleanup_all(self):
        """Cleanup all modules"""
        for sensor in self.sensors.values():
            sensor.cleanup()
        for actuator in self.actuators.values():
            actuator.cleanup()
        # AI modules don't need cleanup typically
        
    def get_sensor_data(self) -> List[SensorData]:
        """Get data from all available sensors"""
        data = []
        for name, sensor in self.sensors.items():
            if sensor.is_available():
                try:
                    sensor_data = sensor.read()
                    data.append(sensor_data)
                except Exception as e:
                    print(f"Error reading sensor {name}: {e}")
        return data
