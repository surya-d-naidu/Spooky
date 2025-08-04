"""
Raspberry Pi Robot Configuration
For distributed setup with laptop AI server
"""

# Get your laptop's IP address and update this
LAPTOP_IP = "192.168.1.100"  # CHANGE THIS TO YOUR LAPTOP'S IP

# Robot configuration for Raspberry Pi
config = {
    'robot_type': 'quadruped_dog',
    'decision_interval': 4.0,  # Slower for WiFi latency
    
    # AI runs on laptop
    'ai': {
        'ollama_url': f'http://{LAPTOP_IP}:11434',
        'model': 'llava:7b',
        'timeout': 45  # Longer timeout for WiFi
    },
    
    # Camera on Pi
    'camera': {
        'camera_id': 0,  # Try 1 if 0 doesn't work
        'width': 320,    # Smaller for faster WiFi transmission
        'height': 240,
        'fps': 10        # Lower FPS for WiFi
    },
    
    # Movement on Pi
    'movement': {
        'servo_update_rate': 50,  # Hz
        'gait_speed': 'normal'
    },
    
    # Dog personality
    'personality': {
        'base_traits': {
            'energy': 0.7,
            'curiosity': 0.8,
            'playfulness': 0.6,
            'sociability': 0.9,
            'courage': 0.6,
            'independence': 0.3,
            'loyalty': 0.8,
            'alertness': 0.7
        }
    },
    
    # WiFi optimization
    'network': {
        'image_quality': 70,     # JPEG quality (lower = faster)
        'retry_attempts': 3,
        'connection_timeout': 10
    }
}

# Hardware pin mappings for Raspberry Pi
SERVO_CHANNELS = {
    'front_left':  {'hip': 4,  'knee': 3,  'calf': 2},
    'front_right': {'hip': 8,  'knee': 9,  'calf': 10},
    'hind_right':  {'hip': 14, 'knee': 12, 'calf': 11},
    'hind_left':   {'hip': 7,  'knee': 6,  'calf': 5}
}

# PCA9685 I2C settings
PCA9685_ADDRESS = 0x40
I2C_BUS = 1

# Gait and servo parameters (for trot.py)
config['gait'] = {
    'l1': 5.0,    # upper leg length
    'l2': 5.0,    # lower leg length
    'L_max': 5.0 + 5.0 * 0.9,
    'L_min': (5.0 + 5.0 * 0.9) * 0.7,
    'T': 2000,    # full gait cycle period in ms
    'update_interval_ms': 20,
    'HIP_FIXED': 110,
    'servo_biases': {
        4: -10.0,
        3: 0.0,
        2: 4.0,
        7: 0.0,
        6: 0.0,
        5: 0.0,
        8: 0.0,
        9: 0.0,
        10: 0.0,
        14: 0.0,
        12: 0.0,
        11: 0.0,
    },
    'legs': {
        'front_left':  {'hip': 4,  'knee': 3,  'calf': 2,  'phase': 0},
        'front_right': {'hip': 8,  'knee': 9,  'calf': 10, 'phase': -2000/4},
        'hind_right':  {'hip': 14, 'knee': 12, 'calf': 11, 'phase': -2000/2},
        'hind_left':   {'hip': 7,  'knee': 6,  'calf': 5,  'phase': -3*2000/4},
    },
    'step_length': 45.0,
    'shift_amount': 3.0,
    'shift_duration': 0.1,
    'turn_offset_knee': 5.0,
    'turn_offset_calf': 5.0,
}
