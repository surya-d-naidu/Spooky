# Random Behavior Controller

A fun system that makes your robotic dog perform random movements, sequences, and special behaviors without any external input. Perfect for demonstrations, testing, or just entertainment!

## Features

### 🎯 Basic Random Mode
- Random walks, turns, sits, stands
- Configurable action durations and pause times
- Weighted probability system for different actions

### 🎭 Advanced Sequences
- **Playful**: Greeting waves, walks, and sits
- **Patrol**: Systematic area coverage
- **Exploration**: Curious looking around and walking
- **Dance**: Fun spinning and waving moves  
- **Exercise**: Forward/backward walking routine
- **Greeting**: Multiple hi waves in different directions

### 🌪️ Special Modes

#### Chaos Mode
- Rapid random actions
- Very short durations (0.5-3 seconds)
- Unpredictable stopping and starting
- Pure randomness!

#### Zen Mode  
- Peaceful, slow movements
- Long meditation pauses
- Only calm actions (sit, stand, gentle turns)
- Relaxing behavior

#### Party Mode
- Dance routines with multiple moves:
  - Spin Dance: Continuous rotations
  - Wave Dance: Repeated greetings
  - Bounce Dance: Sit-stand bouncing
  - Twist Dance: Left-right turning
- High energy and fun!

## Usage

### Quick Start
```bash
python3 run_random_behavior.py
```

### Menu Options
1. **Basic Random Movements** - Standard random behavior
2. **Advanced Sequences** - Predefined behavior patterns  
3. **Chaos Mode** - Crazy unpredictable movements
4. **Zen Mode** - Calm and peaceful behavior
5. **Party Mode** - Dance party time!
6. **Stop Current Mode** - Stop without exiting
0. **Exit** - Quit the program

### Programmatic Usage

```python
from random_behavior import RandomBehaviorController, ChaosMode, PartyMode
from main import animation_controller

# Basic random behavior
controller = RandomBehaviorController(animation_controller)
controller.start()

# Chaos mode
chaos = ChaosMode(animation_controller)  
chaos.start()

# Stop any mode
controller.stop()
```

## Customization

### Adjust Random Behavior Settings

```python
controller = RandomBehaviorController(animation_controller)

# Change action duration range (seconds)
controller.set_action_duration_range(1, 10)

# Change pause duration range (seconds)  
controller.set_pause_duration_range(0.5, 3)

# Adjust action probabilities
new_weights = {
    "walk_forward": 0.4,  # More walking
    "sit": 0.3,          # More sitting
    "hi": 0.1            # Less waving
}
controller.set_action_weights(new_weights)
```

### Available Actions
- `walk_forward` - Walk straight ahead
- `walk_backward` - Walk backwards  
- `turn_left` - Turn left in place
- `turn_right` - Turn right in place
- `sit` - Sit down
- `stand` - Stand up/idle position
- `hi` - Wave greeting
- `rotate_left` - Rotate left

## Safety Features

- Automatic stop on program exit
- Thread-safe operation
- Error handling and recovery
- Graceful shutdown with Ctrl+C

## Tips

- **Testing**: Start with Zen Mode for gentle movements
- **Demo**: Use Party Mode to show off capabilities
- **Development**: Use Basic Random for general testing
- **Fun**: Chaos Mode is hilarious but intense!

The system automatically handles animation transitions and ensures the robot returns to a safe idle position when stopped.
