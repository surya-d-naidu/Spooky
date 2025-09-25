# Quadruped Robot Control System

This system provides a Flask-based web interface for controlling your quadruped robot with proper animation handling.

## Features

- **Web-based Control Interface**: Easy-to-use web interface with directional controls
- **Animation Management**: Proper handling of looped animations (first/last frame optimization)
- **Reverse Animation Support**: Play animations backwards for opposite movements
- **Remote Control**: Control via web browser from any device on the network
- **Real-time Status**: Live status updates and robot state monitoring
- **Emergency Stop**: Quick emergency stop functionality
- **Keyboard Controls**: Use arrow keys or WASD for control

## Available Animations

- `walk_forward` - Forward walking gait
- `walk_backward` - Backward walking (uses walk_forward in reverse)
- `rotate_left` - Left rotation
- `rotate_right` - Right rotation (uses rotate_left in reverse)
- `stand` - Standing position
- `sit` - Sitting position

## Movement Logic

The system uses a smart approach to animations:
- **Forward Walk**: Uses `walk_forward.json`
- **Backward Walk**: Uses `walk_forward.json` played in reverse
- **Left Turn**: Uses `rotate_left.json`
- **Right Turn**: Uses `rotate_left.json` played in reverse

This ensures consistent movement patterns and eliminates the need for separate animation files.

## Quick Start

1. **Start the server:**
   ```bash
   cd /home/spooky/Spooky/utils/animation_pys
   ./start_server.sh
   ```

2. **Access the control interface:**
   - Main interface: http://localhost:5000
   - Simple control panel: http://localhost:5000/control

3. **Use keyboard controls:**
   - Arrow keys or WASD for movement
   - Spacebar or Escape to stop

## API Endpoints

- `GET /api/move/<direction>` - Move robot (forward/backward/left/right/stop/stand/sit)
- `GET /api/emergency_stop` - Emergency stop
- `GET /api/status` - Get robot status
- `GET /api/custom_animation/<name>` - Play custom animation
- `GET /api/reverse_animation/<name>` - Play animation in reverse

## Animation Format

The system handles animations with duplicate first/last frames properly:
- First frame is played at the start of the loop
- Middle frames are looped continuously
- Last frame is skipped during loops (since it's the same as first frame)
- Last frame is only played when stopping the animation

## Files Structure

```
animation_pys/
├── flask_server.py          # Main Flask web server
├── main.py                  # Animation controller and legacy functions
├── servo.py                 # Servo control functions
├── requirements.txt         # Python dependencies
├── start_server.sh         # Startup script
├── templates/              # Web interface templates
│   ├── index.html          # Main control interface
│   └── control.html        # Simple control panel
└── animation_j/            # Animation JSON files
    ├── walk_forward.json
    ├── walk_backward.json
    ├── rotate_left.json
    ├── rotate_right.json
    ├── stand.json
    └── sit.json
```

## Dependencies

- Flask (web framework)
- Adafruit-PCA9685 (servo control)

Dependencies are automatically installed when using `start_server.sh`.

## Troubleshooting

1. **Server won't start**: Make sure virtual environment is activated and dependencies are installed
2. **Robot not responding**: Check servo connections and PCA9685 board
3. **Animation issues**: Verify JSON animation files are properly formatted
4. **Network access**: Server runs on 0.0.0.0:5000 for network access

## Safety

- Always use the emergency stop if needed
- Ensure robot has enough space to move
- Monitor robot during operation
- Use proper power supply for servos
