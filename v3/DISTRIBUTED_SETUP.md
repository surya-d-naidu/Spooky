# Distributed Robot System Configuration

This setup splits the system between:
- **Raspberry Pi (Robot)**: Movement, sensors, robot control
- **Laptop**: Ollama AI API server
- **WiFi**: Communication between them

## Setup Instructions

### 1. Laptop Setup (AI Server)

```bash
# Install Ollama
curl -fsSL https://ollama.ai/install.sh | sh

# Pull the vision model
ollama pull llava:7b

# Start Ollama server (accessible from network)
OLLAMA_HOST=0.0.0.0:11434 ollama serve
```

### 2. Raspberry Pi Setup (Robot)

```bash
# Install Python dependencies
pip install opencv-python numpy requests pillow

# Install hardware libraries (for servo control)
pip install RPi.GPIO adafruit-circuitpython-pca9685

# Clone/copy the v3 folder to the Pi
```

### 3. Network Configuration

1. **Find Laptop IP**:
   ```bash
   # On laptop (Windows)
   ipconfig
   # Look for your WiFi adapter IP (e.g., 192.168.1.100)
   
   # On laptop (Linux/Mac)
   ip addr show
   # or
   ifconfig
   ```

2. **Test Connection**:
   ```bash
   # From Raspberry Pi, test Ollama connection
   curl http://LAPTOP_IP:11434/api/version
   ```

### 4. Robot Configuration

Edit the robot configuration to point to your laptop:

```python
# In robot_config.py or when creating the robot
config = {
    'ai': {
        'ollama_url': 'http://192.168.1.100:11434',  # Replace with laptop IP
        'model': 'llava:7b',
        'timeout': 30
    },
    'camera': {
        'camera_id': 0,  # USB camera or Pi camera
        'width': 640,
        'height': 480
    }
}
```

## File Structure

```
Laptop (AI Server):
├── Just run Ollama service
└── No robot code needed

Raspberry Pi (Robot):
├── v3/ (full robot code)
├── robot_main.py (main entry point)
├── robot_config.py (Pi-specific config)
└── requirements_pi.txt (Pi dependencies)
```

## Usage

1. **Start AI server on laptop**:
   ```bash
   OLLAMA_HOST=0.0.0.0:11434 ollama serve
   ```

2. **Run robot on Raspberry Pi**:
   ```bash
   python robot_main.py
   ```

## How Movement Works

The robot uses a **crawl gait** with 4-leg coordination:

1. **Servo Control**: 12 servos (3 per leg: hip, knee, calf)
2. **Gait Pattern**: Crawl gait - one leg moves at a time
3. **Movement Types**:
   - `crawl_gait_loop()`: Forward walking
   - `rotate_in_place()`: Turn left/right
   - Various behaviors: explore, play, investigate, etc.

4. **Physical Movement**:
   - Each leg follows a swing/stance cycle
   - Inverse kinematics calculates joint angles
   - Body shifts for stability during leg lifts
   - Servo biases for calibration

## AI Decision Flow

```
Camera (Pi) → Image → Ollama API (Laptop) → Decision → Movement (Pi)
     ↑                    ↓
  WiFi ←------------------ WiFi
```

1. Pi captures camera image
2. Sends image to laptop Ollama API over WiFi
3. Ollama analyzes image and decides action
4. Pi receives decision and executes movement
5. Repeats every 3 seconds

## Troubleshooting

- **Connection Issues**: Check firewall, WiFi network
- **Slow AI**: Increase decision_interval, check WiFi speed  
- **Movement Issues**: Check servo wiring, power supply
- **Camera Issues**: Try different camera_id values
