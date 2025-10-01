# Spooky Robot - Random Behavior System

This directory contains the improved random behavior system for the Spooky robot with enhanced reliability and automatic startup capabilities.

## 🚀 Quick Start

### Manual Control
```bash
# Start the robot
./spooky_robot.sh start

# Check status
./spooky_robot.sh status

# View live logs
./spooky_robot.sh logs

# Stop the robot
./spooky_robot.sh stop

# Restart the robot
./spooky_robot.sh restart
```

### Interactive Mode (Original)
```bash
python3 run_random_behavior.py
```

### Continuous Mode (New - Perfect for SSH-free operation)
```bash
python3 run_continuous_random.py
```

## 📁 Files Overview

- **`random_controller.py`** - Core random behavior controller (improved with 6+ second actions)
- **`run_continuous_random.py`** - NEW: Continuous loop version for automatic startup
- **`spooky_robot.sh`** - NEW: Shell script for easy control and systemd integration
- **`spooky-robot.service`** - NEW: Systemd service file template
- **`run_random_behavior.py`** - Original interactive menu system
- **`behavior_sequences.py`** - Advanced behavior sequences
- **`special_modes.py`** - Special modes (Chaos, Zen, Party)

## ⚙️ Improvements Made

### 1. Enhanced Timing
- **Minimum action duration**: 6 seconds (was 2 seconds)
- **Maximum action duration**: 15 seconds (was 8 seconds)
- **Minimum pause duration**: 2 seconds (was 1 second)
- **Maximum pause duration**: 8 seconds (was 5 seconds)

### 2. Better Error Handling
- Exponential backoff on errors
- Maximum consecutive error limit
- Automatic recovery and restart
- Comprehensive logging

### 3. Continuous Operation
- Runs in an infinite loop until stopped
- Automatic restart on unexpected failures
- Graceful shutdown handling
- Status monitoring

### 4. SSH-Free Operation
- Shell script for easy control
- Systemd integration ready
- Background operation support
- PID file management

## 🔧 System Integration

### For Automatic Startup on Boot:

1. **Copy the service file:**
   ```bash
   sudo cp spooky-robot.service /etc/systemd/system/
   ```

2. **Enable and start the service:**
   ```bash
   sudo systemctl daemon-reload
   sudo systemctl enable spooky-robot.service
   sudo systemctl start spooky-robot.service
   ```

3. **Monitor the service:**
   ```bash
   sudo systemctl status spooky-robot.service
   journalctl -u spooky-robot.service -f
   ```

### Alternative: Using the Shell Script with Cron
Add to crontab for startup on reboot:
```bash
@reboot /home/surya/Spooky/Spooky/utils/animation_pys/random_behavior/spooky_robot.sh start
```

## 📊 Monitoring and Logs

- **Main log file**: `/tmp/spooky_startup.log`
- **Python log file**: `/tmp/spooky_random_behavior.log`
- **PID file**: `/tmp/spooky_random_behavior.pid`

View logs in real-time:
```bash
tail -f /tmp/spooky_startup.log
# or
./spooky_robot.sh logs
```

## 🎮 Available Actions

The robot randomly performs these actions with weighted probabilities:

- **Walk Forward** (25% chance) - 6-15 seconds
- **Turn Left** (15% chance) - 6-15 seconds  
- **Turn Right** (15% chance) - 6-15 seconds
- **Walk Backward** (10% chance) - 6-15 seconds
- **Rotate Left** (10% chance) - 6-15 seconds
- **Sit** (10% chance) - 6-15 seconds
- **Stand** (10% chance) - 6-15 seconds
- **Wave Hi** (5% chance) - 6-15 seconds

Between each action, the robot pauses for 2-8 seconds.

## 🔍 Troubleshooting

### Check if running:
```bash
./spooky_robot.sh status
```

### Manual stop if needed:
```bash
pkill -f run_continuous_random.py
rm -f /tmp/spooky_random_behavior.pid
```

### Reset everything:
```bash
./spooky_robot.sh stop
sleep 2
./spooky_robot.sh start
```

### View recent errors:
```bash
grep -i error /tmp/spooky_startup.log
```

## 🚫 Stopping the Robot

Always use proper shutdown methods:
- `./spooky_robot.sh stop` (recommended)
- `sudo systemctl stop spooky-robot.service` (if using systemd)
- `Ctrl+C` (if running in foreground)

This ensures graceful shutdown and proper cleanup of resources.

## 🔄 Migration from Old System

The new system is backward compatible. You can still use:
- `run_random_behavior.py` for interactive mode
- All existing behavior sequences and special modes

The new continuous mode (`run_continuous_random.py`) is designed for unattended operation.
