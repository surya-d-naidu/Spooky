# Robot Movement Simulation Suite

This folder contains comprehensive visualization and analysis tools for the quadruped robot's movement system.

## 🎯 Available Simulations

### 1. **Gait Pattern Simulation** (`gait_simulator.py`)
- **Timeline Analysis**: Shows when each leg is in swing vs stance phase
- **Height Variations**: Plots leg extension changes during gait cycle  
- **Stability Analysis**: Visualizes support patterns and center of gravity
- **Phase Relationships**: Analyzes timing between all four legs

### 2. **Inverse Kinematics Visualization** (`ik_visualizer.py`)
- **Leg Geometry**: Shows joint positions for different leg extensions
- **Reachable Workspace**: Plots all possible foot positions
- **Angle vs Length**: Graphs joint angles vs leg extension
- **Gait Trajectory**: Visualizes foot path during walking cycle

### 3. **Servo Analysis** (`servo_analyzer.py`)
- **Channel Mapping**: Shows which servo controls which joint
- **Bias Analysis**: Analyzes servo calibration offsets
- **Angle Ranges**: Computes expected servo angle ranges during gait
- **Calibration Sequence**: Generates test code for servo calibration

### 4. **Real-time Simulation** (`realtime_simulator.py`)
- **Live Servo Angles**: Real-time display of all servo positions
- **Leg State Visualization**: Shows which legs are swinging/standing
- **Gait Phase Wheel**: Circular diagram of gait timing
- **Text Mode**: Works without graphics for debugging

## 🚀 Quick Start

### Run All Simulations
```bash
cd v3/simulations
python run_simulations.py
```

### Install Dependencies (if needed)
```bash
pip install -r requirements.txt
```

### Run Individual Simulations
```bash
# Gait analysis
python gait_simulator.py

# Kinematics analysis  
python ik_visualizer.py

# Servo analysis (no graphics needed)
python servo_analyzer.py

# Real-time simulation
python realtime_simulator.py
```

## 📊 What You'll Learn

### **Gait Timing**
- How crawl gait coordinates 4 legs
- When each leg lifts and touches down
- Stability margins during walking
- Phase relationships between legs

### **Leg Mechanics** 
- Joint angle ranges for each servo
- Reachable workspace of each leg
- Inverse kinematics solutions
- Foot trajectory patterns

### **Servo Behavior**
- Individual servo calibration needs
- Expected angle ranges during movement
- Bias corrections for each channel
- Real-time servo positions

### **System Integration**
- How gait parameters affect movement
- Timing relationships between components
- Real-time system behavior
- Debugging movement issues

## 🎨 Visualization Features

### **Interactive Plots**
- Zoom, pan, and explore data
- Multiple synchronized views
- Real-time updates
- Export capabilities

### **Color Coding**
- Red: Front Left leg
- Blue: Front Right leg  
- Green: Hind Left leg
- Orange: Hind Right leg

### **Analysis Outputs**
- Numerical summaries
- Range calculations
- Timing analysis
- Performance metrics

## 🔧 Configuration

All simulations use parameters from `../robot_config.py`:
- Leg lengths (`l1`, `l2`)
- Gait timing (`T`, `update_interval_ms`) 
- Servo channels and biases
- Step length and body shifts

Modify `robot_config.py` to see how changes affect the simulation results.

## 📈 Use Cases

### **Robot Design**
- Optimize leg dimensions
- Adjust gait parameters
- Validate servo selection
- Check workspace coverage

### **Calibration**
- Determine servo biases
- Test movement ranges
- Verify timing sequences
- Debug servo issues

### **Performance Analysis**
- Analyze stability margins
- Optimize gait speed
- Study power consumption
- Evaluate smoothness

### **Education & Understanding**
- Learn inverse kinematics
- Understand gait coordination
- Visualize robot behavior
- Debug movement problems

## 🎮 Interactive Features

### **Real-time Controls** (in realtime_simulator.py)
- Start/stop simulation
- Adjust speed
- Change duration
- Switch visualization modes

### **Parameter Exploration**
- Modify `robot_config.py` values
- Re-run simulations instantly
- Compare different configurations
- Save/load parameter sets

## 🐛 Troubleshooting

### **Import Errors**
```bash
# Install missing packages
pip install matplotlib numpy scipy

# Or use the installer
python run_simulations.py
# Choose option 6
```

### **No Display Available**
- Use `servo_analyzer.py` (text-only)
- Use text mode in `realtime_simulator.py`
- Run on system with display support

### **Performance Issues**
- Reduce simulation duration
- Lower update rate in config
- Close other applications
- Use text-mode simulations

## 📚 Mathematical Background

### **Inverse Kinematics Formula**
```
θ = 180° - acos(((l₁² + l₂² - L²) / (2·l₁·l₂)))
knee_angle = atan((l₂·sin(θ)) / (l₁ + l₂·cos(θ)))
calf_angle = θ
```

### **Gait Phase Calculation**
```
normalized_phase = ((time + phase_offset) % period) / period
if 0.15 ≤ phase < 0.35: swing
else: stance
```

### **Leg Height Calculation**
```
L = L_max - progress × (L_max - L_min)  [during lift]
L = L_min + progress × (L_max - L_min)  [during lower]
L = L_max                               [on ground]
```

## 🎓 Educational Value

These simulations help you understand:
- **Robotics Fundamentals**: Kinematics, gait patterns, servo control
- **Mathematics**: Trigonometry, inverse functions, phase relationships
- **Programming**: Real-time systems, visualization, data analysis
- **Engineering**: System integration, calibration, performance optimization

Perfect for students, researchers, and anyone wanting to understand quadruped robot locomotion!

---

**Ready to explore robot movement? Run `python run_simulations.py` to get started! 🤖**
