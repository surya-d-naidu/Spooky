import math
import time
import smbus
from servo import set_servo_angle as s

# ====== MPU6050 Configuration ======
# MPU6050 Registers and Address
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

# Create I2C bus
bus = smbus.SMBus(1)  # Bus 1 is typically used on newer Raspberry Pi versions

# ====== Robot Configuration ======
# Link lengths (units: e.g., centimeters)
l1 = 5.0    # upper leg length
l2 = 5.0    # lower leg length

# Vertical leg position limits
L_max = l1 + l2 * 0.9    # slightly bent knee position when foot is on ground
L_min = L_max * 0.7      # minimum vertical distance when foot is lifted

# Gait parameters
T = 2000                 # full gait cycle period in milliseconds
update_interval_ms = 20  # update every 20 ms for smooth motion

# Default hip angle
HIP_FIXED = 110  # Fixed hip angle (degrees)

# Servo biases and limits
servo_biases = {
    4: 4.0,   # Channel 4 has +4 degree bias
    3: 0.0,
    2: 0.0,
    7: 0.0,
    6: 0.0,
    5: 0.0,
    8: 0.0,
    9: 0.0,
    10: 0.0,
    14: 0.0,
    12: 0.0,
    11: 0.0,
}

# Leg configurations
legs = {
    'front_left':  {'hip': 4,  'knee': 3,  'calf': 2,  'phase': 0},
    'front_right': {'hip': 8,  'knee': 9,  'calf': 10, 'phase': -T/4},
    'hind_right':  {'hip': 14, 'knee': 12, 'calf': 11, 'phase': -T/2},
    'hind_left':   {'hip': 7,  'knee': 6,  'calf': 5,  'phase': -3*T/4},
}

# ====== PID Controller Configuration ======
# PID gains (tune these values for your specific robot)
PID_GAINS = {
    'roll': {'P': 2.0, 'I': 0.01, 'D': 0.5},
    'pitch': {'P': 2.0, 'I': 0.01, 'D': 0.5}
}

# Target angles (degrees) - typically level (0, 0)
TARGET_ANGLES = {
    'roll': 0.0,
    'pitch': 0.0
}

# Maximum correction values (degrees)
MAX_CORRECTION = {
    'roll': 15.0,
    'pitch': 15.0
}

# ====== MPU6050 Functions ======
def initialize_mpu6050():
    """Initialize the MPU6050 sensor."""
    try:
        # Wake up the MPU6050 (out of sleep mode)
        bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
        
        # Set sample rate to 1kHz
        bus.write_byte_data(MPU6050_ADDR, SMPLRT_DIV, 7)
        
        # Set low pass filter
        bus.write_byte_data(MPU6050_ADDR, CONFIG, 0)
        
        # Set gyro range to ±250 degrees/s
        bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, 0)
        
        # Set accelerometer range to ±2g
        bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, 0)
        
        print("MPU6050 initialized successfully")
        return True
    except Exception as e:
        print(f"Error initializing MPU6050: {e}")
        return False

def read_raw_data(addr):
    """Read raw 16-bit data from the MPU6050."""
    # Read high and low 8-bit values and combine
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr + 1)
    
    # Combine high and low for 16-bit value
    value = (high << 8) | low
    
    # Convert to signed value
    if value > 32767:
        value -= 65536
        
    return value

def get_imu_data():
    """Read and process data from MPU6050."""
    try:
        # Read accelerometer data
        acc_x = read_raw_data(ACCEL_XOUT_H)
        acc_y = read_raw_data(ACCEL_XOUT_H + 2)
        acc_z = read_raw_data(ACCEL_XOUT_H + 4)
        
        # Read gyroscope data
        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_XOUT_H + 2)
        gyro_z = read_raw_data(GYRO_XOUT_H + 4)
        
        # Convert accelerometer values to g
        acc_x = acc_x / 16384.0  # ±2g range
        acc_y = acc_y / 16384.0
        acc_z = acc_z / 16384.0
        
        # Convert gyro values to degrees/second
        gyro_x = gyro_x / 131.0  # ±250 degrees/s range
        gyro_y = gyro_y / 131.0
        gyro_z = gyro_z / 131.0
        
        # Calculate roll and pitch angles from accelerometer
        roll = math.atan2(acc_y, acc_z) * 180 / math.pi
        pitch = math.atan2(-acc_x, math.sqrt(acc_y * acc_y + acc_z * acc_z)) * 180 / math.pi
        
        return {
            'roll': roll,
            'pitch': pitch,
            'acc': (acc_x, acc_y, acc_z),
            'gyro': (gyro_x, gyro_y, gyro_z)
        }
    except Exception as e:
        print(f"Error reading IMU data: {e}")
        return {
            'roll': 0,
            'pitch': 0,
            'acc': (0, 0, 0),
            'gyro': (0, 0, 0)
        }

# ====== Robot Control Functions ======
def apply_bias(channel, angle):
    """Apply servo bias to angle."""
    return angle + servo_biases.get(channel, 0.0)

def compute_IK(L, l1, l2):
    """
    Compute knee and calf angles using inverse kinematics.
    Returns (knee_angle, calf_angle) in degrees.
    """
    # Clamp L within physical limits
    if L > (l1 + l2):
        L = l1 + l2
    if L < max(l1, l2) - min(l1, l2) + 0.1:
        L = max(l1, l2) - min(l1, l2) + 0.1

    try:
        # Compute theta in degrees
        theta = 180 - math.degrees(math.acos(((l1**2) + (l2**2) - (L**2)) / (2 * l1 * l2)))
    except ValueError as e:
        print(f"IK error: Invalid L value ({L}). Error: {e}")
        theta = 90.0  # Default to a safe value

    # Compute knee angle
    knee_angle = math.degrees(math.atan((l2 * math.sin(math.radians(theta))) /
                                       (l1 + l2 * math.cos(math.radians(theta)))))
    calf_angle = theta
    
    return knee_angle, calf_angle

# ====== PID Controller Class ======
class PIDController:
    def __init__(self, Kp, Ki, Kd, target=0, output_limits=None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = target
        self.output_limits = output_limits
        
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()
    
    def compute(self, measured_value):
        """Compute PID output value for given reference input and feedback."""
        current_time = time.time()
        dt = current_time - self.last_time
        
        # Handle first update or very small dt
        if dt <= 0:
            dt = 0.001
        
        # Calculate error
        error = self.target - measured_value
        
        # Calculate proportional term
        P = self.Kp * error
        
        # Calculate integral term with anti-windup
        self.integral += error * dt
        if self.output_limits:
            self.integral = max(min(self.integral, self.output_limits[1]/self.Ki), 
                               self.output_limits[0]/self.Ki)
        I = self.Ki * self.integral
        
        # Calculate derivative term (on measurement to avoid derivative kick)
        derivative = (measured_value - self.prev_error) / dt
        D = -self.Kd * derivative  # Note the negative sign
        
        # Calculate output
        output = P + I + D
        
        # Apply output limits if specified
        if self.output_limits:
            output = max(min(output, self.output_limits[1]), self.output_limits[0])
        
        # Store for next iteration
        self.prev_error = measured_value
        self.last_time = current_time
        
        return output

# ====== Robot Stabilization Function ======
def stabilized_crawl_gait():
    """
    Run the crawl gait with MPU6050-based stabilization.
    """
    print("Initializing MPU6050...")
    if not initialize_mpu6050():
        print("Failed to initialize MPU6050. Exiting.")
        return
    
    print("Calibrating IMU, please keep the robot still...")
    time.sleep(1)  # Wait for sensor to stabilize
    
    # Take initial readings to establish baseline
    imu_data = get_imu_data()
    initial_roll = imu_data['roll']
    initial_pitch = imu_data['pitch']
    
    # Adjust target angles based on initial position (if robot isn't perfectly level)
    TARGET_ANGLES['roll'] = initial_roll
    TARGET_ANGLES['pitch'] = initial_pitch
    
    print(f"Calibration complete. Target angles - Roll: {TARGET_ANGLES['roll']:.2f}°, Pitch: {TARGET_ANGLES['pitch']:.2f}°")
    
    # Create PID controllers
    roll_pid = PIDController(
        PID_GAINS['roll']['P'],
        PID_GAINS['roll']['I'],
        PID_GAINS['roll']['D'],
        TARGET_ANGLES['roll'],
        (-MAX_CORRECTION['roll'], MAX_CORRECTION['roll'])
    )
    
    pitch_pid = PIDController(
        PID_GAINS['pitch']['P'],
        PID_GAINS['pitch']['I'],
        PID_GAINS['pitch']['D'],
        TARGET_ANGLES['pitch'],
        (-MAX_CORRECTION['pitch'], MAX_CORRECTION['pitch'])
    )
    
    # Set all legs to ground position first for stability
    for leg_name, params in legs.items():
        knee_angle, calf_angle = compute_IK(L_max, l1, l2)
        s(params['hip'], apply_bias(params['hip'], HIP_FIXED))
        
        # Handle the special case for front-left knee
        if params['knee'] == 3:
            s(params['knee'], apply_bias(params['knee'], max(0, knee_angle - 20)))
        else:
            s(params['knee'], apply_bias(params['knee'], knee_angle))
            
        s(params['calf'], apply_bias(params['calf'], calf_angle))
    
    # Give time to stabilize
    time.sleep(0.5)
    
    print("Starting stabilized crawl gait. Press Ctrl+C to stop.")
    start_time = time.time()
    
    try:
        while True:
            # Get current time for gait calculation
            t_now_ms = (time.time() - start_time) * 1000
            
            # Read IMU data
            imu_data = get_imu_data()
            current_roll = imu_data['roll']
            current_pitch = imu_data['pitch']
            
            # Calculate PID corrections
            roll_correction = roll_pid.compute(current_roll)
            pitch_correction = pitch_pid.compute(current_pitch)
            
            # Print stabilization data every 10 cycles (to reduce console spam)
            if int(t_now_ms / update_interval_ms) % 10 == 0:
                print(f"\nIMU - Roll: {current_roll:.2f}°, Pitch: {current_pitch:.2f}°")
                print(f"Corrections - Roll: {roll_correction:.2f}°, Pitch: {pitch_correction:.2f}°\n")
            
            # Apply corrections to each leg
            for leg_name, params in legs.items():
                phase = params['phase']
                normalized_phase = ((t_now_ms + phase) % T) / T  # 0 to 1
                
                # Define leg behavior based on phase
                if 0.15 <= normalized_phase < 0.25:  # Lifting phase
                    lift_progress = (normalized_phase - 0.15) / 0.1
                    L_desired = L_max - lift_progress * (L_max - L_min)
                    leg_state = "LIFTING"
                elif 0.25 <= normalized_phase < 0.35:  # Lowering phase
                    lower_progress = (normalized_phase - 0.25) / 0.1
                    L_desired = L_min + lower_progress * (L_max - L_min)
                    leg_state = "LOWERING"
                else:  # On ground
                    L_desired = L_max
                    leg_state = "GROUND"
                
                # Apply stabilization adjustments to L_desired based on position
                # Front-left leg
                if leg_name == 'front_left':
                    L_desired += -pitch_correction - roll_correction
                # Front-right leg
                elif leg_name == 'front_right':
                    L_desired += -pitch_correction + roll_correction
                # Hind-left leg
                elif leg_name == 'hind_left':
                    L_desired += pitch_correction - roll_correction
                # Hind-right leg
                elif leg_name == 'hind_right':
                    L_desired += pitch_correction + roll_correction
                
                # Clamp L_desired within safe limits
                L_desired = max(min(L_desired, l1 + l2 * 0.95), L_min * 0.95)
                
                # Compute knee and calf angles
                knee_angle, calf_angle = compute_IK(L_desired, l1, l2)
                
                # Calculate horizontal position adjustment for forward motion
                # The stabilization occurs in the vertical movement, but we still want the robot to walk
                horizontal_offset = 0
                
                # Forward walking motion based on phase
                if normalized_phase < 0.15:  # Prepare to lift - move slightly back
                    prep_progress = normalized_phase / 0.15
                    horizontal_offset = -15.0 * 0.2 * prep_progress
                elif 0.15 <= normalized_phase < 0.35:  # Lifting and lowering - move from back to front
                    swing_progress = (normalized_phase - 0.15) / 0.2
                    horizontal_offset = -15.0 * 0.2 + swing_progress * 15.0
                elif 0.35 <= normalized_phase:  # Stance phase - gradually move from front to back
                    stance_progress = (normalized_phase - 0.35) / 0.65
                    horizontal_offset = 15.0 * 0.8 - stance_progress * 15.0 * 0.8
                
                # Apply horizontal offset to knee angle for forward motion
                modified_knee_angle = knee_angle - horizontal_offset
                
                # Send commands to the servos with biases applied
                hip_channel = params['hip']
                knee_channel = params['knee']
                calf_channel = params['calf']
                
                s(hip_channel, apply_bias(hip_channel, HIP_FIXED))
                
                # Handle the special case for front-left knee
                if knee_channel == 3:
                    delta_angle = modified_knee_angle - 20
                    if delta_angle <= 0:
                        s(knee_channel, apply_bias(knee_channel, 0))
                    else:
                        s(knee_channel, apply_bias(knee_channel, delta_angle))
                else:
                    s(knee_channel, apply_bias(knee_channel, modified_knee_angle))
                    
                s(calf_channel, apply_bias(calf_channel, calf_angle))
                
                # Debug output (once per second to reduce console spam)
                if int(t_now_ms / update_interval_ms) % 50 == 0 and leg_name == 'front_left':
                    print(f"Leg: {leg_name} | State: {leg_state} | L: {L_desired:.2f} | "
                          f"Knee: {modified_knee_angle:.2f}° | Calf: {calf_angle:.2f}°")
            
            time.sleep(update_interval_ms / 1000.0)
            
    except KeyboardInterrupt:
        print("\nStabilized crawl gait stopped.")
    except Exception as e:
        print(f"Error in stabilized crawl gait: {e}")

# ====== Complementary Filter Class (for improved angle estimation) ======
class ComplementaryFilter:
    def __init__(self, alpha=0.98):
        self.alpha = alpha
        self.roll = 0
        self.pitch = 0
        self.last_time = time.time()
    
    def update(self, acc_data, gyro_data):
        # Extract data
        acc_x, acc_y, acc_z = acc_data
        gyro_x, gyro_y, gyro_z = gyro_data
        
        # Calculate time difference
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Avoid division by zero
        if dt <= 0:
            dt = 0.001
        
        # Calculate angles from accelerometer
        accel_roll = math.atan2(acc_y, acc_z) * 180 / math.pi
        accel_pitch = math.atan2(-acc_x, math.sqrt(acc_y * acc_y + acc_z * acc_z)) * 180 / math.pi
        
        # Integrate gyro rates to get angles
        gyro_roll = self.roll + gyro_x * dt
        gyro_pitch = self.pitch + gyro_y * dt
        
        # Complementary filter combines the two estimates
        self.roll = self.alpha * gyro_roll + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * gyro_pitch + (1 - self.alpha) * accel_pitch
        
        return self.roll, self.pitch

# ====== Tuning Function for PID Controller ======
def tune_pid_controller():
    """
    Interactive PID tuning function. Use this to find optimal PID values for your robot.
    """
    print("PID Controller Tuning Mode")
    print("This mode helps you find the best PID parameters for your robot.")
    
    if not initialize_mpu6050():
        print("Failed to initialize MPU6050. Exiting.")
        return
    
    # Create complementary filter for angle estimation
    cf = ComplementaryFilter(alpha=0.98)
    
    # Start with default PID values
    p_value = float(input("Enter initial P value [2.0]: ") or "2.0")
    i_value = float(input("Enter initial I value [0.01]: ") or "0.01")
    d_value = float(input("Enter initial D value [0.5]: ") or "0.5")
    
    # Create PID controllers
    roll_pid = PIDController(p_value, i_value, d_value, 0, (-MAX_CORRECTION['roll'], MAX_CORRECTION['roll']))
    pitch_pid = PIDController(p_value, i_value, d_value, 0, (-MAX_CORRECTION['pitch'], MAX_CORRECTION['pitch']))
    
    print("\nTuning started. Press Ctrl+C to exit.")
    print("Position the robot on a level surface.")
    time.sleep(1)
    
    # Take initial readings to establish baseline
    for _ in range(10):  # Average multiple readings
        imu_data = get_imu_data()
        cf.update(imu_data['acc'], imu_data['gyro'])
        time.sleep(0.05)
    
    roll_target = cf.roll
    pitch_target = cf.pitch
    roll_pid.target = roll_target
    pitch_pid.target = pitch_target
    
    print(f"Baseline set - Roll: {roll_target:.2f}°, Pitch: {pitch_target:.2f}°")
    print("Now introduce disturbances to see how well the PID controller responds.")
    print("Observe the 'Error' and 'Correction' values.")
    
    try:
        while True:
            # Read IMU data
            imu_data = get_imu_data()
            roll, pitch = cf.update(imu_data['acc'], imu_data['gyro'])
            
            # Calculate PID corrections
            roll_correction = roll_pid.compute(roll)
            pitch_correction = pitch_pid.compute(pitch)
            
            # Calculate errors
            roll_error = roll_target - roll
            pitch_error = pitch_target - pitch
            
            # Display data
            print(f"\033[H\033[J")  # Clear screen (ANSI escape code)
            print(f"PID Values - P: {p_value:.4f}, I: {i_value:.4f}, D: {d_value:.4f}")
            print(f"Roll  - Current: {roll:.2f}°, Target: {roll_target:.2f}°, Error: {roll_error:.2f}°, Correction: {roll_correction:.2f}°")
            print(f"Pitch - Current: {pitch:.2f}°, Target: {pitch_target:.2f}°, Error: {pitch_error:.2f}°, Correction: {pitch_correction:.2f}°")
            print("\nControls:")
            print("P+ : Increase P by 0.1    P- : Decrease P by 0.1")
            print("I+ : Increase I by 0.005  I- : Decrease I by 0.005")
            print("D+ : Increase D by 0.05   D- : Decrease D by 0.05")
            print("R  : Reset to current position")
            print("S  : Save current values and exit")
            print("Ctrl+C : Exit without saving\n")
            
            # Check for keyboard input (non-blocking)
            if input_available():
                cmd = input().strip().lower()
                if cmd == 'p+':
                    p_value += 0.1
                elif cmd == 'p-':
                    p_value = max(0, p_value - 0.1)
                elif cmd == 'i+':
                    i_value += 0.005
                elif cmd == 'i-':
                    i_value = max(0, i_value - 0.005)
                elif cmd == 'd+':
                    d_value += 0.05
                elif cmd == 'd-':
                    d_value = max(0, d_value - 0.05)
                elif cmd == 'r':
                    # Reset targets to current position
                    roll_target = roll
                    pitch_target = pitch
                    roll_pid.target = roll_target
                    pitch_pid.target = pitch_target
                    roll_pid.integral = 0
                    pitch_pid.integral = 0
                    print("Position reset to current values.")
                elif cmd == 's':
                    print(f"Saving PID values: P={p_value}, I={i_value}, D={d_value}")
                    # Here you could save these to a config file
                    return p_value, i_value, d_value
                
                # Update PID controllers with new values
                roll_pid.Kp = p_value
                roll_pid.Ki = i_value
                roll_pid.Kd = d_value
                pitch_pid.Kp = p_value
                pitch_pid.Ki = i_value
                pitch_pid.Kd = d_value
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\nTuning ended.")
        return None

def input_available():
    """Check if there's input available without blocking."""
    import select
    import sys
    return select.select([sys.stdin], [], [], 0)[0]

# ====== Main Function ======
if __name__ == '__main__':
    print("Quadruped Robot with MPU6050 Stabilization")
    print("Select mode:")
    print("1) Stabilized Crawl Gait")
    print("2) PID Tuning")
    
    try:
        choice = input("Enter choice (1-2): ").strip()
        
        if choice == '1':
            stabilized_crawl_gait()
        elif choice == '2':
            pid_values = tune_pid_controller()
            if pid_values:
                p, i, d = pid_values
                print(f"Optimized PID values: P={p}, I={i}, D={d}")
                # Update the global PID gains
                PID_GAINS['roll'] = {'P': p, 'I': i, 'D': d}
                PID_GAINS['pitch'] = {'P': p, 'I': i, 'D': d}
                
                # Ask if user wants to try these values
                if input("Try these values with stabilized gait? (y/n): ").lower() == 'y':
                    stabilized_crawl_gait()
        else:
            print("Invalid choice!")
    
    except KeyboardInterrupt:
        print("\nProgram terminated.")
    except Exception as e:
        print(f"Error: {e}")
