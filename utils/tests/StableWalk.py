import math
import time
from servo import set_servo_angle as s  # s(channel, angle)
from mpu6050 import mpu6050        # Assuming you have an mpu6050 library

# ----- Configuration -----
# Link lengths (units: e.g., centimeters)
l1 = 5.0    # upper leg length
l2 = 5.0    # lower leg length

L_max = l1 + l2          # full vertical extension (foot on ground)
L_min = L_max * 0.6      # minimum vertical distance when foot is lifted

T = 1000                 # full gait cycle period in milliseconds
update_interval_ms = 40  # update every 20 ms for smooth motion

# Base hip angle (locked)
HIP_FIXED = 110  

# Define servo channels for each leg and phase offsets.
legs = {
    'front_left':  {'hip': 2,  'knee': 1,  'calf': 0,  'phase': T/2, 'pos': 'front_left'},
    'hind_left':   {'hip': 5,  'knee': 4,  'calf': 3,  'phase': 0,   'pos': 'hind_left'},
    'front_right': {'hip': 10, 'knee': 11, 'calf': 12, 'phase': 0,   'pos': 'front_right'},
    'hind_right':  {'hip': 13, 'knee': 14, 'calf': 15, 'phase': T/2, 'pos': 'hind_right'},
}

# New Variable for Forward/Backward Stepping
delta_amp = 10.0  # amplitude (in degrees) for front/back motion
T_delta = T       # period for delta motion

# Gains for altitude adjustment (tune these experimentally)
k_pitch_alt = 0.1   # gain for forward/backward tilt correction on altitude
k_roll_alt  = 0.1   # gain for side tilt correction on altitude

# Initialize the MPU6050 IMU (adjust address or bus if necessary)
imu = mpu6050(0x68)

# ----- Inverse Kinematics Function -----
def compute_IK(L, l1, l2):
    """
    Given desired vertical distance L (hip-to-foot) and leg link lengths,
    compute the knee and calf angles using the formula:
    
      theta = 180 - degrees( acos( ((l1**2)+(l2**2)-L**2) / (2*l1*l2) ) )
      knee_angle = degrees( atan( (l2*sin(radians(theta))) / (l1 + l2*cos(radians(theta))) ) )
      calf_angle = theta
      
    Returns (knee_angle, calf_angle) in degrees.
    """
    # Clamp L within physical limits.
    if L > L_max:
        L = L_max
    if L < L_min:
        L = L_min

    try:
        theta = 180 - math.degrees(math.acos(((l1**2) + (l2**2) - (L**2)) / (2 * l1 * l2)))
        print(f"(Debug) theta: {theta:.2f}°")
    except ValueError:
        print("Error: Invalid L value. Check L, l1, and l2.")
        theta = 0.0

    knee_angle = math.degrees(math.atan((l2 * math.sin(math.radians(theta))) /
                                          (l1 + l2 * math.cos(math.radians(theta)))))
    calf_angle = theta  # using supplement configuration
    return knee_angle, calf_angle

# ----- Desired Vertical Altitude Function -----
def desired_altitude(t, phase, T, L_max, L_min):
    """
    Compute base desired vertical distance L(t) for a leg.
    When sin() = +1, foot is lifted (L = L_min);
    When sin() = -1, foot is on the ground (L = L_max).
    """
    val = math.sin(2 * math.pi * ((t + phase) % T) / T)
    L = (L_max + L_min) / 2 - (L_max - L_min) / 2 * val
    return L

# ----- Altitude Adjustment Based on IMU Data -----
def adjust_altitude(L_desired, leg_position, pitch, roll):
    """
    Adjusts the desired vertical distance L based on IMU pitch and roll.
    For example:
      - If the robot tilts forward (positive pitch), the front legs will
        have L reduced (contract) to help lift the front.
      - Conversely, hind legs will have L increased.
      - For roll, left/right adjustments are made similarly.
    """
    # Adjust for pitch.
    if 'front' in leg_position:
        L_desired -= k_pitch_alt * pitch
    elif 'hind' in leg_position:
        L_desired += k_pitch_alt * pitch

    # Adjust for roll.
    if 'left' in leg_position:
        L_desired += k_roll_alt * roll
    elif 'right' in leg_position:
        L_desired -= k_roll_alt * roll

    # Clamp adjusted L.
    if L_desired > L_max:
        L_desired = L_max
    if L_desired < L_min:
        L_desired = L_min

    return L_desired

# ----- Delta Calculation for Front/Back Motion -----
def compute_delta(t, phase, T_delta, delta_amp):
    """
    Compute a periodic delta (in degrees) as a sine wave.
    This value will be added to the computed knee angle to produce forward/backward stepping.
    """
    delta = delta_amp * math.sin(2 * math.pi * ((t + phase) % T_delta) / T_delta)
    return delta

# ----- IMU Reading Function -----
def read_imu_angles():
    """
    Reads raw accelerometer data from the MPU6050 and computes pitch and roll angles.
    
    Returns:
      pitch: rotation around the lateral axis (forward/back tilt)
      roll:  rotation around the longitudinal axis (side tilt)
    """
    data = imu.get_accel_data()  # returns a dict with keys 'x', 'y', 'z'
    ax = data['x']
    ay = data['y']
    az = data['z']
    
    # Calculate pitch and roll in degrees.
    pitch = math.degrees(math.atan2(ax, math.sqrt(ay**2 + az**2)))
    roll  = math.degrees(math.atan2(ay, math.sqrt(ax**2 + az**2)))
    return pitch, roll

# ----- Main Gait Loop with Altitude Adjustment for Stability -----
def gait_loop():
    print("Starting stabilized trot gait loop (hips locked at 110°). Press Ctrl+C to stop.")
    start_time = time.time()  # seconds
    try:
        while True:
            t_now_ms = (time.time() - start_time) * 1000  # current time in ms

            # Get IMU data for stabilization.
            pitch, roll = read_imu_angles()
            print(f"(Debug) IMU: pitch={pitch:.2f}°, roll={roll:.2f}°")
            
            for leg_name, params in legs.items():
                phase = params['phase']
                pos = params['pos']
                # Compute base desired vertical altitude L for this leg.
                L_base = desired_altitude(t_now_ms, phase, T, L_max, L_min)
                # Adjust L based on IMU data for stability.
                L_desired = adjust_altitude(L_base, pos, pitch, roll)
                
                # Compute IK for knee and calf angles using the adjusted altitude.
                knee_angle, calf_angle = compute_IK(L_desired, l1, l2)
                # Compute delta for front/back stepping.
                delta = compute_delta(t_now_ms, phase, T_delta, delta_amp)
                modified_knee_angle = knee_angle + delta

                # Hip remains fixed at 110°.
                hip_angle = HIP_FIXED

                # Debug print for leg.
                print(f"{leg_name:12s} | L_base: {L_base:5.2f} -> L_adj: {L_desired:5.2f} | "
                      f"Hip: {hip_angle:5.2f}°, Knee: {modified_knee_angle:5.2f}° "
                      f"(base: {knee_angle:5.2f}°+delta:{delta:5.2f}°), Calf: {calf_angle:5.2f}°")

                # Send commands to the servos.
                s(params['hip'], hip_angle)
                s(params['knee'], modified_knee_angle)
                s(params['calf'], calf_angle)
            time.sleep(update_interval_ms / 1000.0)
    except KeyboardInterrupt:
        print("Gait loop stopped.")

if __name__ == '__main__':
    gait_loop()
