import math
import time
from servo import set_servo_angle as s  # s(channel, angle)
from mpu6050 import mpu6050        # Assuming you have an mpu6050 library

# ----- Configuration -----
# Link lengths (units: e.g., centimeters)
l1 = 5.0    # upper leg length
l2 = 5.0    # lower leg length

# The vertical distance from hip to foot will be between L_min and L_max.
L_max = l1 + l2          # full extension
L_min = L_max * 0.6      # minimum contracted length

# Base altitude for standing (midpoint between L_max and L_min)
BASE_ALTITUDE = (L_max + L_min) / 2

# Base hip angle (locked)
HIP_FIXED = 110

# Define servo channels for each leg.
legs = {
    'front_left':  {'hip': 2,  'knee': 1,  'calf': 0,  'pos': 'front_left'},
    'hind_left':   {'hip': 5,  'knee': 4,  'calf': 3,  'pos': 'hind_left'},
    'front_right': {'hip': 10, 'knee': 11, 'calf': 12, 'pos': 'front_right'},
    'hind_right':  {'hip': 13, 'knee': 14, 'calf': 15, 'pos': 'hind_right'},
}

# Gains for altitude adjustment based on IMU data (tune these experimentally)
k_pitch_alt = 0.1   # forward/back correction gain
k_roll_alt  = 0.1   # side correction gain

# Initialize the MPU6050 IMU (adjust address or bus if necessary)
imu = mpu6050(0x68)

# ----- Inverse Kinematics Function -----
def compute_IK(L, l1, l2):
    """
    Compute knee and calf angles from desired vertical distance L.
    The formula used is:
      theta = 180 - degrees(acos((l1^2 + l2^2 - L^2) / (2*l1*l2)))
      knee_angle = degrees(atan((l2*sin(radians(theta))) / (l1 + l2*cos(radians(theta)))))
      calf_angle = theta   (supplementary angle configuration)
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
    calf_angle = theta  # using supplement configuration for the calf joint
    return knee_angle, calf_angle

# ----- Altitude Adjustment Based on IMU Data -----
def adjust_altitude(L_base, leg_position, pitch, roll):
    """
    Adjust the base altitude L based on IMU pitch and roll data.
    
    For example:
      - Front legs: if pitch is positive (tilt forward), reduce L (contract leg) to lift the front.
      - Hind legs: if pitch is positive, increase L.
      - Left legs: if roll is positive (tilt right), increase L.
      - Right legs: if roll is positive, decrease L.
    """
    L_adjusted = L_base

    # Adjust for pitch.
    if 'front' in leg_position:
        L_adjusted -= k_pitch_alt * pitch
    elif 'hind' in leg_position:
        L_adjusted += k_pitch_alt * pitch

    # Adjust for roll.
    if 'left' in leg_position:
        L_adjusted += k_roll_alt * roll
    elif 'right' in leg_position:
        L_adjusted -= k_roll_alt * roll

    # Clamp the adjusted altitude.
    if L_adjusted > L_max:
        L_adjusted = L_max
    if L_adjusted < L_min:
        L_adjusted = L_min

    return L_adjusted

# ----- IMU Reading Function -----
def read_imu_angles():
    """
    Reads accelerometer data from the MPU6050 and computes pitch and roll angles.
    
    Returns:
      pitch: rotation around the lateral axis (forward/back tilt)
      roll:  rotation around the longitudinal axis (side tilt)
    """
    data = imu.get_accel_data()  # returns dictionary with keys 'x', 'y', 'z'
    ax = data['x']
    ay = data['y']
    az = data['z']
    
    # Calculate pitch and roll in degrees.
    pitch = math.degrees(math.atan2(ax, math.sqrt(ay**2 + az**2)))
    roll  = math.degrees(math.atan2(ay, math.sqrt(ax**2 + az**2)))
    return pitch, roll

# ----- Main Loop for Testing Stabilization -----
def stabilization_test():
    print("Starting stabilization test (standing still). Adjust the platform to see leg corrections.")
    try:
        while True:
            # Read IMU angles.
            pitch, roll = read_imu_angles()
            print(f"(Debug) IMU: pitch={pitch:.2f}°, roll={roll:.2f}°")
            
            # For each leg, compute adjusted altitude and then compute IK for joint angles.
            for leg_name, params in legs.items():
                pos = params['pos']
                # Use BASE_ALTITUDE as the starting point.
                L_base = BASE_ALTITUDE
                L_desired = adjust_altitude(L_base, pos, pitch, roll)
                
                # Compute inverse kinematics.
                knee_angle, calf_angle = compute_IK(L_desired, l1, l2)
                
                # Hip angle remains locked.
                hip_angle = HIP_FIXED

                # Debug print for leg.
                print(f"{leg_name:12s} | Base L: {L_base:5.2f} -> Adjusted L: {L_desired:5.2f} | "
                      f"Hip: {hip_angle:5.2f}°, Knee: {knee_angle:5.2f}°, Calf: {calf_angle:5.2f}°")

                # Send commands to the servos.
                s(params['hip'], hip_angle)
                s(params['knee'], knee_angle)
                s(params['calf'], calf_angle)
            print("-" * 80)
            time.sleep(0.05)  # update every 50 ms
    except KeyboardInterrupt:
        print("Stabilization test stopped.")

if __name__ == '__main__':
    stabilization_test()
