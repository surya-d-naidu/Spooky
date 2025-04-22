import math
import time
from servo import set_servo_angle as s  # s(channel, angle)

# ----- Configuration -----
# Link lengths (units: e.g., centimeters)
l1 = 5.0    # upper leg length
l2 = 5.0    # lower leg length

L_max = l1 + l2          # full vertical extension (foot on ground)
L_min = L_max * 0.6      # minimum vertical distance when foot is lifted

T = 1000                 # full gait cycle period in milliseconds
update_interval_ms = 20  # update every 20 ms for smooth motion

HIP_FIXED = 110  # Fixed hip angle (degrees)

# Define servo channels for each leg and phase offsets.
"""legs = {
    'front_left':  {'hip': 2,  'knee': 1,  'calf': 0,  'phase': T/2},
    'hind_left': {'hip': 5,  'knee': 4,  'calf': 3,  'phase': 0},
    'front_right':   {'hip': 10, 'knee': 11, 'calf': 12, 'phase': 0},
    'hind_right':  {'hip': 13, 'knee': 14, """
legs = {
    'front_left':  {'hip': 4,  'knee': 3,  'calf': 2,  'phase': T/2},
    'hind_left': {'hip': 7,  'knee': 6,  'calf': 5,  'phase': 0},
    'front_right':   {'hip': 8, 'knee': 9, 'calf': 10, 'phase': 0},
    'hind_right':  {'hip': 11, 'knee': 12, 'calf': 13, 'phase': T/2},
}

# ----- Adjusted for Rotation -----
delta_amp = 10.0  # amplitude (in degrees) for knee adjustment
T_delta = T       # period for front/back motion

# ----- IK Function -----
def compute_IK(L, l1, l2):
    if L > L_max:
        L = L_max
    if L < L_min:
        L = L_min

    try:
        theta = 180 - math.degrees(math.acos(((l1**2) + (l2**2) - (L**2)) / (2 * l1 * l2)))
    except ValueError:
        theta = 0.0

    knee_angle = math.degrees(math.atan((l2 * math.sin(math.radians(theta))) /
                              (l1 + l2 * math.cos(math.radians(theta)))))
    calf_angle = theta
    return knee_angle, calf_angle

# ----- Desired Vertical Altitude -----
def desired_altitude(t, phase, T, L_max, L_min):
    val = math.sin(2 * math.pi * ((t + phase) % T) / T)
    L = (L_max + L_min) / 2 - (L_max - L_min) / 2 * val
    return L

# ----- Delta Calculation with Direction -----
def compute_delta(t, phase, T_delta, delta_amp, direction):
    """Added 'direction' parameter (1 for left legs, -1 for right legs)."""
    delta = direction * delta_amp * math.sin(2 * math.pi * ((t + phase) % T_delta) / T_delta)
    return delta

# ----- Main Gait Loop -----
def gait_loop():
    print("Starting rotation gait loop. Press Ctrl+C to stop.")
    start_time = time.time()  # seconds
    try:
        while True:
            t_now_ms = (time.time() - start_time) * 1000  # current time in ms
            for leg_name, params in legs.items():
                phase = params['phase']
                # Compute sine value for phase synchronization
                val = math.sin(2 * math.pi * ((t_now_ms + phase) % T) / T)
                
                # Set hip angle based on leg side
                if 'left' in leg_name:
                    # Left legs: stance (val < 0) -> 140° (backward), swing (val > 0) -> 80° (forward)
                    hip_angle = HIP_FIXED - 30 * val
                else:
                    # Right legs: stance (val < 0) -> 80° (forward), swing (val > 0) -> 140° (backward)
                    hip_angle = HIP_FIXED + 30 * val
                
                # Compute desired vertical altitude L
                L_desired = desired_altitude(t_now_ms, phase, T, L_max, L_min)
                
                # Compute IK for knee and calf angles
                knee_angle, calf_angle = compute_IK(L_desired, l1, l2)
                
                # No delta since delta_amp = 0
                modified_knee_angle = knee_angle
                
                # Debug print
                print(f"{leg_name:12s} | L: {L_desired:5.2f} | Hip: {hip_angle:5.2f}°, "
                      f"Knee: {modified_knee_angle:5.2f}°, Calf: {calf_angle:5.2f}°")
                
                # Send commands to the servos
                s(params['hip'], hip_angle)
                s(params['knee'], modified_knee_angle)
                s(params['calf'], calf_angle)
            time.sleep(update_interval_ms / 1000.0)
    except KeyboardInterrupt:
        print("Gait loop stopped.")
"""def gait_loop():
    print("Starting counter-clockwise rotation gait. Press Ctrl+C to stop.")
    start_time = time.time()
    try:
        while True:
            t_now_ms = (time.time() - start_time) * 1000
            for leg_name, params in legs.items():
                phase = params['phase']
                # Determine direction: left legs push forward, right legs push backward
                direction = 1 if 'left' in leg_name else -1
                L_desired = desired_altitude(t_now_ms, phase, T, L_max, L_min)
                knee_angle, calf_angle = compute_IK(L_desired, l1, l2)
                # Compute delta with direction for rotation
                delta = compute_delta(t_now_ms, phase, T_delta, delta_amp, direction)
                modified_knee_angle = knee_angle + delta

                hip_angle = HIP_FIXED

                print(f"{leg_name:12s} | L: {L_desired:5.2f} | Hip: {hip_angle:5.2f}°, "
                      f"Knee: {modified_knee_angle:5.2f}° (base: {knee_angle:5.2f}°+delta:{delta:5.2f}°), "
                      f"Calf: {calf_angle:5.2f}°")

                s(params['hip'], hip_angle)
                s(params['knee'], modified_knee_angle)
                s(params['calf'], calf_angle)
            time.sleep(update_interval_ms / 1000.0)
    except KeyboardInterrupt:
        print("Gait loop stopped.")
"""
if __name__ == '__main__':
    gait_loop()
