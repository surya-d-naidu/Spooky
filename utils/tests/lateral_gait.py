import math
import time
from servo import set_servo_angle as s  # s(channel, angle)

# ----- Configuration -----
# Leg geometry (units, e.g., centimeters)
l1 = 5.0    # upper leg length
l2 = 5.0    # lower leg length

L_max = l1 + l2          # full vertical extension (foot on ground)
L_min = L_max * 0.6      # minimum vertical distance when foot is lifted

# Gait timing (milliseconds)
T = 1000                 # full cycle period for vertical motion (1 second)
update_interval_ms = 20  # update every 20 ms for smooth motion

# Fixed hip angle (the hip servos remain at this value)
HIP_FIXED = 100  # degrees

# Define servo channels, phase offsets, and side for each leg.
legs = {
    'front_left':  {'hip': 2,  'knee': 1,  'calf': 0,  'phase': T/2, 'side': 'left'},
    'front_right': {'hip': 5,  'knee': 4,  'calf': 3,  'phase': 0,   'side': 'right'},
    'hind_left':   {'hip': 10, 'knee': 11, 'calf': 12, 'phase': 0,   'side': 'left'},
    'hind_right':  {'hip': 13, 'knee': 14, 'calf': 15, 'phase': T/2, 'side': 'right'},
}

# ----- Lateral (X-axis) Stepping Parameters -----
x_amp = 10.0   # amplitude (in degrees) for lateral stepping offset
T_x = T        # period for lateral stepping (using the same cycle period here)

# ----- IK Function (Vertical) -----
def compute_IK(L, l1, l2):
    """
    Given desired vertical distance L (hip-to-foot) and leg link lengths,
    compute the knee and calf angles using the formula:
    
       theta = 180 - degrees( acos( ((l1^2)+(l2^2)-L^2) / (2*l1*l2) ) )
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
        # Debug: print base theta value.
        print(f"(Debug) theta: {theta:.2f}°")
    except ValueError:
        print("Error: Invalid L value. Check L, l1, and l2.")
        theta = 0.0
    # Compute knee angle (convert theta to radians when using sin and cos).
    knee_angle = math.degrees(math.atan((l2 * math.sin(math.radians(theta))) /
                                          (l1 + l2 * math.cos(math.radians(theta)))))
    calf_angle = theta  # Per the provided formula.
    return knee_angle, calf_angle

# ----- Vertical Altitude Trajectory -----
def desired_altitude(t, phase, T, L_max, L_min):
    """
    Compute the desired vertical distance L(t) for a leg.
    Uses a sine function (with phase offset):
       When sin() = +1 → L = L_min (foot lifted)
       When sin() = -1 → L = L_max (foot on ground)
    """
    val = math.sin(2 * math.pi * ((t + phase) % T) / T)
    L = (L_max + L_min) / 2 - (L_max - L_min) / 2 * val
    return L

# ----- Lateral Delta for X-axis (Sideways) Motion -----
def compute_delta_lateral(t, T_x, x_amp):
    """
    Compute a lateral delta (in degrees) using a sine wave.
    This value will be added (for left legs) or subtracted (for right legs)
    to modify the knee angle, causing the foot to move sideways.
    """
    delta = x_amp * math.sin(2 * math.pi * t / T_x)
    return delta

# ----- Main Lateral Gait Loop -----
def lateral_gait_loop():
    print("Starting lateral gait loop (walking right/left). Press Ctrl+C to stop.")
    start_time = time.time()  # seconds
    try:
        while True:
            t_now_ms = (time.time() - start_time) * 1000  # current time in ms
            # Compute lateral delta from global time.
            delta_lateral = compute_delta_lateral(t_now_ms, T_x, x_amp)
            for leg_name, params in legs.items():
                phase = params['phase']
                side = params['side']
                # Compute desired vertical altitude for this leg.
                L_desired = desired_altitude(t_now_ms, phase, T, L_max, L_min)
                # Compute IK for the vertical motion.
                base_knee, calf_angle = compute_IK(L_desired, l1, l2)
                # For lateral stepping, adjust the knee angle based on side:
                if side == 'left':
                    modified_knee = base_knee + delta_lateral
                else:
                    modified_knee = base_knee - delta_lateral

                # Hip remains fixed.
                hip_angle = HIP_FIXED

                print(f"{leg_name:12s} | L: {L_desired:5.2f} | Hip: {hip_angle:5.2f}°, "
                      f"Knee: {modified_knee:5.2f}° (base: {base_knee:5.2f}° ± delta: {delta_lateral:5.2f}°), "
                      f"Calf: {calf_angle:5.2f}°")
                
                # Send commands to servos.
                s(params['hip'], hip_angle)
                s(params['knee'], modified_knee)
                s(params['calf'], calf_angle)
            time.sleep(update_interval_ms / 1000.0)
    except KeyboardInterrupt:
        print("Lateral gait loop stopped.")

if __name__ == '__main__':
    lateral_gait_loop()

