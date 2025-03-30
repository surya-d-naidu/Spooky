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
# For trot, phase offsets (in ms) are set to alternate stepping:
legs = {
    'front_left':  {'hip': 2,  'knee': 1,  'calf': 0,  'phase': T/2},
    'hind_left': {'hip': 5,  'knee': 4,  'calf': 3,  'phase': 0},
    'front_right':   {'hip': 10, 'knee': 11, 'calf': 12, 'phase': 0},
    'hind_right':  {'hip': 13, 'knee': 14, 'calf': 15, 'phase': T/2},
}

# ----- New Variable for Forward/Backward Stepping -----
delta_amp = 10.0  # amplitude (in degrees) to add to the knee angle for front/back motion
T_delta = T       # using the same period; adjust if you want a different rhythm

# ----- IK Function Using Your Formula -----
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
        # Compute theta in degrees.
        theta = 180 - math.degrees(math.acos(((l1**2) + (l2**2) - (L**2)) / (2 * l1 * l2)))
        print(f"(Debug) theta: {theta:.2f}°")
    except ValueError:
        print("Error: Invalid L value. Check L, l1, and l2.")
        theta = 0.0

    # Compute knee angle (convert theta to radians for sin and cos)
    knee_angle = math.degrees(math.atan((l2 * math.sin(math.radians(theta))) /
                                          (l1 + l2 * math.cos(math.radians(theta)))))
    calf_angle = theta  # As per the formula.
    return knee_angle, calf_angle

# ----- Desired Vertical Altitude Function -----
def desired_altitude(t, phase, T, L_max, L_min):
    """
    Compute desired vertical distance L(t) for a leg.
    Using a sine function with phase offset:
       When sin() = +1, foot is lifted (L = L_min);
       When sin() = -1, foot is on the ground (L = L_max).
    """
    val = math.sin(2 * math.pi * ((t + phase) % T) / T)
    L = (L_max + L_min) / 2 - (L_max - L_min) / 2 * val
    return L

# ----- Delta Calculation for Front/Back Motion -----
def compute_delta(t, phase, T_delta, delta_amp):
    """
    Compute a periodic delta (in degrees) as a sine wave.
    This value will be added to the computed knee angle to produce forward/backward stepping.
    """
    delta = delta_amp * math.sin(2 * math.pi * ((t + phase) % T_delta) / T_delta)
    return delta

# ----- Main Gait Loop -----
def gait_loop():
    print("Starting trot gait loop with stepping. Press Ctrl+C to stop.")
    start_time = time.time()  # seconds
    try:
        while True:
            t_now_ms = (time.time() - start_time) * 1000  # current time in ms
            for leg_name, params in legs.items():
                phase = params['phase']
                # Compute desired vertical altitude L for this leg.
                L_desired = desired_altitude(t_now_ms, phase, T, L_max, L_min)
                # Compute IK for knee and calf angles.
                knee_angle, calf_angle = compute_IK(L_desired, l1, l2)
                # Compute delta for front/back stepping.
                delta = compute_delta(t_now_ms, phase, T_delta, delta_amp)
                # Add delta to the knee angle.
                modified_knee_angle = knee_angle + delta

                # Hip remains fixed.
                hip_angle = HIP_FIXED

                # Debug print.
                print(f"{leg_name:12s} | L: {L_desired:5.2f} | Hip: {hip_angle:5.2f}°, "
                      f"Knee: {modified_knee_angle:5.2f}° (base: {knee_angle:5.2f}°+delta:{delta:5.2f}°), "
                      f"Calf: {calf_angle:5.2f}°")

                # Send commands to the servos.
                s(params['hip'], hip_angle)
                s(params['knee'], modified_knee_angle)
                s(params['calf'], calf_angle)
            time.sleep(update_interval_ms / 1000.0)
    except KeyboardInterrupt:
        print("Gait loop stopped.")

if __name__ == '__main__':
    gait_loop()

