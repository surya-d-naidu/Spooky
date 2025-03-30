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
T = 1000                 # full cycle period (1 second)
update_interval_ms = 40  # update every 20 ms for smooth motion

# Base hip angles:
HIP_FRONT = 110   # front hip base angle (degrees)
HIP_HIND  = 110  # hind hip base angle (degrees)

# Define servo channels, phase offsets, and leg position for each leg.
legs = {
    'front_left':  {'hip': 2,  'knee': 1,  'calf': 0,  'phase': T/2, 'position': 'front'},
    'hind_left': {'hip': 5,  'knee': 4,  'calf': 3,  'phase': 0,   'position': 'hind'},
    'front_right':   {'hip': 10, 'knee': 11, 'calf': 12, 'phase': 0,   'position':'front'},
    'hind_right':  {'hip': 13, 'knee': 14, 'calf': 15, 'phase': T/2, 'position': 'hind'},
}

# ----- Forward Stepping Parameters -----
delta_amp = 10.0  # amplitude (degrees) for forward/back swing offset
T_delta = T       # using the same period for stepping

# ----- IK Function Using Your Formula -----
def compute_IK(L, l1, l2):
    """
    Compute knee and calf angles given desired vertical distance L (hip-to-foot)
    using the formula:
    
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
        print(f"(Debug) theta: {theta:.2f}°")
    except ValueError:
        print("Error: Invalid L value. Check L, l1, and l2.")
        theta = 0.0
    # Compute knee angle; convert theta to radians for sin and cos.
    knee_angle = math.degrees(math.atan((l2 * math.sin(math.radians(theta))) /
                                          (l1 + l2 * math.cos(math.radians(theta)))))
    calf_angle = theta  # per the formula
    return knee_angle, calf_angle

# ----- Vertical Altitude Trajectory -----
def desired_altitude(t, phase, T, L_max, L_min):
    """
    Compute desired vertical distance L(t) for a leg.
    Uses a sine function with phase offset:
       When sin() = +1 → L = L_min (foot lifted)
       When sin() = -1 → L = L_max (foot on ground)
    """
    val = math.sin(2 * math.pi * ((t + phase) % T) / T)
    L = (L_max + L_min) / 2 - (L_max - L_min) / 2 * val
    return L

# ----- Delta for Forward/Backward Swing -----
def compute_delta(t, phase, T_delta, delta_amp):
    """
    Compute a periodic delta (in degrees) using a sine wave.
    This delta is subtracted from the knee angle to produce forward stepping.
    """
    delta = delta_amp * math.sin(2 * math.pi * ((t + phase) % T_delta) / T_delta)
    return delta

# ----- Get Hip Angle Based on Leg Position -----
def get_hip_angle(position):
    """
    Return the base hip angle based on leg position.
    Front legs: HIP_FRONT, hind legs: HIP_HIND.
    """
    return HIP_FRONT if position == 'front' else HIP_HIND

# ----- Main Forward Gait Loop -----
def gait_loop():
    print("Starting forward gait loop. Press Ctrl+C to stop.")
    start_time = time.time()  # seconds
    try:
        while True:
            t_now_ms = (time.time() - start_time) * 1000  # current time in ms
            for leg_name, params in legs.items():
                phase = params['phase']
                position = params['position']
                # Compute desired vertical distance L for this leg.
                L_desired = desired_altitude(t_now_ms, phase, T, L_max, L_min)
                # Compute base knee and calf angles via IK.
                knee_angle, calf_angle = compute_IK(L_desired, l1, l2)
                # Compute the forward/backward delta.
                delta = compute_delta(t_now_ms, phase, T_delta, delta_amp)
                # Subtract delta to produce a forward step.
                modified_knee_angle = knee_angle - delta
                # Determine hip angle based on leg position.
                hip_angle = get_hip_angle(position)

                print(f"{leg_name:12s} | L: {L_desired:5.2f} | Hip: {hip_angle:5.2f}°, "
                      f"Knee: {modified_knee_angle:5.2f}° (base: {knee_angle:5.2f}° - delta: {delta:5.2f}°), "
                      f"Calf: {calf_angle:5.2f}°")
                
                # Send commands to servos.
                s(params['hip'], hip_angle)
                s(params['knee'], modified_knee_angle)
                s(params['calf'], calf_angle)
            time.sleep(update_interval_ms / 1000.0)
    except KeyboardInterrupt:
        print("Gait loop stopped.")

if __name__ == '__main__':
    gait_loop()

