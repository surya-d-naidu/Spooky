import math
import time
from servo import set_servo_angle as s  # s(channel, angle)

# Import gait and servo parameters from robot_config.py
try:
    # Try relative import first (when run as module)
    from ..robot_config import config
except ImportError:
    # Fallback to absolute import (when run directly)
    import sys
    import os
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from robot_config import config

# Link lengths (units: e.g., centimeters)
l1 = config['gait']['l1']
l2 = config['gait']['l2']
L_max = config['gait']['L_max']
L_min = config['gait']['L_min']
T = config['gait']['T']
update_interval_ms = config['gait']['update_interval_ms']
HIP_FIXED = config['gait']['HIP_FIXED']
servo_biases = config['gait']['servo_biases']
legs = config['gait']['legs']
step_length = config['gait']['step_length']

# ----- Function to Apply Servo Bias -----
def apply_bias(channel, angle):
    """Apply the calibration bias to the servo angle before setting it."""
    return angle + servo_biases.get(channel, 0.0)

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
    if L > (l1 + l2):
        L = l1 + l2
    if L < max(l1, l2) - min(l1, l2) + 0.1:  # Add a small buffer to avoid singularity
        L = max(l1, l2) - min(l1, l2) + 0.1

    try:
        # Compute theta in degrees.
        theta = 180 - math.degrees(math.acos(((l1**2) + (l2**2) - (L**2)) / (2 * l1 * l2)))
    except ValueError as e:
        print(f"Error: Invalid L value ({L}). Check L, l1, and l2. Error: {e}")
        # Default to a safe value
        theta = 90.0

    # Compute knee angle (convert theta to radians for sin and cos)
    knee_angle = math.degrees(math.atan((l2 * math.sin(math.radians(theta))) /
                                        (l1 + l2 * math.cos(math.radians(theta)))))
    calf_angle = theta  # As per the formula.
    return knee_angle, calf_angle


# ----- One Limb at a Time Gait Loop (Crawl Gait) -----
shift_amount = config['gait']['shift_amount']
shift_duration = config['gait']['shift_duration']
prev_active_leg = None  # track when a step just completed

def shift_body_forward():
    """Lean the body forward on all stance legs, then return to neutral."""
    # lean forward
    for params in legs.values():
        s(params['hip'], apply_bias(params['hip'], HIP_FIXED + shift_amount))
    time.sleep(shift_duration)
    # reset hips back to center
    for params in legs.values():
        s(params['hip'], apply_bias(params['hip'], HIP_FIXED))

def crawl_gait_loop(duration=None):
    global prev_active_leg
    print("Starting crawl gait loop with body shifts. Ctrl+C to stop.")
    start_time = time.time()
    end_time = start_time + duration if duration else float('inf')

    # initial stance: set all legs on ground
    for name, p in legs.items():
        knee0, calf0 = compute_IK(L_max, l1, l2)
        s(p['hip'], apply_bias(p['hip'], HIP_FIXED))
        s(p['knee'], apply_bias(p['knee'], knee0))
        s(p['calf'], apply_bias(p['calf'], calf0))
    time.sleep(0.5)

    try:
        while time.time() < end_time:
            t_ms = (time.time() - start_time) * 1000
            active_leg = None

            # 1) compute each leg’s state & command its servos
            for leg_name, params in legs.items():
                ph = params['phase']
                norm = ((t_ms + ph) % T) / T

                # Decide L_desired and whether this is the swinging leg
                if 0.15 <= norm < 0.25:
                    # swing up & forward
                    prog = (norm - 0.15) / 0.1
                    L_des = L_max - prog * (L_max - L_min)
                    active_leg = leg_name
                elif 0.25 <= norm < 0.35:
                    # swing down
                    prog = (norm - 0.25) / 0.1
                    L_des = L_min + prog * (L_max - L_min)
                    active_leg = leg_name
                else:
                    # on ground
                    L_des = L_max

                # IK for vertical
                knee_ang, calf_ang = compute_IK(L_des, l1, l2)

                # horizontal foot trajectory
                if norm < 0.15:
                    off = -step_length * (norm / 0.15)
                elif norm < 0.35:
                    off = -step_length * 0.2 + step_length * ((norm - 0.15) / 0.2)
                else:
                    off = step_length * 0.8 * (1 - (norm - 0.35) / 0.65)

                mod_knee = knee_ang - off

                # command the servos
                s(params['hip'], apply_bias(params['hip'], HIP_FIXED))
                # special bias for front-left
                if params['knee'] == 3:
                    a = max(0, mod_knee - 20)
                    s(params['knee'], apply_bias(params['knee'], a))
                else:
                    s(params['knee'], apply_bias(params['knee'], mod_knee))
                s(params['calf'], apply_bias(params['calf'], calf_ang))

            # 2) once the active leg *just changed*, shift the body
            if active_leg and active_leg != prev_active_leg:
                shift_body_forward()
                prev_active_leg = active_leg

            time.sleep(update_interval_ms / 1000.0)

    except KeyboardInterrupt:
        print("Crawl gait stopped.")

# ----- Bi-Diagonal Rotation Without Hip Servo -----

turn_offset_knee = config['gait']['turn_offset_knee']
turn_offset_calf = config['gait']['turn_offset_calf']

def rotate_in_place(direction, duration=3):
    """
    Rotate the robot in place using the crawl gait pattern but with turning offsets.
    """
    print(f"Starting rotation to the {direction} for {duration} seconds.")
    rot_start_time = time.time()
    while time.time() - rot_start_time < duration:
        t_now_ms = (time.time() - rot_start_time) * 1000  # elapsed time in ms for leg phases
        for leg_name, params in legs.items():
            phase = params['phase']
            normalized_phase = ((t_now_ms + phase) % T) / T  # 0 to 1
            
            # Calculate L (vertical distance) based on phase
            if 0.15 <= normalized_phase < 0.25:  # Lifting phase
                lift_progress = (normalized_phase - 0.15) / 0.1  # 0 to 1 within lifting phase
                L_desired = L_max - lift_progress * (L_max - L_min)
                leg_state = "LIFTING"
            elif 0.25 <= normalized_phase < 0.35:  # Lowering phase
                lower_progress = (normalized_phase - 0.25) / 0.1  # 0 to 1 within lowering phase
                L_desired = L_min + lower_progress * (L_max - L_min)
                leg_state = "LOWERING"
            else:  # On ground
                L_desired = L_max
                leg_state = "GROUND"
            
            # Compute base angles
            knee_angle, calf_angle = compute_IK(L_desired, l1, l2)
            
            # Apply turning offsets based on which legs are on the ground
            if leg_state == "GROUND":
                if direction.lower() == 'right':
                    if 'left' in leg_name:
                        # Left legs push backward for right turn
                        knee_angle -= turn_offset_knee
                    else:  # right leg
                        # Right legs push forward for right turn
                        knee_angle += turn_offset_knee
                elif direction.lower() == 'left':
                    if 'left' in leg_name:
                        # Left legs push forward for left turn
                        knee_angle += turn_offset_knee
                    else:  # right leg
                        # Right legs push backward for left turn
                        knee_angle -= turn_offset_knee
            
            # Send commands to the servos with biases applied
            hip_channel = params['hip']
            knee_channel = params['knee']
            calf_channel = params['calf']
            
            s(hip_channel, apply_bias(hip_channel, HIP_FIXED))
            
            # Handle the special case for front-left knee
            if knee_channel == 3:
                delta_angle = knee_angle - 20
                if delta_angle <= 0:
                    s(knee_channel, apply_bias(knee_channel, 0))
                else:
                    s(knee_channel, apply_bias(knee_channel, delta_angle))
            else:
                s(knee_channel, apply_bias(knee_channel, knee_angle))
                
            s(calf_channel, apply_bias(calf_channel, calf_angle))
            
            # Debug output
            print(f"{leg_name:12s} | {leg_state:7s} | L: {L_desired:5.2f} | "
                  f"Knee: {knee_angle:5.1f}° | Calf: {calf_angle:5.1f}°")
            
        time.sleep(update_interval_ms / 1000.0)
    print("Rotation complete. Resuming normal gait.")

def key_control_rotation():
    """
    Wait for a key press to trigger a rotation using the crawl gait pattern.
    """
    key = input("Press 'r' for right turn, 'l' for left turn: ").strip().lower()
    if key == 'r' or key == 'l':
        rotate_in_place('right' if key == 'r' else 'left')
    else:
        print("No rotation activated.")

if __name__ == '__main__':
    print("Select mode:")
    print("  1) Crawl gait with forward movement (one limb at a time)")
    print("  2) Rotation in place (using crawl pattern)")
    choice = input("Enter 1 or 2: ").strip()
    if choice == '1':
        crawl_gait_loop()
    elif choice == '2':
        key_control_rotation()
    else:
        print("Invalid choice. Running crawl gait by default.")
        crawl_gait_loop()
