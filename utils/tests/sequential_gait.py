import math
import time
from servo import set_servo_angle as s  # s(channel, angle)

# ----- Configuration -----
# Link lengths (units: e.g., centimeters)
l1 = 5.0    # upper leg length
l2 = 5.0    # lower leg length

L_max = l1 + l2 * 0.9    # slightly bent knee position when foot is on ground
L_min = L_max * 0.7      # minimum vertical distance when foot is lifted

# Timing configuration for sequential movement
step_duration = 1000     # time in ms for one leg to complete its step
pause_between_steps = 200  # pause between leg movements in ms
update_interval_ms = 20  # update every 20 ms for smooth motion

HIP_FIXED = 110  # Fixed hip angle (degrees) - NEVER CHANGES

# ----- Servo Biases -----
servo_biases = {
    4: -10.0,   # Channel 4 has +4 degree bias as specified
    3: 0.0,
    2: 4.0,
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

# Define servo channels for each leg in sequential order
legs = {
    'front_left':  {'hip': 4,  'knee': 3,  'calf': 2,  'order': 0},
    'front_right': {'hip': 8,  'knee': 9,  'calf': 10, 'order': 1},
    'hind_right':  {'hip': 14, 'knee': 12, 'calf': 11, 'order': 2},
    'hind_left':   {'hip': 7,  'knee': 6,  'calf': 5,  'order': 3},
}

# Movement parameters
step_length = 20.0  # Horizontal movement in degrees
weight_shift_amount = 10.0  # Degrees to shift weight using knee/calf

# ----- Function to Apply Servo Bias -----
def apply_bias(channel, angle):
    """Apply the calibration bias to the servo angle before setting it."""
    return angle + servo_biases.get(channel, 0.0)

# ----- IK Function -----
def compute_IK(L, l1, l2):
    """
    Given desired vertical distance L (hip-to-foot) and leg link lengths,
    compute the knee and calf angles.
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
        print(f"Error: Invalid L value ({L}). Using default theta=90. Error: {e}")
        theta = 90.0

    # Compute knee angle
    knee_angle = math.degrees(math.atan((l2 * math.sin(math.radians(theta))) /
                                        (l1 + l2 * math.cos(math.radians(theta)))))
    calf_angle = theta
    return knee_angle, calf_angle

# ----- Weight Shifting Functions -----
def shift_weight_to_three_legs(moving_leg_name):
    """
    Shift weight to the three support legs by adjusting their knee/calf angles.
    The hip servos remain completely static.
    """
    print(f"Shifting weight away from {moving_leg_name}")
    
    for leg_name, params in legs.items():
        if leg_name != moving_leg_name:
            # These are the support legs - shift weight to them
            # Increase L slightly to lower the body and shift weight
            support_L = L_max + 0.5
            knee_angle, calf_angle = compute_IK(support_L, l1, l2)
            
            # Apply special handling for front-left knee
            if params['knee'] == 3:
                adjusted_knee = max(0, knee_angle - 20)
                s(params['knee'], apply_bias(params['knee'], adjusted_knee))
            else:
                s(params['knee'], apply_bias(params['knee'], knee_angle))
            
            s(params['calf'], apply_bias(params['calf'], calf_angle))

def restore_normal_stance():
    """
    Restore all legs to normal stance position.
    """
    print("Restoring normal stance")
    
    for leg_name, params in legs.items():
        knee_angle, calf_angle = compute_IK(L_max, l1, l2)
        
        # Apply special handling for front-left knee
        if params['knee'] == 3:
            adjusted_knee = max(0, knee_angle - 20)
            s(params['knee'], apply_bias(params['knee'], adjusted_knee))
        else:
            s(params['knee'], apply_bias(params['knee'], knee_angle))
        
        s(params['calf'], apply_bias(params['calf'], calf_angle))

# ----- Individual Leg Movement -----
def move_single_leg(leg_name, forward_offset=0):
    """
    Move a single leg through its step cycle while others remain stationary.
    1. Lift the leg
    2. Move it forward/backward
    3. Lower it back down
    """
    params = legs[leg_name]
    print(f"Moving {leg_name}")
    
    # Phase 1: Lift the leg
    print(f"  Lifting {leg_name}")
    for i in range(10):
        progress = i / 9.0  # 0 to 1
        L_current = L_max - progress * (L_max - L_min)
        knee_angle, calf_angle = compute_IK(L_current, l1, l2)
        
        # Apply forward/backward movement during lift
        horizontal_offset = forward_offset * progress
        modified_knee = knee_angle + horizontal_offset
        
        # Apply special handling for front-left knee
        if params['knee'] == 3:
            adjusted_knee = max(0, modified_knee - 20)
            s(params['knee'], apply_bias(params['knee'], adjusted_knee))
        else:
            s(params['knee'], apply_bias(params['knee'], modified_knee))
        
        s(params['calf'], apply_bias(params['calf'], calf_angle))
        time.sleep(step_duration / 1000.0 / 20)  # 1/20th of step duration per update
    
    # Phase 2: Move leg forward/backward while lifted
    print(f"  Moving {leg_name} horizontally")
    knee_angle, calf_angle = compute_IK(L_min, l1, l2)
    modified_knee = knee_angle + forward_offset
    
    # Apply special handling for front-left knee
    if params['knee'] == 3:
        adjusted_knee = max(0, modified_knee - 20)
        s(params['knee'], apply_bias(params['knee'], adjusted_knee))
    else:
        s(params['knee'], apply_bias(params['knee'], modified_knee))
    
    s(params['calf'], apply_bias(params['calf'], calf_angle))
    time.sleep(step_duration / 1000.0 / 10)  # Hold position briefly
    
    # Phase 3: Lower the leg
    print(f"  Lowering {leg_name}")
    for i in range(10):
        progress = i / 9.0  # 0 to 1
        L_current = L_min + progress * (L_max - L_min)
        knee_angle, calf_angle = compute_IK(L_current, l1, l2)
        
        # Maintain horizontal position while lowering
        modified_knee = knee_angle + forward_offset
        
        # Apply special handling for front-left knee
        if params['knee'] == 3:
            adjusted_knee = max(0, modified_knee - 20)
            s(params['knee'], apply_bias(params['knee'], adjusted_knee))
        else:
            s(params['knee'], apply_bias(params['knee'], modified_knee))
        
        s(params['calf'], apply_bias(params['calf'], calf_angle))
        time.sleep(step_duration / 1000.0 / 20)

# ----- Sequential Gait Loop -----
def sequential_gait_loop():
    """
    Main gait loop where only one leg moves at a time in sequence.
    Hip servos remain completely static throughout.
    """
    print("Starting sequential gait (one leg at a time). Ctrl+C to stop.")
    print("Hip servos will remain static throughout the movement.")
    
    # Initialize all legs to ground position with static hips
    print("Initializing all legs to ground position...")
    for leg_name, params in legs.items():
        # Set hip to fixed position and NEVER change it
        s(params['hip'], apply_bias(params['hip'], HIP_FIXED))
        
        # Set knee and calf to ground position
        knee_angle, calf_angle = compute_IK(L_max, l1, l2)
        
        # Apply special handling for front-left knee
        if params['knee'] == 3:
            adjusted_knee = max(0, knee_angle - 20)
            s(params['knee'], apply_bias(params['knee'], adjusted_knee))
        else:
            s(params['knee'], apply_bias(params['knee'], knee_angle))
        
        s(params['calf'], apply_bias(params['calf'], calf_angle))
    
    time.sleep(1.0)  # Allow robot to settle
    
    try:
        step_count = 0
        while True:
            # Get ordered list of legs
            leg_order = sorted(legs.items(), key=lambda x: x[1]['order'])
            
            for leg_name, params in leg_order:
                print(f"\n--- Step {step_count + 1}: Moving {leg_name} ---")
                
                # 1. Shift weight to the other three legs
                shift_weight_to_three_legs(leg_name)
                time.sleep(0.1)  # Brief pause for weight shift
                
                # 2. Move the selected leg
                forward_offset = step_length if step_count % 2 == 0 else -step_length
                move_single_leg(leg_name, forward_offset)
                
                # 3. Restore normal stance
                restore_normal_stance()
                
                # 4. Pause between leg movements
                print(f"Pausing before next leg...")
                time.sleep(pause_between_steps / 1000.0)
                
                step_count += 1
            
            print(f"\nCompleted full cycle (all 4 legs moved)")
            
    except KeyboardInterrupt:
        print("\nSequential gait stopped.")
        # Return all legs to neutral position
        restore_normal_stance()

# ----- Rotation Mode -----
def sequential_rotation(direction, cycles=2):
    """
    Rotate the robot by moving legs sequentially with directional bias.
    """
    print(f"Starting sequential rotation {direction} for {cycles} cycles")
    
    # Initialize to ground position
    for leg_name, params in legs.items():
        s(params['hip'], apply_bias(params['hip'], HIP_FIXED))
        knee_angle, calf_angle = compute_IK(L_max, l1, l2)
        
        if params['knee'] == 3:
            adjusted_knee = max(0, knee_angle - 20)
            s(params['knee'], apply_bias(params['knee'], adjusted_knee))
        else:
            s(params['knee'], apply_bias(params['knee'], knee_angle))
        
        s(params['calf'], apply_bias(params['calf'], calf_angle))
    
    time.sleep(1.0)
    
    try:
        for cycle in range(cycles):
            leg_order = sorted(legs.items(), key=lambda x: x[1]['order'])
            
            for leg_name, params in leg_order:
                print(f"\nCycle {cycle + 1}: Rotating {leg_name} {direction}")
                
                # Calculate rotation offset based on direction and leg position
                if direction.lower() == 'right':
                    if 'left' in leg_name:
                        rotation_offset = -step_length  # Left legs move backward
                    else:
                        rotation_offset = step_length   # Right legs move forward
                elif direction.lower() == 'left':
                    if 'left' in leg_name:
                        rotation_offset = step_length   # Left legs move forward
                    else:
                        rotation_offset = -step_length  # Right legs move backward
                else:
                    rotation_offset = 0
                
                # Shift weight and move leg
                shift_weight_to_three_legs(leg_name)
                time.sleep(0.1)
                move_single_leg(leg_name, rotation_offset)
                restore_normal_stance()
                time.sleep(pause_between_steps / 1000.0)
        
        print(f"Rotation complete!")
        
    except KeyboardInterrupt:
        print("\nRotation stopped.")
        restore_normal_stance()

# ----- Main Function -----
if __name__ == '__main__':
    print("Sequential Gait Controller")
    print("=========================")
    print("Features:")
    print("- Only ONE leg moves at a time")
    print("- Hip servos remain completely STATIC")
    print("- Weight shifting uses only knee/calf motors")
    print("- Clear sequential movement pattern")
    print()
    print("Select mode:")
    print("  1) Sequential forward gait")
    print("  2) Sequential rotation (left/right)")
    print("  3) Test single leg movement")
    
    choice = input("Enter 1, 2, or 3: ").strip()
    
    if choice == '1':
        sequential_gait_loop()
    elif choice == '2':
        direction = input("Enter rotation direction (left/right): ").strip()
        if direction.lower() in ['left', 'right']:
            sequential_rotation(direction)
        else:
            print("Invalid direction. Using 'right' as default.")
            sequential_rotation('right')
    elif choice == '3':
        leg_name = input("Enter leg name (front_left, front_right, hind_left, hind_right): ").strip()
        if leg_name in legs:
            print(f"Testing movement of {leg_name}")
            # Initialize all hips to static position
            for name, params in legs.items():
                s(params['hip'], apply_bias(params['hip'], HIP_FIXED))
            restore_normal_stance()
            time.sleep(1)
            move_single_leg(leg_name, step_length)
        else:
            print("Invalid leg name.")
    else:
        print("Invalid choice. Running sequential gait by default.")
        sequential_gait_loop()
