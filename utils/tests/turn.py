import keyboard   # pip install keyboard

# ----- Rotation-without-hip Configuration -----
turn_offset_knee = 5.0    # degrees to skew knee for turning
turn_offset_calf = 5.0    # degrees to skew calf for turning
ROT_DURATION = 3.0        # seconds per key‑press rotation

# State variables
rotation_direction = None
rotation_end_time = 0.0

def gait_loop_with_key_rotation():
    """
    Continuous trot gait. While walking, pressing 'r' or 'l' triggers
    a 3‑second bi‑diagonal in‑place rotation (no hip servo) using knee+calf.
    """
    global rotation_direction, rotation_end_time
    print("Walking... Press R or L to rotate, Ctrl+C to stop.")
    start_time = time.time()
    try:
        while True:
            now = time.time()
            t_now_ms = (now - start_time) * 1000

            # --- check for keypress and schedule rotation ---
            if keyboard.is_pressed('r'):
                rotation_direction = 'right'
                rotation_end_time = now + ROT_DURATION
                print("→ RIGHT rotation!")
            elif keyboard.is_pressed('l'):
                rotation_direction = 'left'
                rotation_end_time = now + ROT_DURATION
                print("← LEFT rotation!")

            # clear rotation when time’s up
            if rotation_direction and now >= rotation_end_time:
                print("Rotation done. Resuming straight walk.")
                rotation_direction = None

            rotating = rotation_direction is not None

            # --- per‑leg commands ---
            for leg_name, params in legs.items():
                phase = params['phase']
                L_desired = desired_altitude(t_now_ms, phase, T, L_max, L_min)
                knee_angle, calf_angle = compute_IK(L_desired, l1, l2)
                delta = compute_delta(t_now_ms, phase, T_delta, delta_amp)
                mod_knee = knee_angle + delta

                # if rotating and this leg is in stance, apply turn offsets
                if rotating:
                    val = math.sin(2 * math.pi * ((t_now_ms + phase) % T) / T)
                    if val < 0:  # stance phase
                        if rotation_direction == 'right':
                            if 'left' in leg_name:
                                mod_knee += turn_offset_knee
                                calf_angle += turn_offset_calf
                            else:
                                mod_knee -= turn_offset_knee
                                calf_angle -= turn_offset_calf
                        else:  # left turn
                            if 'left' in leg_name:
                                mod_knee -= turn_offset_knee
                                calf_angle -= turn_offset_calf
                            else:
                                mod_knee += turn_offset_knee
                                calf_angle += turn_offset_calf

                hip_angle = HIP_FIXED  # hip still fixed

                # send to servos
                s(params['hip'], hip_angle)
                s(params['knee'], mod_knee)
                s(params['calf'], calf_angle)

            time.sleep(update_interval_ms / 1000.0)

    except KeyboardInterrupt:
        print("Stopped.")

# In your main, just call this instead of gait_loop():
if __name__ == '__main__':
    gait_loop_with_key_rotation()

