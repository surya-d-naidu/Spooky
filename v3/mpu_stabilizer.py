# mpu_stabilizer.py
"""
Stabilization logic for Spooky robot using MPU6050 IMU.
Reads pitch/roll and computes hip servo corrections for each leg.
"""

# You must implement get_pitch_roll() in your mpu6050.py to return (pitch, roll) in degrees.
from mpu6050 import get_pitch_roll

# PID parameters for stabilization
default_P_GAIN = 0.8  # Proportional gain for correction
MAX_CORRECTION = 8.0  # Maximum degrees to correct

# List of leg names must match your gait code
LEG_NAMES = ['front_left', 'front_right', 'hind_right', 'hind_left']


def get_stabilization_offsets(P_GAIN=default_P_GAIN):
    """
    Reads the IMU and returns hip angle corrections for each leg.
    Returns a dict: {leg_name: correction_angle}
    """
    pitch, roll = get_pitch_roll()  # in degrees

    corrections = {}
    for leg in LEG_NAMES:
        if 'front' in leg:
            pitch_corr = -pitch  # If nose up, push front legs down
        else:
            pitch_corr = pitch   # If nose up, pull hind legs up

        if 'left' in leg:
            roll_corr = -roll   # If left up, push left legs down
        else:
            roll_corr = roll    # If right up, push right legs down

        # Combine and clamp
        total_corr = P_GAIN * (pitch_corr + roll_corr)
        total_corr = max(-MAX_CORRECTION, min(MAX_CORRECTION, total_corr))
        corrections[leg] = total_corr
    return corrections

if __name__ == '__main__':
    # Simple test loop
    import time
    while True:
        offsets = get_stabilization_offsets()
        print(offsets)
        time.sleep(0.1)
