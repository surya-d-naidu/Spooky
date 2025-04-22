import time
from pid import PID
from mpu6050 import MPU6050
import Adafruit_PCA9685

# Initialize the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685(0x60, busnum=1)

# Constants
NEUTRAL_KNEE = 120  # Adjust these as per your servo calibration
NEUTRAL_CALF = 60
SERVO_CHANNELS = {
    'FL_knee': 1, 'FL_calf': 2,
    'FR_knee': 4, 'FR_calf': 5,
    'BL_knee': 7, 'BL_calf': 8,
    'BR_knee': 10, 'BR_calf': 11,
}
PITCH_SENS = 0.8  # Degrees of change per degree of pitch
ROLL_SENS = 0.8   # Degrees of change per degree of roll

# Initialize hardware
mpu = MPU6050()
pwm.set_pwm_freq(50)

# Initialize PID (P, I, D)
pid_pitch = PID(1.2, 0.01, 0.3)
pid_roll = PID(1.2, 0.01, 0.3)

pid_pitch.setpoint = 0  # target pitch = 0
pid_roll.setpoint = 0   # target roll = 0

def set_servo_angle(channel, angle):
    pulse = int((angle / 180.0) * 2000 + 500)
    pwm.set_pwm(channel, 0, pulse)

def stand_neutral():
    for name, ch in SERVO_CHANNELS.items():
        if 'knee' in name:
            set_servo_angle(ch, NEUTRAL_KNEE)
        elif 'calf' in name:
            set_servo_angle(ch, NEUTRAL_CALF)

def stabilize_once():
    pitch, roll = mpu.get_pitch_roll()

    pitch_correction = pid_pitch(pitch)
    roll_correction = pid_roll(roll)

    for leg in ['FL', 'FR', 'BL', 'BR']:
        knee_ch = SERVO_CHANNELS[f'{leg}_knee']
        calf_ch = SERVO_CHANNELS[f'{leg}_calf']

        # Basic rule: 
        # - pitch backward => lift front legs
        # - pitch forward => lift back legs
        # - roll right => lift left legs, etc.

        pitch_dir = -1 if leg.startswith('F') else 1
        roll_dir = -1 if leg in ['FL', 'BL'] else 1

        knee_angle = NEUTRAL_KNEE + pitch_dir * pitch_correction * PITCH_SENS + roll_dir * roll_correction * ROLL_SENS
        calf_angle = NEUTRAL_CALF + pitch_dir * pitch_correction * PITCH_SENS + roll_dir * roll_correction * ROLL_SENS

        # Clamp angles
        knee_angle = max(60, min(160, knee_angle))
        calf_angle = max(30, min(120, calf_angle))

        set_servo_angle(knee_ch, knee_angle)
        set_servo_angle(calf_ch, calf_angle)

def main():
    stand_neutral()
    print("Standing neutral... Starting stabilization.")
    time.sleep(2)

    while True:
        stabilize_once()
        time.sleep(0.05)  # 20Hz loop

if __name__ == "__main__":
    main()
