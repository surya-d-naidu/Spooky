import time
import Adafruit_PCA9685

# Initialize the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685(0x40, busnum=1)

# Set frequency to 60Hz, good for servos.
pwm.set_pwm_freq(60)

def set_servo_angle(channel, angle):
    """Set the servo to a specific angle."""
    # Convert angle to pulse length
    pulse_length = 1000000  # 1,000,000 us per second
    pulse_length //= 60     # 60 Hz
    pulse_length //= 4096   # 12 bits of resolution
    pulse = int(int(angle) * (2000 / 180) + 500)  # Convert angle to pulse width (500-2500 us)
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)

def move_servo_relative(channel, target_angle):
    """Move the servo relative to its current position."""
    # Assume the current position is 0° (initial position)
    current_angle = 0  # Treat the current position as 0°
    print(f"Moving servo on channel {channel} from {current_angle}° to {target_angle}°")
    
    # Move the servo to the target angle
    set_servo_angle(channel, target_angle)
    time.sleep(2)  # Wait for the servo to reach the target position

    # Update the current angle to the target angle
    current_angle = target_angle
    return current_angle

def setFromDict(dicto):
    set_servo_angle(0, dicto["slider_1"])
    set_servo_angle(1, dicto["slider_2"])
    set_servo_angle(2, dicto["slider_3"])
    set_servo_angle(3, dicto["slider_4"])
    set_servo_angle(4, dicto["slider_5"])
    set_servo_angle(5, dicto["slider_6"])
    set_servo_angle(10, dicto["slider_7"])
    set_servo_angle(11, dicto["slider_8"])
    set_servo_angle(12, dicto["slider_9"])
    set_servo_angle(13, dicto["slider_10"])
    set_servo_angle(14, dicto["slider_11"])
    set_servo_angle(15, dicto["slider_12"])

if __name__ == "__main__":
    # Define the channel (pin) where the servo is connected
    servo_channel = 13

    try:
        # Assume the current position is 0° (initial position)
        current_angle = 0

        while True:
            # Move to 90° relative to the current position
            current_angle = move_servo_relative(servo_channel, 90)

            # Move to 180° relative to the current position
            current_angle = move_servo_relative(servo_channel, 180)

            # Move back to 0° relative to the current position
            current_angle = move_servo_relative(servo_channel, 0)

    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
