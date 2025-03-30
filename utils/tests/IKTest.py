import math
import time
from servo import set_servo_angle as s  # use s(channel, angle)

# ----- Configuration -----
l1 = 5.0    # upper leg length (units)
l2 = 5.0    # lower leg length (units)
L_max = l1 + l2  # full vertical extension
L_min = L_max * 0.6  # minimum altitude for the dance motion

T = 2000  # full cycle period (milliseconds)
update_interval_ms = 50  # update every 50 ms for smooth motion

# ----- Helper Functions -----
def compute_angles_for_vertical_altitude(L, l1, l2):
    """
    Given a desired vertical altitude L (hip-to-foot distance) and link lengths,
    compute the knee and calf angles using the provided formula.
    
    Formula:
       theta = 180 - acos( ((L**2) - (l1**2) - (l2**2)) / (2*l1*l2) )
       knee_angle = atan( (l2*sin(theta))/(l1 + (l2*cos(theta)) ) )
       calf_angle = theta
       
    Note:
       - math.acos returns a value in radians. We convert it to degrees.
       - When using sin and cos on theta, we convert theta from degrees to radians.
    """
    # Clamp L within physical limits.
    if L > L_max:
        L = L_max
    if L < L_min:
        L = L_min

    a = 0  # temporary variable for debugging
    try:
        # Use extra parentheses to ensure correct evaluation order.
        theta = 180 - math.degrees(math.acos(((l1**2) + (l2**2) - (L**2)) / (2 * l1 * l2)))
        print(theta - 180)
        print(f"theta: {theta:.2f}°")
    except ValueError:
        print("Error: Invalid L value. Check L, l1, and l2.")
        print(a)
        theta = 0.0

    # Compute knee angle using your formula.
    # Convert theta to radians when passing to sin and cos.
    knee_angle = math.degrees(math.atan((l2 * math.sin(math.radians(theta))) /
                                          (l1 + l2 * math.cos(math.radians(theta)))))
    calf_angle = theta  # To keep the foot vertical.
    return knee_angle, calf_angle

def desired_altitude(t, L_min, L_max, T):
    """
    Compute the desired vertical altitude L(t) as a cosine function.
    At t=0, L = L_max; at t=T/2, L = L_min.
    """
    omega = 2 * math.pi / T  # angular frequency in radians per millisecond
    L = (L_max + L_min) / 2 + ((L_max - L_min) / 2) * math.cos(omega * t)
    return L

# ----- Main Test Loop -----
def dance_test_loop():
    print("Starting IK Dance Test. Press Ctrl+C to stop.")
    start_time = time.time()  # seconds
    try:
        while True:
            # Current time in milliseconds (modulo T for periodicity)
            t_now_ms = (time.time() - start_time) * 1000
            t_mod = t_now_ms % T

            # Compute the desired altitude L at this time.
            L_desired = desired_altitude(t_mod, L_min, L_max, T)

            # Compute the corresponding knee and calf angles.
            knee_angle, calf_angle = compute_angles_for_vertical_altitude(L_desired, l1, l2)

            # Print out the values for debugging.
            print(f"t: {t_mod:6.0f} ms | Altitude: {L_desired:5.2f} | Knee: {knee_angle:5.2f}°, Calf: {calf_angle:5.2f}°")

            # Send angles to the servos.
            # Here, we send knee_angle to channel 1 and calf_angle to channel 0.
            s(1, knee_angle)
            s(0, calf_angle)

            # Wait for the update interval.
            time.sleep(update_interval_ms / 1000.0)
    except KeyboardInterrupt:
        print("Dance test stopped.")

if __name__ == '__main__':
    dance_test_loop()

