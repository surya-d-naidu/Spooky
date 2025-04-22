import math
import time
from servo import set_servo_angle as s  # s(channel, angle)
from mpu6050 import mpu6050        # Assuming you have an mpu6050 library

# ----- Configuration -----
# Link lengths (units: e.g., centimeters)
l1 = 5.0    # upper leg length
l2 = 5.0    # lower leg length

# The vertical distance from hip to foot will be between L_min and L_max.
L_max = l1 + l2          # full extension
L_min = L_max * 0.6      # minimum contracted length

# Base altitude for standing (midpoint between L_max and L_min)
BASE_ALTITUDE = (L_max + L_min) / 2

# Base hip angle (locked)
HIP_FIXED = 110

# Define servo channels for each leg.
"""legs = {
    'front_left':  {'hip': 2,  'knee': 1,  'calf': 0,  'pos': 'front_left'},
    'hind_left':   {'hip': 5,  'knee': 4,  'calf': 3,  'pos': 'hind_left'},
    'front_right': {'hip': 10, 'knee': 11, 'calf': 12, 'pos': 'front_right'},
    'hind_right':  {'hip': 13, 'knee': 14, 'calf': 15, 'pos': 'hind_right'},
}"""

legs = {
    'front_left':  {'hip': 4,  'knee': 3,  'calf': 2,  'pos': 'front_left'},
    'hind_left': {'hip': 7,  'knee': 6,  'calf': 5,  'pos': 'hind_left'},
    'front_right':   {'hip': 8, 'knee': 9, 'calf': 10, 'pos': 'front_right'},
    'hind_right':  {'hip': 11, 'knee': 12, 'calf': 13, 'pos': 'hind_right'},
}

# Gains for altitude adjustment based on IMU data (tune these experimentally)
k_pitch_alt = 0.1   # forward/back correction gain
k_roll_alt  = 0.1   # side correction gain

# Initialize the MPU6050 IMU (adjust address or bus if necessary)
imu = mpu6050(0x68)

# ----- Inverse Kinematics Function -----
def compute_IK(L, l1, l2):
    """
    Compute knee and calf angles from desired vertical distance L.
    The formula used is:
      theta = 180 - degrees(acos((l1^2 + l2^2 - L^2) / (2*l1*l2)))
      knee_angle = degrees(atan((l2*sin(radians(theta))) / (l1 + l2*cos(radians(theta)))))
      calf_angle = theta   (supplementary angle configuration)
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

    knee_angle = math.degrees(math.atan((l2 * math.sin(math.radians(theta))) /
                                          (l1 + l2 * math.cos(math.radians(theta)))))
    calf_angle = theta  # using supplement configuration for the calf joint
    return knee_angle, calf_angle

# ----- Altitude Adjustment Based on IMU Data -----
def adjust_altitude(L_base, leg_position, pitch, roll):
    """
    Adjust the base altitude L based on IMU pitch and roll data.
    
    For example:
      - Front legs: if pitch is positive (tilt forward), reduce L (contract leg) to lift the front.
      - Hind legs: if pitch is positive, increase L.
      - Left legs: if roll is positive (tilt right), increase L.
      - Right legs: if roll is positive, decrease L.
    """
    L_adjusted = L_base

    # Adjust for pitch.
    if 'front' in leg_position:
        L_adjusted -= k_pitch_alt * pitch
    elif 'hind' in leg_position:
        L_adjusted += k_pitch_alt * pitch

    # Adjust for roll.
    if 'left' in leg_position:
        L_adjusted += k_roll_alt * roll
    elif 'right' in leg_position:
        L_adjusted -= k_roll_alt * roll

    # Clamp the adjusted altitude.
    if L_adjusted > L_max:
        L_adjusted = L_max
    if L_adjusted < L_min:
        L_adjusted = L_min

    return L_adjusted

# ----- Kalman Filter Implementation -----
class KalmanFilter:
    def __init__(self, dt, process_noise, measurement_noise):
        self.dt = dt
        # State vector: [angle, bias]
        self.x = [0, 0]
        # State covariance matrix
        self.P = [[1, 0], [0, 1]]
        # State transition matrix A
        self.A = [[1, -dt],
                  [0, 1]]
        # Observation matrix H (we only measure angle)
        self.H = [1, 0]
        # Process noise covariance Q
        self.Q = [[process_noise, 0],
                  [0, process_noise]]
        # Measurement noise covariance R
        self.R = measurement_noise

    def update(self, measurement):
        # Prediction step:
        x_prior = [
            self.A[0][0] * self.x[0] + self.A[0][1] * self.x[1],
            self.A[1][0] * self.x[0] + self.A[1][1] * self.x[1]
        ]
        # Covariance prediction: P_prior = A*P*A^T + Q
        P_prior = [
            [
                self.A[0][0]*self.P[0][0] + self.A[0][1]*self.P[1][0],
                self.A[0][0]*self.P[0][1] + self.A[0][1]*self.P[1][1]
            ],
            [
                self.A[1][0]*self.P[0][0] + self.A[1][1]*self.P[1][0],
                self.A[1][0]*self.P[0][1] + self.A[1][1]*self.P[1][1]
            ]
        ]
        # Add process noise Q
        P_prior[0][0] += self.Q[0][0]
        P_prior[0][1] += self.Q[0][1]
        P_prior[1][0] += self.Q[1][0]
        P_prior[1][1] += self.Q[1][1]

        # Kalman Gain: K = P_prior * H^T / (H * P_prior * H^T + R)
        # Since H = [1, 0], S = P_prior[0][0] + R.
        S = P_prior[0][0] + self.R
        K = [P_prior[0][0] / S, P_prior[1][0] / S]

        # Update step:
        innovation = measurement - (self.H[0]*x_prior[0] + self.H[1]*x_prior[1])
        self.x[0] = x_prior[0] + K[0] * innovation
        self.x[1] = x_prior[1] + K[1] * innovation

        # Covariance update: P = (I - K*H)*P_prior
        self.P[0][0] = (1 - K[0]*self.H[0]) * P_prior[0][0]
        self.P[0][1] = (1 - K[0]*self.H[0]) * P_prior[0][1]
        self.P[1][0] = -K[1]*self.H[0]*P_prior[0][0] + P_prior[1][0]
        self.P[1][1] = -K[1]*self.H[0]*P_prior[0][1] + P_prior[1][1]

        return self.x[0]  # Return the filtered angle

# ----- PID Controller Implementation -----
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.integral = 0
        self.prev_error = 0

    def update(self, measured_value, dt):
        error = self.setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

# ----- IMU Reading Function with Kalman Filter -----
def read_imu_angles_kalman(kf_pitch, kf_roll):
    data = imu.get_accel_data()  # returns dictionary with keys 'x', 'y', 'z'
    ax, ay, az = data['x'], data['y'], data['z']
    # Raw angle measurements.
    raw_pitch = math.degrees(math.atan2(ax, math.sqrt(ay**2 + az**2)))
    raw_roll  = math.degrees(math.atan2(ay, math.sqrt(ax**2 + az**2)))
    # Apply the Kalman filter.
    filtered_pitch = kf_pitch.update(raw_pitch)
    filtered_roll = kf_roll.update(raw_roll)
    return filtered_pitch, filtered_roll

# ----- Main Loop for Stabilization with Kalman Filter and PID Controller -----
def stabilization_test_with_filters():
    print("Starting stabilization test with Kalman and PID control.")
    dt = 0.05  # 50 ms update interval
    kf_pitch = KalmanFilter(dt, process_noise=0.001, measurement_noise=0.03)
    kf_roll = KalmanFilter(dt, process_noise=0.001, measurement_noise=0.03)
    # Initialize a PID controller to adjust altitude toward BASE_ALTITUDE.
    pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.05, setpoint=BASE_ALTITUDE)
    
    last_time = time.time()
    try:
        while True:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time

            # Get filtered IMU angles.
            pitch, roll = read_imu_angles_kalman(kf_pitch, kf_roll)
            print(f"(Debug) Filtered IMU: pitch={pitch:.2f}°, roll={roll:.2f}°")

            # Process each leg.
            for leg_name, params in legs.items():
                pos = params['pos']
                L_base = BASE_ALTITUDE
                # Adjust altitude based on IMU readings.
                L_desired = adjust_altitude(L_base, pos, pitch, roll)
                # Apply PID to refine the altitude toward the setpoint.
                L_corrected = L_desired + pid.update(L_desired, dt)
                # Clamp the corrected altitude.
                L_corrected = max(L_min, min(L_corrected, L_max))

                # Compute inverse kinematics for the corrected altitude.
                knee_angle, calf_angle = compute_IK(L_corrected, l1, l2)
                hip_angle = HIP_FIXED

                print(f"{leg_name:12s} | L_desired: {L_desired:5.2f} -> L_corrected: {L_corrected:5.2f} | "
                      f"Hip: {hip_angle:5.2f}°, Knee: {knee_angle:5.2f}°, Calf: {calf_angle:5.2f}°")

                # Send commands to the servos.
                s(params['hip'], hip_angle)
                s(params['knee'], knee_angle)
                s(params['calf'], calf_angle)
            print("-" * 80)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("Stabilization test stopped.")

if __name__ == '__main__':
    stabilization_test_with_filters()
