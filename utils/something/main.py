
import time
from imu_balance import get_data, apply_kalman_filter, set_angle_after_balance

L1 = 10.0  # Upper leg length in cm
L2 = 10.0  # Lower leg length in cm

def main():
    while True:
        raw_angles = get_data()
        filtered_angles = apply_kalman_filter(raw_angles)
        joint_angles = set_angle_after_balance(filtered_angles, L1, L2)

        print("Filtered Angles:", filtered_angles)
        print("Joint Angles (per leg):", joint_angles)

        time.sleep(0.05)

def walk():
    # Simple gait logic: shift pitch and walk forward
    for step in range(10):
        # Simulate shifting center of gravity
        desired_angles = (5.0 * ((step % 2) * 2 - 1), 0.0)
        joint_angles = set_angle_after_balance(desired_angles, L1, L2)
        print(f"Step {step+1}: Joint Angles: {joint_angles}")
        time.sleep(0.5)

if __name__ == "__main__":
    walk()
