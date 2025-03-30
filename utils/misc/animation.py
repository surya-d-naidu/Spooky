
from __future__ import division
import numpy as np
import time
import Adafruit_PCA9685
import smbus
#from mpu6050 import MPU6050
import math

############# SETUP ##############
pwm = Adafruit_PCA9685.PCA9685(0x40, busnum=1)
pwm.set_pwm_freq(60)
"""
bus = smbus.SMBus(1)
mpu = MPU6050(0x68, bus)"""
##################################

class PIDController():
    def __init__(self, kp, ki, kd, setpoint=0.0, sample_time=0.02, output_limits=(None, None)):
        """
        Simple PID controller.
        :param kp: Proportional gain.
        :param ki: Integral gain.
        :param kd: Derivative gain.
        :param setpoint: The target value.
        :param sample_time: Minimum time between updates.
        :param output_limits: Tuple (min, max) to limit output.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.sample_time = sample_time
        self.output_limits = output_limits
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
    
    def update(self, measurement):
        now = time.time()
        dt = now - self.last_time
        if dt >= self.sample_time:
            error = self.setpoint - measurement
            self.integral += error * dt
            derivative = (error - self.last_error) / dt if dt > 0 else 0.0
            output = self.kp * error + self.ki * self.integral + self.kd * derivative

            # Apply output limits.
            lower, upper = self.output_limits
            if lower is not None:
                output = max(lower, output)
            if upper is not None:
                output = min(upper, output)

            self.last_error = error
            self.last_time = now
            return output
        else:
            return None

class Leg:
    def __init__(self, t_length, c_length, hip_channel, thigh_channel, calf_channel):
        self.t_length = t_length
        self.c_length = c_length
        self.hip_channel = hip_channel
        self.thigh_channel = thigh_channel
        self.calf_channel = calf_channel

    def get_angles(self, x, y, z):
        """
        Compute inverse kinematics based on (x, y, z) position.
        Adjust the equations based on your robot’s geometry.
        """
        d = np.sqrt(x**2 + y**2)
        # Avoid division by zero.
        if d == 0:
            d = 1e-6
        angle_calf = np.arccos((d**2 + self.t_length**2 - self.c_length**2) / (2 * self.t_length * d))
        angle_thigh = np.arccos((self.t_length**2 + self.c_length**2 - d**2) / (2 * self.t_length * self.c_length))
        return angle_thigh, angle_calf

    def get_semicircular_trajectory(self, start_pos, end_pos, steps, arc_height):
        """
        Generate a semicircular trajectory. The x and y coordinates are linearly interpolated,
        while the z coordinate follows a sine curve (creating an arc) with a peak at arc_height.
        """
        trajectory = []
        for i in range(steps):
            t = i / (steps - 1)
            # Linear interpolation for x and y.
            pos = (1 - t) * start_pos + t * end_pos
            # Use a sine curve to add an arc in z.
            # (Assumes start_pos[2] and end_pos[2] are the base heights, e.g. 0)
            pos[2] = (1 - t) * start_pos[2] + t * end_pos[2] + arc_height * math.sin(math.pi * t)
            trajectory.append(pos)
        return trajectory

    def get_square_trajectory(self, start_pos, end_pos, seg_steps, arc_height):
        """
        Generate a square trajectory consisting of three segments:
          1. Vertical rise from start_pos to a raised height.
          2. Horizontal move at the raised height.
          3. Vertical descent to end_pos.
        seg_steps is the number of steps for each vertical segment.
        """
        traj = []
        # Segment 1: vertical rise.
        for i in range(seg_steps):
            t = i / (seg_steps - 1) if seg_steps > 1 else 1.0
            pos = np.copy(start_pos)
            pos[2] = start_pos[2] + t * arc_height
            traj.append(pos)
        
        # Segment 2: horizontal movement at raised height.
        # Use the same number of steps as the vertical segments.
        raised_start = start_pos + np.array([0, 0, arc_height])
        raised_end = end_pos + np.array([0, 0, arc_height])
        for i in range(seg_steps):
            t = i / (seg_steps - 1) if seg_steps > 1 else 1.0
            pos = (1 - t) * raised_start + t * raised_end
            traj.append(pos)
        
        # Segment 3: vertical descent.
        for i in range(seg_steps):
            t = i / (seg_steps - 1) if seg_steps > 1 else 1.0
            pos = np.copy(end_pos)
            pos[2] = end_pos[2] + arc_height * (1 - t)
            traj.append(pos)
        
        return traj

    def move_leg(self, start_pos, end_pos, duration=1.0, mode="semicircular", arc_height=20, max_step_height=None):
        """
        Moves the leg from start_pos to end_pos following one of the available trajectory modes.
        
        Parameters:
          - start_pos, end_pos: np.array with [x, y, z] coordinates.
          - duration: total time to complete the motion.
          - mode: "semicircular" or "square".
          - arc_height: maximum additional height (in cm) of the foot during the move.
          - max_step_height: maximum vertical change (in cm) allowed between consecutive steps.
        
        If max_step_height is provided, the number of steps is computed so that the vertical increment
        in the arc does not exceed max_step_height.
        """
        # Determine number of steps if max_step_height is provided.
        if max_step_height is not None:
            if mode == "semicircular":
                # For the sine arc: the maximum derivative is roughly arc_height * pi.
                # dt ~ 1/(steps-1), so require: arc_height*pi/(steps-1) <= max_step_height.
                steps = math.ceil((arc_height * math.pi) / max_step_height) + 1
            elif mode == "square":
                # For the vertical segments in square mode, each vertical segment goes through arc_height.
                # So we need: (arc_height / (seg_steps - 1)) <= max_step_height
                seg_steps = math.ceil(arc_height / max_step_height) + 1
                # We will use seg_steps for both rise and fall and also for horizontal movement.
                steps = seg_steps  # for each segment, then total steps = 3 * seg_steps.
            else:
                # For linear trajectory, compute steps based on overall vertical displacement.
                total_vertical = abs(end_pos[2] - start_pos[2])
                steps = math.ceil(total_vertical / max_step_height) + 1
        else:
            # If max_step_height is not specified, default to a fixed number of steps.
            steps = 20
            if mode == "square":
                seg_steps = steps  # Use the same for each segment.
        
        # Build the trajectory.
        if mode == "semicircular":
            trajectory = self.get_semicircular_trajectory(start_pos, end_pos, steps, arc_height)
            total_steps = steps
        elif mode == "square":
            trajectory = self.get_square_trajectory(start_pos, end_pos, seg_steps, arc_height)
            total_steps = len(trajectory)
        else:
            # Default: linear interpolation for all coordinates.
            trajectory = [ (1 - t) * start_pos + t * end_pos for t in np.linspace(0, 1, steps) ]
            total_steps = steps
        
        step_delay = duration / total_steps
        
        # Execute the movement along the trajectory.
        for pos in trajectory:
            x, y, z = pos
            angle_thigh, angle_calf = self.get_angles(x, y, z)
            self.move_servo(self.thigh_channel, angle_thigh)
            self.move_servo(self.calf_channel, angle_calf)
            time.sleep(step_delay)

    def move_servo(self, channel, angle):
        """
        Convert angle (in radians) to pulse width and set the servo.
        """
        angle_deg = np.degrees(angle)
        pulse_length = 1000000  # 1,000,000 us per second.
        pulse_length //= 60     # 60 Hz.
        pulse_length //= 4096   # 12 bits of resolution.
        pulse = int(angle_deg * (2000 / 180) + 500)  # Map 0-180° to 500-2500 us.
        pulse //= pulse_length
        pwm.set_pwm(channel, 0, pulse)

    def set_angles(self, angle_thigh, angle_calf):
        # self.move_servo(self.hip_channel, angle_hip)
        self.move_servo(self.thigh_channel, angle_thigh)
        self.move_servo(self.calf_channel, angle_calf)

class Animator:
    def __init__(self, fl_leg, fr_leg, bl_leg, br_leg):
        self.fl_leg = fl_leg
        self.fr_leg = fr_leg
        self.bl_leg = bl_leg
        self.br_leg = br_leg

    def move_leg(self, leg, start_pos, end_pos, time_duration, use_semicircular=True, arc_height=20):
        """
        Move a single leg along a trajectory.
        """
        if use_semicircular:
            trajectory = leg.get_semicircular_trajectory(start_pos, end_pos, time_duration, arc_height=arc_height)
        else:
            trajectory = leg.get_trajectory(start_pos, end_pos, time_duration)
        for pos in trajectory:
            angle_hip, angle_calf, angle_thigh = leg.get_angles(pos[0], pos[1], pos[2])
            leg.move_servo(leg.hip_channel, angle_hip)
            leg.move_servo(leg.thigh_channel, angle_thigh)
            leg.move_servo(leg.calf_channel, angle_calf)
            time.sleep(0.02)

    def move_all_legs(self, front_distance, back_distance, time_duration, use_semicircular=True, arc_height=20):
        """
        Move all legs sequentially.
        """
        fl_start_pos = np.array([0, 0, 0])
        fr_start_pos = np.array([0, 0, 0])
        fl_end_pos = np.array([front_distance, 0, 0])
        fr_end_pos = np.array([front_distance, 0, 0])
        self.move_leg(self.fl_leg, fl_start_pos, fl_end_pos, time_duration, use_semicircular, arc_height)
        self.move_leg(self.fr_leg, fr_start_pos, fr_end_pos, time_duration, use_semicircular, arc_height)

        bl_start_pos = np.array([0, 0, 0])
        br_start_pos = np.array([0, 0, 0])
        bl_end_pos = np.array([back_distance, 0, 0])
        br_end_pos = np.array([back_distance, 0, 0])
        self.move_leg(self.bl_leg, bl_start_pos, bl_end_pos, time_duration, use_semicircular, arc_height)
        self.move_leg(self.br_leg, br_start_pos, br_end_pos, time_duration, use_semicircular, arc_height)
"""
class Stabilizer():
    def __init__(self, mpu, roll_pid_params, pitch_pid_params, alpha=0.1):
        \"""
        Stabilizes the robot using MPU6050 sensor data with PID controllers.
        :param mpu: Instance of MPU6050.
        :param roll_pid_params: Tuple of PID parameters for roll 
                                (kp, ki, kd, setpoint, sample_time, output_limits).
        :param pitch_pid_params: Tuple of PID parameters for pitch 
                                 (kp, ki, kd, setpoint, sample_time, output_limits).
        :param alpha: Smoothing factor for low-pass filtering.
        \"""
        self.mpu = mpu
        self.alpha = alpha
        self.prev_roll = 0.0
        self.prev_pitch = 0.0
        self.roll_pid = PIDController(*roll_pid_params)
        self.pitch_pid = PIDController(*pitch_pid_params)

    def get_orientation(self):
        \"""
        Calculate filtered roll and pitch based on accelerometer data.
        \"""
        accel = self.mpu.get_accel_data()
        acc_x = accel['x']
        acc_y = accel['y']
        acc_z = accel['z']
        roll = np.arctan2(acc_y, acc_z)
        pitch = np.arctan2(-acc_x, np.sqrt(acc_y**2 + acc_z**2))
        roll = self.alpha * roll + (1 - self.alpha) * self.prev_roll
        pitch = self.alpha * pitch + (1 - self.alpha) * self.prev_pitch
        self.prev_roll = roll
        self.prev_pitch = pitch
        return roll, pitch

    def apply_stabilization(self, leg_positions):
        \"""
        Adjust leg target positions using PID-based corrections from the current orientation.
        :param leg_positions: Dictionary with keys 'fl', 'fr', 'bl', 'br' and np.array positions.
        :return: Dictionary with adjusted positions.
        \"""
        roll, pitch = self.get_orientation()
        roll_correction = self.roll_pid.update(roll)
        pitch_correction = self.pitch_pid.update(pitch)

        # Use zero correction if PID hasn't updated yet.
        if roll_correction is None:
            roll_correction = 0.0
        if pitch_correction is None:
            pitch_correction = 0.0

        adjustments = {}
        for leg in leg_positions:
            pos = leg_positions[leg].copy()
            # Adjust in x-direction based on pitch.
            if leg in ['fl', 'fr']:
                pos[0] -= pitch_correction
            elif leg in ['bl', 'br']:
                pos[0] += pitch_correction
            # Adjust in y-direction based on roll.
            if leg in ['fl', 'bl']:
                pos[1] += roll_correction
            elif leg in ['fr', 'br']:
                pos[1] -= roll_correction
            adjustments[leg] = pos
        return adjustments
"""
