from Physics import *
from animation import *

class Robot:
    def __init__(self):
        # Initialize body transform.
        self.body = Transform(position=Vector3(0, 0, 0), rotation=Quaternion())
        # Define leg offsets relative to the body center (in centimeters).
        self.leg_offsets = {
            'front_left': Vector3(-10, 0, 20),
            'front_right': Vector3(10, 0, 20),
            'back_left': Vector3(-10, 0, -20),
            'back_right': Vector3(10, 0, -20)
        }
        # Create four Leg objects (adjust lengths and channel numbers as needed).
        self.legs = {
            'front_left': Leg(t_length=100, c_length=100, hip_channel=0, thigh_channel=1, calf_channel=2),
            'front_right': Leg(t_length=100, c_length=100, hip_channel=3, thigh_channel=4, calf_channel=5),
            'back_left': Leg(t_length=100, c_length=100, hip_channel=6, thigh_channel=7, calf_channel=8),
            'back_right': Leg(t_length=100, c_length=100, hip_channel=9, thigh_channel=10, calf_channel=11)
        }
        # Initialize each foot’s world position.
        self.foot_positions = {}
        for key, offset in self.leg_offsets.items():
            self.foot_positions[key] = self.get_leg_world_position(offset)

    def get_leg_world_position(self, relative_offset: Vector3):
        # Rotate the offset by the body’s rotation and add the body position.
        rotated_q = self.body.rotation.rotate(relative_offset)
        rotated = Vector3(rotated_q.quaternion[1],
                          rotated_q.quaternion[2],
                          rotated_q.quaternion[3])
        return self.body.position + rotated

    def rotate_towards(self, target: Vector3):
        # Determine the vector from body to target.
        to_target = target - self.body.position
        to_target_norm = to_target.normalize()
        # Current forward direction.
        current_forward = self.body.get_forward().normalize()
        # Compute angle between current_forward and to_target.
        dot = current_forward.dot(to_target_norm)
        print(dot)
        dot = max(-1.0, min(1.0, dot))
        angle = math.acos(dot)
        # Determine the sign via the y-component of the cross product.
        cross = current_forward.cross(to_target_norm)
        if cross.vector[1] < 0:
            angle = -angle
        # Create a rotation quaternion about the y-axis.
        sin_half = math.sin(angle / 2)
        cos_half = math.cos(angle / 2)
        rot_quat = Quaternion(cos_half, 0, sin_half, 0)
        self.body.rotate(rot_quat)
        print(f"Rotated by {math.degrees(angle):.2f} degrees towards target.")

    def step_leg(self, leg_name, step_distance=5.0, duration=0.5, mode="semicircular", arc_height=5, max_step_height=1):
        # Get current foot position (as an array).
        start_pos = self.foot_positions[leg_name].vector
        # Calculate new foot position by moving it in the body's forward direction.
        forward = self.body.get_forward().normalize()
        new_pos = start_pos + forward.vector * step_distance
        # Command the leg to move.
        self.legs[leg_name].move_leg(start_pos, new_pos, duration=duration, mode=mode, arc_height=arc_height, max_step_height=max_step_height)
        # Update the stored foot position.
        self.foot_positions[leg_name] = Vector3(*new_pos)
        print(f"{leg_name} leg stepped from {start_pos} to {new_pos}.")

    def move_forward_cycle(self, step_distance=5.0, duration=2.0, mode="semicircular", arc_height=5, max_step_height=1):
        # Move legs one at a time.
        leg_order = ["front_left", "front_right", "back_left", "back_right"]
        for leg_name in leg_order:
            self.step_leg(leg_name, step_distance, duration / len(leg_order), mode, arc_height, max_step_height)
        # After all legs have stepped, update the body position.
        forward = self.body.get_forward().normalize()
        self.body.position = self.body.position + forward * step_distance
        # Reset foot positions relative to the new body position.
        for leg_name, offset in self.leg_offsets.items():
            self.foot_positions[leg_name] = self.get_leg_world_position(offset)
        print(f"Robot body moved to {self.body.position}.")

    def move_to(self, target: Vector3, cycles=5, step_distance=5.0, duration=2.0, mode="semicircular", arc_height=5, max_step_height=1):
        # First, rotate towards the target.
        self.rotate_towards(target)
        # Then execute several cycles of leg movements.
        for _ in range(cycles):
            self.move_forward_cycle(step_distance, duration, mode, arc_height, max_step_height)
            print(f"Current body position: {self.body.position}")
        print("Reached target (simulation).")

if __name__ == '__main__':
    # Initialize the robot.
    robot = Robot()
    # Define a target position (for example, 100 cm forward).
    target_position = Vector3(0, 0, 100)
    # Move the robot toward the target.
    robot.move_to(target_position, cycles=10, step_distance=10.0, duration=4.0,
                  mode="semicircular", arc_height=2, max_step_height=2)
