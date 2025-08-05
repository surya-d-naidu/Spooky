"""
Servo Analysis and Calibration Tool
Analyze servo biases, ranges, and calibration
"""

import sys
import os
import time

# Add parent directory to path to import robot_config
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot_config import config

class ServoAnalyzer:
    def __init__(self):
        self.gait_config = config['gait']
        self.servo_biases = self.gait_config['servo_biases']
        self.legs = self.gait_config['legs']
        
    def analyze_servo_mapping(self):
        """Analyze servo channel mapping and biases"""
        print("🔧 Servo Channel Analysis")
        print("=" * 60)
        
        # Group servos by leg
        print("\n📍 Servo Mapping by Leg:")
        print("-" * 40)
        
        for leg_name, params in self.legs.items():
            hip_ch = params['hip']
            knee_ch = params['knee']
            calf_ch = params['calf']
            
            print(f"\n{leg_name.upper()}: ")
            print(f"  Hip  (Channel {hip_ch:2d}): Bias = {self.servo_biases.get(hip_ch, 0):+5.1f}°")
            print(f"  Knee (Channel {knee_ch:2d}): Bias = {self.servo_biases.get(knee_ch, 0):+5.1f}°")
            print(f"  Calf (Channel {calf_ch:2d}): Bias = {self.servo_biases.get(calf_ch, 0):+5.1f}°")
        
        # Analyze bias distribution
        print(f"\n📊 Bias Statistics:")
        print("-" * 30)
        
        all_biases = list(self.servo_biases.values())
        non_zero_biases = [b for b in all_biases if b != 0]
        
        print(f"Total servos: {len(all_biases)}")
        print(f"Servos with bias: {len(non_zero_biases)}")
        print(f"Servos without bias: {len(all_biases) - len(non_zero_biases)}")
        
        if non_zero_biases:
            print(f"Max bias: {max(non_zero_biases):+5.1f}°")
            print(f"Min bias: {min(non_zero_biases):+5.1f}°")
            print(f"Avg bias (non-zero): {sum(non_zero_biases)/len(non_zero_biases):+5.1f}°")
    
    def analyze_servo_ranges(self):
        """Analyze expected servo angle ranges during gait"""
        print("\n🎯 Servo Angle Range Analysis")
        print("=" * 50)
        
        # Simulate one gait cycle to find min/max angles
        T = self.gait_config['T']
        L_max = self.gait_config['L_max']
        L_min = self.gait_config['L_min']
        HIP_FIXED = self.gait_config['HIP_FIXED']
        step_length = self.gait_config['step_length']
        
        servo_ranges = {}
        
        # Initialize ranges
        for leg_name, params in self.legs.items():
            for joint in ['hip', 'knee', 'calf']:
                ch = params[joint]
                servo_ranges[ch] = {'min': float('inf'), 'max': float('-inf'), 'joint': joint, 'leg': leg_name}
        
        # Simulate gait cycle
        time_steps = [i * T / 100 for i in range(100)]  # 100 steps through cycle
        
        for t_ms in time_steps:
            for leg_name, params in self.legs.items():
                phase = params['phase']
                norm = ((t_ms + phase) % T) / T
                
                # Compute L_desired
                if 0.15 <= norm < 0.25:
                    prog = (norm - 0.15) / 0.1
                    L_des = L_max - prog * (L_max - L_min)
                elif 0.25 <= norm < 0.35:
                    prog = (norm - 0.25) / 0.1
                    L_des = L_min + prog * (L_max - L_min)
                else:
                    L_des = L_max
                
                # Compute angles (simplified IK)
                knee_ang, calf_ang = self.compute_IK(L_des)
                
                # Horizontal offset
                if norm < 0.15:
                    off = -step_length * (norm / 0.15)
                elif norm < 0.35:
                    off = -step_length * 0.2 + step_length * ((norm - 0.15) / 0.2)
                else:
                    off = step_length * 0.8 * (1 - (norm - 0.35) / 0.65)
                
                mod_knee = knee_ang - off
                
                # Apply special rules
                if params['knee'] == 3:  # Front-left special case
                    mod_knee = max(0, mod_knee - 20)
                
                # Apply biases and update ranges
                angles = {
                    params['hip']: self.apply_bias(params['hip'], HIP_FIXED),
                    params['knee']: self.apply_bias(params['knee'], mod_knee),
                    params['calf']: self.apply_bias(params['calf'], calf_ang)
                }
                
                for ch, angle in angles.items():
                    servo_ranges[ch]['min'] = min(servo_ranges[ch]['min'], angle)
                    servo_ranges[ch]['max'] = max(servo_ranges[ch]['max'], angle)
        
        # Print results
        print("\nServo Angle Ranges (with biases applied):")
        print("-" * 50)
        print(f"{'Channel':<8} | {'Joint':<5} | {'Leg':<12} | {'Min':<6} | {'Max':<6} | {'Range':<6}")
        print("-" * 50)
        
        for ch in sorted(servo_ranges.keys()):
            data = servo_ranges[ch]
            range_val = data['max'] - data['min']
            print(f"{ch:<8} | {data['joint']:<5} | {data['leg']:<12} | "
                  f"{data['min']:<6.1f} | {data['max']:<6.1f} | {range_val:<6.1f}")
    
    def compute_IK(self, L):
        """Simplified IK computation"""
        import math
        
        l1 = self.gait_config['l1']
        l2 = self.gait_config['l2']
        
        # Clamp L
        if L > (l1 + l2):
            L = l1 + l2
        if L < abs(l1 - l2) + 0.1:
            L = abs(l1 - l2) + 0.1
        
        try:
            theta = 180 - math.degrees(math.acos(((l1**2) + (l2**2) - (L**2)) / (2 * l1 * l2)))
            knee_angle = math.degrees(math.atan((l2 * math.sin(math.radians(theta))) /
                                              (l1 + l2 * math.cos(math.radians(theta)))))
            calf_angle = theta
            return knee_angle, calf_angle
        except:
            return 45.0, 90.0
    
    def apply_bias(self, channel, angle):
        """Apply servo bias"""
        return angle + self.servo_biases.get(channel, 0.0)
    
    def test_servo_calibration(self):
        """Interactive servo calibration test"""
        print("\n🔧 Servo Calibration Test")
        print("=" * 40)
        print("This would normally move servos to test positions.")
        print("Since we're in simulation mode, showing expected commands:\n")
        
        test_angles = [0, 45, 90, 135, 180]
        
        for leg_name, params in self.legs.items():
            print(f"\n{leg_name.upper()} LEG TEST:")
            print("-" * 25)
            
            for joint_name, channel in [('Hip', params['hip']), 
                                       ('Knee', params['knee']), 
                                       ('Calf', params['calf'])]:
                print(f"\n{joint_name} (Channel {channel}):")
                
                for test_angle in test_angles:
                    biased_angle = self.apply_bias(channel, test_angle)
                    print(f"  Command {test_angle:3d}° → Servo moves to {biased_angle:6.1f}°")
    
    def generate_calibration_sequence(self):
        """Generate a calibration sequence"""
        print("\n📋 Calibration Sequence Generator")
        print("=" * 45)
        
        print("# Servo Calibration Sequence")
        print("# Copy this code to test each servo individually\n")
        
        print("from servo import set_servo_angle")
        print("import time\n")
        
        print("def calibrate_servo(channel, name):")
        print("    print(f'Calibrating {name} (Channel {channel})')")
        print("    angles = [90, 0, 45, 135, 180, 90]  # Test sequence")
        print("    for angle in angles:")
        print("        print(f'  Setting to {angle}°')")
        print("        set_servo_angle(channel, angle)")
        print("        time.sleep(2)")
        print("    print('Done\\n')\n")
        
        for leg_name, params in self.legs.items():
            print(f"# {leg_name.upper()} LEG")
            for joint_name, channel in [('Hip', params['hip']), 
                                       ('Knee', params['knee']), 
                                       ('Calf', params['calf'])]:
                servo_name = f"{leg_name}_{joint_name.lower()}"
                print(f"calibrate_servo({channel}, '{servo_name}')")
            print()
    
    def run_analysis(self):
        """Run complete servo analysis"""
        print("🔍 Servo Analysis and Calibration Tool")
        print("=" * 50)
        
        self.analyze_servo_mapping()
        self.analyze_servo_ranges()
        self.test_servo_calibration()
        self.generate_calibration_sequence()
        
        print("\n✅ Analysis complete!")

def main():
    """Main function"""
    analyzer = ServoAnalyzer()
    analyzer.run_analysis()

if __name__ == "__main__":
    main()
