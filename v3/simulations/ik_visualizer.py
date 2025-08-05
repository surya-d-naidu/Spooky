"""
Inverse Kinematics Visualization
Shows leg geometry, reachable workspace, and IK solutions
"""

import matplotlib.pyplot as plt
import numpy as np
import math
import sys
import os

# Add parent directory to path to import robot_config
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot_config import config

class IKVisualizer:
    def __init__(self):
        self.gait_config = config['gait']
        self.l1 = self.gait_config['l1']  # Upper leg length
        self.l2 = self.gait_config['l2']  # Lower leg length
        self.L_max = self.gait_config['L_max']
        self.L_min = self.gait_config['L_min']
        
        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=(15, 12))
        self.fig.suptitle('Inverse Kinematics Analysis', fontsize=16)
        
    def compute_IK(self, L):
        """
        Compute IK using the formula from trot.py
        Returns (knee_angle, calf_angle) in degrees
        """
        # Clamp L within physical limits
        if L > (self.l1 + self.l2):
            L = self.l1 + self.l2
        if L < max(self.l1, self.l2) - min(self.l1, self.l2) + 0.1:
            L = max(self.l1, self.l2) - min(self.l1, self.l2) + 0.1
        
        try:
            # Compute theta in degrees
            theta = 180 - math.degrees(math.acos(((self.l1**2) + (self.l2**2) - (L**2)) / (2 * self.l1 * self.l2)))
        except ValueError:
            theta = 90.0
        
        # Compute knee angle
        knee_angle = math.degrees(math.atan((self.l2 * math.sin(math.radians(theta))) /
                                          (self.l1 + self.l2 * math.cos(math.radians(theta)))))
        calf_angle = theta
        
        return knee_angle, calf_angle
    
    def plot_leg_geometry(self):
        """Plot leg geometry and joint positions"""
        self.ax1.clear()
        self.ax1.set_title('Leg Geometry and Joint Configuration')
        
        # Test different L values
        L_values = [self.L_min, (self.L_min + self.L_max)/2, self.L_max]
        colors = ['red', 'blue', 'green']
        labels = ['Min Extension (Lifted)', 'Mid Extension', 'Max Extension (Ground)']
        
        for L, color, label in zip(L_values, colors, labels):
            knee_angle, calf_angle = self.compute_IK(L)
            
            # Convert to coordinates for visualization
            # Hip at origin (0, 0)
            hip_x, hip_y = 0, 0
            
            # Knee position (relative to hip)
            knee_x = self.l1 * math.cos(math.radians(knee_angle))
            knee_y = -self.l1 * math.sin(math.radians(knee_angle))  # Negative for downward
            
            # Foot position (relative to knee)
            foot_x = knee_x + self.l2 * math.cos(math.radians(knee_angle - calf_angle))
            foot_y = knee_y - self.l2 * math.sin(math.radians(knee_angle - calf_angle))
            
            # Plot leg segments
            self.ax1.plot([hip_x, knee_x], [hip_y, knee_y], color=color, linewidth=3, 
                         label=f'{label} (L={L:.2f})')
            self.ax1.plot([knee_x, foot_x], [knee_y, foot_y], color=color, linewidth=3)
            
            # Plot joints
            self.ax1.scatter([hip_x, knee_x, foot_x], [hip_y, knee_y, foot_y], 
                           c=color, s=100, zorder=5)
            
            # Annotations
            self.ax1.annotate(f'Knee: {knee_angle:.1f}°', 
                            (knee_x, knee_y), xytext=(5, 5), 
                            textcoords='offset points', fontsize=8, color=color)
            self.ax1.annotate(f'Calf: {calf_angle:.1f}°', 
                            (foot_x, foot_y), xytext=(5, 5), 
                            textcoords='offset points', fontsize=8, color=color)
        
        self.ax1.set_xlabel('X Position')
        self.ax1.set_ylabel('Y Position') 
        self.ax1.set_aspect('equal')
        self.ax1.grid(True, alpha=0.3)
        self.ax1.legend()
        self.ax1.set_xlim(-2, 12)
        self.ax1.set_ylim(-12, 2)
        
    def plot_workspace(self):
        """Plot reachable workspace of the leg"""
        self.ax2.clear()
        self.ax2.set_title('Leg Reachable Workspace')
        
        # Generate all possible foot positions
        foot_positions = []
        
        # Sweep through all possible L values
        L_range = np.linspace(abs(self.l1 - self.l2) + 0.1, self.l1 + self.l2 - 0.1, 100)
        
        for L in L_range:
            knee_angle, calf_angle = self.compute_IK(L)
            
            # Convert to foot position
            knee_x = self.l1 * math.cos(math.radians(knee_angle))
            knee_y = -self.l1 * math.sin(math.radians(knee_angle))
            
            foot_x = knee_x + self.l2 * math.cos(math.radians(knee_angle - calf_angle))
            foot_y = knee_y - self.l2 * math.sin(math.radians(knee_angle - calf_angle))
            
            foot_positions.append((foot_x, foot_y))
        
        # Plot workspace boundary
        if foot_positions:
            xs, ys = zip(*foot_positions)
            self.ax2.plot(xs, ys, 'b-', linewidth=2, label='Workspace Boundary')
            
        # Mark operating range
        L_op_range = np.linspace(self.L_min, self.L_max, 20)
        op_positions = []
        
        for L in L_op_range:
            knee_angle, calf_angle = self.compute_IK(L)
            knee_x = self.l1 * math.cos(math.radians(knee_angle))
            knee_y = -self.l1 * math.sin(math.radians(knee_angle))
            foot_x = knee_x + self.l2 * math.cos(math.radians(knee_angle - calf_angle))
            foot_y = knee_y - self.l2 * math.sin(math.radians(knee_angle - calf_angle))
            op_positions.append((foot_x, foot_y))
        
        if op_positions:
            xs, ys = zip(*op_positions)
            self.ax2.scatter(xs, ys, c='red', s=30, alpha=0.7, label='Operating Range')
        
        # Hip position
        self.ax2.scatter([0], [0], c='black', s=100, marker='s', label='Hip Joint')
        
        self.ax2.set_xlabel('X Position')
        self.ax2.set_ylabel('Y Position')
        self.ax2.set_aspect('equal')
        self.ax2.grid(True, alpha=0.3)
        self.ax2.legend()
        
    def plot_angle_vs_length(self):
        """Plot joint angles vs leg extension"""
        self.ax3.clear()
        self.ax3.set_title('Joint Angles vs Leg Extension')
        
        # Range of L values
        L_values = np.linspace(self.L_min, self.L_max, 50)
        knee_angles = []
        calf_angles = []
        
        for L in L_values:
            knee_angle, calf_angle = self.compute_IK(L)
            knee_angles.append(knee_angle)
            calf_angles.append(calf_angle)
        
        self.ax3.plot(L_values, knee_angles, 'b-', linewidth=2, label='Knee Angle')
        self.ax3.plot(L_values, calf_angles, 'r-', linewidth=2, label='Calf Angle')
        
        # Mark operating points
        self.ax3.axvline(x=self.L_min, color='gray', linestyle='--', alpha=0.7, label='Min Extension (Lift)')
        self.ax3.axvline(x=self.L_max, color='gray', linestyle=':', alpha=0.7, label='Max Extension (Ground)')
        
        self.ax3.set_xlabel('Leg Extension L')
        self.ax3.set_ylabel('Joint Angle (degrees)')
        self.ax3.grid(True, alpha=0.3)
        self.ax3.legend()
        
    def plot_gait_trajectory(self):
        """Plot foot trajectory during gait cycle"""
        self.ax4.clear()
        self.ax4.set_title('Foot Trajectory During Gait Cycle')
        
        # Simulate one gait cycle
        time_steps = np.linspace(0, 1, 100)  # Normalized time 0-1
        foot_positions = []
        swing_positions = []
        stance_positions = []
        
        for norm_phase in time_steps:
            # Compute L based on gait phase (from gait_simulator.py logic)
            if 0.15 <= norm_phase < 0.25:
                # Lifting phase
                prog = (norm_phase - 0.15) / 0.1
                L = self.L_max - prog * (self.L_max - self.L_min)
                is_swing = True
            elif 0.25 <= norm_phase < 0.35:
                # Lowering phase
                prog = (norm_phase - 0.25) / 0.1
                L = self.L_min + prog * (self.L_max - self.L_min)
                is_swing = True
            else:
                # On ground
                L = self.L_max
                is_swing = False
            
            # Convert to foot position
            knee_angle, calf_angle = self.compute_IK(L)
            knee_x = self.l1 * math.cos(math.radians(knee_angle))
            knee_y = -self.l1 * math.sin(math.radians(knee_angle))
            foot_x = knee_x + self.l2 * math.cos(math.radians(knee_angle - calf_angle))
            foot_y = knee_y - self.l2 * math.sin(math.radians(knee_angle - calf_angle))
            
            # Add horizontal movement (simplified)
            step_length = config['gait']['step_length'] * 0.01  # Scale down for visualization
            if norm_phase < 0.15:
                horizontal_offset = -step_length * (norm_phase / 0.15)
            elif norm_phase < 0.35:
                horizontal_offset = -step_length * 0.2 + step_length * ((norm_phase - 0.15) / 0.2)
            else:
                horizontal_offset = step_length * 0.8 * (1 - (norm_phase - 0.35) / 0.65)
            
            foot_x += horizontal_offset
            
            if is_swing:
                swing_positions.append((foot_x, foot_y))
            else:
                stance_positions.append((foot_x, foot_y))
            
            foot_positions.append((foot_x, foot_y))
        
        # Plot trajectory
        if foot_positions:
            xs, ys = zip(*foot_positions)
            self.ax4.plot(xs, ys, 'k-', linewidth=2, alpha=0.7, label='Full Trajectory')
        
        # Plot swing phase
        if swing_positions:
            xs, ys = zip(*swing_positions)
            self.ax4.scatter(xs, ys, c='red', s=20, alpha=0.7, label='Swing Phase')
        
        # Plot stance phase
        if stance_positions:
            xs, ys = zip(*stance_positions)
            self.ax4.scatter(xs, ys, c='blue', s=20, alpha=0.7, label='Stance Phase')
        
        # Hip position
        self.ax4.scatter([0], [0], c='black', s=100, marker='s', label='Hip Joint')
        
        self.ax4.set_xlabel('X Position (Forward)')
        self.ax4.set_ylabel('Y Position (Vertical)')
        self.ax4.set_aspect('equal')
        self.ax4.grid(True, alpha=0.3)
        self.ax4.legend()
        
    def run_analysis(self):
        """Run complete IK analysis"""
        print("🦾 Running Inverse Kinematics Analysis...")
        print(f"Upper leg length (l1): {self.l1}")
        print(f"Lower leg length (l2): {self.l2}")
        print(f"Operating range: {self.L_min:.2f} - {self.L_max:.2f}")
        
        # Generate all plots
        self.plot_leg_geometry()
        self.plot_workspace()
        self.plot_angle_vs_length()
        self.plot_gait_trajectory()
        
        plt.tight_layout()
        plt.show()
        
        # Print analysis
        print("\n📊 IK Analysis Results:")
        print("=" * 50)
        
        # Test key positions
        test_positions = [
            ("Ground Contact", self.L_max),
            ("Mid Position", (self.L_min + self.L_max) / 2),
            ("Max Lift", self.L_min)
        ]
        
        for name, L in test_positions:
            knee_angle, calf_angle = self.compute_IK(L)
            print(f"{name:15s}: L={L:5.2f}, Knee={knee_angle:6.1f}°, Calf={calf_angle:6.1f}°")

def main():
    """Main function to run IK analysis"""
    print("🔧 Inverse Kinematics Visualizer")
    print("=" * 35)
    
    visualizer = IKVisualizer()
    visualizer.run_analysis()
    
    print("\n✅ Analysis complete!")
    print("Close the plot window to continue...")

if __name__ == "__main__":
    main()
