"""
Gait Pattern Simulation and Visualization
Shows crawl gait timing, leg phases, and coordination
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import sys
import os

# Add parent directory to path to import robot_config
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot_config import config

class GaitSimulator:
    def __init__(self):
        self.gait_config = config['gait']
        self.T = self.gait_config['T']  # Gait cycle period in ms
        self.legs = self.gait_config['legs']
        self.L_max = self.gait_config['L_max']
        self.L_min = self.gait_config['L_min']
        
        # Colors for each leg
        self.leg_colors = {
            'front_left': 'red',
            'front_right': 'blue', 
            'hind_left': 'green',
            'hind_right': 'orange'
        }
        
        # Setup the plot
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(12, 10))
        self.fig.suptitle('Quadruped Crawl Gait Analysis', fontsize=16)
        
    def compute_leg_height(self, normalized_phase):
        """Compute leg height based on normalized phase (0-1)"""
        if 0.15 <= normalized_phase < 0.25:
            # Lifting phase
            prog = (normalized_phase - 0.15) / 0.1
            return self.L_max - prog * (self.L_max - self.L_min)
        elif 0.25 <= normalized_phase < 0.35:
            # Lowering phase
            prog = (normalized_phase - 0.25) / 0.1
            return self.L_min + prog * (self.L_max - self.L_min)
        else:
            # On ground
            return self.L_max
    
    def get_leg_state(self, normalized_phase):
        """Get leg state as string"""
        if 0.15 <= normalized_phase < 0.25:
            return "LIFT"
        elif 0.25 <= normalized_phase < 0.35:
            return "LOWER"
        else:
            return "GROUND"
    
    def plot_gait_timeline(self):
        """Plot gait timeline showing when each leg is in swing/stance"""
        self.ax1.clear()
        self.ax1.set_title('Gait Timeline - Swing vs Stance Phases')
        
        # Time array for one complete gait cycle
        time_ms = np.linspace(0, self.T, 1000)
        
        y_positions = {'front_left': 3, 'front_right': 2, 'hind_right': 1, 'hind_left': 0}
        
        for leg_name, params in self.legs.items():
            phase_offset = params['phase']
            y_pos = y_positions[leg_name]
            color = self.leg_colors[leg_name]
            
            swing_periods = []
            stance_periods = []
            
            for t in time_ms:
                norm_phase = ((t + phase_offset) % self.T) / self.T
                if 0.15 <= norm_phase < 0.35:  # Swing phase
                    swing_periods.append((t, y_pos))
                else:  # Stance phase
                    stance_periods.append((t, y_pos))
            
            # Plot stance phases (thick line)
            if stance_periods:
                stance_times = [p[0] for p in stance_periods]
                stance_y = [p[1] for p in stance_periods]
                self.ax1.scatter(stance_times, stance_y, c=color, s=2, alpha=0.7, label=f'{leg_name} stance')
            
            # Plot swing phases (thin line)
            if swing_periods:
                swing_times = [p[0] for p in swing_periods]
                swing_y = [p[1] for p in swing_periods]
                self.ax1.scatter(swing_times, swing_y, c=color, s=8, marker='x', alpha=0.9)
        
        self.ax1.set_xlabel('Time (ms)')
        self.ax1.set_ylabel('Leg')
        self.ax1.set_yticks([0, 1, 2, 3])
        self.ax1.set_yticklabels(['Hind Left', 'Hind Right', 'Front Right', 'Front Left'])
        self.ax1.grid(True, alpha=0.3)
        self.ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        
    def plot_leg_heights(self):
        """Plot leg height variations during gait cycle"""
        self.ax2.clear()
        self.ax2.set_title('Leg Height Variations During Gait Cycle')
        
        # Time array for one complete gait cycle
        time_ms = np.linspace(0, self.T, 1000)
        
        for leg_name, params in self.legs.items():
            phase_offset = params['phase']
            color = self.leg_colors[leg_name]
            heights = []
            
            for t in time_ms:
                norm_phase = ((t + phase_offset) % self.T) / self.T
                height = self.compute_leg_height(norm_phase)
                heights.append(height)
            
            self.ax2.plot(time_ms, heights, color=color, linewidth=2, label=leg_name)
        
        self.ax2.axhline(y=self.L_max, color='gray', linestyle='--', alpha=0.5, label='Ground Level')
        self.ax2.axhline(y=self.L_min, color='gray', linestyle=':', alpha=0.5, label='Max Lift Height')
        self.ax2.set_xlabel('Time (ms)')
        self.ax2.set_ylabel('Leg Extension (units)')
        self.ax2.legend()
        self.ax2.grid(True, alpha=0.3)
        
    def plot_stability_diagram(self):
        """Plot stability diagram showing center of support"""
        self.ax3.clear()
        self.ax3.set_title('Robot Stability Analysis - Support Pattern')
        
        # Robot footprint (rough approximation)
        robot_length = 10  # arbitrary units
        robot_width = 8
        
        # Leg positions (x, y) relative to robot center
        leg_positions = {
            'front_left': (-robot_length/2, robot_width/2),
            'front_right': (-robot_length/2, -robot_width/2),
            'hind_left': (robot_length/2, robot_width/2),
            'hind_right': (robot_length/2, -robot_width/2)
        }
        
        # Check stability at different time points
        time_points = np.linspace(0, self.T, 50)
        
        for i, t in enumerate(time_points):
            supporting_legs = []
            
            for leg_name, params in self.legs.items():
                phase_offset = params['phase']
                norm_phase = ((t + phase_offset) % self.T) / self.T
                
                if not (0.15 <= norm_phase < 0.35):  # If on ground
                    supporting_legs.append(leg_name)
            
            # Plot supporting legs
            if len(supporting_legs) >= 3:  # Stable (3+ legs on ground)
                alpha = 0.8
                marker_size = 30
            else:  # Potentially unstable
                alpha = 0.3
                marker_size = 15
            
            for leg_name in supporting_legs:
                x, y = leg_positions[leg_name]
                color = self.leg_colors[leg_name]
                self.ax3.scatter(x + i*0.1, y, c=color, alpha=alpha, s=marker_size)
        
        # Draw robot outline
        robot_x = [-robot_length/2, robot_length/2, robot_length/2, -robot_length/2, -robot_length/2]
        robot_y = [-robot_width/2, -robot_width/2, robot_width/2, robot_width/2, -robot_width/2]
        self.ax3.plot(robot_x, robot_y, 'k--', alpha=0.5, linewidth=2)
        
        # Mark leg positions
        for leg_name, (x, y) in leg_positions.items():
            self.ax3.scatter(x, y, c=self.leg_colors[leg_name], s=100, marker='s', 
                           alpha=0.7, edgecolors='black', linewidth=2)
            self.ax3.annotate(leg_name.replace('_', '\n'), (x, y), 
                            xytext=(5, 5), textcoords='offset points', fontsize=8)
        
        self.ax3.set_xlabel('Forward Direction')
        self.ax3.set_ylabel('Lateral Direction') 
        self.ax3.set_aspect('equal')
        self.ax3.grid(True, alpha=0.3)
        self.ax3.set_xlim(-8, 8)
        self.ax3.set_ylim(-6, 6)
        
    def run_simulation(self):
        """Run the complete gait simulation"""
        print("🤖 Running Gait Pattern Simulation...")
        print(f"Gait Cycle Period: {self.T} ms")
        print(f"Leg Extension Range: {self.L_min:.2f} - {self.L_max:.2f} units")
        
        # Generate all plots
        self.plot_gait_timeline()
        self.plot_leg_heights()
        self.plot_stability_diagram()
        
        plt.tight_layout()
        plt.show()
        
        # Print gait analysis
        print("\n📊 Gait Analysis:")
        print("=" * 50)
        
        # Analyze each leg's timing
        for leg_name, params in self.legs.items():
            phase_deg = (params['phase'] / self.T) * 360
            swing_start = (phase_deg + 54) % 360  # 0.15 * 360
            swing_end = (phase_deg + 126) % 360   # 0.35 * 360
            print(f"{leg_name:12s}: Phase offset {phase_deg:6.1f}°, "
                  f"Swing {swing_start:6.1f}° - {swing_end:6.1f}°")
        
        print(f"\nSwing Phase Duration: {0.2 * self.T:.0f} ms (20% of cycle)")
        print(f"Stance Phase Duration: {0.8 * self.T:.0f} ms (80% of cycle)")

def main():
    """Main function to run gait simulation"""
    print("🦾 Quadruped Gait Pattern Simulator")
    print("=" * 40)
    
    simulator = GaitSimulator()
    simulator.run_simulation()
    
    print("\n✅ Simulation complete!")
    print("Close the plot window to continue...")

if __name__ == "__main__":
    main()
