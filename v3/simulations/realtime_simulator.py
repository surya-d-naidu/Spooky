"""
Real-time Robot Movement Simulator
Shows real-time gait execution with servo angles and leg positions
"""

import time
import threading
import sys
import os

# Add parent directory to path to import robot_config
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot_config import config

try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    import numpy as np
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False

class RealTimeSimulator:
    def __init__(self):
        self.gait_config = config['gait']
        self.running = False
        self.current_time = 0
        self.servo_angles = {}
        
        # Initialize servo angles to neutral
        for leg_name, params in self.gait_config['legs'].items():
            self.servo_angles[params['hip']] = self.gait_config['HIP_FIXED']
            self.servo_angles[params['knee']] = 0
            self.servo_angles[params['calf']] = 90
            
        if MATPLOTLIB_AVAILABLE:
            self.setup_plots()
        
    def setup_plots(self):
        """Setup real-time plots"""
        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=(15, 10))
        self.fig.suptitle('Real-Time Robot Movement Simulation', fontsize=16)
        
        # Colors for each leg
        self.leg_colors = {
            'front_left': 'red',
            'front_right': 'blue', 
            'hind_left': 'green',
            'hind_right': 'orange'
        }
        
    def compute_IK(self, L):
        """Compute inverse kinematics"""
        l1 = self.gait_config['l1']
        l2 = self.gait_config['l2']
        
        # Clamp L within physical limits
        if L > (l1 + l2):
            L = l1 + l2
        if L < abs(l1 - l2) + 0.1:
            L = abs(l1 - l2) + 0.1
        
        try:
            import math
            theta = 180 - math.degrees(math.acos(((l1**2) + (l2**2) - (L**2)) / (2 * l1 * l2)))
            knee_angle = math.degrees(math.atan((l2 * math.sin(math.radians(theta))) /
                                              (l1 + l2 * math.cos(math.radians(theta)))))
            calf_angle = theta
            return knee_angle, calf_angle
        except:
            return 45.0, 90.0  # Default safe angles
    
    def apply_bias(self, channel, angle):
        """Apply servo bias from config"""
        return angle + self.gait_config['servo_biases'].get(channel, 0.0)
    
    def simulate_servo_command(self, channel, angle):
        """Simulate servo command (replace actual servo control)"""
        biased_angle = self.apply_bias(channel, angle)
        self.servo_angles[channel] = biased_angle
        print(f"Servo {channel:2d}: {biased_angle:6.1f}°")
    
    def update_gait_step(self):
        """Update one step of the gait simulation"""
        import math
        
        T = self.gait_config['T']
        L_max = self.gait_config['L_max']
        L_min = self.gait_config['L_min']
        HIP_FIXED = self.gait_config['HIP_FIXED']
        step_length = self.gait_config['step_length']
        legs = self.gait_config['legs']
        
        t_ms = self.current_time
        
        print(f"\n⏱️  Time: {t_ms:6.0f} ms")
        print("=" * 50)
        
        for leg_name, params in legs.items():
            phase = params['phase']
            norm = ((t_ms + phase) % T) / T
            
            # Determine leg state and L_desired
            if 0.15 <= norm < 0.25:
                # Swing up & forward
                prog = (norm - 0.15) / 0.1
                L_des = L_max - prog * (L_max - L_min)
                state = "LIFT"
            elif 0.25 <= norm < 0.35:
                # Swing down
                prog = (norm - 0.25) / 0.1
                L_des = L_min + prog * (L_max - L_min)
                state = "LOWER"
            else:
                # On ground
                L_des = L_max
                state = "GROUND"
            
            # Compute joint angles
            knee_ang, calf_ang = self.compute_IK(L_des)
            
            # Horizontal foot trajectory
            if norm < 0.15:
                off = -step_length * (norm / 0.15)
            elif norm < 0.35:
                off = -step_length * 0.2 + step_length * ((norm - 0.15) / 0.2)
            else:
                off = step_length * 0.8 * (1 - (norm - 0.35) / 0.65)
            
            mod_knee = knee_ang - off
            
            # Command servos (simulation)
            hip_channel = params['hip']
            knee_channel = params['knee']
            calf_channel = params['calf']
            
            print(f"{leg_name:12s} | {state:6s} | Phase: {norm:5.2f} | L: {L_des:5.2f}")
            
            self.simulate_servo_command(hip_channel, HIP_FIXED)
            
            # Special handling for front-left knee
            if knee_channel == 3:
                a = max(0, mod_knee - 20)
                self.simulate_servo_command(knee_channel, a)
            else:
                self.simulate_servo_command(knee_channel, mod_knee)
                
            self.simulate_servo_command(calf_channel, calf_ang)
    
    def run_text_simulation(self, duration=10):
        """Run text-based simulation"""
        print("🏃 Starting Real-Time Gait Simulation (Text Mode)")
        print(f"Duration: {duration} seconds")
        print("=" * 60)
        
        self.running = True
        start_time = time.time()
        
        try:
            while self.running and (time.time() - start_time) < duration:
                self.current_time = (time.time() - start_time) * 1000  # Convert to ms
                
                self.update_gait_step()
                
                # Update at configured interval
                time.sleep(self.gait_config['update_interval_ms'] / 1000.0)
                
        except KeyboardInterrupt:
            print("\n\n⏹️  Simulation stopped by user")
        
        self.running = False
        print(f"\n✅ Simulation completed after {time.time() - start_time:.1f} seconds")
    
    def animate_plots(self, frame):
        """Animation function for matplotlib plots"""
        if not MATPLOTLIB_AVAILABLE:
            return
            
        self.current_time = frame * self.gait_config['update_interval_ms']
        self.update_gait_step()
        
        # Clear all axes
        for ax in [self.ax1, self.ax2, self.ax3, self.ax4]:
            ax.clear()
        
        # Plot 1: Servo angles over time
        self.ax1.set_title('Servo Angles (Real-time)')
        channels = sorted(self.servo_angles.keys())
        angles = [self.servo_angles[ch] for ch in channels]
        colors = ['red' if ch in [2,3,4] else 'blue' if ch in [8,9,10] else 
                 'green' if ch in [5,6,7] else 'orange' for ch in channels]
        
        bars = self.ax1.bar(channels, angles, color=colors, alpha=0.7)
        self.ax1.set_xlabel('Servo Channel')
        self.ax1.set_ylabel('Angle (degrees)')
        self.ax1.set_ylim(0, 180)
        self.ax1.grid(True, alpha=0.3)
        
        # Add value labels on bars
        for bar, angle in zip(bars, angles):
            height = bar.get_height()
            self.ax1.text(bar.get_x() + bar.get_width()/2., height + 2,
                         f'{angle:.0f}°', ha='center', va='bottom', fontsize=8)
        
        # Plot 2: Leg states
        self.ax2.set_title('Leg States')
        leg_names = list(self.gait_config['legs'].keys())
        states = []
        colors = []
        
        T = self.gait_config['T']
        for leg_name in leg_names:
            params = self.gait_config['legs'][leg_name]
            phase = params['phase']
            norm = ((self.current_time + phase) % T) / T
            
            if 0.15 <= norm < 0.35:
                states.append(1)  # Swing
                colors.append('red')
            else:
                states.append(0)  # Stance
                colors.append('blue')
        
        self.ax2.bar(range(len(leg_names)), states, color=colors, alpha=0.7)
        self.ax2.set_xticks(range(len(leg_names)))
        self.ax2.set_xticklabels([name.replace('_', '\n') for name in leg_names])
        self.ax2.set_ylabel('State (0=Stance, 1=Swing)')
        self.ax2.set_ylim(-0.1, 1.1)
        
        # Plot 3: Gait phase diagram
        self.ax3.set_title('Gait Phase Wheel')
        import math
        
        for i, (leg_name, params) in enumerate(self.gait_config['legs'].items()):
            phase = params['phase']
            norm = ((self.current_time + phase) % T) / T
            angle = norm * 2 * math.pi
            
            x = math.cos(angle)
            y = math.sin(angle)
            
            color = self.leg_colors[leg_name]
            self.ax3.scatter(x, y, c=color, s=100, label=leg_name)
            self.ax3.arrow(0, 0, x*0.8, y*0.8, head_width=0.05, 
                          head_length=0.05, fc=color, ec=color, alpha=0.7)
        
        # Draw phase sectors
        swing_start = 0.15 * 2 * math.pi
        swing_end = 0.35 * 2 * math.pi
        
        theta = [swing_start + i * (swing_end - swing_start) / 50 for i in range(51)]
        x_swing = [math.cos(t) for t in theta]
        y_swing = [math.sin(t) for t in theta]
        
        self.ax3.fill(x_swing + [0], y_swing + [0], alpha=0.2, color='red', label='Swing Phase')
        
        circle = plt.Circle((0, 0), 1, fill=False, linestyle='--', alpha=0.5)
        self.ax3.add_patch(circle)
        self.ax3.set_xlim(-1.2, 1.2)
        self.ax3.set_ylim(-1.2, 1.2)
        self.ax3.set_aspect('equal')
        self.ax3.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        
        # Plot 4: Time series
        self.ax4.set_title(f'Time: {self.current_time/1000:.1f}s')
        time_text = f"Gait Cycle: {(self.current_time % T)/T:.2f}\n"
        time_text += f"Update Rate: {1000/self.gait_config['update_interval_ms']:.0f} Hz\n"
        time_text += f"Cycle Period: {T/1000:.1f}s"
        
        self.ax4.text(0.1, 0.5, time_text, fontsize=14, verticalalignment='center',
                     transform=self.ax4.transAxes)
        self.ax4.set_xlim(0, 1)
        self.ax4.set_ylim(0, 1)
        self.ax4.axis('off')
        
        plt.tight_layout()
    
    def run_visual_simulation(self, duration=20):
        """Run visual simulation with matplotlib"""
        if not MATPLOTLIB_AVAILABLE:
            print("❌ Matplotlib not available. Running text simulation instead...")
            self.run_text_simulation(duration)
            return
            
        print("🎬 Starting Real-Time Visual Simulation")
        print(f"Duration: {duration} seconds")
        print("Close the plot window to stop...")
        
        # Calculate number of frames
        fps = 1000 / self.gait_config['update_interval_ms']  # Frames per second
        total_frames = int(duration * fps)
        
        ani = animation.FuncAnimation(
            self.fig, self.animate_plots, frames=range(total_frames),
            interval=self.gait_config['update_interval_ms'], blit=False, repeat=False
        )
        
        plt.show()
        print("✅ Visual simulation completed!")

def main():
    """Main function to run real-time simulation"""
    print("🤖 Real-Time Robot Movement Simulator")
    print("=" * 40)
    
    simulator = RealTimeSimulator()
    
    print("\nChoose simulation mode:")
    print("1. Text-based simulation (works without matplotlib)")
    print("2. Visual simulation (requires matplotlib)")
    
    try:
        choice = input("Enter choice (1 or 2): ").strip()
        duration = float(input("Duration in seconds (default 10): ") or "10")
        
        if choice == "1":
            simulator.run_text_simulation(duration)
        elif choice == "2":
            simulator.run_visual_simulation(duration)
        else:
            print("Invalid choice. Running text simulation...")
            simulator.run_text_simulation(duration)
            
    except KeyboardInterrupt:
        print("\n⏹️  Simulation cancelled by user")
    except Exception as e:
        print(f"❌ Error: {e}")
        print("Running text simulation as fallback...")
        simulator.run_text_simulation(10)

if __name__ == "__main__":
    main()
