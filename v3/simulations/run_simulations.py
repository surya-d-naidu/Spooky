"""
Simulation Suite - Robot Movement Analysis and Visualization
Run comprehensive analysis of gait patterns, kinematics, and servo behavior
"""

import sys
import os
import subprocess

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def check_dependencies():
    """Check if required dependencies are installed"""
    required = ['matplotlib', 'numpy']
    missing = []
    
    for package in required:
        try:
            __import__(package)
        except ImportError:
            missing.append(package)
    
    return missing

def install_dependencies():
    """Install missing dependencies"""
    print("📦 Installing simulation dependencies...")
    try:
        subprocess.check_call([sys.executable, '-m', 'pip', 'install', '-r', 'requirements.txt'])
        print("✅ Dependencies installed successfully!")
        return True
    except subprocess.CalledProcessError:
        print("❌ Failed to install dependencies")
        return False

def run_gait_simulation():
    """Run gait pattern simulation"""
    print("\n🦵 Running Gait Pattern Simulation...")
    try:
        from gait_simulator import main as gait_main
        gait_main()
    except Exception as e:
        print(f"❌ Error running gait simulation: {e}")

def run_ik_analysis():
    """Run inverse kinematics analysis"""
    print("\n🔧 Running Inverse Kinematics Analysis...")
    try:
        from ik_visualizer import main as ik_main
        ik_main()
    except Exception as e:
        print(f"❌ Error running IK analysis: {e}")

def run_servo_analysis():
    """Run servo analysis"""
    print("\n⚙️ Running Servo Analysis...")
    try:
        from servo_analyzer import main as servo_main
        servo_main()
    except Exception as e:
        print(f"❌ Error running servo analysis: {e}")

def run_realtime_simulation():
    """Run real-time movement simulation"""
    print("\n⏱️ Running Real-time Movement Simulation...")
    try:
        from realtime_simulator import main as realtime_main
        realtime_main()
    except Exception as e:
        print(f"❌ Error running real-time simulation: {e}")

def show_menu():
    """Display main menu"""
    print("\n🤖 Robot Simulation Suite")
    print("=" * 40)
    print("1. Gait Pattern Analysis")
    print("2. Inverse Kinematics Visualization") 
    print("3. Servo Analysis & Calibration")
    print("4. Real-time Movement Simulation")
    print("5. Run All Simulations")
    print("6. Install Dependencies")
    print("0. Exit")
    print("-" * 40)

def main():
    """Main simulation suite"""
    print("🎬 Robot Movement Simulation Suite")
    print("=" * 50)
    
    # Check dependencies
    missing = check_dependencies()
    if missing:
        print(f"⚠️  Missing dependencies: {', '.join(missing)}")
        print("Some simulations may not work properly.")
        print("Choose option 6 to install dependencies.\n")
    else:
        print("✅ All dependencies available!\n")
    
    while True:
        show_menu()
        
        try:
            choice = input("\nEnter your choice (0-6): ").strip()
            
            if choice == "0":
                print("👋 Goodbye!")
                break
            elif choice == "1":
                run_gait_simulation()
            elif choice == "2":
                run_ik_analysis()
            elif choice == "3":
                run_servo_analysis()
            elif choice == "4":
                run_realtime_simulation()
            elif choice == "5":
                print("\n🚀 Running All Simulations...")
                run_servo_analysis()
                input("\nPress Enter to continue to next simulation...")
                run_ik_analysis()
                input("\nPress Enter to continue to next simulation...")
                run_gait_simulation()
                input("\nPress Enter to continue to next simulation...")
                run_realtime_simulation()
                print("\n🎉 All simulations completed!")
            elif choice == "6":
                if install_dependencies():
                    print("✅ Dependencies installed! You can now run all simulations.")
                else:
                    print("❌ Installation failed. You may need to install manually:")
                    print("pip install matplotlib numpy scipy")
            else:
                print("❌ Invalid choice. Please try again.")
                
        except KeyboardInterrupt:
            print("\n\n👋 Simulation suite interrupted. Goodbye!")
            break
        except Exception as e:
            print(f"❌ Error: {e}")
            print("Please try again.")

if __name__ == "__main__":
    main()
