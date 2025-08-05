#!/usr/bin/env python3
"""
Simple Robot Runner - ensures proper path setup
Run this script to start the robot with proper imports
"""

import sys
import os

# Ensure we're in the v3 directory
script_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_dir)

# Add current directory to Python path
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

# Add Machine directory to path for trot.py and servo.py
machine_dir = os.path.join(script_dir, "Machine")
if os.path.exists(machine_dir) and machine_dir not in sys.path:
    sys.path.insert(0, machine_dir)

def main():
    """Main entry point"""
    print("🤖 Simple Robot Runner")
    print("=" * 30)
    print(f"Working directory: {os.getcwd()}")
    print(f"Python path includes: {script_dir}")
    
    # Import and run robot_main
    try:
        import robot_main
        robot_main.main()
    except KeyboardInterrupt:
        print("\n👋 Robot stopped by user")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
