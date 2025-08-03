#!/usr/bin/env python3
"""
Main robot script for Raspberry Pi
Distributed setup with laptop AI server
"""

import sys
import signal
import time
from robot_config import config, LAPTOP_IP

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print("\nShutting down robot...")
    sys.exit(0)

def test_laptop_connection():
    """Test connection to laptop AI server"""
    import requests
    
    print(f"Testing connection to laptop at {LAPTOP_IP}...")
    try:
        response = requests.get(f"http://{LAPTOP_IP}:11434/api/version", timeout=5)
        if response.status_code == 200:
            print("✅ Connected to Ollama AI server on laptop!")
            return True
        else:
            print(f"❌ Ollama server responded with status {response.status_code}")
            return False
    except requests.exceptions.RequestException as e:
        print(f"❌ Cannot connect to laptop: {e}")
        print(f"\nTroubleshooting:")
        print(f"1. Make sure laptop is on same WiFi network")
        print(f"2. Check laptop IP is correct: {LAPTOP_IP}")
        print(f"3. Start Ollama on laptop: OLLAMA_HOST=0.0.0.0:11434 ollama serve")
        print(f"4. Check firewall settings on laptop")
        return False

def main():
    """Main robot execution"""
    print("🤖 Starting Raspberry Pi Robot...")
    print(f"📡 Laptop AI Server: {LAPTOP_IP}:11434")
    
    # Test connection first
    if not test_laptop_connection():
        print("Cannot start robot without AI connection.")
        return
    
    # Import robot system
    try:
        from autonomous_robot import create_dog_robot
        print("✅ Robot modules loaded")
    except ImportError as e:
        print(f"❌ Failed to import robot modules: {e}")
        print("Make sure you're running this from the v3 directory")
        return
    
    # Setup signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # Create robot with Pi configuration
        print("🔧 Initializing robot systems...")
        robot = create_dog_robot(config)
        
        # Start robot
        print("🚀 Starting autonomous behavior...")
        print("Robot will now behave autonomously!")
        print("Press Ctrl+C to stop")
        
        robot.start()
        
        # Keep main thread alive
        try:
            while robot.running:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
            
    except Exception as e:
        print(f"❌ Robot error: {e}")
    finally:
        if 'robot' in locals():
            robot.stop()
        print("🛑 Robot stopped")

if __name__ == "__main__":
    main()
