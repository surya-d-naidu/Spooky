#!/usr/bin/env python3
"""
Continuous Random Behavior Runner
Runs the robot in basic random behavior mode continuously until stopped.
Perfect for systemd integration and automatic startup.
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import main
from random_controller import RandomBehaviorController
import time
import signal
import logging

# Set up logging
logging.basicConfig(
    level=logging.INFO, 
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('/tmp/spooky_random_behavior.log')
    ]
)
logger = logging.getLogger(__name__)

current_controller = None
shutdown_requested = False

def signal_handler(signum, frame):
    global current_controller, shutdown_requested
    logger.info("Shutdown signal received, stopping random behavior...")
    shutdown_requested = True
    if current_controller:
        current_controller.stop()
    sys.exit(0)

def main():
    global current_controller, shutdown_requested
    
    # Register signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    logger.info("🤖 Starting Continuous Random Behavior System...")
    
    # Add a startup delay to allow system to stabilize
    logger.info("Waiting 10 seconds for system initialization...")
    time.sleep(10)
    
    try:
        while not shutdown_requested:
            try:
                logger.info("Starting Basic Random Behavior Controller...")
                current_controller = RandomBehaviorController()
                
                # Set longer durations for better behavior
                current_controller.set_action_duration_range(6, 15)
                current_controller.set_pause_duration_range(2, 8)
                
                # Start the random behavior
                current_controller.start()
                logger.info("✅ Random behavior started successfully!")
                
                # Monitor the controller and restart if it stops unexpectedly
                while current_controller.is_running and not shutdown_requested:
                    time.sleep(5)  # Check every 5 seconds
                    
                    # Log current status periodically
                    status = current_controller.get_status()
                    if hasattr(current_controller, '_last_log_time'):
                        if time.time() - current_controller._last_log_time > 60:  # Log every minute
                            logger.info(f"Status: {status['current_action']} - Mode: {status['mode']}")
                            current_controller._last_log_time = time.time()
                    else:
                        current_controller._last_log_time = time.time()
                
                if not shutdown_requested:
                    logger.warning("Random behavior stopped unexpectedly, restarting in 5 seconds...")
                    if current_controller:
                        current_controller.stop()
                    time.sleep(5)
                    
            except Exception as e:
                logger.error(f"Error in main loop: {str(e)}")
                if current_controller:
                    current_controller.stop()
                logger.info("Restarting after error in 10 seconds...")
                time.sleep(10)
                
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received")
    finally:
        if current_controller:
            logger.info("Stopping random behavior controller...")
            current_controller.stop()
        logger.info("🛑 Continuous random behavior system stopped")

if __name__ == "__main__":
    main()
