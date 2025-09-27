import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import main
from random_controller import RandomBehaviorController
from behavior_sequences import AdvancedRandomBehavior
from special_modes import ChaosMode, ZenMode, PartyMode
import time
import signal
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

current_controller = None

def signal_handler(signum, frame):
    global current_controller
    logger.info("Shutting down random behavior...")
    if current_controller:
        current_controller.stop()
    sys.exit(0)

def show_menu():
    print("\n" + "="*50)
    print("🤖 RANDOM BEHAVIOR ROBOT CONTROLLER 🤖")
    print("="*50)
    print("1. Basic Random Movements")
    print("2. Advanced Sequences") 
    print("3. Chaos Mode (Crazy Random)")
    print("4. Zen Mode (Peaceful)")
    print("5. Party Mode (Dance Party)")
    print("6. Stop Current Mode")
    print("0. Exit")
    print("="*50)
    return input("Choose mode (0-6): ").strip()

def main():
    global current_controller
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    logger.info("Random Behavior System Starting...")
    
    try:
        while True:
            choice = show_menu()
            
            if choice == "0":
                if current_controller:
                    current_controller.stop()
                logger.info("Exiting...")
                break
                
            elif choice == "6":
                if current_controller:
                    current_controller.stop()
                    current_controller = None
                    logger.info("Stopped current behavior mode")
                else:
                    logger.info("No active behavior mode")
                continue
                
            if current_controller:
                logger.info("Stopping current mode first...")
                current_controller.stop()
                time.sleep(1)
                
            if choice == "1":
                logger.info("Starting Basic Random Movements...")
                current_controller = RandomBehaviorController()
                current_controller.start()
                
            elif choice == "2":
                logger.info("Starting Advanced Sequences...")
                current_controller = AdvancedRandomBehavior()
                current_controller.start()
                
            elif choice == "3":
                logger.info("Starting Chaos Mode...")
                current_controller = ChaosMode()
                current_controller.start()
                
            elif choice == "4":
                logger.info("Starting Zen Mode...")
                current_controller = ZenMode()
                current_controller.start()
                
            elif choice == "5":
                logger.info("Starting Party Mode...")
                current_controller = PartyMode()
                current_controller.start()
                
            else:
                print("Invalid choice. Please try again.")
                continue
                
            print(f"\n✅ Mode activated! Press Ctrl+C to stop or choose option 6 to change modes.")
            time.sleep(2)
            
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received")
    except Exception as e:
        logger.error(f"Error in main loop: {str(e)}")
    finally:
        if current_controller:
            current_controller.stop()
        logger.info("Random behavior system stopped")

if __name__ == "__main__":
    main()
