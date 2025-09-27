import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from main import animation_controller
from ai_vision import AIVisionController
import time
import signal
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def signal_handler(signum, frame):
    logger.info("Shutting down AI Dog...")
    if 'ai_controller' in globals():
        ai_controller.stop()
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        logger.info("Starting AI Dog System...")
        
        ollama_host = os.getenv('OLLAMA_HOST', 'localhost')
        camera_index = int(os.getenv('CAMERA_INDEX', '0'))
        
        ai_controller = AIVisionController(
            animation_controller=animation_controller,
            ollama_host=ollama_host,
            camera_index=camera_index
        )
        
        logger.info(f"Connecting to Ollama at {ollama_host}")
        logger.info(f"Using camera index {camera_index}")
        
        ai_controller.start()
        
        logger.info("AI Dog is now active! Press Ctrl+C to stop.")
        
        while True:
            status = ai_controller.get_status()
            if status['is_running']:
                personality_state = status['personality_state']
                logger.info(f"Current emotion: {personality_state['current_emotion']} "
                          f"(intensity: {personality_state['emotion_intensity']:.2f})")
            time.sleep(30)
            
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received")
    except Exception as e:
        logger.error(f"Error in main loop: {str(e)}")
    finally:
        if 'ai_controller' in locals():
            ai_controller.stop()
        logger.info("AI Dog system stopped")
