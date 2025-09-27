import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from ai_vision import CameraStream, OllamaAI
import time
import cv2

def test_camera():
    print("Testing camera stream...")
    try:
        camera = CameraStream(0)
        camera.start()
        time.sleep(2)
        
        frame = camera.get_frame()
        if frame is not None:
            print(f"✓ Camera working - Frame size: {frame.shape}")
            
            cv2.imshow("Camera Test", frame)
            print("Press any key to close camera test window...")
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        else:
            print("✗ Camera not producing frames")
            
        camera.stop()
        
    except Exception as e:
        print(f"✗ Camera test failed: {str(e)}")

def test_ollama():
    print("\nTesting Ollama connection...")
    try:
        ai = OllamaAI()
        
        test_image = "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mP8/5+hHgAHggJ/PchI7wAAAABJRU5ErkJggg=="
        
        response = ai.analyze_image(test_image)
        if response:
            print("✓ Ollama connection successful")
            print(f"  Response: {response}")
        else:
            print("✗ Ollama not responding properly")
            
    except Exception as e:
        print(f"✗ Ollama test failed: {str(e)}")

def test_integration():
    print("\nTesting full integration...")
    try:
        camera = CameraStream(0)
        ai = OllamaAI()
        
        camera.start()
        time.sleep(2)
        
        frame_b64 = camera.get_base64_frame()
        if frame_b64:
            print("✓ Got base64 frame from camera")
            
            response = ai.analyze_image(frame_b64)
            if response:
                print("✓ AI analysis successful")
                print(f"  Emotion: {response.get('emotion')}")
                print(f"  Action: {response.get('action')}")
                print(f"  Confidence: {response.get('confidence')}")
            else:
                print("✗ AI analysis failed")
        else:
            print("✗ Failed to get frame from camera")
            
        camera.stop()
        
    except Exception as e:
        print(f"✗ Integration test failed: {str(e)}")

if __name__ == "__main__":
    print("AI Vision Dog System Tests")
    print("=" * 30)
    
    test_camera()
    test_ollama() 
    test_integration()
    
    print("\n" + "=" * 30)
    print("Tests completed!")
