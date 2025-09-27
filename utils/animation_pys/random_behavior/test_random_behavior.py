import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from random_controller import RandomBehaviorController
from special_modes import PartyMode
import time

class MockAnimationController:
    def start_animation(self, animation_name):
        print(f"🎬 Starting animation: {animation_name}")
        
    def start_reverse_animation(self, animation_name):
        print(f"🎬 Starting reverse animation: {animation_name}")
        
    def stop_current_animation(self):
        print("⏹️  Stopping current animation")
        
    def set_idle_position(self):
        print("🧍 Setting idle position")

def test_random_controller():
    print("Testing Random Behavior Controller...")
    mock_controller = MockAnimationController()
    
    random_behavior = RandomBehaviorController(mock_controller)
    random_behavior.set_action_duration_range(1, 3)
    random_behavior.set_pause_duration_range(0.5, 1)
    
    print("Starting random behavior for 10 seconds...")
    random_behavior.start()
    time.sleep(10)
    random_behavior.stop()
    print("✅ Random controller test completed")

def test_party_mode():
    print("\nTesting Party Mode...")
    mock_controller = MockAnimationController()
    
    party = PartyMode(mock_controller)
    print("Starting party mode for 8 seconds...")
    party.start()
    time.sleep(8)
    party.stop()
    print("✅ Party mode test completed")

if __name__ == "__main__":
    print("🤖 Random Behavior System Tests")
    print("=" * 40)
    
    test_random_controller()
    test_party_mode()
    
    print("\n" + "=" * 40)
    print("All tests completed!")
