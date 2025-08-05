#!/usr/bin/env python3
"""
Test script to verify import fixes
Run this to check if all modules can be imported correctly
"""

import sys
import os

# Add the v3 directory to Python path
v3_path = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, v3_path)

def test_imports():
    """Test all module imports"""
    print("🧪 Testing Module Imports...")
    print("=" * 40)
    
    tests = [
        ("robot_config", "Robot configuration"),
        ("core.interfaces", "Core interfaces"),
        ("ai.ollama_ai", "AI module"),
        ("sensors.camera", "Camera sensor"),
        ("actuators.quadruped_movement", "Movement actuator"),
        ("personality.dog_personality", "Personality system"),
    ]
    
    success_count = 0
    
    for module_name, description in tests:
        try:
            __import__(module_name)
            print(f"✅ {description:<25} - OK")
            success_count += 1
        except ImportError as e:
            print(f"❌ {description:<25} - FAILED: {e}")
        except Exception as e:
            print(f"⚠️  {description:<25} - ERROR: {e}")
    
    print("\n" + "=" * 40)
    print(f"📊 Results: {success_count}/{len(tests)} modules loaded successfully")
    
    if success_count == len(tests):
        print("🎉 All imports working correctly!")
        return True
    else:
        print("⚠️  Some imports failed. Check error messages above.")
        return False

def test_movement_imports():
    """Test movement-specific imports"""
    print("\n🏃 Testing Movement System...")
    print("=" * 30)
    
    try:
        # Test trot.py import
        if os.path.exists("Machine/trot.py"):
            sys.path.insert(0, os.path.join(v3_path, "Machine"))
            import trot
            print("✅ trot.py - OK")
        else:
            print("⚠️  trot.py not found in Machine/ folder")
            
        # Test servo.py import
        if os.path.exists("Machine/servo.py"):
            import servo
            print("✅ servo.py - OK")
        else:
            print("⚠️  servo.py not found in Machine/ folder")
            
    except Exception as e:
        print(f"❌ Movement import error: {e}")

def test_autonomous_robot():
    """Test main autonomous robot import"""
    print("\n🤖 Testing Main Robot System...")
    print("=" * 35)
    
    try:
        import autonomous_robot
        print("✅ autonomous_robot.py - OK")
        
        # Test factory function
        from autonomous_robot import create_dog_robot
        print("✅ create_dog_robot function - OK")
        
    except Exception as e:
        print(f"❌ Autonomous robot error: {e}")

def main():
    """Main test function"""
    print("🔧 Robot System Import Test")
    print("=" * 50)
    
    # Run all tests
    imports_ok = test_imports()
    test_movement_imports() 
    test_autonomous_robot()
    
    print("\n" + "=" * 50)
    if imports_ok:
        print("✅ System ready! You can now run:")
        print("   python robot_main.py")
        print("   python autonomous_robot.py")
        print("   python examples.py")
    else:
        print("❌ Some issues found. Check the error messages above.")
        print("   Make sure all dependencies are installed:")
        print("   pip install -r requirements_pi.txt")

if __name__ == "__main__":
    main()
