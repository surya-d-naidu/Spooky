"""
Simulation package initialization
"""

__version__ = "1.0.0"
__author__ = "AIR Research Team"

# Import main simulation classes for easy access
try:
    from .gait_simulator import GaitSimulator
    from .ik_visualizer import IKVisualizer  
    from .servo_analyzer import ServoAnalyzer
    from .realtime_simulator import RealTimeSimulator
    SIMULATIONS_AVAILABLE = True
except ImportError:
    SIMULATIONS_AVAILABLE = False

def run_all_simulations():
    """Convenience function to run all simulations"""
    if not SIMULATIONS_AVAILABLE:
        print("❌ Simulation dependencies not available")
        return
        
    from .run_simulations import main
    main()

def check_simulation_dependencies():
    """Check if simulation dependencies are available"""
    missing = []
    
    try:
        import matplotlib
        import numpy
    except ImportError as e:
        missing.append(str(e))
    
    return missing

__all__ = [
    'GaitSimulator', 
    'IKVisualizer', 
    'ServoAnalyzer', 
    'RealTimeSimulator',
    'run_all_simulations',
    'check_simulation_dependencies'
]
