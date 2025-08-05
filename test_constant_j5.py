#!/usr/bin/env python3
"""
Test constant J5 position functionality
"""

import sys
import os

# Add the robot_control package to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_control.robot_controller.executor import TaskExecutor

def test_constant_j5():
    """Test that J5 maintains constant position during movements."""
    
    print("🧪 Testing Constant J5 Position")
    print("📍 Connecting to robot at 192.168.1.241")
    print("📹 J5 should stay at 90° (camera pointing up) for all movements")
    
    try:
        # Test plan with multiple movements
        test_plan = {
            "goal": "test constant J5 position",
            "steps": [
                {
                    "action": "MOVE_TO_POSE",
                    "pose": {"xyz_mm": [400, -100, 200], "rpy_deg": [0, 0, 0]},
                    "pose_name": "test_position_1"
                },
                {
                    "action": "MOVE_TO_POSE", 
                    "pose": {"xyz_mm": [400, 100, 200], "rpy_deg": [0, 0, 0]},
                    "pose_name": "test_position_2"
                },
                {
                    "action": "MOVE_TO_POSE",
                    "pose": {"xyz_mm": [300, 0, 300], "rpy_deg": [0, 90, 0]},
                    "pose_name": "home"
                }
            ]
        }
        
        print(f"\n📋 Executing test plan...")
        executor = TaskExecutor("192.168.1.241", world_yaml="config/robot/world_model.yaml", sim=False, dry_run=False)
        executor.execute(test_plan)
        executor.shutdown()
        
        print(f"\n✅ Constant J5 test completed!")
        print("📹 J5 should have stayed at 90° for movements 1 and 2")
        print("🏠 J5 should have changed for the home movement")
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_constant_j5() 