#!/usr/bin/env python3
"""
Test J5 constant position with Y alignment to objects
"""

import sys
import os

# Add the robot_control package to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_control.robot_controller.executor import TaskExecutor

def test_j5_alignment():
    """Test that J5 stays constant while Y aligns to detected objects."""
    
    print("🧪 Testing J5 Constant Position with Y Alignment")
    print("📍 Connecting to robot at 192.168.1.241")
    print("📹 J5 should stay at 90° (camera pointing up)")
    print("🎯 Y should align to detected object position")
    
    try:
        # Test plan with object detection and approach
        test_plan = {
            "goal": "test J5 constant with Y alignment",
            "steps": [
                {
                    "action": "SCAN_FOR_OBJECTS",
                    "pattern": "horizontal",
                    "sweep_mm": 200,
                    "steps": 3,
                    "pause_sec": 2.0
                },
                {
                    "action": "APPROACH_OBJECT",
                    "label": "bottle",
                    "hover_mm": 100,
                    "timeout_sec": 10
                }
            ]
        }
        
        print(f"\n📋 Executing alignment test plan...")
        executor = TaskExecutor("192.168.1.241", world_yaml="config/robot/world_model.yaml", sim=False, dry_run=False)
        executor.execute(test_plan)
        executor.shutdown()
        
        print(f"\n✅ J5 alignment test completed!")
        print("📹 J5 should have stayed at 90° throughout")
        print("🎯 Y should have aligned to the detected bottle")
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_j5_alignment() 