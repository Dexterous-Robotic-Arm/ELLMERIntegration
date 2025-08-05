#!/usr/bin/env python3
"""
Test can detection and movement toward the can
"""

import sys
import os
import time

# Add the robot_control package to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_control.robot_controller import XArmRunner
from robot_control.robot_controller.executor import TaskExecutor

def test_can_detection_and_movement(robot_ip: str = "192.168.1.241"):
    """Test detecting a can and moving toward it."""
    
    print("🤖 Testing Can Detection and Movement")
    print(f"📍 Connecting to robot at {robot_ip}")
    print("🥫 Place a can directly in front of the robot")
    
    try:
        # Test 1: Scan for objects (including the can)
        print("\n📋 Step 1: Scanning for objects...")
        scan_plan = {
            "goal": "scan for can",
            "steps": [
                {
                    "action": "SCAN_FOR_OBJECTS",
                    "pattern": "horizontal",
                    "sweep_mm": 400,
                    "steps": 7,
                    "pause_sec": 2.0
                }
            ]
        }
        
        executor = TaskExecutor(robot_ip, sim=False, dry_run=False)
        executor.execute(scan_plan)
        executor.shutdown()
        
        # Test 2: Approach the detected can
        print("\n📋 Step 2: Approaching detected can...")
        approach_plan = {
            "goal": "approach can",
            "steps": [
                {
                    "action": "APPROACH_OBJECT",
                    "label": "can",
                    "hover_mm": 150,
                    "timeout_sec": 10
                }
            ]
        }
        
        executor2 = TaskExecutor(robot_ip, sim=False, dry_run=False)
        executor2.execute(approach_plan)
        executor2.shutdown()
        
        print("\n✅ Can detection and movement test completed!")
        print("🥫 Robot should have moved toward the can")
        
    except Exception as e:
        print(f"❌ Error during can detection test: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        print("\n🔌 Test completed")

if __name__ == "__main__":
    # Get robot IP from command line or use default
    robot_ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.241"
    test_can_detection_and_movement(robot_ip) 