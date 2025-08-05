#!/usr/bin/env python3
"""
Simple command to detect objects and move toward them
"""

import sys
import os
import time

# Add the robot_control package to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_control.robot_controller import XArmRunner
from robot_control.robot_controller.executor import TaskExecutor

def detect_and_move(robot_ip: str = "192.168.1.241", object_type: str = "bottle"):
    """Detect an object and move toward it."""
    
    print(f"🤖 Detect and Move: {object_type}")
    print(f"📍 Connecting to robot at {robot_ip}")
    print(f"🎯 Place a {object_type} in front of the robot")
    
    try:
        # Step 1: Scan for objects
        print(f"\n📋 Step 1: Scanning for {object_type}...")
        scan_plan = {
            "goal": f"scan for {object_type}",
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
        
        # Step 2: Approach the detected object
        print(f"\n📋 Step 2: Approaching {object_type}...")
        approach_plan = {
            "goal": f"approach {object_type}",
            "steps": [
                {
                    "action": "APPROACH_OBJECT",
                    "label": object_type,
                    "hover_mm": 150,
                    "timeout_sec": 10
                }
            ]
        }
        
        executor2 = TaskExecutor(robot_ip, sim=False, dry_run=False)
        executor2.execute(approach_plan)
        executor2.shutdown()
        
        print(f"\n✅ Successfully detected and moved toward {object_type}!")
        
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python detect_and_move.py [object_type]")
        print("")
        print("Examples:")
        print("  python detect_and_move.py bottle")
        print("  python detect_and_move.py cup")
        print("  python detect_and_move.py person")
        sys.exit(1)
    
    object_type = sys.argv[1] if len(sys.argv) > 1 else "bottle"
    robot_ip = "192.168.1.241"
    
    detect_and_move(robot_ip, object_type) 