#!/usr/bin/env python3
"""
Quick scan and detect interface
"""

import sys
import os
import time

# Add the robot_control package to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_control.robot_controller import XArmRunner
from robot_control.robot_controller.executor import TaskExecutor

def quick_scan(robot_ip: str = "192.168.1.241", sweep_mm: int = 300, steps: int = 5):
    """Quick scan with specified parameters."""
    
    print(f"🤖 Quick Scan: {sweep_mm}mm sweep, {steps} steps")
    print(f"📍 Connecting to robot at {robot_ip}")
    
    try:
        # Initialize robot connection
        runner = XArmRunner(robot_ip, sim=False)
        
        # Create scan plan
        scan_plan = {
            "goal": f"quick scan {sweep_mm}mm",
            "steps": [
                {
                    "action": "SCAN_FOR_OBJECTS",
                    "pattern": "horizontal",
                    "sweep_mm": sweep_mm,
                    "steps": steps,
                    "pause_sec": 1.5
                }
            ]
        }
        
        # Execute scan
        executor = TaskExecutor(robot_ip, sim=False, dry_run=False)
        executor.execute(scan_plan)
        executor.shutdown()
        
        print("✅ Quick scan completed!")
        
    except Exception as e:
        print(f"❌ Error during quick scan: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Clean up
        if 'runner' in locals():
            runner.disconnect()
        print("\n🔌 Disconnected from robot")

def approach_object(robot_ip: str = "192.168.1.241", object_type: str = "can"):
    """Approach a detected object."""
    
    print(f"🤖 Approach Object: {object_type}")
    print(f"📍 Connecting to robot at {robot_ip}")
    
    try:
        # Initialize robot connection
        runner = XArmRunner(robot_ip, sim=False)
        
        # Create approach plan
        approach_plan = {
            "goal": f"approach {object_type}",
            "steps": [
                {
                    "action": "APPROACH_OBJECT",
                    "label": object_type,
                    "hover_mm": 100,
                    "timeout_sec": 5
                }
            ]
        }
        
        # Execute approach
        executor = TaskExecutor(robot_ip, sim=False, dry_run=False)
        executor.execute(approach_plan)
        executor.shutdown()
        
        print("✅ Approach completed!")
        
    except Exception as e:
        print(f"❌ Error during approach: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Clean up
        if 'runner' in locals():
            runner.disconnect()
        print("\n🔌 Disconnected from robot")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python quick_scan.py scan [sweep_mm] [steps]")
        print("  python quick_scan.py approach [object_type]")
        print("")
        print("Examples:")
        print("  python quick_scan.py scan 300 5")
        print("  python quick_scan.py scan 500 7")
        print("  python quick_scan.py approach can")
        print("  python quick_scan.py approach bottle")
        sys.exit(1)
    
    command = sys.argv[1]
    robot_ip = "192.168.1.241"
    
    if command == "scan":
        sweep_mm = int(sys.argv[2]) if len(sys.argv) > 2 else 300
        steps = int(sys.argv[3]) if len(sys.argv) > 3 else 5
        quick_scan(robot_ip, sweep_mm, steps)
    
    elif command == "approach":
        object_type = sys.argv[2] if len(sys.argv) > 2 else "can"
        approach_object(robot_ip, object_type)
    
    else:
        print(f"Unknown command: {command}")
        sys.exit(1) 