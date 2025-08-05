#!/usr/bin/env python3
"""
Simple test for scanning, detecting objects, and moving towards them
"""

import sys
import os
import time

# Add the robot_control package to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_control.robot_controller import XArmRunner
from robot_control.robot_controller.executor import TaskExecutor

def test_scan_and_detect(robot_ip: str = "192.168.1.241"):
    """Test scanning, detecting objects, and moving towards them."""
    
    print("🤖 Testing Scan and Detect Functionality")
    print(f"📍 Connecting to robot at {robot_ip}")
    
    try:
        # Initialize robot connection
        runner = XArmRunner(robot_ip, sim=False)
        
        # Test 1: Basic scan
        print("\n📋 Test 1: Basic scan (300mm sweep)")
        scan_plan = {
            "goal": "scan for objects",
            "steps": [
                {
                    "action": "SCAN_FOR_OBJECTS",
                    "pattern": "horizontal",
                    "sweep_mm": 300,
                    "steps": 5,
                    "pause_sec": 2.0
                }
            ]
        }
        
        executor = TaskExecutor(robot_ip, sim=False, dry_run=False)
        executor.execute(scan_plan)
        executor.shutdown()
        
        # Test 2: Large scan
        print("\n📋 Test 2: Large scan (500mm sweep)")
        large_scan_plan = {
            "goal": "large scan for objects",
            "steps": [
                {
                    "action": "SCAN_FOR_OBJECTS",
                    "pattern": "horizontal",
                    "sweep_mm": 500,
                    "steps": 7,
                    "pause_sec": 1.5
                }
            ]
        }
        
        executor2 = TaskExecutor(robot_ip, sim=False, dry_run=False)
        executor2.execute(large_scan_plan)
        executor2.shutdown()
        
        # Test 3: Approach object (simulated)
        print("\n📋 Test 3: Approach object (simulated)")
        approach_plan = {
            "goal": "approach detected object",
            "steps": [
                {
                    "action": "APPROACH_OBJECT",
                    "label": "can",
                    "hover_mm": 100,
                    "timeout_sec": 5
                }
            ]
        }
        
        executor3 = TaskExecutor(robot_ip, sim=False, dry_run=False)
        executor3.execute(approach_plan)
        executor3.shutdown()
        
        print("\n✅ All tests completed!")
        
    except Exception as e:
        print(f"❌ Error during test: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Clean up
        if 'runner' in locals():
            runner.disconnect()
        print("\n🔌 Disconnected from robot")

def test_simple_movement(robot_ip: str = "192.168.1.241"):
    """Test simple movement to verify robot works."""
    
    print("🤖 Testing Simple Movement")
    print(f"📍 Connecting to robot at {robot_ip}")
    
    try:
        # Initialize robot connection
        runner = XArmRunner(robot_ip, sim=False)
        
        # Get current position
        current_pos = runner.get_current_position()
        print(f"Current position: {current_pos}")
        
        if current_pos is not None:
            # Move to scan center
            scan_center = [400, 0, current_pos[2]]
            print(f"Moving to scan center: {scan_center}")
            
            runner.move_pose(scan_center, [0, -45, 0])  # Face camera outward at 45-degree angle
            time.sleep(2)
            
            # Check new position
            new_pos = runner.get_current_position()
            print(f"New position: {new_pos}")
            
            if new_pos is not None:
                x_moved = new_pos[0] - current_pos[0]
                print(f"X movement: {x_moved:.2f}mm")
                
                if abs(x_moved) > 50:
                    print("✅ Movement successful!")
                else:
                    print("❌ Movement failed - robot barely moved")
        
        print("\n✅ Simple movement test completed!")
        
    except Exception as e:
        print(f"❌ Error during simple movement test: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Clean up
        if 'runner' in locals():
            runner.disconnect()
        print("\n🔌 Disconnected from robot")

if __name__ == "__main__":
    # Get robot IP from command line or use default
    robot_ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.241"
    
    # Test simple movement first
    test_simple_movement(robot_ip)
    
    # Then test scan and detect
    test_scan_and_detect(robot_ip) 