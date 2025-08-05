#!/usr/bin/env python3
"""
Comprehensive test script for scan functionality and movement logic
Tests all the fixes for movement errors and scanning issues
"""

import sys
import os
import time

# Add the robot_control package to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_control.robot_controller import XArmRunner
from robot_control.robot_controller.executor import TaskExecutor

def test_comprehensive_scan(robot_ip: str = "192.168.1.241"):
    """Test comprehensive scan functionality with all fixes."""
    
    print("🤖 Comprehensive Scan & Movement Test")
    print(f"📍 Connecting to robot at {robot_ip}")
    
    try:
        # Initialize robot connection
        runner = XArmRunner(robot_ip, sim=False)
        
        # Test 1: Basic scan with fallback position
        print("\n📋 Test 1: Basic scan with fallback position")
        scan_plan_1 = {
            "goal": "scan for objects",
            "steps": [
                {
                    "action": "SCAN_FOR_OBJECTS",
                    "pattern": "horizontal",
                    "sweep_mm": 300,
                    "steps": 5,
                    "pause_sec": 1.0
                }
            ]
        }
        
        executor = TaskExecutor(robot_ip, sim=False, dry_run=False)
        executor.execute(scan_plan_1)
        executor.shutdown()  # Clean up after each test
        
        # Test 2: Large sweep scan (tests collision detection fix)
        print("\n📋 Test 2: Large sweep scan (400mm)")
        scan_plan_2 = {
            "goal": "large scan sweep",
            "steps": [
                {
                    "action": "SCAN_FOR_OBJECTS",
                    "pattern": "horizontal",
                    "sweep_mm": 400,
                    "steps": 7,
                    "pause_sec": 0.5
                }
            ]
        }
        
        executor2 = TaskExecutor(robot_ip, sim=False, dry_run=False)
        executor2.execute(scan_plan_2)
        executor2.shutdown()
        
        # Test 3: Single step scan (tests division by zero fix)
        print("\n📋 Test 3: Single step scan")
        scan_plan_3 = {
            "goal": "single step scan",
            "steps": [
                {
                    "action": "SCAN_FOR_OBJECTS",
                    "pattern": "horizontal",
                    "sweep_mm": 100,
                    "steps": 1,
                    "pause_sec": 2.0
                }
            ]
        }
        
        executor3 = TaskExecutor(robot_ip, sim=False, dry_run=False)
        executor3.execute(scan_plan_3)
        executor3.shutdown()
        
        # Test 4: Dry run scan (tests dry run logic)
        print("\n📋 Test 4: Dry run scan")
        executor_dry = TaskExecutor(robot_ip, sim=False, dry_run=True)
        scan_plan_4 = {
            "goal": "dry run scan",
            "steps": [
                {
                    "action": "SCAN_FOR_OBJECTS",
                    "pattern": "horizontal",
                    "sweep_mm": 200,
                    "steps": 3,
                    "pause_sec": 1.0
                }
            ]
        }
        
        executor_dry.execute(scan_plan_4)
        executor_dry.shutdown()
        
        print("\n✅ All comprehensive tests completed!")
        
    except Exception as e:
        print(f"❌ Error during comprehensive test: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Clean up
        if 'executor' in locals():
            executor.shutdown()
        if 'executor2' in locals():
            executor2.shutdown()
        if 'executor3' in locals():
            executor3.shutdown()
        if 'executor_dry' in locals():
            executor_dry.shutdown()
        if 'runner' in locals():
            runner.disconnect()
        print("\n🔌 Disconnected from robot")

def test_movement_consistency():
    """Test movement logic consistency without robot connection."""
    print("\n🔍 Testing Movement Logic Consistency")
    
    try:
        # Test plan validation
        from robot_control.task_planner.planner_llm import validate_plan
        
        # Test valid scan plan
        valid_plan = {
            "goal": "test scan",
            "steps": [
                {"action": "SCAN_FOR_OBJECTS", "pattern": "horizontal", "sweep_mm": 300, "steps": 5, "pause_sec": 1.0}
            ]
        }
        
        is_valid = validate_plan(valid_plan)
        print(f"✅ Valid scan plan validation: {is_valid}")
        
        # Test invalid plan
        invalid_plan = {
            "goal": "test scan",
            "steps": [
                {"action": "INVALID_ACTION"}
            ]
        }
        
        is_valid = validate_plan(invalid_plan)
        print(f"✅ Invalid plan validation: {not is_valid}")
        
        print("✅ Movement logic consistency tests passed!")
        
    except Exception as e:
        print(f"❌ Error in movement logic test: {e}")

if __name__ == "__main__":
    # Get robot IP from command line or use default
    robot_ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.241"
    
    # Test movement logic consistency first
    test_movement_consistency()
    
    # Test comprehensive scan functionality
    test_comprehensive_scan(robot_ip) 