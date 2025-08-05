#!/usr/bin/env python3
"""
Simple test script to verify hardcoded scan movement
"""

import sys
import os

# Add the robot_control package to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_control.robot_controller import XArmRunner
from robot_control.robot_controller.executor import TaskExecutor

def test_hardcoded_scan(robot_ip: str = "192.168.1.241"):
    """Test the hardcoded scan movement."""
    
    print("🤖 Testing Hardcoded Scan Movement")
    print(f"📍 Connecting to robot at {robot_ip}")
    
    try:
        # Initialize robot connection
        runner = XArmRunner(robot_ip, sim=False)
        
        # Test 1: Simple scan with hardcoded movement
        print("\n📋 Test 1: Simple scan with hardcoded movement")
        scan_plan = {
            "goal": "hardcoded scan test",
            "steps": [
                {
                    "action": "SCAN_FOR_OBJECTS",
                    "pattern": "horizontal",
                    "sweep_mm": 200,  # Smaller sweep for testing
                    "steps": 3,
                    "pause_sec": 2.0
                }
            ]
        }
        
        executor = TaskExecutor(robot_ip, sim=False, dry_run=False)
        executor.execute(scan_plan)
        executor.shutdown()
        
        print("\n✅ Hardcoded scan test completed!")
        
    except Exception as e:
        print(f"❌ Error during hardcoded scan test: {e}")
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
    test_hardcoded_scan(robot_ip) 