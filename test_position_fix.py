#!/usr/bin/env python3
"""
Simple test script to verify position fix and basic movement
"""

import sys
import os

# Add the robot_control package to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_control.robot_controller import XArmRunner

def test_position_fix(robot_ip: str = "192.168.1.241"):
    """Test the position fix and basic movement."""
    
    print("🤖 Testing Position Fix and Basic Movement")
    print(f"📍 Connecting to robot at {robot_ip}")
    
    try:
        # Initialize robot connection
        runner = XArmRunner(robot_ip, sim=False)
        
        # Test 1: Get current position
        print("\n📋 Test 1: Get current position")
        current_pos = runner.get_current_position()
        print(f"Current position: {current_pos}")
        
        if current_pos is not None:
            print(f"Position type: {type(current_pos)}")
            print(f"Position length: {len(current_pos)}")
            print(f"Position values: X={current_pos[0]:.2f}, Y={current_pos[1]:.2f}, Z={current_pos[2]:.2f}")
        
        # Test 2: Simple movement (if position is valid)
        if current_pos is not None:
            print("\n📋 Test 2: Simple movement test")
            target_pos = [current_pos[0], current_pos[1] + 50, current_pos[2]]  # Move 50mm in Y
            print(f"Moving to: {target_pos}")
            
            try:
                runner.move_pose(target_pos, [0, 0, 0])  # Keep same orientation
                print("✅ Movement successful!")
            except Exception as e:
                print(f"❌ Movement failed: {e}")
        
        print("\n✅ Position fix test completed!")
        
    except Exception as e:
        print(f"❌ Error during position test: {e}")
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
    test_position_fix(robot_ip) 