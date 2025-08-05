#!/usr/bin/env python3
"""
Test camera orientation - verify the wrist faces outward at 45-degree angle
"""

import sys
import os
import time

# Add the robot_control package to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_control.robot_controller import XArmRunner

def test_camera_orientation(robot_ip: str = "192.168.1.241"):
    """Test the new camera orientation."""
    
    print("🤖 Testing Camera Orientation")
    print(f"📍 Connecting to robot at {robot_ip}")
    print("📷 Camera should now face outward at 45-degree angle")
    
    try:
        # Initialize robot connection
        runner = XArmRunner(robot_ip, sim=False)
        
        # Get current position
        current_pos = runner.get_current_position()
        print(f"Current position: {current_pos}")
        
        if current_pos is not None:
            # Test 1: Move to scan center with new orientation
            scan_center = [400, 0, current_pos[2]]
            print(f"\n📋 Test 1: Moving to scan center with camera facing outward")
            print(f"Target: {scan_center}")
            print(f"Orientation: [0, -45, 0] (camera faces outward at 45°)")
            
            runner.move_pose(scan_center, [0, -45, 0])
            time.sleep(2)
            
            # Test 2: Small movement to verify orientation
            test_pos = [400, 50, current_pos[2]]
            print(f"\n📋 Test 2: Small movement to verify orientation")
            print(f"Target: {test_pos}")
            
            runner.move_pose(test_pos, [0, -45, 0])
            time.sleep(2)
            
            # Test 3: Return to original position
            print(f"\n📋 Test 3: Returning to original position")
            runner.move_pose(current_pos, [0, -45, 0])
            time.sleep(2)
            
            print("\n✅ Camera orientation test completed!")
            print("📷 Camera should now be facing outward at 45-degree angle")
            print("🔍 This allows the camera to see objects in front of the robot")
        
    except Exception as e:
        print(f"❌ Error during camera orientation test: {e}")
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
    test_camera_orientation(robot_ip) 