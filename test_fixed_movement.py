#!/usr/bin/env python3
"""
Test fixed robot movement with proper state management
"""

import sys
import os
import time

# Add the robot_control package to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_control.robot_controller import XArmRunner

def test_fixed_movement(robot_ip: str = "192.168.1.241"):
    """Test fixed robot movement with proper state management."""
    
    print("🤖 Testing Fixed Robot Movement")
    print(f"📍 Connecting to robot at {robot_ip}")
    
    try:
        # Initialize robot connection
        runner = XArmRunner(robot_ip, sim=False)
        
        # Test 1: Get initial position
        print("\n📋 Test 1: Get initial position")
        initial_pos = runner.get_current_position()
        print(f"Initial position: {initial_pos}")
        
        # Test 2: Simple movement using our fixed move_pose
        print("\n📋 Test 2: Fixed movement test")
        if initial_pos is not None:
            # Move 50mm in Y direction
            target_pos = [initial_pos[0], initial_pos[1] + 50, initial_pos[2]]
            print(f"Moving from Y={initial_pos[1]:.2f} to Y={target_pos[1]:.2f}")
            
            try:
                runner.move_pose(target_pos, [180, 0, 0])
                time.sleep(3)  # Wait for movement
                
                # Check new position
                new_pos = runner.get_current_position()
                print(f"New position: {new_pos}")
                if new_pos is not None:
                    y_moved = new_pos[1] - initial_pos[1]
                    print(f"Y movement: {y_moved:.2f}mm")
                    
                    if abs(y_moved) > 10:  # Should have moved more than 10mm
                        print("✅ Movement successful!")
                    else:
                        print("❌ Movement failed - robot barely moved")
                        
            except Exception as e:
                print(f"❌ Movement failed: {e}")
        
        # Test 3: Scan movement test
        print("\n📋 Test 3: Scan movement test")
        if initial_pos is not None:
            # Move to scan center (400mm X, 0mm Y)
            scan_center = [400, 0, initial_pos[2]]
            print(f"Moving to scan center: {scan_center}")
            
            try:
                runner.move_pose(scan_center, [180, 0, 0])
                time.sleep(3)  # Wait for movement
                
                # Check new position
                scan_pos = runner.get_current_position()
                print(f"Scan center position: {scan_pos}")
                if scan_pos is not None:
                    x_moved = scan_pos[0] - initial_pos[0]
                    print(f"X movement to scan center: {x_moved:.2f}mm")
                    
                    if abs(x_moved) > 100:  # Should have moved more than 100mm
                        print("✅ Scan center movement successful!")
                    else:
                        print("❌ Scan center movement failed - robot barely moved")
                        
            except Exception as e:
                print(f"❌ Scan center movement failed: {e}")
        
        print("\n✅ Fixed movement test completed!")
        
    except Exception as e:
        print(f"❌ Error during fixed movement test: {e}")
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
    test_fixed_movement(robot_ip) 