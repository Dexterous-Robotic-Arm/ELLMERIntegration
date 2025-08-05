#!/usr/bin/env python3
"""
Simple movement test to verify robot can actually move
"""

import sys
import os
import time

# Add the robot_control package to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_control.robot_controller import XArmRunner

def test_simple_movement(robot_ip: str = "192.168.1.241"):
    """Test simple robot movement to verify it works."""
    
    print("🤖 Testing Simple Robot Movement")
    print(f"📍 Connecting to robot at {robot_ip}")
    
    try:
        # Initialize robot connection
        runner = XArmRunner(robot_ip, sim=False)
        
        # Test 1: Get initial position
        print("\n📋 Test 1: Get initial position")
        initial_pos = runner.get_current_position()
        print(f"Initial position: {initial_pos}")
        
        # Test 2: Simple Y movement (50mm)
        print("\n📋 Test 2: Simple Y movement (+50mm)")
        if initial_pos is not None:
            target_y = initial_pos[1] + 50
            target_pos = [initial_pos[0], target_y, initial_pos[2]]
            print(f"Moving from Y={initial_pos[1]:.2f} to Y={target_y:.2f}")
            print(f"Target position: {target_pos}")
            
            try:
                runner.move_pose(target_pos, [180, 0, 0])
                time.sleep(2)  # Wait for movement
                
                # Check new position
                new_pos = runner.get_current_position()
                print(f"New position: {new_pos}")
                if new_pos is not None:
                    y_moved = new_pos[1] - initial_pos[1]
                    print(f"Y movement: {y_moved:.2f}mm")
            except Exception as e:
                print(f"❌ Movement failed: {e}")
        
        # Test 3: Simple X movement (100mm)
        print("\n📋 Test 3: Simple X movement (+100mm)")
        current_pos = runner.get_current_position()
        if current_pos is not None:
            target_x = current_pos[0] + 100
            target_pos = [target_x, current_pos[1], current_pos[2]]
            print(f"Moving from X={current_pos[0]:.2f} to X={target_x:.2f}")
            print(f"Target position: {target_pos}")
            
            try:
                runner.move_pose(target_pos, [180, 0, 0])
                time.sleep(2)  # Wait for movement
                
                # Check new position
                new_pos = runner.get_current_position()
                print(f"New position: {new_pos}")
                if new_pos is not None:
                    x_moved = new_pos[0] - current_pos[0]
                    print(f"X movement: {x_moved:.2f}mm")
            except Exception as e:
                print(f"❌ Movement failed: {e}")
        
        # Test 4: Check robot status
        print("\n📋 Test 4: Check robot status")
        try:
            status = runner.get_safety_status()
            print(f"Robot status: {status}")
        except Exception as e:
            print(f"❌ Could not get status: {e}")
        
        print("\n✅ Simple movement test completed!")
        
    except Exception as e:
        print(f"❌ Error during movement test: {e}")
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
    test_simple_movement(robot_ip) 