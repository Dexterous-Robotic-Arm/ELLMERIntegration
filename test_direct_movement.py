#!/usr/bin/env python3
"""
Test direct robot movement bypassing safety checks
"""

import sys
import os
import time

# Add the robot_control package to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_control.robot_controller import XArmRunner

def test_direct_movement(robot_ip: str = "192.168.1.241"):
    """Test direct robot movement bypassing safety checks."""
    
    print("🤖 Testing Direct Robot Movement (Bypassing Safety)")
    print(f"📍 Connecting to robot at {robot_ip}")
    
    try:
        # Initialize robot connection
        runner = XArmRunner(robot_ip, sim=False)
        
        # Test 1: Get initial position
        print("\n📋 Test 1: Get initial position")
        initial_pos = runner.get_current_position()
        print(f"Initial position: {initial_pos}")
        
        # Test 2: Direct movement using XArm API (bypassing our wrapper)
        print("\n📋 Test 2: Direct XArm API movement")
        if initial_pos is not None and runner.arm is not None:
            # Try direct movement using the XArm API
            target_x = initial_pos[0] + 50  # Move 50mm in X
            target_y = initial_pos[1] + 50  # Move 50mm in Y
            
            print(f"Moving from X={initial_pos[0]:.2f},Y={initial_pos[1]:.2f} to X={target_x:.2f},Y={target_y:.2f}")
            
            try:
                # Direct XArm API call
                runner.arm.set_position(
                    x=target_x, 
                    y=target_y, 
                    z=initial_pos[2],
                    roll=180, 
                    pitch=0, 
                    yaw=0,
                    speed=50,  # Slow speed for safety
                    wait=True
                )
                
                time.sleep(3)  # Wait for movement
                
                # Check new position
                new_pos = runner.get_current_position()
                print(f"New position: {new_pos}")
                if new_pos is not None:
                    x_moved = new_pos[0] - initial_pos[0]
                    y_moved = new_pos[1] - initial_pos[1]
                    print(f"X movement: {x_moved:.2f}mm, Y movement: {y_moved:.2f}mm")
                    
            except Exception as e:
                print(f"❌ Direct movement failed: {e}")
        
        # Test 3: Check robot state and errors
        print("\n📋 Test 3: Check robot state")
        if runner.arm is not None:
            try:
                # Get robot state
                state = runner.arm.get_state()
                print(f"Robot state: {state}")
                
                # Get error code
                error_code = runner.arm.get_err_code()
                print(f"Error code: {error_code}")
                
                # Get warning code
                warn_code = runner.arm.get_warn_code()
                print(f"Warning code: {warn_code}")
                
            except Exception as e:
                print(f"❌ Could not get robot state: {e}")
        
        # Test 4: Try enabling motion
        print("\n📋 Test 4: Enable motion")
        if runner.arm is not None:
            try:
                # Enable motion
                runner.arm.motion_enable(enable=True)
                print("Motion enabled")
                
                # Set mode to position control
                runner.arm.set_mode(0)  # Position control mode
                print("Set to position control mode")
                
                # Set state to ready
                runner.arm.set_state(0)  # Ready state
                print("Set to ready state")
                
            except Exception as e:
                print(f"❌ Could not enable motion: {e}")
        
        print("\n✅ Direct movement test completed!")
        
    except Exception as e:
        print(f"❌ Error during direct movement test: {e}")
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
    test_direct_movement(robot_ip) 