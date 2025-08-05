#!/usr/bin/env python3
"""
Comprehensive test for full workspace scanning
Makes the robot actually move and scan its entire workspace
"""

import sys
import os
import time

# Add the robot_control package to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_control.robot_controller import XArmRunner
from robot_control.robot_controller.executor import TaskExecutor

def test_full_workspace_scan(robot_ip: str = "192.168.1.241"):
    """Test full workspace scanning with movement verification."""
    
    print("🤖 Full Workspace Scanning Test")
    print(f"📍 Connecting to robot at {robot_ip}")
    
    try:
        # Initialize robot connection
        runner = XArmRunner(robot_ip, sim=False)
        
        # Test 1: Verify robot can move
        print("\n📋 Test 1: Basic movement verification")
        initial_pos = runner.get_current_position()
        print(f"Initial position: {initial_pos}")
        
        if initial_pos is not None:
            # Test small movement first
            test_pos = [initial_pos[0], initial_pos[1] + 30, initial_pos[2]]
            print(f"Testing movement to: {test_pos}")
            
            runner.move_pose(test_pos, [0, -45, 0])  # Face camera outward at 45-degree angle
            time.sleep(3)
            
            new_pos = runner.get_current_position()
            print(f"Position after test movement: {new_pos}")
            
            if new_pos is not None:
                y_moved = new_pos[1] - initial_pos[1]
                print(f"Y movement: {y_moved:.2f}mm")
                
                if abs(y_moved) > 5:  # Should have moved at least 5mm
                    print("✅ Basic movement verified!")
                else:
                    print("❌ Basic movement failed - robot not moving")
                    return
            else:
                print("❌ Could not get position after movement")
                return
        
        # Test 2: Move to scan center
        print("\n📋 Test 2: Move to scan center")
        current_pos = runner.get_current_position()
        if current_pos is not None:
            scan_center = [400, 0, current_pos[2]]  # Center of workspace
            print(f"Moving to scan center: {scan_center}")
            
            runner.move_pose(scan_center, [0, -45, 0])  # Face camera outward at 45-degree angle
            time.sleep(3)
            
            center_pos = runner.get_current_position()
            print(f"Position at scan center: {center_pos}")
            
            if center_pos is not None:
                x_moved = center_pos[0] - current_pos[0]
                print(f"X movement to center: {x_moved:.2f}mm")
                
                if abs(x_moved) > 50:  # Should have moved significantly
                    print("✅ Scan center movement successful!")
                else:
                    print("❌ Scan center movement failed")
                    return
        
        # Test 3: Full workspace scan
        print("\n📋 Test 3: Full workspace scan")
        scan_plan = {
            "goal": "full workspace scan",
            "steps": [
                {
                    "action": "SCAN_FOR_OBJECTS",
                    "pattern": "horizontal",
                    "sweep_mm": 500,  # Large sweep to test full workspace
                    "steps": 7,
                    "pause_sec": 1.0
                }
            ]
        }
        
        executor = TaskExecutor(robot_ip, sim=False, dry_run=False)
        executor.execute(scan_plan)
        executor.shutdown()
        
        # Test 4: Verify final position
        print("\n📋 Test 4: Verify final position")
        final_pos = runner.get_current_position()
        print(f"Final position: {final_pos}")
        
        if final_pos is not None and initial_pos is not None:
            total_x_moved = final_pos[0] - initial_pos[0]
            total_y_moved = final_pos[1] - initial_pos[1]
            print(f"Total X movement: {total_x_moved:.2f}mm")
            print(f"Total Y movement: {total_y_moved:.2f}mm")
            
            if abs(total_x_moved) > 100 or abs(total_y_moved) > 100:
                print("✅ Full workspace scan successful!")
            else:
                print("❌ Full workspace scan failed - insufficient movement")
        
        print("\n✅ Full workspace scan test completed!")
        
    except Exception as e:
        print(f"❌ Error during full workspace test: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Clean up
        if 'runner' in locals():
            runner.disconnect()
        print("\n🔌 Disconnected from robot")

def show_workspace_info():
    """Show workspace information."""
    print("\n📐 Workspace Information:")
    print("X range: 150mm to 650mm (500mm total)")
    print("Y range: -300mm to +300mm (600mm total)")
    print("Z range: 50mm to 500mm (450mm total)")
    print("\n🎯 Scan Strategy:")
    print("1. Move to X=400mm (workspace center)")
    print("2. Perform Y-axis sweep from -250mm to +250mm")
    print("3. Verify actual movement at each step")

if __name__ == "__main__":
    # Get robot IP from command line or use default
    robot_ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.241"
    
    # Show workspace information
    show_workspace_info()
    
    # Test full workspace scanning
    test_full_workspace_scan(robot_ip) 