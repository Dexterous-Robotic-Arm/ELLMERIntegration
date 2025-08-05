#!/usr/bin/env python3
"""
Test script for full workspace utilization scanning
Demonstrates different scan patterns that use the entire robot workspace
"""

import sys
import os

# Add the robot_control package to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_control.robot_controller import XArmRunner
from robot_control.robot_controller.executor import TaskExecutor

def test_full_workspace_scan(robot_ip: str = "192.168.1.241"):
    """Test full workspace utilization scanning."""
    
    print("🤖 Full Workspace Utilization Scan Test")
    print(f"📍 Connecting to robot at {robot_ip}")
    
    try:
        # Initialize robot connection
        runner = XArmRunner(robot_ip, sim=False)
        
        # Test 1: Full Y-axis sweep (600mm total range)
        print("\n📋 Test 1: Full Y-axis sweep (600mm range)")
        scan_plan_1 = {
            "goal": "full Y-axis sweep",
            "steps": [
                {
                    "action": "SCAN_FOR_OBJECTS",
                    "pattern": "horizontal",
                    "sweep_mm": 600,  # Full Y range
                    "steps": 7,
                    "pause_sec": 1.0
                }
            ]
        }
        
        executor = TaskExecutor(robot_ip, sim=False, dry_run=False)
        executor.execute(scan_plan_1)
        executor.shutdown()
        
        # Test 2: Large sweep with more steps
        print("\n📋 Test 2: Large sweep with more steps (500mm, 10 steps)")
        scan_plan_2 = {
            "goal": "large sweep with more steps",
            "steps": [
                {
                    "action": "SCAN_FOR_OBJECTS",
                    "pattern": "horizontal",
                    "sweep_mm": 500,
                    "steps": 10,
                    "pause_sec": 0.5
                }
            ]
        }
        
        executor2 = TaskExecutor(robot_ip, sim=False, dry_run=False)
        executor2.execute(scan_plan_2)
        executor2.shutdown()
        
        # Test 3: Multiple scan positions at different Z heights
        print("\n📋 Test 3: Multiple Z-level scans")
        
        # Get current position for Z reference
        current_pos = runner.get_current_position()
        if current_pos is not None:
            base_z = current_pos[2]
            
            # Test different Z heights
            z_positions = [base_z, base_z + 50, base_z - 50]
            
            for i, z_pos in enumerate(z_positions):
                print(f"\n--- Z Level {i+1}: {z_pos:.1f}mm ---")
                
                # Move to this Z level first
                if z_pos != base_z:
                    target_pos = [400, 0, z_pos]  # Center position
                    print(f"Moving to Z level: {target_pos}")
                    runner.move_pose(target_pos, [180, 0, 0])
                
                # Scan at this Z level
                scan_plan_z = {
                    "goal": f"scan at Z={z_pos:.1f}",
                    "steps": [
                        {
                            "action": "SCAN_FOR_OBJECTS",
                            "pattern": "horizontal",
                            "sweep_mm": 400,
                            "steps": 5,
                            "pause_sec": 0.8
                        }
                    ]
                }
                
                executor_z = TaskExecutor(robot_ip, sim=False, dry_run=False)
                executor_z.execute(scan_plan_z)
                executor_z.shutdown()
        
        print("\n✅ Full workspace scan tests completed!")
        
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
    print("\n🎯 Optimal scan centers:")
    print("X center: 400mm (middle of workspace)")
    print("Y center: 0mm (middle of workspace)")
    print("Z: Use current position or specific heights")

if __name__ == "__main__":
    # Get robot IP from command line or use default
    robot_ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.241"
    
    # Show workspace information
    show_workspace_info()
    
    # Test full workspace utilization
    test_full_workspace_scan(robot_ip) 