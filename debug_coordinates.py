#!/usr/bin/env python3
"""
Debug coordinate systems between object detection and robot
"""

import sys
import os
import time

# Add the robot_control package to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_control.robot_controller.executor import TaskExecutor

def debug_coordinates():
    """Debug coordinate systems."""
    
    print("🔍 Debugging Coordinate Systems")
    print("📍 Connecting to robot at 192.168.1.241")
    print("📊 Will show object detection vs robot coordinates")
    
    try:
        # Create executor to access object detection
        executor = TaskExecutor("192.168.1.241", world_yaml="config/robot/world_model.yaml", sim=False, dry_run=True)
        
        print("🔍 Monitoring coordinates for 30 seconds...")
        print("📊 Object detection coordinates vs Robot coordinates:")
        
        # Monitor for 30 seconds
        start_time = time.time()
        while time.time() - start_time < 30:
            # Get robot position
            robot_pos = executor.runner.get_current_position()
            
            # Check for detected objects
            detected_objects = list(executor.obj_index.latest_mm.keys())
            
            if detected_objects:
                print(f"\n⏰ Time: {time.time() - start_time:.1f}s")
                print(f"🤖 Robot position: {robot_pos}")
                
                for obj in detected_objects:
                    obj_pos = executor.obj_index.latest_mm[obj]
                    print(f"🎯 {obj} detected at: {obj_pos}")
                    
                    if robot_pos and len(robot_pos) >= 3:
                        print(f"📏 Distance: X_diff={obj_pos[0]-robot_pos[0]:.1f}, Y_diff={obj_pos[1]-robot_pos[1]:.1f}, Z_diff={obj_pos[2]-robot_pos[2]:.1f}")
                        
                        # Check coordinate system
                        if abs(obj_pos[0]) > 1000 or abs(obj_pos[1]) > 1000 or abs(obj_pos[2]) > 1000:
                            print(f"⚠️  {obj} coordinates seem to be in different system (large numbers)")
                        else:
                            print(f"✅ {obj} coordinates seem to be in same system")
            else:
                print(f"⏳ No objects detected... (Robot: {robot_pos})")
            
            time.sleep(2)  # Check every 2 seconds
        
        executor.shutdown()
        print(f"\n✅ Coordinate debug completed!")
        
    except Exception as e:
        print(f"❌ Debug failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    debug_coordinates() 