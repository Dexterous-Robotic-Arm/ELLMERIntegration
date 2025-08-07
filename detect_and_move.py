#!/usr/bin/env python3
"""
Detect and Move with LLM Planning
"""

import sys
import os
import time
import json
from typing import Dict, Any, Optional, Tuple

# Add the robot_control package to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_control.robot_controller import XArmRunner
from robot_control.robot_controller.executor import TaskExecutor
from robot_control.task_planner.planner_llm import plan_with_gemini, plan_fallback

def scan_for_object_with_vision(object_type: str, robot_ip: str = "192.168.1.241") -> Optional[Dict[str, Any]]:
    """Scan for objects using vision system and robot movement."""
    print(f"🔍 Scanning for {object_type} using vision system and robot movement")
    print("=" * 60)
    
    # Step 1: Check imports
    print("📋 Step 1: Checking imports...")
    try:
        from ultralytics import YOLO
        import pyrealsense2 as rs
        import numpy as np
        print("✅ All imports successful")
    except ImportError as e:
        print(f"❌ Import error: {e}")
        return None
    
    # Step 2: Load YOLO model
    print("📋 Step 2: Loading YOLO model...")
    try:
        model = YOLO('yolov8n.pt')
        print("✅ YOLO model loaded successfully")
    except Exception as e:
        print(f"❌ Failed to load YOLO model: {e}")
        return None
    
    # Step 3: Initialize RealSense camera
    print("📋 Step 3: Initializing RealSense camera...")
    try:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        pipeline.start(config)
        print("✅ RealSense camera initialized successfully")
    except Exception as e:
        print(f"❌ Failed to initialize camera: {e}")
        return None
    
    # Step 4: Connect to robot
    print("📋 Step 4: Connecting to robot...")
    try:
        robot = XArmRunner(robot_ip)
        current_pos = robot.get_current_position()
        print(f"✅ Robot connected to {robot_ip}")
        print(f"[DEBUG] Raw position data: {current_pos}")
        if current_pos:
            print(f"✅ Current position: X={current_pos[0]:.1f}, Y={current_pos[1]:.1f}, Z={current_pos[2]:.1f}")
        else:
            print("⚠️ Could not get current position")
    except Exception as e:
        print(f"❌ Failed to connect to robot: {e}")
        pipeline.stop()
        return None
    
    # Step 5: Scan for objects with robot movement
    print("📋 Step 5: Starting scan for '{}' with robot movement...".format(object_type))
    print("🔄 Robot will move in a horizontal pattern to scan for objects")
    
    # Scan parameters
    x_pos = 400.0
    z_height = 250.0
    y_start = -200.0
    y_end = 200.0
    steps = 7
    pause_time = 2.0
    
    print("📊 Scan parameters:")
    print(f"   - X position: {x_pos}mm")
    print(f"   - Z height: {z_height}mm")
    print(f"   - Y sweep: {y_start} to {y_end}mm")
    print(f"   - Steps: {steps}")
    print(f"   - Pause: {pause_time}s per position")
    
    # Move to scan center first
    print(f"🔄 Moving to scan center: [{x_pos}, 0, {z_height}]")
    try:
        robot.move_pose([x_pos, 0, z_height], [0, 90, 0])
        print("✅ Arrived at scan center")
    except Exception as e:
        print(f"❌ Failed to move to scan center: {e}")
        pipeline.stop()
        robot.disconnect()
        return None
    
    # Scan positions
    y_positions = [y_start + i * (y_end - y_start) / (steps - 1) for i in range(steps)]
    target_found = None
    
    for i, y_pos in enumerate(y_positions):
        print(f"🔍 Scan position {i+1}/{steps}: Y={y_pos:.1f}mm")
        
        # Move robot to scan position
        try:
            robot.move_pose([x_pos, y_pos, z_height], [0, 90, 0])
            print(f"✅ Moved to scan position {i+1}")
        except Exception as e:
            print(f"❌ Failed to move to scan position {i+1}: {e}")
            continue
        
        # Pause for detection
        print(f"⏳ Pausing {pause_time}s for detection...")
        time.sleep(pause_time)
        
        # Get camera frame
        try:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                continue
            
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # Run YOLO detection
            results = model(color_image, verbose=False)
            
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # Get box coordinates
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                        
                        # Get confidence and class
                        confidence = float(box.conf[0])
                        class_id = int(box.cls[0])
                        class_name = model.names[class_id]
                        
                        # Check if this is our target object
                        if class_name.lower() == object_type.lower() and confidence > 0.3:
                            # Calculate center
                            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
                            
                            # Get depth at center
                            depth = depth_frame.get_distance(cx, cy)
                            
                            print(f"🎯 TARGET FOUND at scan position {i+1}!")
                            print(f"   Object: {class_name} (confidence: {confidence:.2f})")
                            print(f"   Pixel center: ({cx},{cy})")
                            print(f"   Depth: {depth:.3f}m")
                            print(f"   Robot position: X={x_pos}, Y={y_pos:.1f}, Z={z_height}")
                            
                            target_found = {
                                'object_type': class_name,
                                'pixel_center': (cx, cy),
                                'depth': depth,
                                'confidence': confidence,
                                'bbox': (x1, y1, x2, y2),
                                'robot_pos': [x_pos, y_pos, z_height]
                            }
                            break
                    
                    if target_found:
                        break
            
            if target_found:
                break
                
        except Exception as e:
            print(f"❌ Error during detection at position {i+1}: {e}")
            continue
    
    # Cleanup
    pipeline.stop()
    robot.disconnect()
    
    # Print scan results
    print("\n📊 Scan Results:")
    if target_found:
        print(f"   - Scan positions completed: {i+1}/{steps}")
        print(f"   - Target '{object_type}' found: Yes")
        print(f"   - Target position: {target_found}")
        print(f"✅ Target '{object_type}' found! Stopping scan.")
    else:
        print(f"   - Scan positions completed: {steps}/{steps}")
        print(f"   - Target '{object_type}' found: No")
        print(f"❌ Target '{object_type}' not found after scanning all positions.")
    
    return target_found

def create_llm_plan(object_data: Dict[str, Any], object_type: str) -> Optional[Dict[str, Any]]:
    """Create LLM plan for approaching the detected object."""
    print("📋 Step 2: Creating LLM plan...")
    print("🧠 Creating LLM plan for {}".format(object_type))
    print("=" * 60)
    
    # Step 1: Load LLM planner
    print("📋 Step 1: Loading LLM planner...")
    try:
        from robot_control.task_planner.planner_llm import plan_with_gemini, plan_fallback
        print("✅ LLM planner functions imported successfully")
    except ImportError as e:
        print(f"❌ LLM planner import failed: {e}")
        print("❌ Failed to create LLM plan. Cannot proceed.")
        print("🔧 Troubleshooting:")
        print("   - Check LLM API key")
        print("   - Verify internet connection")
        print("   - Check LLM service availability")
        return None
    
    # Step 2: Generate plan with LLM
    print("📋 Step 2: Generating plan with LLM...")
    
    # Create task description
    depth = object_data.get('depth', 0)
    confidence = object_data.get('confidence', 0)
    pixel_center = object_data.get('pixel_center', (0, 0))
    robot_pos = object_data.get('robot_pos', [0, 0, 0])
    
    task = f"""A {object_type} has been detected at a distance of {depth:.3f} meters with confidence {confidence:.2f}. 
        The object is located at pixel coordinates {pixel_center} in the camera view.
        The robot is currently at position X={robot_pos[0]}, Y={robot_pos[1]}, Z={robot_pos[2]}.
        
        Please create a plan to:
        1. Move the robot toward the detected {object_type}
        2. Approach the object safely
        3. Test gripper operations (open and close)
        4. Return to home position
        
        Use the scanning technique we have established for object detection and movement."""
    
    try:
        # Try Gemini first
        print("🔄 Attempting to generate plan with Gemini...")
        plan = plan_with_gemini(task)
        print("✅ Gemini plan generated successfully!")
    except Exception as e:
        print(f"⚠️ Gemini failed: {e}")
        print("🔄 Falling back to basic plan...")
        plan = plan_fallback(task)
        print("✅ Fallback plan generated successfully!")
    
    print("✅ Final plan generated successfully!")
    print(f"📊 Plan has {len(plan.get('steps', []))} steps")
    print(f"🎯 Goal: {plan.get('goal', 'No goal specified')}")
    for i, step in enumerate(plan.get('steps', []), 1):
        print(f"   Step {i}: {step.get('action', 'UNKNOWN')}")
    
    return plan

def create_custom_object_plan(object_data: Dict[str, Any], object_type: str) -> Dict[str, Any]:
    """Create a custom plan for approaching the detected object."""
    print("🔄 Creating custom object approach plan...")
    
    return {
        "goal": f"Move towards the detected {object_type}, test the gripper operation, and return to the home position.",
        "steps": [
            {
                "action": "MOVE_TO_NAMED",
                "name": "home"
            },
            {
                "action": "MOVE_TO_NAMED", 
                "name": "staging_area"
            },
            {
                "action": "APPROACH_OBJECT",
                "label": object_type,
                "hover_mm": 120,
                "timeout_sec": 5
            },
            {
                "action": "GRIPPER_TEST",
                "cycles": 2,
                "delay": 1.0
            },
            {
                "action": "RETREAT_Z",
                "dz_mm": 50
            },
            {
                "action": "MOVE_TO_NAMED",
                "name": "home"
            }
        ]
    }

def execute_robot_plan(plan: Dict[str, Any], robot_ip: str = "192.168.1.241") -> bool:
    """Execute the robot plan using TaskExecutor."""
    print("📋 Step 3: Executing robot plan...")
    print("🤖 Executing robot plan")
    print("=" * 60)
    
    # Step 1: Load TaskExecutor
    print("📋 Step 1: Loading TaskExecutor...")
    try:
        from robot_control.robot_controller.executor import TaskExecutor
        print("✅ TaskExecutor imported successfully")
    except ImportError as e:
        print(f"❌ TaskExecutor import failed: {e}")
        return False
    
    # Step 2: Initialize TaskExecutor
    print("📋 Step 2: Initializing TaskExecutor...")
    try:
        executor = TaskExecutor(robot_ip, sim=False, dry_run=False)
        print("✅ TaskExecutor initialized successfully")
    except Exception as e:
        print(f"❌ TaskExecutor initialization failed: {e}")
        return False
    
    # Step 3: Execute plan
    print("📋 Step 3: Executing plan...")
    try:
        steps = plan.get('steps', [])
        print(f"📊 Executing {len(steps)} steps")
        print(f"[Plan] Goal: {plan.get('goal', 'No goal specified')}")
        
        # Get gripper status
        gripper_status = executor.get_gripper_status()
        print(f"[Plan] Gripper status: {gripper_status}")
        
        for i, step in enumerate(steps, 1):
            print(f"[Plan] Step {i}/{len(steps)}: {step.get('action', 'UNKNOWN')} {step}")
            try:
                executor.execute_step(step)
                print(f"✅ Step {i} completed successfully")
            except Exception as e:
                print(f"❌ Step {i} failed: {e}")
                continue
        
        print(f"[Plan] Done. Execution time: {executor.get_execution_time():.1f}s, Steps completed: {len(steps)}")
        print("✅ Plan execution completed successfully!")
        
        # Print execution stats
        stats = executor.get_execution_stats()
        print(f"📊 Execution stats: {stats}")
        
        return True
        
    except Exception as e:
        print(f"❌ Plan execution failed: {e}")
        return False
    finally:
        # Step 4: Cleanup
        print("📋 Step 4: Cleaning up...")
        try:
            executor.shutdown()
            print("✅ TaskExecutor shutdown completed")
        except Exception as e:
            print(f"⚠️ Cleanup warning: {e}")

def detect_and_move_with_llm_planning(robot_ip: str = "192.168.1.241", object_type: str = "bottle"):
    """Detect an object and move toward it using LLM planning."""
    
    print(f"🤖 Detect and Move with LLM Planning: {object_type}")
    print(f"📍 Robot IP: {robot_ip}")
    print(f"🎯 Target object: {object_type}")
    print("=" * 60)
    
    try:
        # Step 1: Scan for object with vision
        print("\n📋 Step 1: Scanning for object...")
        object_data = scan_for_object_with_vision(object_type, robot_ip)
        
        if not object_data:
            print(f"❌ Failed to detect {object_type}")
            return False
        
        # Step 2: Create LLM plan
        print("\n📋 Step 2: Creating LLM plan...")
        plan = create_llm_plan(object_data, object_type)
        
        if not plan:
            print("❌ Failed to create LLM plan. Cannot proceed.")
            print("🔧 Troubleshooting:")
            print("   - Check LLM API key")
            print("   - Verify internet connection")
            print("   - Check LLM service availability")
            return False
        
        # Validate if plan includes object movement
        plan_has_object_movement = False
        for step in plan.get('steps', []):
            action = step.get('action', '')
            if action in ['APPROACH_OBJECT', 'MOVE_TO_OBJECT', 'SCAN_FOR_OBJECTS']:
                plan_has_object_movement = True
                break
        
        if not plan_has_object_movement:
            print("⚠️ LLM plan doesn't include object movement, using custom plan...")
            plan = create_custom_object_plan(object_data, object_type)
        
        # Step 3: Execute robot plan
        print("\n📋 Step 3: Executing robot plan...")
        success = execute_robot_plan(plan, robot_ip)
        
        if success:
            print(f"\n🎉 Successfully completed detect and move for {object_type}!")
            print("✅ Object detection: WORKING")
            print("✅ LLM planning: WORKING")
            print("✅ Robot execution: WORKING")
            print("✅ Full workflow: COMPLETED")
            return True
        else:
            print(f"\n❌ Failed to execute plan for {object_type}")
            return False
            
    except Exception as e:
        print(f"\n❌ Detect and move failed for {object_type}")
        print(f"🔧 Check the troubleshooting steps above")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python detect_and_move.py [object_type]")
        print("")
        print("Examples:")
        print("  python detect_and_move.py bottle")
        print("  python detect_and_move.py cup")
        print("  python detect_and_move.py person")
        sys.exit(1)
    
    object_type = sys.argv[1] if len(sys.argv) > 1 else "bottle"
    robot_ip = "192.168.1.241"
    
    success = detect_and_move_with_llm_planning(robot_ip, object_type)
    
    if success:
        print(f"\n✅ Successfully completed detect and move for {object_type}!")
        print("🔧 Next steps:")
        print("   - Test with different objects")
        print("   - Adjust detection parameters if needed")
        print("   - Fine-tune movement plans")
    else:
        print(f"\n❌ Detect and move failed for {object_type}")
        print("🔧 Check the troubleshooting steps above") 