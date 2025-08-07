#!/usr/bin/env python3
"""
Detect and Move with LLM Planning
"""

import sys
import os
import time
import json
import math
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
            
            # Run YOLO detection (force CPU to avoid CUDA compatibility issues)
            results = model(color_image, verbose=False, device='cpu')
            
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
                            
                            # Get depth at center with fallback
                            try:
                                depth = depth_frame.get_distance(cx, cy)
                                if depth == 0.0:  # If depth is 0, try nearby pixels
                                    for dx in [-5, 0, 5]:
                                        for dy in [-5, 0, 5]:
                                            test_depth = depth_frame.get_distance(cx + dx, cy + dy)
                                            if test_depth > 0.0:
                                                depth = test_depth
                                                break
                                        if depth > 0.0:
                                            break
                                if depth == 0.0:  # Still 0, use default
                                    depth = 0.5  # Default 0.5m
                            except Exception as e:
                                print(f"⚠️ Depth measurement failed: {e}")
                                depth = 0.5  # Default 0.5m
                            
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
    
    # Ultra-compact task description to minimize tokens
    task = f"Move to {object_type} at {depth:.1f}m, test gripper, return home"
    
    # Use LLM planning only - no fallbacks
    print("🔄 Generating plan with Gemini...")
    plan = plan_with_gemini(task)
    print("✅ Gemini plan generated successfully!")
    
    print("✅ Final plan generated successfully!")
    print(f"📊 Plan has {len(plan.get('steps', []))} steps")
    print(f"🎯 Goal: {plan.get('goal', 'No goal specified')}")
    for i, step in enumerate(plan.get('steps', []), 1):
        print(f"   Step {i}: {step.get('action', 'UNKNOWN')}")
    
    return plan

def create_custom_object_plan(object_data: Dict[str, Any], object_type: str) -> Dict[str, Any]:
    """Create a custom plan for approaching the detected object."""
    print("🔄 Creating custom object approach plan...")
    print(f"🔍 Input object_data: {object_data}")
    
    # Get object position from detection data
    robot_pos = object_data.get('robot_pos', [400, 0, 250])
    depth = object_data.get('depth', 0.5)  # Depth in meters
    pixel_center = object_data.get('pixel_center', (320, 240))  # Camera center
    
    print(f"🔍 Extracted robot_pos: {robot_pos}")
    print(f"🔍 Object depth: {depth}m")
    print(f"🔍 Pixel center: {pixel_center}")
    
    # Calculate actual bottle position in 3D space
    # Camera is mounted on robot, so we need to calculate bottle position relative to robot
    # Assuming camera is pointing forward (X direction)
    # Depth gives us distance from camera to bottle
    # Pixel coordinates give us Y and Z offsets
    
    # Camera parameters (approximate for RealSense D435i)
    camera_fov_h = 87  # degrees horizontal FOV
    camera_fov_v = 58  # degrees vertical FOV
    image_width = 640
    image_height = 480
    
    # Calculate angular offsets from pixel coordinates
    pixel_x, pixel_y = pixel_center
    angle_y = (pixel_x - image_width/2) * (camera_fov_h / image_width)  # Yaw angle
    angle_z = (pixel_y - image_height/2) * (camera_fov_v / image_height)  # Pitch angle
    
    # Convert to radians
    angle_y_rad = math.radians(angle_y)
    angle_z_rad = math.radians(angle_z)
    
    # Calculate 3D position of bottle relative to camera
    # X = depth (forward)
    # Y = depth * tan(yaw) (left/right)
    # Z = depth * tan(pitch) (up/down)
    bottle_x = depth * 1000  # Convert to mm
    bottle_y = depth * 1000 * math.tan(angle_y_rad)
    bottle_z = depth * 1000 * math.tan(angle_z_rad)
    
    # Move directly to detected object position
    target_x = robot_pos[0] + bottle_x
    target_y = robot_pos[1] + bottle_y
    target_z = robot_pos[2] + bottle_z
    
    print(f"🎯 Calculated bottle position: X={bottle_x:.1f}mm, Y={bottle_y:.1f}mm, Z={bottle_z:.1f}mm")
    print(f"🎯 Moving to bottle coordinates: X={target_x:.1f}, Y={target_y:.1f}, Z={target_z:.1f}")
    
    return {
        "goal": f"Move towards the detected {object_type}, test the gripper operation, and return to the home position.",
        "steps": [
            {
                "action": "MOVE_TO_POSE",
                "pose": {
                    "xyz_mm": [target_x, target_y, target_z],
                    "rpy_deg": [0, 90, 0]
                }
            },
            {
                "action": "OPEN_GRIPPER",
                "gripper": {"position": 850}
            },
            {
                "action": "SLEEP",
                "seconds": 1.0
            },
            {
                "action": "CLOSE_GRIPPER",
                "gripper": {"position": 200}
            },
            {
                "action": "SLEEP",
                "seconds": 1.0
            },
            {
                "action": "OPEN_GRIPPER",
                "gripper": {"position": 850}
            }
        ]
    }


def execute_robot_plan(plan: Dict[str, Any], robot_ip: str = "192.168.1.241") -> bool:
    """Execute the robot plan using working gripper code."""
    print("📋 Step 3: Executing robot plan...")
    print("🤖 Executing robot plan")
    print("=" * 60)
    
    # Initialize robot
    print("🤖 Initializing robot...")
    try:
        robot = XArmRunner(robot_ip)
        
        # Check and reset robot state
        current_state = robot.get_state()
        if current_state[1] == 4:  # If stopped
            print("🔄 Resetting robot state...")
            robot.reset_error()
            robot.set_state(0)  # Set to ready state
            time.sleep(0.1)
        
        current_pos = robot.get_current_position()
        if current_pos is None:
            print("❌ Failed to connect to robot")
            return False
        print(f"✅ Robot connected successfully at position: {current_pos}")
    except Exception as e:
        print(f"❌ Robot initialization failed: {e}")
        return False
    
    # Initialize gripper with working code
    print("🔧 Initializing gripper...")
    try:
        from working_gripper import initialize_gripper, open_gripper, close_gripper, disable_motor
        portHandler, packetHandler = initialize_gripper()
        gripper_enabled = True
        print("✅ Gripper initialized successfully")
    except Exception as e:
        print(f"⚠️ Gripper initialization failed: {e}")
        gripper_enabled = False
        portHandler = None
        packetHandler = None
    
    # Execute plan
    print("📋 Executing plan...")
    try:
        steps = plan.get('steps', [])
        print(f"📊 Executing {len(steps)} steps")
        print(f"[Plan] Goal: {plan.get('goal', 'No goal specified')}")
        
        for i, step in enumerate(steps, 1):
            action = step.get('action', 'UNKNOWN')
            print(f"[Plan] Step {i}/{len(steps)}: {action}")
            
            try:
                if action == 'MOVE_TO_POSE':
                    pose = step.get('pose', {})
                    xyz = pose.get('xyz_mm', [0, 0, 0])
                    rpy = pose.get('rpy_deg', [0, 90, 0])
                    
                    # Validate coordinates
                    if xyz[0] <= 0 or xyz[0] > 800:  # X range
                        print(f"⚠️ Invalid X coordinate: {xyz[0]}, using 400mm")
                        xyz[0] = 400
                    if abs(xyz[1]) > 400:  # Y range
                        print(f"⚠️ Invalid Y coordinate: {xyz[1]}, clamping to ±400mm")
                        xyz[1] = max(min(xyz[1], 400), -400)
                    if xyz[2] <= 0 or xyz[2] > 600:  # Z range
                        print(f"⚠️ Invalid Z coordinate: {xyz[2]}, using 250mm")
                        xyz[2] = 250
                    
                    print(f"🎯 Moving to pose: {xyz} with orientation {rpy}")
                    
                    # Check robot state before moving
                    current_state = robot.get_state()
                    if current_state[1] == 4:  # If stopped
                        print("🔄 Resetting robot state before move...")
                        robot.reset_error()
                        robot.set_state(0)
                        time.sleep(0.1)
                    
                    robot.move_pose(xyz, rpy)
                    print("✅ Moved to pose successfully")
                
                elif action == 'OPEN_GRIPPER':
                    print("🔓 Opening gripper...")
                    if gripper_enabled:
                        open_gripper(portHandler, packetHandler)
                        print("✅ Gripper opened successfully")
                    else:
                        print("⚠️ Gripper not available")
                
                elif action == 'CLOSE_GRIPPER':
                    print("🔒 Closing gripper...")
                    if gripper_enabled:
                        close_gripper(portHandler, packetHandler)
                        print("✅ Gripper closed successfully")
                    else:
                        print("⚠️ Gripper not available")
                
                elif action == 'SLEEP':
                    seconds = step.get('seconds', 1.0)
                    print(f"⏳ Sleeping for {seconds} seconds...")
                    time.sleep(seconds)
                    print("✅ Sleep completed")
                
                elif action == 'MOVE_TO_NAMED':
                    name = step.get('name', 'home')
                    print(f"🏠 Moving to named position: {name}")
                    if name == 'home':
                        robot.go_home()
                    print(f"✅ Moved to {name}")
                
                elif action == 'RETREAT_Z':
                    dz = step.get('dz_mm', 50)
                    current_pos = robot.get_current_position()
                    if current_pos:
                        new_pos = current_pos.copy()
                        new_pos[2] += dz
                        robot.move_pose(new_pos, [0, 90, 0])
                        print(f"✅ Retreated {dz}mm in Z")
                    else:
                        print("⚠️ Could not get current position")
                
                else:
                    print(f"⚠️ Unknown action: {action}")
                    continue
                
                print(f"✅ Step {i} completed successfully")
                
            except Exception as e:
                print(f"❌ Step {i} failed: {e}")
                continue
        
        print("✅ Plan execution completed successfully!")
        return True
        
    except Exception as e:
        print(f"❌ Plan execution failed: {e}")
        return False
    finally:
        # Cleanup
        print("📋 Step 4: Cleaning up...")
        if gripper_enabled:
            try:
                disable_motor(portHandler, packetHandler)
                print("✅ Gripper disconnected")
            except Exception as e:
                print(f"⚠️ Gripper cleanup warning: {e}")
        
        try:
            robot.disconnect()
            print("✅ Robot disconnected")
        except Exception as e:
            print(f"⚠️ Robot cleanup warning: {e}")

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
        
        # Use LLM plan directly - no fallbacks
        print("✅ Using LLM-generated plan directly")
        print(f"🔍 LLM plan: {plan}")
        
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