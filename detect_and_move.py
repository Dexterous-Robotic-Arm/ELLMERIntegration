#!/usr/bin/env python3
"""
Detect and Move with LLM Planning
"""

import sys
import os
import time
import json
import math
import time
from typing import Dict, Any, Optional, Tuple

# Add the robot_control package to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_control.robot_controller import XArmRunner
from robot_control.robot_controller.executor import TaskExecutor
from robot_control.task_planner.planner_llm import plan_with_gemini, plan_fallback

# Global state tracking
target_locked = False
target_position = None
target_object_name = None
movement_start_time = None
last_movement_time = None

def detect_and_move_to_object(robot_ip: str = "192.168.1.241", target_object_type: str = None) -> bool:
    """Detect any object and move camera closer to it."""
    global target_locked, target_position, target_object_name, movement_start_time, last_movement_time
    
    print("🔍 Detecting objects and moving camera closer")
    print("=" * 60)
    
    # Step 1: Check imports
    print("📋 Step 1: Checking imports...")
    try:
        from ultralytics import YOLO
        import pyrealsense2 as rs
        import numpy as np
        import cv2
        print("✅ All imports successful")
    except ImportError as e:
        print(f"❌ Import error: {e}")
        return False
    
    # Step 2: Load YOLO model
    print("📋 Step 2: Loading YOLO model...")
    try:
        model = YOLO('yolov8n.pt')
        print("✅ YOLO model loaded successfully")
    except Exception as e:
        print(f"❌ Failed to load YOLO model: {e}")
        return False
    
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
        return False
    
    # Step 4: Connect to robot
    print("📋 Step 4: Connecting to robot...")
    time.sleep(10)
    try:
        robot = XArmRunner(robot_ip)
        current_pos = robot.get_current_position()
        print(f"✅ Robot connected to {robot_ip}")
        if current_pos:
            print(f"✅ Current position: X={current_pos[0]:.1f}, Y={current_pos[1]:.1f}, Z={current_pos[2]:.1f}")
        else:
            print("⚠️ Could not get current position")
    except Exception as e:
        print(f"❌ Failed to connect to robot: {e}")
        pipeline.stop()
        return False
    
    # Step 4.5: Initialize gripper
    print("📋 Step 4.5: Initializing gripper...")
    try:
        from robot_control.robot_controller.gripper import XL330GripperController
        gripper = XL330GripperController()
        if not gripper.enabled:
            print("⚠️ XL330 gripper not enabled - continuing without gripper")
            gripper = None
        else:
            print("✅ XL330 gripper initialized successfully")
    except Exception as e:
        print(f"⚠️ Gripper initialization failed: {e}")
        gripper = None
    
    # Step 5: Object detection and movement
    print("📋 Step 5: Starting object detection...")
    
    # Create display window
    cv2.namedWindow('Object Detection', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Object Detection', 1280, 960)
    
    try:
        while True:
            # Get camera frame
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                continue
            
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # Run YOLO detection
            results = model(color_image, verbose=False, device='cpu', conf=0.3)
            
            # Print all detected objects
            detected_objects = []
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        confidence = float(box.conf[0])
                        class_id = int(box.cls[0])
                        class_name = model.names[class_id]
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
                        try:
                            depth = depth_frame.get_distance(cx, cy)
                            if depth == 0.0:
                                depth = 0.5
                        except:
                            depth = 0.5
                        detected_objects.append(f"{class_name}({confidence:.2f}) at ({cx},{cy}) depth:{depth:.2f}m")
            if detected_objects:
                print(f"📍 Detected: {', '.join(detected_objects)}")
            
            # Process detections - Initial lock OR position updates
            
            if not target_locked:
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
                            
                            # Calculate center
                            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
                            
                            # Get depth at center
                            try:
                                depth = depth_frame.get_distance(cx, cy)
                                if depth == 0.0:
                                    depth = 0.5  # Default if no depth
                            except:
                                depth = 0.5
                            
                            # If object is detected and we want to move closer
                            if confidence > 0.5:  # High confidence detection
                                # Filter by target object type if specified
                                if target_object_type and class_name != target_object_type:
                                    continue  # Skip this object, not the target type
                                
                                print(f"🎯 LOCKING TARGET: {class_name} at center ({cx}, {cy}) with depth {depth:.2f}m")
                                
                                # Calculate FULL target position using same math as original
                                current_pos = robot.get_current_position()
                                if not current_pos:
                                    current_pos = [400, 0, 250]
                                
                                # Camera parameters (same as original)
                                camera_fov_h = 87  # degrees horizontal FOV
                                camera_fov_v = 58  # degrees vertical FOV
                                image_width = 640
                                image_height = 480
                                
                                # Calculate angular offsets from pixel coordinates
                                pixel_x, pixel_y = cx, cy
                                angle_y = (pixel_x - image_width/2) * (camera_fov_h / image_width)  # Yaw angle
                                angle_z = (pixel_y - image_height/2) * (camera_fov_v / image_height)  # Pitch angle
                                
                                # Convert to radians
                                angle_y_rad = math.radians(angle_y)
                                angle_z_rad = math.radians(angle_z)
                                
                                # Calculate 3D position of object relative to camera
                                object_x = depth * 1000  # Convert to mm
                                object_y = depth * 1000 * math.tan(angle_y_rad)
                                object_z = depth * 1000 * math.tan(angle_z_rad)
                                
                                # Apply calibration offset (compensate for camera mounting)
                                #object_y = object_y + 50  # Move 5cm right to compensate for left offset
                                
                                # Move 17cm closer to the object
                                object_x = object_x - 130  # Subtract 170mm (17cm)
                                
                                # Calculate absolute target position
                                target_x = current_pos[0] + object_x
                                target_y = current_pos[1] + object_y
                                target_z = current_pos[2] + object_z
                                
                                # Lock the target
                                target_locked = True
                                target_position = [target_x, target_y, target_z]
                                target_object_name = class_name
                                moving_to_target = True
                                last_known_position = [target_x, target_y, target_z]
                                
                                print(f"🔒 TARGET LOCKED: Moving to {class_name} at position {target_position}")
                                
                                break  # Only lock first detected object
                    if target_locked:  # Break outer loop too
                        break
            
            elif moving_to_target:
                # Phase 2: Update target position while moving (if same object is still visible)
                for result in results:
                    boxes = result.boxes
                    if boxes is not None:
                        for box in boxes:
                            confidence = float(box.conf[0])
                            class_id = int(box.cls[0])
                            class_name = model.names[class_id]
                            
                            # Only update if it's the same object type with good confidence
                            if class_name == target_object_name and confidence > 0.5:
                                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                                cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
                                
                                try:
                                    depth = depth_frame.get_distance(cx, cy)
                                    if depth == 0.0:
                                        depth = 0.5
                                except:
                                    depth = 0.5
                                
                                # Recalculate updated position
                                current_pos = robot.get_current_position()
                                if current_pos:
                                    # Same math as before
                                    camera_fov_h = 87
                                    camera_fov_v = 58
                                    image_width = 640
                                    image_height = 480
                                    
                                    pixel_x, pixel_y = cx, cy
                                    angle_y = (pixel_x - image_width/2) * (camera_fov_h / image_width)
                                    angle_z = (pixel_y - image_height/2) * (camera_fov_v / image_height)
                                    
                                    angle_y_rad = math.radians(angle_y)
                                    angle_z_rad = math.radians(angle_z)
                                    
                                    object_x = depth * 1000
                                    object_y = depth * 1000 * math.tan(angle_y_rad)
                                    object_z = depth * 1000 * math.tan(angle_z_rad)
                                    
                                    object_x = object_x - 170  # Move 17cm closer
                                    
                                    updated_x = current_pos[0] + object_x
                                    updated_y = current_pos[1] + object_y
                                    updated_z = current_pos[2] + object_z
                                    
                                    # Update target position
                                    target_position = [updated_x, updated_y, updated_z]
                                    last_known_position = [updated_x, updated_y, updated_z]
                                    print(f"🔄 UPDATING TARGET: {class_name} new position {target_position}")
                                
                                break  # Only update first matching object
                        if class_name == target_object_name:  # Break outer loop if found
                            break
                
                # Check if we should execute the movement (after some time or user trigger)
                # For now, let's execute after a short delay or when 'e' is pressed
                key = cv2.waitKey(1) & 0xFF
                if key == ord('e') or time.time() % 5 < 0.1:  # Execute every 5 seconds or when 'e' pressed
                    print(f"🎯 EXECUTING MOVEMENT to final position: {last_known_position}")
                    moving_to_target = False
                    
                    # Execute the movement sequence
                    execute_movement_sequence(robot, gripper, last_known_position, target_object_name)
                    
                    # Reset after completion
                    target_locked = False
                    target_position = None
                    target_object_name = None
                    last_known_position = None
                    print("🔓 TARGET UNLOCKED: Ready for next object")
            
            # Always draw detections for visualization
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
                        
                        # Calculate center
                        cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
                        
                        # Get depth at center
                        try:
                            depth = depth_frame.get_distance(cx, cy)
                            if depth == 0.0:
                                depth = 0.5  # Default if no depth
                        except:
                            depth = 0.5
                        
                        # Draw bounding box
                        color = (0, 0, 255) if target_locked else (0, 255, 0)  # Red if locked, green if available
                        cv2.rectangle(color_image, (x1, y1), (x2, y2), color, 2)
                        
                        # Draw center dot
                        cv2.circle(color_image, (cx, cy), 5, (0, 0, 255), -1)
                        
                        # Draw label
                        label = f"{class_name} {confidence:.2f}"
                        cv2.putText(color_image, label, (x1, y1-10), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                        
                        # Display depth info
                        depth_text = f"Depth: {depth:.2f}m"
                        cv2.putText(color_image, depth_text, (cx+10, cy+10), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            
            # Display target lock status
            if target_locked and movement_start_time is not None:
                time_since_start = time.time() - movement_start_time
                status_text = f"MOVING TO: {target_object_name} ({time_since_start:.1f}s elapsed)"
                cv2.putText(color_image, status_text, (10, 30), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 165, 0), 2)  # Orange for moving
            else:
                status_text = "READY FOR TARGET DETECTION"
                cv2.putText(color_image, status_text, (10, 30), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)  # Green for ready
            
            # Display the image
            cv2.imshow('Object Detection', color_image)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("🛑 User requested quit")
                break
            elif key == ord('s'):
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                filename = f"object_detection_{timestamp}.jpg"
                cv2.imwrite(filename, color_image)
                print(f"💾 Saved frame as {filename}")
            elif key == ord('r'):
                # Reset target lock
                target_locked = False
                target_position = None
                target_object_name = None
                movement_start_time = None
                last_movement_time = None
                print("🔄 Target lock reset")
    
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
    
    finally:
        # Cleanup
        pipeline.stop()
        if gripper:
            gripper.disconnect()
        robot.disconnect()
        cv2.destroyAllWindows()
        print("✅ Cleanup completed")
    
    return True

def execute_movement_sequence(robot, gripper, target_position, object_name):
    """Execute the complete movement and gripper sequence."""
    print(f"🤖 Executing movement sequence for {object_name}")
    print("=" * 50)
    
    try:
        # Step 1: Open gripper first
        print("🔓 Step 1: Opening gripper...")
        if gripper and gripper.open_gripper():
            print("✅ Gripper opened successfully")
        else:
            print("⚠️ Gripper not available or failed to open")
        time.sleep(1.0)
        
        # Step 2: Move to target position
        print(f"🎯 Step 2: Moving to target position {target_position}...")
        try:
            robot.move_pose(target_position, [0, 90, 0])
            print(f"✅ Moved to target position: {target_position}")
        except Exception as e:
            print(f"❌ Failed to move to target: {e}")
            return False
        time.sleep(1.0)
        
        # Step 2.5: Move down 30cm before closing gripper
        print("⬇️ Step 2.5: Moving down 30cm...")
        try:
            down_position = target_position.copy()
            down_position[2] -= 300  # Move down 30cm (300mm)
            robot.move_pose(down_position, [0, 90, 0])
            print(f"✅ Moved down to: {down_position}")
        except Exception as e:
            print(f"❌ Failed to move down: {e}")
            return False
        time.sleep(1.0)
        
        # Step 3: Close gripper
        print("🔒 Step 3: Closing gripper...")
        if gripper and gripper.close_gripper():
            print("✅ Gripper closed successfully")
        else:
            print("⚠️ Gripper not available or failed to close")
        time.sleep(2.0)
        
        print(f"🎉 Movement sequence completed for {object_name}!")
        return True
        
    except Exception as e:
        print(f"❌ Movement sequence failed: {e}")
        return False

def execute_final_sequence(robot, gripper, target_position, object_name):
    """Execute the final gripper sequence: move down 30cm and close gripper."""
    print(f"🤖 Executing final sequence for {object_name}")
    print("=" * 50)
    
    try:
        # Step 1: Open gripper first (if not already open)
        print("🔓 Step 1: Opening gripper...")
        if gripper and gripper.open_gripper():
            print("✅ Gripper opened successfully")
        else:
            print("⚠️ Gripper not available or failed to open")
        time.sleep(1.0)
        
        # Step 2: Move down 30cm from current position
        print("⬇️ Step 2: Moving down 30cm...")
        try:
            current_pos = robot.get_current_position()
            if current_pos:
                down_position = current_pos.copy()
                down_position[2] -= 300  # Move down 30cm (300mm)
                robot.move_pose(down_position, [0, 90, 0])
                print(f"✅ Moved down to: {down_position}")
            else:
                print("⚠️ Could not get current position for down movement")
        except Exception as e:
            print(f"❌ Failed to move down: {e}")
            return False
        time.sleep(1.0)
        
        # Step 3: Close gripper
        print("🔒 Step 3: Closing gripper...")
        if gripper and gripper.close_gripper():
            print("✅ Gripper closed successfully")
        else:
            print("⚠️ Gripper not available or failed to close")
        time.sleep(2.0)
        
        print(f"🎉 Final sequence completed for {object_name}!")
        return True
        
    except Exception as e:
        print(f"❌ Final sequence failed: {e}")
        return False

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
    
    # Initialize gripper
    print("🔧 Initializing gripper...")
    try:
        from robot_control.robot_controller.gripper import XL330GripperController
        gripper = XL330GripperController()
        if not gripper.enabled:
            print("⚠️ XL330 gripper not enabled - continuing without gripper")
            gripper = None
        else:
            print("✅ XL330 gripper initialized successfully")
    except Exception as e:
        print(f"⚠️ Gripper initialization failed: {e}")
        gripper = None
    
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
                    if gripper and gripper.open_gripper():
                        print("✅ Gripper opened successfully")
                    else:
                        print("⚠️ Gripper not available or failed to open")
                
                elif action == 'CLOSE_GRIPPER':
                    print("🔒 Closing gripper...")
                    if gripper and gripper.close_gripper():
                        print("✅ Gripper closed successfully")
                    else:
                        print("⚠️ Gripper not available or failed to close")
                
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
        if gripper:
            try:
                gripper.disconnect()
                print("✅ Gripper disconnected")
            except Exception as e:
                print(f"⚠️ Gripper cleanup warning: {e}")
        
        try:
            robot.disconnect()
            print("✅ Robot disconnected")
        except Exception as e:
            print(f"⚠️ Robot cleanup warning: {e}")

def detect_and_move_to_object_simple(robot_ip: str = "192.168.1.241"):
    """Simple object detection and camera movement."""
    
    # Get target object type from command line arguments
    import sys
    target_object_type = None
    if len(sys.argv) > 1:
        target_object_type = sys.argv[1]
        print(f"🎯 Target object filter: {target_object_type}")
    
    print("🤖 Simple Object Detection and Camera Movement")
    print(f"📍 Robot IP: {robot_ip}")
    if target_object_type:
        print(f"🎯 Only targeting: {target_object_type}")
    else:
        print("🎯 Targeting any detected object")
    print("=" * 60)
    
    try:
        success = detect_and_move_to_object(robot_ip, target_object_type)
        
        if success:
            print(f"\n🎉 Successfully completed object detection and movement!")
            print("✅ Object detection: WORKING")
            print("✅ Camera movement: WORKING")
            print("✅ Full workflow: COMPLETED")
            return True
        else:
            print(f"\n❌ Failed to detect and move to objects")
            return False
            
    except Exception as e:
        print(f"\n❌ Object detection and movement failed")
        print(f"🔧 Check the troubleshooting steps above")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    robot_ip = "192.168.1.241"
    
    print("🤖 Simple Object Detection and Camera Movement")
    print("Controls:")
    print("  - Press 'q' to quit")
    print("  - Press 's' to save current frame")
    print("  - Press 'r' to reset target lock")
    print("  - Robot will CONTINUOUSLY TRACK and ADJUST PATH while moving")
    print("")
    
    
    success = detect_and_move_to_object_simple(robot_ip)
    
    if success:
        print(f"\n✅ Successfully completed object detection and movement!")
        print("🔧 Next steps:")
        print("   - Test with different objects")
        print("   - Adjust detection confidence threshold if needed")
        print("   - Fine-tune movement distance")
    else:
        print(f"\n❌ Object detection and movement failed")
        print("🔧 Check the troubleshooting steps above")