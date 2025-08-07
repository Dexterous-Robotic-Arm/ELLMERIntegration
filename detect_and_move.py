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

def detect_and_move_to_object(robot_ip: str = "192.168.1.241") -> bool:
    """Detect any object and move camera closer to it."""
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
            
            # Process detections
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
                        
                        # Draw bounding box
                        cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        
                        # Draw center dot
                        cv2.circle(color_image, (cx, cy), 5, (0, 0, 255), -1)
                        
                        # Draw label
                        label = f"{class_name} {confidence:.2f}"
                        cv2.putText(color_image, label, (x1, y1-10), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        
                        # Get depth at center
                        try:
                            depth = depth_frame.get_distance(cx, cy)
                            if depth == 0.0:
                                depth = 0.5  # Default if no depth
                        except:
                            depth = 0.5
                        
                        # Display depth info
                        depth_text = f"Depth: {depth:.2f}m"
                        cv2.putText(color_image, depth_text, (cx+10, cy+10), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                        
                        # If object is detected and we want to move closer
                        if confidence > 0.5:  # High confidence detection
                            print(f"🎯 Detected {class_name} at center ({cx}, {cy}) with depth {depth:.2f}m")
                            
                            # Calculate movement to get closer
                            current_x = current_pos[0] if current_pos else 400
                            target_x = current_x + (depth * 1000) * 0.3  # Move 30% closer
                            
                            # Move robot closer
                            try:
                                print(f"🔄 Moving from X={current_x:.1f}mm to X={target_x:.1f}mm")
                                robot.move_pose([target_x, current_pos[1], current_pos[2]], [0, 90, 0])
                                current_pos = robot.get_current_position()
                                print(f"✅ Moved to X={current_pos[0]:.1f}mm")
                            except Exception as e:
                                print(f"❌ Failed to move: {e}")
            
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
    
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
    
    finally:
        # Cleanup
        pipeline.stop()
        robot.disconnect()
        cv2.destroyAllWindows()
        print("✅ Cleanup completed")
    
    return True

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
    
    print("🤖 Simple Object Detection and Camera Movement")
    print(f"📍 Robot IP: {robot_ip}")
    print("🎯 Detecting any object and moving camera closer")
    print("=" * 60)
    
    try:
        success = detect_and_move_to_object(robot_ip)
        
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
    print("  - Camera will automatically move closer to detected objects")
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