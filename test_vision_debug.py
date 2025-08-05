#!/usr/bin/env python3
"""
Debug vision system to see what's being detected
"""

import sys
import os
import time
import json
import numpy as np
# Add the robot_control package to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def test_vision_debug():
    """Test vision system and see what's being detected."""
    
    print("🔍 Debugging Vision System")
    print("📷 Checking YOLO + RealSense detection")
    
    try:
        # Test 1: Check if vision system can be imported
        print("\n📋 Test 1: Importing vision system...")
        try:
            from robot_control.vision_system.pose_recorder import PoseRecorder
            print("✅ Vision system imported successfully")
        except Exception as e:
            print(f"❌ Failed to import vision system: {e}")
            return
        
        # Test 2: Check if YOLO is available
        print("\n📋 Test 2: Checking YOLO availability...")
        try:
            from ultralytics import YOLO
            # Force CPU usage to avoid CUDA compatibility issues
            model = YOLO("yolov8n.pt")
            print("✅ YOLO model loaded successfully")
            print("🖥️ Using CPU for inference (GPU compatibility issue)")
        except Exception as e:
            print(f"❌ YOLO not available: {e}")
            return
        
        # Test 3: Check if RealSense is available
        print("\n📋 Test 3: Checking RealSense availability...")
        pipeline = None
        try:
            import pyrealsense2 as rs
            pipeline = rs.pipeline()
            cfg = rs.config()
            cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            pipeline.start(cfg)
            print("✅ RealSense camera connected successfully")
            
            # Test camera frames
            print("📷 Testing camera frames...")
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if color_frame and depth_frame:
                print("✅ Camera frames received successfully")
                color_image = np.asanyarray(color_frame.get_data())
                print(f"📸 Color frame shape: {color_image.shape}")
                print(f"📸 Color frame dtype: {color_image.dtype}")
            else:
                print("❌ No camera frames received")
                return
                
        except Exception as e:
            print(f"❌ RealSense not available: {e}")
            return
        
        # Test 4: Test YOLO detection on camera image
        print("\n📋 Test 4: Testing YOLO detection on camera image...")
        try:
            import numpy as np
            # Get a real camera frame for testing
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())
            
            print(f"📸 Testing YOLO on camera image: {color_image.shape}")
            # Try GPU first, fallback to CPU if CUDA issues
            try:
                results = model(color_image, device='cuda')
                print("🚀 Using GPU for YOLO inference")
            except Exception as gpu_error:
                print(f"⚠️ GPU failed: {gpu_error}")
                print("🖥️ Falling back to CPU")
                results = model(color_image, device='cpu')
            print("✅ YOLO detection test successful")
            print(f"📊 YOLO classes available: {len(model.names)}")
            print(f"📋 Sample classes: {list(model.names.values())[:10]}")
            
            # Check if 'can' is in the class names
            if 'can' in model.names.values():
                print("✅ 'can' class found in YOLO model")
            else:
                print("❌ 'can' class NOT found in YOLO model")
                print("🔍 Available classes that might work:")
                for i, name in model.names.items():
                    if any(word in name.lower() for word in ['bottle', 'cup', 'container', 'object']):
                        print(f"  - {name} (class {i})")
            
            # Show what was detected in the current frame
            if results and len(results) > 0:
                result = results[0]
                if result.boxes is not None and len(result.boxes) > 0:
                    print(f"🎯 Detected {len(result.boxes)} objects in current frame:")
                    for box in result.boxes:
                        cls_id = int(box.cls[0])
                        conf = float(box.conf[0])
                        class_name = model.names[cls_id]
                        print(f"  - {class_name} (confidence: {conf:.2f})")
                else:
                    print("❌ No objects detected in current frame")
            else:
                print("❌ No detection results")
                        
        except Exception as e:
            print(f"❌ YOLO detection test failed: {e}")
            return
        finally:
            # Clean up RealSense
            if pipeline:
                try:
                    pipeline.stop()
                    print("🔌 RealSense pipeline stopped")
                except:
                    pass
        
        # Test 5: Test ROS2 topic subscription
        print("\n📋 Test 5: Testing ROS2 topic subscription...")
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import String
            
            rclpy.init()
            node = Node('vision_debug')
            received_messages = []
            
            def callback(msg):
                received_messages.append(msg.data)
                print(f"📡 Received detection: {msg.data}")
            
            subscription = node.create_subscription(String, '/detected_objects', callback, 10)
            
            print("✅ ROS2 topic subscription created")
            print("⏳ Listening for 10 seconds...")
            
            start_time = time.time()
            while time.time() - start_time < 10:
                rclpy.spin_once(node, timeout_sec=0.1)
            
            if received_messages:
                print(f"✅ Received {len(received_messages)} detection messages")
                for i, msg in enumerate(received_messages[-3:]):  # Show last 3
                    print(f"  Message {i+1}: {msg}")
            else:
                print("❌ No detection messages received")
                
            node.destroy_node()
            rclpy.shutdown()
            
        except Exception as e:
            print(f"❌ ROS2 test failed: {e}")
        
        print("\n✅ Vision debug test completed!")
        
    except Exception as e:
        print(f"❌ Error during vision debug: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_vision_debug() 