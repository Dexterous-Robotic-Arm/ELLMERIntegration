#!/usr/bin/env python3
"""
Debug vision system to see what's being detected
"""

import sys
import os
import time
import json

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
            model = YOLO("yolov8n.pt")
            print("✅ YOLO model loaded successfully")
        except Exception as e:
            print(f"❌ YOLO not available: {e}")
            return
        
        # Test 3: Check if RealSense is available
        print("\n📋 Test 3: Checking RealSense availability...")
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
                color_image = color_frame.get_data()
                print(f"📸 Color frame size: {len(color_image)} bytes")
            else:
                print("❌ No camera frames received")
                return
                
        except Exception as e:
            print(f"❌ RealSense not available: {e}")
            return
        
        # Test 4: Test YOLO detection on a simple image
        print("\n📋 Test 4: Testing YOLO detection...")
        try:
            import numpy as np
            # Create a simple test image (black image)
            test_image = np.zeros((480, 640, 3), dtype=np.uint8)
            results = model(test_image)
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
                        
        except Exception as e:
            print(f"❌ YOLO detection test failed: {e}")
            return
        
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