#!/usr/bin/env python3
"""
Complete vision system debug - test every component
"""

import sys
import os
import time
import json
import numpy as np

# Add the robot_control package to the path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def debug_vision_complete():
    """Complete debug of vision system."""
    
    print("🔍 COMPLETE VISION SYSTEM DEBUG")
    print("=" * 50)
    
    try:
        # Test 1: Import all components
        print("\n📋 Test 1: Importing all components...")
        try:
            from ultralytics import YOLO
            print("✅ YOLO imported")
        except Exception as e:
            print(f"❌ YOLO import failed: {e}")
            return
            
        try:
            import pyrealsense2 as rs
            print("✅ RealSense imported")
        except Exception as e:
            print(f"❌ RealSense import failed: {e}")
            return
            
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import String
            print("✅ ROS2 imported")
        except Exception as e:
            print(f"❌ ROS2 import failed: {e}")
            return
        
        # Test 2: Initialize RealSense
        print("\n📋 Test 2: Initializing RealSense...")
        try:
            pipeline = rs.pipeline()
            cfg = rs.config()
            cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            pipeline.start(cfg)
            print("✅ RealSense pipeline started")
        except Exception as e:
            print(f"❌ RealSense initialization failed: {e}")
            return
        
        # Test 3: Initialize YOLO
        print("\n📋 Test 3: Initializing YOLO...")
        try:
            model = YOLO("yolov8n.pt")
            print("✅ YOLO model loaded")
            print(f"📊 Available classes: {len(model.names)}")
        except Exception as e:
            print(f"❌ YOLO initialization failed: {e}")
            return
        
        # Test 4: Test camera frames
        print("\n📋 Test 4: Testing camera frames...")
        try:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if color_frame and depth_frame:
                color_image = np.asanyarray(color_frame.get_data())
                print(f"✅ Camera frames received: {color_image.shape}")
                print(f"📸 Image stats - min: {color_image.min()}, max: {color_image.max()}, mean: {color_image.mean():.1f}")
            else:
                print("❌ No camera frames")
                return
        except Exception as e:
            print(f"❌ Camera frame test failed: {e}")
            return
        
        # Test 5: Test YOLO detection
        print("\n📋 Test 5: Testing YOLO detection...")
        try:
            # Try GPU first, fallback to CPU
            try:
                results = model(color_image, device='cuda')
                print("🚀 Using GPU for YOLO")
            except Exception as gpu_error:
                print(f"⚠️ GPU failed: {gpu_error}")
                results = model(color_image, device='cpu')
                print("🖥️ Using CPU for YOLO")
            
            if results and len(results) > 0:
                result = results[0]
                if result.boxes is not None and len(result.boxes) > 0:
                    print(f"🎯 Detected {len(result.boxes)} objects:")
                    for i, box in enumerate(result.boxes):
                        cls_id = int(box.cls[0])
                        conf = float(box.conf[0])
                        class_name = model.names[cls_id]
                        print(f"  {i+1}. {class_name} (confidence: {conf:.2f})")
                        
                        # Check if it's a bottle
                        if class_name == 'bottle':
                            print(f"    🥤 BOTTLE DETECTED! Confidence: {conf:.2f}")
                else:
                    print("❌ No objects detected in frame")
            else:
                print("❌ No YOLO results")
        except Exception as e:
            print(f"❌ YOLO detection failed: {e}")
            return
        
        # Test 6: Test ROS2 publisher
        print("\n📋 Test 6: Testing ROS2 publisher...")
        try:
            rclpy.init()
            node = Node('vision_debug')
            publisher = node.create_publisher(String, '/detected_objects', 10)
            print("✅ ROS2 publisher created")
            
            # Create a test message
            test_detections = [
                {"class": "bottle", "pos": [0.5, 0.0, 0.3], "conf": 0.8}
            ]
            test_payload = {
                "t": time.time(),
                "units": "m", 
                "items": test_detections
            }
            test_msg = String(data=json.dumps(test_payload))
            
            # Publish test message
            publisher.publish(test_msg)
            print("✅ Test message published")
            
            # Wait a moment
            time.sleep(1)
            
            node.destroy_node()
            rclpy.shutdown()
            print("✅ ROS2 test completed")
            
        except Exception as e:
            print(f"❌ ROS2 test failed: {e}")
        
        # Test 7: Test complete vision pipeline
        print("\n📋 Test 7: Testing complete vision pipeline...")
        try:
            from robot_control.vision_system.pose_recorder import PoseRecorder
            
            print("🚀 Starting complete vision pipeline...")
            recorder = PoseRecorder()
            
            # Run for 10 seconds
            print("⏳ Running vision pipeline for 10 seconds...")
            start_time = time.time()
            detection_count = 0
            
            while time.time() - start_time < 10:
                try:
                    # Call scan_and_publish directly
                    recorder.scan_and_publish()
                    detection_count += 1
                    print(f"📡 Detection cycle {detection_count}")
                    time.sleep(1)
                except Exception as e:
                    print(f"❌ Detection cycle failed: {e}")
                    break
            
            print(f"✅ Vision pipeline completed: {detection_count} cycles")
            
        except Exception as e:
            print(f"❌ Complete vision pipeline failed: {e}")
        
        # Test 8: Test ROS2 topic subscription
        print("\n📋 Test 8: Testing ROS2 topic subscription...")
        try:
            rclpy.init()
            node = Node('subscription_test')
            received_messages = []
            
            def callback(msg):
                received_messages.append(msg.data)
                print(f"📡 Received: {msg.data}")
            
            subscription = node.create_subscription(String, '/detected_objects', callback, 10)
            print("✅ ROS2 subscription created")
            
            # Listen for 5 seconds
            print("⏳ Listening for 5 seconds...")
            start_time = time.time()
            while time.time() - start_time < 5:
                rclpy.spin_once(node, timeout_sec=0.1)
            
            if received_messages:
                print(f"✅ Received {len(received_messages)} messages")
                for i, msg in enumerate(received_messages[-2:]):  # Show last 2
                    print(f"  Message {i+1}: {msg}")
            else:
                print("❌ No messages received")
                
            node.destroy_node()
            rclpy.shutdown()
            
        except Exception as e:
            print(f"❌ ROS2 subscription test failed: {e}")
        
        # Cleanup
        try:
            pipeline.stop()
            print("\n🔌 RealSense pipeline stopped")
        except:
            pass
        
        print("\n✅ COMPLETE VISION DEBUG FINISHED")
        print("=" * 50)
        
    except Exception as e:
        print(f"❌ Complete debug failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    debug_vision_complete() 