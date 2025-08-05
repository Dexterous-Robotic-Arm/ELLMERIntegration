#!/usr/bin/env python3
"""
Test bottle detection during scanning
"""

import sys
import os
import time
import subprocess

# Add the robot_control package to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def test_bottle_detection():
    """Test bottle detection during scanning."""
    
    print("🥤 Testing Bottle Detection During Scanning")
    print("=" * 50)
    print("🎯 Place a bottle in front of the robot")
    print("📷 Camera should detect it during scan")
    
    try:
        # Start vision system
        print("\n🚀 Starting vision system...")
        vision_process = subprocess.Popen([
            sys.executable, "robot_control/vision_system/pose_recorder.py"
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        print("⏳ Waiting for vision system to start...")
        time.sleep(5)
        
        # Test ROS2 topic for bottle detection
        print("\n📡 Testing bottle detection...")
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import String
            import json
            
            rclpy.init()
            node = Node('bottle_test')
            received_messages = []
            bottle_detected = False
            
            def callback(msg):
                nonlocal bottle_detected
                received_messages.append(msg.data)
                try:
                    data = json.loads(msg.data)
                    items = data.get('items', [])
                    for item in items:
                        if item.get('class') == 'bottle':
                            bottle_detected = True
                            print(f"🥤 BOTTLE DETECTED! Position: {item.get('pos')}, Confidence: {item.get('conf')}")
                except:
                    pass
                print(f"📡 Detection: {msg.data}")
            
            subscription = node.create_subscription(String, '/detected_objects', callback, 10)
            print("✅ ROS2 subscription created")
            
            # Listen for 20 seconds
            print("⏳ Listening for 20 seconds...")
            print("🤖 Robot will scan during this time")
            
            start_time = time.time()
            while time.time() - start_time < 20:
                rclpy.spin_once(node, timeout_sec=0.1)
            
            if bottle_detected:
                print("\n✅ SUCCESS: Bottle detected during scanning!")
            else:
                print("\n❌ No bottle detected during scanning")
                print("🔍 Check if bottle is visible to camera")
            
            if received_messages:
                print(f"📊 Total detection messages: {len(received_messages)}")
            else:
                print("❌ No detection messages received")
                
            node.destroy_node()
            rclpy.shutdown()
            
        except Exception as e:
            print(f"❌ ROS2 test failed: {e}")
        
        # Stop vision system
        print("\n🛑 Stopping vision system...")
        vision_process.terminate()
        try:
            vision_process.wait(timeout=2)
            print("✅ Vision system stopped")
        except:
            vision_process.kill()
            vision_process.wait(timeout=1)
            print("✅ Vision system force stopped")
        
        print("\n✅ Bottle detection test completed!")
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_bottle_detection() 