#!/usr/bin/env python3
"""
Test if vision system is publishing to ROS2 topic
"""

import sys
import os
import time
import json
import subprocess
import threading

# Add the robot_control package to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def test_vision_publishing():
    """Test if vision system is publishing to ROS2 topic."""
    
    print("🔍 Testing Vision System Publishing")
    print("=" * 40)
    
    try:
        # Start vision system in background
        print("🚀 Starting vision system...")
        vision_process = subprocess.Popen([
            sys.executable, "robot_control/vision_system/pose_recorder.py"
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        print("⏳ Waiting for vision system to start...")
        time.sleep(5)
        
        # Test ROS2 topic subscription
        print("📡 Testing ROS2 topic subscription...")
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import String
            
            rclpy.init()
            node = Node('vision_test')
            received_messages = []
            
            def callback(msg):
                received_messages.append(msg.data)
                print(f"📡 Received detection: {msg.data}")
            
            subscription = node.create_subscription(String, '/detected_objects', callback, 10)
            print("✅ ROS2 subscription created")
            
            # Listen for 15 seconds
            print("⏳ Listening for 15 seconds...")
            print("🥫 Place a bottle in front of the camera!")
            
            start_time = time.time()
            while time.time() - start_time < 15:
                rclpy.spin_once(node, timeout_sec=0.1)
            
            if received_messages:
                print(f"✅ Received {len(received_messages)} detection messages")
                print("📋 Last 3 messages:")
                for i, msg in enumerate(received_messages[-3:]):
                    print(f"  {i+1}. {msg}")
            else:
                print("❌ No detection messages received")
                print("🔍 This means the vision system is not publishing to /detected_objects")
                
            node.destroy_node()
            rclpy.shutdown()
            
        except Exception as e:
            print(f"❌ ROS2 test failed: {e}")
        
        # Stop vision system
        print("🛑 Stopping vision system...")
        vision_process.terminate()
        vision_process.wait(timeout=5)
        print("✅ Vision system stopped")
        
        print("\n✅ Vision publishing test completed!")
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_vision_publishing() 