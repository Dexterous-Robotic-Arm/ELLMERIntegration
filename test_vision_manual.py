#!/usr/bin/env python3
"""
Manual test of vision system - run it directly and see what it detects
"""

import sys
import os
import time
import json

# Add the robot_control package to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def test_vision_manual():
    """Manually test the vision system."""
    
    print("🔍 Manual Vision System Test")
    print("📷 Running vision system directly to see detections")
    print("🥫 Place a can in front of the camera")
    
    try:
        # Import and run the vision system
        from robot_control.vision_system.pose_recorder import PoseRecorder
        
        print("\n📋 Starting vision system...")
        recorder = PoseRecorder()
        
        print("⏳ Running vision system for 30 seconds...")
        print("📡 Check for detection messages below:")
        
        # Run for 30 seconds
        start_time = time.time()
        while time.time() - start_time < 30:
            try:
                # Call the scan_and_publish method directly
                recorder.scan_and_publish()
                time.sleep(1)  # Wait 1 second between detections
            except Exception as e:
                print(f"❌ Error in vision system: {e}")
                break
        
        print("\n✅ Manual vision test completed!")
        
    except Exception as e:
        print(f"❌ Error during manual vision test: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_vision_manual() 