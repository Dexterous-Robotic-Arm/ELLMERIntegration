# Testing Guide for Robot Control System

This guide provides comprehensive testing procedures for the robot control system on Ubuntu 22.04 with real hardware.

## 🛠️ Prerequisites

Before running tests, ensure you have:

- ✅ **Ubuntu 22.04 LTS** installed
- ✅ **ROS2 Humble** installed and sourced
- ✅ **RealSense D435/D435i** camera connected
- ✅ **ufactory850 robot** powered and connected to network
- ✅ **Python virtual environment** activated
- ✅ **All dependencies** installed

## 🚀 Quick Test Setup

### 1. Environment Setup
```bash
# Set environment variables
export GEMINI_API_KEY="your-gemini-api-key"
export XARM_IP="192.168.1.241"

# Setup environment
source setup_env.sh

# Verify setup
./quick_start.sh
```

### 2. Basic System Test
```bash
# Run basic structure tests
python3 tests/test_basic_structure.py

# Test robot connection
python3 -c "
from robot_control.robot_controller import XArmRunner
runner = XArmRunner()
print('Robot connected:', runner.connect())
runner.disconnect()
"
```

## 🔍 Component Testing

### Robot Controller Testing

#### 1. Basic Connection Test
```bash
python3 -c "
from robot_control.robot_controller import XArmRunner
import time

runner = XArmRunner()
print('Connecting to robot...')
if runner.connect():
    print('✅ Robot connected successfully')
    
    print('Enabling robot...')
    if runner.enable():
        print('✅ Robot enabled successfully')
        
        # Test basic movement
        print('Testing home position...')
        if runner.move_to_position([0, 0, 0, 0, 0, 0]):
            print('✅ Movement successful')
        
        runner.disable()
    else:
        print('❌ Failed to enable robot')
    
    runner.disconnect()
else:
    print('❌ Failed to connect to robot')
"
```

#### 2. Gripper Testing
```bash
python3 -c "
from robot_control.robot_controller import XArmRunner
import time

runner = XArmRunner()
if runner.connect() and runner.enable():
    print('Testing gripper...')
    
    # Test gripper open
    print('Opening gripper...')
    if runner.open_gripper():
        print('✅ Gripper opened')
    
    time.sleep(1)
    
    # Test gripper close
    print('Closing gripper...')
    if runner.close_gripper():
        print('✅ Gripper closed')
    
    runner.disable()
    runner.disconnect()
"
```

#### 3. Safety Testing
```bash
python3 -c "
from robot_control.robot_controller import XArmRunner

runner = XArmRunner()
if runner.connect() and runner.enable():
    print('Testing safety features...')
    
    # Test emergency stop
    print('Testing emergency stop...')
    runner.emergency_stop()
    print('✅ Emergency stop activated')
    
    # Test error reset
    print('Resetting error state...')
    if runner.reset_error():
        print('✅ Error state reset')
    
    runner.disconnect()
"
```

### Vision System Testing

#### 1. Camera Connection Test
```bash
python3 -c "
import pyrealsense2 as rs
import numpy as np

try:
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    # Start streaming
    profile = pipeline.start(config)
    print('✅ RealSense camera connected successfully')
    
    # Get a frame
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    
    if not depth_frame or not color_frame:
        print('❌ Failed to get frames')
    else:
        print('✅ Camera frames received successfully')
    
    pipeline.stop()
    
except Exception as e:
    print(f'❌ Camera test failed: {e}')
"
```

#### 2. Object Detection Test
```bash
# Test YOLO object detection
python3 -c "
from ultralytics import YOLO
import cv2
import numpy as np

# Load model
model = YOLO('yolov8n.pt')

# Create test image
test_image = np.zeros((480, 640, 3), dtype=np.uint8)
test_image[:] = (128, 128, 128)  # Gray background

# Test detection
results = model(test_image)
print(f'✅ YOLO model loaded successfully')
print(f'Detection test completed with {len(results)} results')
"
```

#### 3. Vision System Integration Test
```bash
# Test the complete vision system
python3 robot_control/vision_system/pose_recorder.py --test-mode
```

### Task Planner Testing

#### 1. LLM Connection Test
```bash
python3 -c "
import google.generativeai as genai
import os

api_key = os.getenv('GEMINI_API_KEY')
if not api_key:
    print('❌ GEMINI_API_KEY not set')
    exit(1)

try:
    genai.configure(api_key=api_key)
    model = genai.GenerativeModel('gemini-pro')
    
    # Test simple query
    response = model.generate_content('Hello, can you help me plan a robot task?')
    print('✅ LLM connection successful')
    print(f'Response: {response.text[:100]}...')
    
except Exception as e:
    print(f'❌ LLM test failed: {e}')
"
```

#### 2. Task Planning Test
```bash
python3 -c "
from robot_control.rag_system.planner.rag_planner import RAGPlanner

# Create RAG planner in simulation mode
planner = RAGPlanner(
    robot_controller=None,
    vision_system=None,
    config_path='config/',
    max_retries=3,
    scan_timeout=10.0
)

# Test simple task planning
task = 'pick up the red cup'
try:
    context = planner._create_rag_context(task)
    plan = planner._generate_plan_with_llm(context)
    print('✅ Task planning successful')
    print(f'Generated plan with {len(plan.get(\"steps\", []))} steps')
except Exception as e:
    print(f'❌ Task planning failed: {e}')
"
```

## 🧪 Integration Testing

### 1. Full System Test (Dry Run)
```bash
# Test complete system without executing movements
python3 robot_control/main.py --task "pick up the red cup" --dry-run
```

### 2. Simple Movement Test
```bash
# Test basic robot movement
python3 robot_control/main.py --task "move to home position"
```

### 3. Vision Integration Test
```bash
# Test vision with robot
python3 robot_control/main.py --task "detect objects in workspace" --wait-detections
```

### 4. Complete Pick and Place Test
```bash
# Test full pick and place operation
python3 robot_control/main.py --task "pick up the blue block and place it on the table"
```

## 🔧 Advanced Testing

### 1. Performance Testing
```bash
# Test system performance
python3 -c "
import time
from robot_control.robot_controller import XArmRunner

runner = XArmRunner()
start_time = time.time()

if runner.connect() and runner.enable():
    # Test multiple movements
    for i in range(5):
        runner.move_to_position([0, 0, 0, 0, 0, 0])
        time.sleep(0.5)
    
    end_time = time.time()
    print(f'Performance test completed in {end_time - start_time:.2f} seconds')
    
    runner.disable()
    runner.disconnect()
"
```

### 2. Safety Limit Testing
```bash
# Test safety limits
python3 -c "
from robot_control.robot_controller import XArmRunner

runner = XArmRunner()
if runner.connect() and runner.enable():
    # Test workspace limits
    print('Testing workspace limits...')
    
    # Try to move outside workspace
    result = runner.move_to_pose([1000, 1000, 1000, 0, 0, 0])
    if not result:
        print('✅ Safety limits working correctly')
    else:
        print('❌ Safety limits failed')
    
    runner.disable()
    runner.disconnect()
"
```

### 3. Error Recovery Testing
```bash
# Test error recovery
python3 -c "
from robot_control.robot_controller import XArmRunner
import time

runner = XArmRunner()
if runner.connect() and runner.enable():
    print('Testing error recovery...')
    
    # Trigger emergency stop
    runner.emergency_stop()
    print('Emergency stop activated')
    
    # Wait and reset
    time.sleep(2)
    if runner.reset_error():
        print('✅ Error recovery successful')
    else:
        print('❌ Error recovery failed')
    
    runner.disable()
    runner.disconnect()
"
```

## 🐛 Troubleshooting Tests

### 1. Network Connectivity Test
```bash
# Test robot network connectivity
ping -c 4 192.168.1.241

# Test robot port
telnet 192.168.1.241 8080
```

### 2. USB Device Test
```bash
# Check RealSense camera
lsusb | grep RealSense

# Test RealSense viewer
realsense-viewer
```

### 3. ROS2 Test
```bash
# Test ROS2 installation
ros2 --version

# Test ROS2 topics
ros2 topic list

# Test ROS2 nodes
ros2 node list
```

### 4. Python Environment Test
```bash
# Test Python packages
python3 -c "
import sys
print(f'Python version: {sys.version}')

packages = [
    'xarm', 'pyrealsense2', 'rclpy', 'google.generativeai',
    'ultralytics', 'opencv-python', 'numpy', 'yaml'
]

for package in packages:
    try:
        __import__(package)
        print(f'✅ {package}')
    except ImportError:
        print(f'❌ {package}')
"
```

## 📊 Test Results

### Expected Test Outcomes

| Test | Expected Result |
|------|----------------|
| Robot Connection | ✅ Connected successfully |
| Robot Movement | ✅ Movement completed |
| Gripper Control | ✅ Gripper responds |
| Camera Connection | ✅ Frames received |
| Object Detection | ✅ Objects detected |
| Task Planning | ✅ Plan generated |
| Safety Limits | ✅ Limits enforced |
| Error Recovery | ✅ Recovery successful |

### Test Checklist

- [ ] **Environment Setup**: All dependencies installed
- [ ] **Robot Connection**: Can connect to ufactory850
- [ ] **Robot Movement**: Can move to positions and poses
- [ ] **Gripper Control**: Can open/close gripper
- [ ] **Camera Connection**: RealSense camera working
- [ ] **Object Detection**: YOLO model detecting objects
- [ ] **Task Planning**: LLM generating plans
- [ ] **Safety Features**: Limits and emergency stop working
- [ ] **Integration**: Full system working together

## 🚨 Safety Reminders

### Before Testing
1. **Clear Workspace**: Remove all objects from robot workspace
2. **Check Safety**: Ensure emergency stop is accessible
3. **Verify Limits**: Confirm safety limits are properly set
4. **Test Emergency Stop**: Verify emergency stop functionality

### During Testing
1. **Monitor Robot**: Keep visual contact with robot
2. **Stay Alert**: Be ready to use emergency stop
3. **Check Limits**: Verify robot stays within workspace
4. **Log Issues**: Document any problems encountered

### After Testing
1. **Return to Home**: Move robot to safe home position
2. **Disable Robot**: Properly disable robot control
3. **Document Results**: Record test outcomes
4. **Clean Up**: Remove any test objects

## 📝 Test Reporting

### Test Report Template
```
Test Date: _______________
Tester: _________________
System Version: __________

Test Results:
□ Robot Connection: Pass/Fail
□ Basic Movement: Pass/Fail
□ Gripper Control: Pass/Fail
□ Vision System: Pass/Fail
□ Task Planning: Pass/Fail
□ Safety Features: Pass/Fail
□ Integration: Pass/Fail

Issues Found:
- Issue 1: Description and resolution
- Issue 2: Description and resolution

Notes:
- Additional observations
- Recommendations
```

---

**🎯 Use this guide to ensure your robot control system is working correctly before deploying to production!**