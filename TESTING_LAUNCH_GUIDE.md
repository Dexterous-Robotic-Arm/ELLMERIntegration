# Testing Launch Guide for ufactory850 Robot

This guide provides step-by-step instructions for safely testing the ufactory850 robot with RealSense D435 camera and enhanced gripper control.

## Table of Contents
- [Prerequisites](#prerequisites)
- [Safety Preparation](#safety-preparation)
- [Environment Setup](#environment-setup)
- [Complete System Launch](#complete-system-launch)
- [Basic Testing](#basic-testing)
- [Advanced Testing](#advanced-testing)
- [Troubleshooting](#troubleshooting)
- [Safety Guidelines](#safety-guidelines)

## Prerequisites

### Hardware Requirements
- ✅ ufactory850 robot arm
- ✅ Intel RealSense D435 camera
- ✅ Gripper (single servo-controlled)
- ✅ Emergency stop button
- ✅ Clear workspace 
- ✅ Network connection to robot 

### Software Requirements
- ✅ ROS2 (Humble or later)
- ✅ Python 3.8+
- ✅ XArm Python SDK
- ✅ Google Gemini API key
- ✅ YOLO model file (yolov8n.pt)

## Safety Preparation

### ⚠️ Critical Safety Checklist
Before any testing, ensure:

- [ ] **Emergency stop button** is within immediate reach
- [ ] **Workspace is clear** of obstacles and fragile items
- [ ] **Robot is powered on** and in ready state
- [ ] **Camera is connected** and functioning
- [ ] **Gripper is attached** and operational
- [ ] **Someone is monitoring** the robot at all times
- [ ] **Safety limits are configured** for your workspace

### 🚨 Emergency Procedures
- **Immediate Stop**: Press emergency stop button on robot
- **Software Stop**: Use `Ctrl+C` to stop execution
- **Emergency Stop**: Call `runner.trigger_emergency_stop()` in code
- **Power Off**: If needed, power off robot immediately

## Environment Setup

### 1. Install Dependencies
```bash
# Install Python packages
pip install xarm-python-sdk google-generativeai pyyaml rclpy ultralytics pyrealsense2

# Download YOLO model
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt
```

### 2. Build ROS Messages
```bash
# Navigate to ros_xarm directory
cd ros_xarm

# Build custom messages
colcon build --packages-select ufactory_ellmer_msgs

# Source the workspace
source install/setup.bash
```

### 3. Configure Environment Variables (Optional)
```bash
# Set Gemini API key (required)
export GEMINI_API_KEY="your_api_key_here"

# Set world configuration (optional)
export WORLD_YAML="arm_config/world_model.yaml"

# Set custom robot IP (optional - defaults to 192.168.1.241)
export XARM_IP="192.168.1.241"
```

### 4. Configure Safety Limits
Edit `arm_config/safety_config.yaml` for your workspace:

```yaml
# Adjust these values for your specific setup
workspace_limits:
  x_min: 150    # Front limit (mm)
  x_max: 650    # Back limit (mm)
  y_min: -300   # Left limit (mm)
  y_max: 300    # Right limit (mm)
  z_min: 50     # Down limit (mm)
  z_max: 500    # Up limit (mm)

speed_limits:
  max_cartesian_speed: 150  # Conservative speed limit
  max_joint_speed: 30       # Conservative joint speed
```

## Complete System Launch

### 🚀 Full Autonomous System Launch

The complete system integrates LLM planning, computer vision, and robot control for autonomous task execution.

#### 1. Complete System Architecture
```
User Task Input → LLM Planning → Vision Detection → Robot Execution → Task Completion
     ↓              ↓              ↓              ↓              ↓
Natural Language → Action Plan → Object Detection → Movement Control → Success/Failure
```

#### 2. Launch Complete Autonomous System
```bash
# Navigate to project root
cd ~/ELLMERIntegration

# Launch the complete autonomous system (uses default IP 192.168.1.241)
python main.py --task "pick up the red cup and place it in the bin"
```

#### 3. System Components (Automatically Launched)
When you run the main command, the system automatically:

1. **Robot Connection**: Connects to robot at 192.168.1.241 (or custom IP)
2. **LLM Planning**: Generates action plan using Gemini AI
3. **Vision System**: Starts RealSense camera and YOLO detection
4. **Robot Control**: Connects to ufactory850 and initializes gripper
5. **Execution Engine**: Executes the complete task autonomously
6. **Safety Monitoring**: Continuously monitors safety limits

#### 4. Complete Autonomous Task Examples

##### Basic Pick and Place
```bash
# Complete autonomous pick and place (uses default IP)
python main.py --task "pick up the cup and place it in the bin"

# System will:
# 1. Connect to robot at 192.168.1.241
# 2. Generate plan: [approach cup, grasp, lift, move to bin, release, return home]
# 3. Detect cup using RealSense + YOLO
# 4. Execute movements with gripper control
# 5. Complete task autonomously
```

##### Complex Multi-Step Tasks
```bash
# Complex autonomous task (uses default IP)
python main.py --task "find the red cup, pick it up, move it to the table, then find the blue bottle and place it next to the cup"

# System will:
# 1. Connect to robot at 192.168.1.241
# 2. Plan multi-step sequence
# 3. Detect multiple objects
# 4. Execute complex pick and place operations
# 5. Handle object detection and positioning
```

##### Interactive Autonomous Mode
```bash
# Interactive autonomous mode (uses default IP)
python main.py --interactive

# Enter tasks one by one:
# Task: "pick up the cup"
# Task: "move it to the left"
# Task: "release it gently"
# Task: "return to home position"
```

##### Continuous Autonomous Operation
```bash
# Continuous autonomous operation (uses default IP)
python main.py --loop

# System will:
# 1. Connect to robot at 192.168.1.241
# 2. Accept continuous task inputs
# 3. Execute each task autonomously
# 4. Wait for next task
# 5. Continue until stopped
```

#### 5. Autonomous System Features

##### Simple Robot Connection
- **Default IP**: Uses 192.168.1.241 by default
- **Environment Override**: Uses XARM_IP if set
- **Manual Override**: Use --ip flag for custom IP
- **Connection Testing**: Verifies robot connectivity before execution
- **Error Handling**: Clear error messages if connection fails

##### Automatic Component Management
- **Vision System**: Automatically starts RealSense camera and YOLO detection
- **Robot Control**: Automatically connects to robot and initializes gripper
- **Safety Systems**: Automatically enables all safety limits and monitoring
- **Error Recovery**: Automatically handles errors and retries when safe

##### Real-Time Object Detection
- **Continuous Detection**: RealSense camera continuously scans workspace
- **Object Recognition**: YOLO identifies objects (cups, bottles, etc.)
- **Pose Estimation**: Calculates 3D positions of detected objects
- **Dynamic Updates**: Updates object positions in real-time

##### Intelligent Planning
- **Natural Language Understanding**: Converts user commands to robot actions
- **Task Decomposition**: Breaks complex tasks into simple steps
- **Safety Validation**: Ensures all planned actions are safe
- **Error Handling**: Plans include error recovery steps

##### Autonomous Execution
- **Sequential Execution**: Executes planned steps in order
- **Real-Time Monitoring**: Monitors execution progress and safety
- **Adaptive Behavior**: Adjusts based on object detection results
- **Completion Verification**: Confirms task completion

#### 6. Complete System Launch Options

##### Standard Autonomous Launch
```bash
# Basic autonomous task execution (uses default IP)
python main.py --task "YOUR_TASK_DESCRIPTION"
```

##### Autonomous Launch with Custom Configuration
```bash
# Custom world model
python main.py --task "pick up the cup" --world arm_config/my_workspace.yaml

# Custom action schema
python main.py --task "grasp the object" --contract custom_actions.md

# Simulation mode (no real robot movement)
python main.py --task "test pick and place" --sim

# Dry run (plan only, no execution)
python main.py --task "complex task" --dry-run
```

##### Advanced Autonomous Launch
```bash
# With vision system optimization
python main.py --task "pick up the cup" --wait-detections 10 --min-detection-items 1

# With fallback planning
python main.py --task "move object" --use-fallback

# With custom robot IP
python main.py --task "pick up cup" --ip 192.168.1.200
```

#### 7. Autonomous System Monitoring

##### Real-Time Status Monitoring
```bash
# Monitor system status
ros2 topic echo /detected_objects

# Check robot status
python -c "
from ros_xarm.actions_xarm import XArmRunner
from main import get_robot_ip
ip = get_robot_ip()
runner = XArmRunner(ip)
print('Robot status:', runner.get_safety_status())
print('Gripper status:', runner.get_gripper_status())
"
```

##### Execution Progress Tracking
The system provides real-time feedback:
- `[Config] Using default robot IP: 192.168.1.241`
- `[Connection] Successfully connected to robot at 192.168.1.241`
- `[Plan] Step 1/5: MOVE_TO_NAMED home`
- `[Vision] Detected objects: [{'class': 'cup', 'pos': [400, 0, 45]}]`
- `[Gripper] Opened to position 850`
- `[Safety] Moving to [400, 0, 125] at speed 150`
- `[Plan] Done. Execution time: 45.2s, Steps completed: 5`

#### 8. Autonomous Task Examples

##### Simple Object Manipulation
```bash
# Pick up and move object (uses default IP)
python main.py --task "pick up the red cup and move it to the table"

# Expected autonomous execution:
# 1. Connect to robot at 192.168.1.241
# 2. Move to home position
# 3. Open gripper
# 4. Detect red cup using camera
# 5. Approach cup position
# 6. Grasp cup with gripper
# 7. Lift cup
# 8. Move to table position
# 9. Release cup
# 10. Return to home
```

##### Complex Multi-Object Tasks
```bash
# Sort multiple objects (uses default IP)
python main.py --task "pick up all cups and place them in the bin, then pick up all bottles and place them on the table"

# Expected autonomous execution:
# 1. Connect to robot at 192.168.1.241
# 2. Detect all cups and bottles
# 3. Plan sequence for each object
# 4. Execute pick and place for each cup
# 5. Execute pick and place for each bottle
# 6. Verify all objects are sorted
```

##### Precision Tasks
```bash
# Gentle handling (uses default IP)
python main.py --task "gently pick up the fragile glass and place it carefully in the box"

# Expected autonomous execution:
# 1. Connect to robot at 192.168.1.241
# 2. Use soft gripper settings
# 3. Approach with extra caution
# 4. Gentle grasp with low force
# 5. Slow, careful movement
# 6. Precise placement
```

#### 9. System Integration Verification

##### Verify Complete System
```bash
# Test complete autonomous pipeline (uses default IP)
python main.py --task "test complete system: detect cup, approach, grasp, lift, move, release, return home"

# Check all components are working:
# ✅ Robot connects to 192.168.1.241
# ✅ LLM planning generates valid plan
# ✅ Vision system detects objects
# ✅ Robot responds to commands
# ✅ Gripper operates correctly
# ✅ Safety systems are active
# ✅ Task completes successfully
```

##### System Health Check
```bash
# Comprehensive system check
python -c "
import sys
sys.path.append('.')

# Test robot connection
from main import get_robot_ip
ip = get_robot_ip()
print('✅ Robot IP:', ip)

# Test LLM planning
from llm_planning_gemini.custom_gemini_examples.planner_llm import plan_with_gemini
plan = plan_with_gemini('test task', [])
print('✅ LLM Planning:', 'Working' if plan else 'Failed')

# Test robot connection
from ros_xarm.actions_xarm import XArmRunner
runner = XArmRunner(ip)
status = runner.get_safety_status()
print('✅ Robot Control:', 'Working' if status else 'Failed')

# Test gripper
gripper_status = runner.get_gripper_status()
print('✅ Gripper:', 'Working' if gripper_status['enabled'] else 'Failed')

print('\\n🎯 Complete System Status: READY FOR AUTONOMOUS OPERATION')
"
```

## Basic Testing

### 1. Dry-Run Testing (No Robot Movement)
```bash
# Test plan generation without moving robot
python main.py --task "open and close the gripper" --dry-run

# Test object detection plan
python main.py --task "find and approach the red cup" --dry-run

# Test pick and place plan
python main.py --task "pick up the cup and place it in the bin" --dry-run
```

### 2. Basic Movement Testing
```bash
# Test robot connection and home position (uses default IP)
python main.py --task "move to home position"

# Test gripper basic operations (uses default IP)
python main.py --task "open gripper, close gripper, open gripper"

# Test gripper cycle (uses default IP)
python main.py --task "test gripper by cycling open and close 3 times"
```

### 3. Gripper Position Testing
```bash
# Test specific gripper positions (uses default IP)
python main.py --task "set gripper to half open position"

# Test gentle grasping (uses default IP)
python main.py --task "gently grasp with soft close"

# Test gripper release (uses default IP)
python main.py --task "release grasped object"
```

## Advanced Testing

### 4. Object Detection Testing
```bash
# Start vision system in background
python ros_xarm/pose_recorder.py &

# Test object detection and approach (uses default IP)
python main.py --task "approach the red cup" --wait-detections 10

# Test object interaction (uses default IP)
python main.py --task "touch the cup gently"
```

### 5. Pick and Place Testing
```bash
# Simple pick and place (uses default IP)
python main.py --task "pick up the cup and move it 10cm to the right"

# Full pick and place sequence (uses default IP)
python main.py --task "pick up the red cup and place it in the bin, then return home"

# Test with multiple objects (uses default IP)
python main.py --task "pick up the closest cup or bottle"
```

### 6. Interactive Testing
```bash
# Interactive mode for multiple tasks (uses default IP)
python main.py --interactive

# Loop mode for continuous testing (uses default IP)
python main.py --loop
```

### 7. Custom Task Testing
```bash
# Test specific movements (uses default IP)
python main.py --task "move to position x=400 y=0 z=200"

# Test complex sequences (uses default IP)
python main.py --task "approach cup, grasp gently, lift 50mm, move right 100mm, release"
```

## Safety Testing

### 8. Emergency Stop Testing
```python
# In Python console or script
from ros_xarm.actions_xarm import XArmRunner
from main import get_robot_ip

# Connect to robot (uses default IP)
ip = get_robot_ip()
runner = XArmRunner(ip)

# Test emergency stop
runner.trigger_emergency_stop()

# Clear emergency stop
runner.clear_emergency_stop()
```

### 9. Safety Limit Testing
```bash
# Test workspace limits (should fail safely, uses default IP)
python main.py --task "move to position x=1000 y=0 z=300"

# Test speed limits (uses default IP)
python main.py --task "move to home at high speed"
```

## Troubleshooting

### Common Issues and Solutions

#### Robot Connection Issues
```bash
# Check robot connectivity (uses default IP)
python -c "
from main import get_robot_ip, test_robot_connection
ip = get_robot_ip()
print('Using IP:', ip)
print('Connection test:', test_robot_connection(ip))
"

# Manual IP test
ping 192.168.1.241

# Test XArm SDK connection
python -c "
from xarm.wrapper import XArmAPI
from main import get_robot_ip
ip = get_robot_ip()
arm = XArmAPI(ip)
print('Connected:', arm.connected)
"
```

#### Vision System Issues
```bash
# Check ROS topics
ros2 topic list
ros2 topic echo /detected_objects

# Check camera connection
python -c "
import pyrealsense2 as rs
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)
print('Camera connected successfully')
"
```

#### Gripper Issues
```bash
# Check gripper status (uses default IP)
python -c "
from ros_xarm.actions_xarm import XArmRunner
from main import get_robot_ip
ip = get_robot_ip()
runner = XArmRunner(ip)
print('Gripper status:', runner.get_gripper_status())
"
```

### Debug Mode
```bash
# Enable verbose logging (uses default IP)
python main.py --task "test gripper" --verbose

# Test with fallback planner (uses default IP)
python main.py --task "move to home" --use-fallback

# Test without vision validation (uses default IP)
python main.py --task "pick up cup" --no-validate

# Use custom IP if needed
python main.py --task "test gripper" --ip 192.168.1.200
```

## Testing Progression

### Recommended Testing Order

1. **Environment Setup**
   - [ ] Install dependencies
   - [ ] Build ROS messages
   - [ ] Configure environment variables (optional)
   - [ ] Test robot connection

2. **Dry-Run Testing**
   - [ ] Test plan generation
   - [ ] Verify action schema
   - [ ] Check safety validation

3. **Basic Movement**
   - [ ] Test home position
   - [ ] Test simple movements
   - [ ] Verify workspace limits

4. **Gripper Testing**
   - [ ] Test open/close
   - [ ] Test position control
   - [ ] Test grasp operations

5. **Vision Testing**
   - [ ] Test camera connection
   - [ ] Test object detection
   - [ ] Test pose estimation

6. **Integration Testing**
   - [ ] Test pick and place
   - [ ] Test error handling
   - [ ] Test safety systems

7. **Advanced Testing**
   - [ ] Test complex tasks
   - [ ] Test edge cases
   - [ ] Test performance

8. **Complete Autonomous System**
   - [ ] Test full autonomous pipeline
   - [ ] Test multi-step tasks
   - [ ] Test interactive mode
   - [ ] Test continuous operation

### Success Criteria

- ✅ **Robot Connection**: Connects to 192.168.1.241 successfully
- ✅ **Robot Response**: Robot responds to commands
- ✅ **Movement**: Robot moves to commanded positions
- ✅ **Gripper**: Gripper opens/closes reliably
- ✅ **Vision**: Object detection works consistently
- ✅ **Safety**: Safety limits prevent dangerous movements
- ✅ **Error Handling**: System stops on errors appropriately
- ✅ **Emergency Stop**: Immediate halt when triggered
- ✅ **LLM Planning**: Generates valid action plans
- ✅ **Autonomous Execution**: Completes tasks without intervention
- ✅ **System Integration**: All components work together seamlessly

## Safety Guidelines

### During Testing
- **Never leave robot unattended** during testing
- **Keep emergency stop accessible** at all times
- **Start with simple tasks** and gradually increase complexity
- **Monitor robot behavior** for unexpected movements
- **Stop immediately** if anything seems unsafe

### Workspace Safety
- **Clear workspace** of all obstacles
- **Remove fragile items** from robot reach
- **Ensure stable footing** for robot base
- **Check for electrical hazards** near robot
- **Have clear escape path** if needed

### Robot Safety
- **Verify robot is in ready state** before testing
- **Check gripper is properly attached**
- **Ensure camera is securely mounted**
- **Verify all safety limits are set**
- **Test emergency stop before each session**

## Configuration Files

### Key Configuration Files
- `arm_config/safety_config.yaml` - Safety limits and parameters
- `arm_config/world_model.yaml` - Workspace poses and configuration
- `llm_planning_gemini/custom_gemini_examples/custom_gpt_action_schema.md` - Action definitions

### Environment Variables (Optional)
- `XARM_IP` - Robot IP address (defaults to 192.168.1.241)
- `GEMINI_API_KEY` - Google Gemini API key (required)
- `WORLD_YAML` - World configuration file path

## Support

### Getting Help
- Check the main README.md for general information
- Review error messages for specific issues
- Test individual components separately
- Use dry-run mode to debug planning issues
- Check ROS topics for vision system problems

### Emergency Contacts
- **Immediate Safety**: Use emergency stop button
- **Technical Issues**: Check troubleshooting section
- **Hardware Problems**: Contact robot manufacturer
- **Software Issues**: Review error logs and documentation

---