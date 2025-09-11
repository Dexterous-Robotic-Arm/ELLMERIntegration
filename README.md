# ELLMER Integration - April Tags Robot Control

A robot control system using April Tags for object detection with Intel RealSense camera and ufactory850 robotic arm. Configured for extreme reasoning and precise millimeter-level movements.

## 🚀 Quick Setup

### 1. Build Everything
```bash
# Clone and build the complete system
git clone <your-repo> ELLMERIntegration
cd ELLMERIntegration
chmod +x setup.sh
./setup.sh
```

### 2. Setup Environment
```bash
source scripts/setup_env.sh
```

## 🏷️ April Tags Configuration

- **Tag Family**: TagStandard41h12
- **Tag Size**: 100mm (for desk objects)
- **Object Mapping**:
  - ID 0 = bottle
  - ID 1 = book  
  - ID 2 = cup
  - ID 3 = pen
  - ID 4 = phone
  - ID 5 = laptop
  - ID 6 = notebook
  - ID 7 = stapler
  - ID 8 = keyboard
  - ID 9 = mouse
  - ID 10 = calculator

**Print April Tags**: Download from [AprilRobotics/apriltag-imgs](https://github.com/AprilRobotics/apriltag-imgs)

## 🤖 Robot Configuration

- **Speed**: 50mm/s (reduced for precision)
- **Precision**: Millimeter-level movements
- **Collision Avoidance**: Enabled
- **Force Feedback**: Disabled
- **Continuous Vision**: Enabled

## 🥤 Pick Up Water Bottle (Example)

### Method 1: Direct Control
```bash
# Simulation mode (safe)
python3 robot_control/main.py --task "pick up the bottle" --sim --dry-run

# Real robot mode
python3 robot_control/main.py --task "pick up the bottle"
```

### Method 2: ROS Integration
```bash
# Terminal 1: Launch complete system
cd ros_workspace
source install/setup.bash
ros2 launch april_tags_vision complete_system.launch.py

# Terminal 2: Run robot control
source scripts/setup_env.sh
python3 robot_control/main.py --task "pick up the bottle"
```

## 🎯 Complex Tasks Examples

### Find Object in Pile
```bash
python3 robot_control/main.py --task "find the book in the pile and grab it"
```

### Organize Objects
```bash
python3 robot_control/main.py --task "organize the desk objects by type"
```

### Interactive Mode
```bash
python3 robot_control/main.py --interactive
```

## 🔧 ROS Launch Commands (Matches Your Friend's Setup)

### 1. RealSense Camera
```bash
source /opt/ros/humble/setup.bash
ros2 launch april_tags_vision realsense_camera.launch.py \
  camera_name:=cam_hand \
  serial_no:="153122078759" \
  camera_info_url:=file:///home/apriltag_ws/calibration/cam_hand.yaml
```

### 2. Image Processing
```bash
source /opt/ros/humble/setup.bash
source ~/apriltag_ws/install/setup.bash
ros2 launch april_tags_vision image_proc.launch.py
```

### 3. April Tags Detection
```bash
source /opt/ros/humble/setup.bash
source ~/apriltag_ws/install/setup.bash
ros2 launch april_tags_vision april_tags_detection.launch.py \
  tag_size:=0.1 \
  tag_ids:="[0,1,2,3,4,5,6,7,8,9,10]"
```

### 4. Complete System
```bash
source /opt/ros/humble/setup.bash
source ~/apriltag_ws/install/setup.bash
ros2 launch april_tags_vision complete_system.launch.py
```

## 🧠 System Capabilities

### Extreme Reasoning
- **Object Prioritization**: System prioritizes the object mentioned in the prompt
- **Stacking Analysis**: Determines object stacking order using depth information
- **Collision Avoidance**: Moves objects out of the way when needed
- **Grasp Strategy**: Reasons about optimal grasp approaches

### Precise Movement
- **Millimeter Precision**: Sub-millimeter accuracy for delicate operations
- **Slow Speed**: 50mm/s for safe, controlled movements
- **Collision Detection**: Real-time collision avoidance
- **Smooth Trajectories**: Optimized acceleration profiles

### Vision System
- **Continuous Detection**: Runs continuously in background
- **Multiple Tags**: Detects up to 10 different objects simultaneously
- **3D Positioning**: Accurate 3D coordinates in robot base frame
- **Coordinate Stabilization**: Smooth, stable positioning

## 📋 System Requirements

- **Python 3.8+**
- **Intel RealSense D435** camera
- **ufactory850** robot arm
- **April Tags** (TagStandard41h12, 100mm)
- **ROS2 Humble** (optional)

## 🚨 Safety Notes

- Always test in simulation mode first (`--sim --dry-run`)
- Ensure robot is properly calibrated
- Check workspace boundaries
- Use emergency stop when needed
- Monitor robot movements closely

## 🔍 Troubleshooting

### April Tags Not Detected
- Check tag family: Must be `TagStandard41h12`
- Check tag size: Should be 100mm
- Improve lighting conditions
- Ensure tags are clearly visible

### Robot Movement Issues
- Check robot IP: `export XARM_IP="192.168.1.241"`
- Verify robot is powered and connected
- Check safety limits in configuration

### Vision System Issues
- Install RealSense SDK: `pip install pyrealsense2`
- Check camera connection
- Verify camera calibration file

## 🎯 Usage Examples

```bash
# Basic object pickup
python3 robot_control/main.py --task "pick up the bottle"

# Complex manipulation
python3 robot_control/main.py --task "find the book in the pile and grab it"

# Interactive control
python3 robot_control/main.py --interactive

# Simulation testing
python3 robot_control/main.py --task "test" --sim --dry-run
```

The system is now ready for extreme reasoning and precise manipulation of any object with an April Tag!