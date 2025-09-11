# April Tags Vision - ROS2 Package

ROS2 package for April Tags detection with RealSense camera and robot integration.

## 🚀 Quick Start

### Build the Package
```bash
cd ros_workspace
colcon build --packages-select april_tags_vision
source install/setup.bash
```

### Launch Components

#### 1. RealSense Camera
```bash
ros2 launch april_tags_vision realsense_camera.launch.py \
  camera_name:=cam_hand \
  serial_no:="153122078759" \
  camera_info_url:=file:///home/apriltag_ws/calibration/cam_hand.yaml
```

#### 2. Image Processing
```bash
ros2 launch april_tags_vision image_proc.launch.py
```

#### 3. April Tags Detection
```bash
ros2 launch april_tags_vision april_tags_detection.launch.py \
  tag_size:=0.1 \
  tag_ids:="[0,1,2,3,4,5,6,7,8,9,10]"
```

#### 4. Complete System
```bash
ros2 launch april_tags_vision complete_system.launch.py
```

### Test the Package
```bash
# Terminal 1: Launch detector
ros2 launch april_tags_vision april_tags_detector.launch.py

# Terminal 2: Test detections
ros2 run april_tags_vision test_april_tags.py
```

## 🏷️ Configuration

- **Tag Family**: TagStandard41h12
- **Tag Size**: 100mm
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

## 📦 Package Structure

```
april_tags_vision/
├── package.xml                    # Package metadata
├── CMakeLists.txt                 # Build configuration
├── msg/                          # Custom messages
│   ├── AprilTagDetection.msg
│   └── AprilTagDetectionArray.msg
├── launch/                       # Launch files
│   ├── april_tags_detector.launch.py
│   ├── realsense_camera.launch.py
│   ├── image_proc.launch.py
│   ├── april_tags_detection.launch.py
│   └── complete_system.launch.py
├── config/                      # Configuration files
│   └── april_tags.yaml
├── scripts/                     # Executable scripts
│   ├── april_tags_detector_node.py
│   └── test_april_tags.py
├── april_tags_vision/           # Python package
│   ├── __init__.py
│   └── april_tag_detector.py
└── README.md                    # This file
```

## 🔧 Features

- **ROS2 Integration**: Full ROS2 package
- **RealSense Support**: Direct camera integration
- **3D Pose Estimation**: Accurate positioning
- **Object Mapping**: Tag ID to object name
- **Custom Messages**: Structured detection data
- **Launch Files**: Easy system startup
- **Configuration**: YAML-based settings

## 📋 Dependencies

- ROS2 Humble
- RealSense2 Camera
- April Tags ROS
- Image Processing
- OpenCV
- NumPy
- SciPy

## 🎯 Integration with Robot Control

The package integrates with the robot control system:

```python
# In robot_control/vision_system/pose_recorder.py
from april_tags_vision import AprilTagDetector

detector = AprilTagDetector()
detections = detector.detect_tags(color_image)
```

## 🚀 ROS2 Commands

```bash
# List topics
ros2 topic list

# View detections
ros2 topic echo /april_tags/detections

# View debug image
ros2 run rqt_image_view rqt_image_view /april_tags/debug_image

# Check node status
ros2 node list
ros2 node info /april_tags_detector
```

## 🔧 Troubleshooting

### Build Issues
```bash
# Install missing dependencies
sudo apt install ros-humble-ament-cmake ros-humble-ament-cmake-auto

# Clean and rebuild
cd ros_workspace
rm -rf build install
colcon build --packages-select april_tags_vision
```

### Camera Issues
```bash
# Check camera connection
ros2 run realsense2_camera rs-enumerate-devices

# Test camera
ros2 launch realsense2_camera rs_launch.py
```

### Detection Issues
```bash
# Check topics
ros2 topic list | grep april_tags

# View detection data
ros2 topic echo /april_tags/detections
```
