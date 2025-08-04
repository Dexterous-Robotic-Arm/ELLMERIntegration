# ELLMER Integration - Robot Control System

A comprehensive robot control system for the ufactory850 robotic arm with Intel RealSense D435 camera, designed for real-world autonomous tasks using LLM planning and computer vision.

## 🤖 System Overview

This system enables autonomous robot control for complex tasks using:
- **Natural Language Planning**: Google Gemini LLM for task understanding and planning
- **Computer Vision**: YOLO object detection with RealSense depth sensing
- **Direct Robot Control**: XArm Python SDK for precise robot manipulation
- **Safety First**: Comprehensive safety limits and emergency stop functionality
- **Modular Architecture**: Clean separation of planning, vision, and execution

### Key Features
- ✅ **Real Hardware Support**: ufactory850 robot arm + RealSense D435 camera
- ✅ **LLM Task Planning**: Google Gemini integration with fallback planning
- ✅ **Computer Vision**: YOLO + RealSense for 3D object detection
- ✅ **Gripper Control**: Full gripper manipulation with force feedback
- ✅ **Safety Monitoring**: Real-time safety checks and limits
- ✅ **ROS2 Integration**: Optional ROS2 support for vision data
- ✅ **Simulation Mode**: Test without hardware connection

## 🛠️ System Requirements

### Hardware
- **Robot**: ufactory850 6-DOF robotic arm
- **Camera**: Intel RealSense D435 or D435i
- **Computer**: Ubuntu 22.04 LTS (ARM64 or x86_64)
- **Network**: Robot and computer on same network (default: 192.168.1.241)

### Software Prerequisites
- **Ubuntu 22.04 LTS**
- **Python 3.8+**
- **ROS2 Humble** (optional, for vision data)
- **Intel RealSense SDK**

## 🚀 Quick Installation

### One-Command Installation (Ubuntu 22.04)
```bash
# Clone the repository
git clone <your-repo-url> ELLMERIntegration
cd ELLMERIntegration

# Run the installation script
chmod +x scripts/install_dependencies.sh
./scripts/install_dependencies.sh
```

### Manual Installation Steps

1. **Install ROS2 Humble**:
   ```bash
   # Set locale
   sudo apt update && sudo apt install locales
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8

   # Setup sources
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   sudo apt update

   # Install ROS2
   sudo apt install ros-humble-desktop ros-dev-tools python3-rclpy python3-ros2cli python3-ros2cli-common-extensions python3-colcon-common-extensions ros-humble-std-msgs ros-humble-geometry-msgs ros-humble-sensor-msgs ros-humble-nav-msgs ros-humble-tf2-ros ros-humble-tf2-msgs ros-humble-visualization-msgs ros-humble-cv-bridge build-essential cmake python3-pip python3-rosdep

   # Setup environment
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source /opt/ros/humble/setup.bash
   sudo rosdep init
   rosdep update
   ```

2. **Install RealSense SDK**:
   ```bash
   sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
   sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"
   sudo apt update
   sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev
   ```

3. **Setup Python environment**:
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   pip install -r requirements.txt
   ```

4. **Build ROS workspace** (optional):
   ```bash
   cd ros_workspace
   source /opt/ros/humble/setup.bash
   colcon build --packages-select ufactory_ellmer_msgs
   cd ..
   ```

## 🎯 Quick Start

### 1. Environment Setup
```bash
# Set your API key and robot IP
export GEMINI_API_KEY="your-gemini-api-key"
export XARM_IP="192.168.1.241"

# Setup environment
source scripts/setup_env.sh
```

### 2. Test System (No Hardware Required)
```bash
# Test in simulation mode
python3 robot_control/main.py --task "move to home" --sim --dry-run

# Test LLM planning
python3 robot_control/task_planner/llm_planner.py --task "pick up the cup" --use-fallback
```

### 3. Run with Real Hardware
```bash
# Simple pick and place
python3 robot_control/main.py --task "pick up the red cup"

# Interactive mode
python3 robot_control/main.py --interactive

# Loop mode for continuous operation
python3 robot_control/main.py --task "sort objects by color" --loop
```

## 📁 Project Structure

```
ELLMERIntegration/
├── robot_control/              # Main application package
│   ├── main.py                # Main orchestrator
│   ├── robot_controller/      # Robot control module
│   │   ├── actions_xarm.py    # XArm robot interface
│   │   └── executor.py        # Plan execution engine
│   ├── vision_system/         # Computer vision module
│   │   └── pose_recorder.py   # YOLO + RealSense integration
│   ├── task_planner/          # LLM planning module
│   │   ├── planner_llm.py     # LLM integration
│   │   └── llm_planner.py     # Standalone planner
│   └── utils/                 # Utilities
│       └── config_manager.py  # Configuration management
├── config/                    # Configuration files
│   ├── robot/                 # Robot configuration
│   │   ├── world_model.yaml   # Named poses and workspace
│   │   └── safety_config.yaml # Safety limits
│   ├── vision/                # Vision configuration
│   └── llm/                   # LLM configuration
│       └── action_schema.md   # LLM action definitions
├── ros_workspace/             # ROS2 workspace
│   └── src/                   # ROS2 packages
├── scripts/                   # Utility scripts
│   ├── install_dependencies.sh # Installation script
│   ├── setup_env.sh           # Environment setup
│   └── quick_start.sh         # Quick test script
└── tests/                     # Test files
```

## 🔧 Configuration

### Robot Configuration
- **World Model**: `config/robot/world_model.yaml` - Named poses and workspace
- **Safety Limits**: `config/robot/safety_config.yaml` - Movement and force limits
- **Default IP**: 192.168.1.241 (configurable via `XARM_IP` env var)

### Vision Configuration
- **Camera Settings**: `config/vision/camera_config.yaml`
- **YOLO Model**: `yolov8n.pt` (auto-downloaded)
- **Detection Thresholds**: Configurable in vision system

### LLM Configuration
- **Action Schema**: `config/llm/action_schema.md` - Complete action definitions
- **API Key**: Set via `GEMINI_API_KEY` environment variable

## 🎮 Usage Examples

### Basic Commands
```bash
# Execute a simple task
python3 robot_control/main.py --task "move to home position"

# Pick and place with vision
python3 robot_control/main.py --task "pick up the blue block and place it on the table"

# Interactive mode for testing
python3 robot_control/main.py --interactive

# Continuous operation
python3 robot_control/main.py --task "sort objects" --loop
```

### Simulation and Testing
```bash
# Test without hardware (simulation mode)
python3 robot_control/main.py --task "pick up cup" --sim --dry-run

# Test LLM planning only
python3 robot_control/task_planner/llm_planner.py --task "pick up the cup" --use-fallback

# Test with custom world configuration
python3 robot_control/main.py --task "pick up cup" --world config/robot/my_workspace.yaml
```

### Advanced Options
```bash
# Wait for minimum detections
python3 robot_control/main.py --task "pick up object" --wait-detections --min-detection-items 3

# Use custom vision script
python3 robot_control/main.py --task "detect objects" --vision-script custom_vision.py

# Loop mode with custom task
python3 robot_control/main.py --task "sort objects by color" --loop
```

## 🛡️ Safety Features

### Built-in Safety
- **Joint Limits**: Enforced for all movements
- **Workspace Boundaries**: Configurable safety zones
- **Velocity Limits**: Maximum speed restrictions
- **Emergency Stop**: Immediate halt capability
- **Collision Detection**: Real-time monitoring
- **Force Limits**: Maximum force and torque limits
- **Timeout Protection**: Automatic timeout for long operations

### Safety Configuration
```yaml
# config/robot/safety_config.yaml
max_velocity: 100.0        # mm/s
max_acceleration: 500.0    # mm/s²
max_force: 100.0          # N
workspace_limits:
  x: [-400, 400]          # mm
  y: [-400, 400]          # mm
  z: [0, 800]             # mm
```

## 🔍 Testing

### Basic Tests (No Hardware Required)
```bash
# Test system structure
python3 tests/test_basic_structure.py

# Test LLM planning
python3 robot_control/task_planner/llm_planner.py --task "pick up cup" --use-fallback

# Test in simulation mode
python3 robot_control/main.py --task "move to home" --sim --dry-run
```

### Hardware Testing
```bash
# Test robot connection
python3 -c "from robot_control.robot_controller import XArmRunner; runner = XArmRunner('192.168.1.241'); print('Connected successfully')"

# Test robot movement
python3 robot_control/main.py --task "move to home" --dry-run

# Test vision system
python3 robot_control/vision_system/pose_recorder.py

# Test gripper
python3 -c "from robot_control.robot_controller import XArmRunner; runner = XArmRunner('192.168.1.241'); runner.open_gripper()"
```

## 🐛 Troubleshooting

### Common Issues

**Robot Connection Failed**
```bash
# Check network connectivity
ping 192.168.1.241

# Verify robot is powered and connected
# Check robot IP in xArm Studio
```

**Camera Not Detected**
```bash
# Check USB connection
lsusb | grep RealSense

# Test RealSense viewer
realsense-viewer
```

**ROS2 Issues**
```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Check ROS2 installation
ros2 --version
```

**Import Errors**
```bash
# Clear Python cache
find . -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null || true

# Check Python path
echo $PYTHONPATH
```

### Getting Help
- Check logs for detailed error messages
- Verify all dependencies are installed
- Ensure hardware is properly connected
- Review configuration files

## 📚 How It Works

### 1. Task Planning
- User provides natural language task
- LLM (Gemini) generates structured action plan
- Plan includes movement, gripper, and vision actions
- Fallback planning available without API key

### 2. Vision System
- YOLO detects objects in camera feed
- RealSense provides depth information
- 3D pose estimation for detected objects
- Publishes object data via ROS2 (optional)

### 3. Robot Control
- Direct XArm SDK integration
- Safety monitoring and limits
- Gripper control with force feedback
- Emergency stop capability

### 4. Execution Engine
- Step-by-step plan execution
- Safety checks between steps
- Error handling and recovery
- Real-time monitoring

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests for new functionality
5. Submit a pull request

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 🆘 Support

For issues and questions:
1. Check the troubleshooting section
2. Review the documentation
3. Open an issue on GitHub
4. Contact the development team

---

**🎯 Ready to automate your robot tasks with AI-powered planning!**

The system is now fully functional with proper LLM integration, comprehensive safety features, and support for both simulation and real hardware operation.
