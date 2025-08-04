# Robot Control System

A comprehensive robot control system for the ufactory850 robotic arm with Intel RealSense D435 camera, designed for real-world autonomous tasks using LLM planning.

## 🤖 System Overview

This system enables autonomous robot control for complex tasks like:
- **Pick and place operations**
- **Object manipulation**
- **Assembly tasks**
- **Sorting and organizing**
- **Custom task execution via natural language**

### Key Features
- ✅ **Real Hardware Support**: ufactory850 robot arm + RealSense D435 camera
- ✅ **LLM Task Planning**: Google Gemini integration for natural language task planning
- ✅ **Computer Vision**: YOLO object detection with RealSense depth sensing
- ✅ **Safety First**: Comprehensive safety limits and emergency stop functionality
- ✅ **Modular Design**: Clean separation of robot control, vision, and planning
- ✅ **ROS2 Integration**: Full ROS2 Humble support for inter-process communication

## 🛠️ System Requirements

### Hardware
- **Robot**: ufactory850 6-DOF robotic arm
- **Camera**: Intel RealSense D435 or D435i
- **Computer**: Ubuntu 22.04 LTS with sufficient USB ports
- **Network**: Robot and computer on same network (default: 192.168.1.241)

### Software Prerequisites
- **Ubuntu 22.04 LTS**
- **Python 3.8+**
- **ROS2 Humble**
- **Intel RealSense SDK**

## 🚀 Quick Installation

### One-Command Installation
```bash
# Clone the repository
git clone <your-repo-url> ELLMERIntegration
cd ELLMERIntegration

# Run the installation script
chmod +x scripts/install_dependencies.sh
./scripts/install_dependencies.sh
```

### Manual Installation
If you prefer step-by-step installation:

1. **Install ROS2 Humble**:
   ```bash
   sudo apt update
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   sudo apt update
   sudo apt install ros-humble-desktop
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

4. **Build ROS workspace**:
   ```bash
   cd ros_workspace
   source /opt/ros/humble/setup.bash
   colcon build --packages-select ufactory_ellmer_msgs
   cd ..
   ```

## 🎯 Quick Start

### 1. Setup Environment
```bash
# Set your API key and robot IP
export GEMINI_API_KEY="your-gemini-api-key"
export XARM_IP="192.168.1.241"

# Setup environment
source setup_env.sh
```

### 2. Test Robot Connection
```bash
./quick_start.sh
```

### 3. Run Your First Task
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
│   ├── vision_system/         # Computer vision module
│   ├── task_planner/          # LLM planning module
│   └── utils/                 # Utilities
├── config/                    # Configuration files
│   ├── robot/                 # Robot configuration
│   ├── vision/                # Vision configuration
│   └── llm/                   # LLM configuration
├── ros_workspace/             # Standard ROS2 workspace
│   └── src/                   # ROS2 packages
├── scripts/                   # Utility scripts
├── docs/                      # Documentation
└── tests/                     # Test files
```

## 🔧 Configuration

### Robot Configuration
- **Safety Limits**: `config/robot/safety_config.yaml`
- **World Model**: `config/robot/world_model.yaml`
- **Default IP**: 192.168.1.241 (configurable via `XARM_IP` env var)

### Vision Configuration
- **Camera Settings**: `config/vision/camera_config.yaml`
- **YOLO Model**: `yolov8n.pt` (auto-downloaded)
- **Detection Thresholds**: Configurable in vision system

### LLM Configuration
- **Action Schema**: `config/llm/action_schema.md`
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

### Advanced Options
```bash
# Use custom world configuration
python3 robot_control/main.py --task "pick up cup" --world config/robot/my_workspace.yaml

# Wait for minimum detections
python3 robot_control/main.py --task "pick up object" --wait-detections --min-detection-items 3

# Use custom vision script
python3 robot_control/main.py --task "detect objects" --vision-script custom_vision.py
```

## 🛡️ Safety Features

### Built-in Safety
- **Joint Limits**: Enforced for all movements
- **Workspace Boundaries**: Configurable safety zones
- **Velocity Limits**: Maximum speed restrictions
- **Emergency Stop**: Immediate halt capability
- **Collision Detection**: Real-time monitoring

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

### Basic Tests
```bash
# Run structure tests
python3 tests/test_basic_structure.py

# Test robot connection
python3 -c "from robot_control.robot_controller import XArmRunner; runner = XArmRunner(); print('Connected:', runner.connect())"
```

### Hardware Testing
```bash
# Test robot movement
python3 robot_control/main.py --task "move to home" --dry-run

# Test vision system
python3 robot_control/vision_system/pose_recorder.py

# Test gripper
python3 -c "from robot_control.robot_controller import XArmRunner; runner = XArmRunner(); runner.connect(); runner.open_gripper()"
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

### Getting Help
- Check logs for detailed error messages
- Verify all dependencies are installed
- Ensure hardware is properly connected
- Review configuration files

## 📚 Documentation

- **Testing Guide**: `docs/testing_guide.md` - Detailed testing instructions
- **Configuration**: `config/` - Configuration examples and documentation
- **API Reference**: Code documentation in respective modules

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
