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
- ✅ **Dynamixel Gripper Control**: Full servo-based gripper manipulation with force feedback
- ✅ **Safety Monitoring**: Real-time safety checks and limits
- ✅ **ROS2 Integration**: Optional ROS2 support for vision data
- ✅ **Simulation Mode**: Test without hardware connection

## 🛠️ System Requirements

### Hardware
- **Robot**: ufactory850 6-DOF robotic arm
- **Camera**: Intel RealSense D435 or D435i
- **Gripper**: Dynamixel servo (MX-28, MX-64, MX-106, or AX-12 series)
- **Computer**: Ubuntu 22.04 LTS (ARM64 or x86_64)
- **Network**: Robot and computer on same network (default: 192.168.1.241)

### Software Prerequisites
- **Ubuntu 22.04 LTS**
- **Python 3.8+**
- **ROS2 Humble** (optional, for vision data)
- **Intel RealSense SDK**

## 🚀 Quick Installation & Setup

### Step 1: Clone and Install (Ubuntu 22.04)
```bash
# Clone the repository
git clone <your-repo-url> ELLMERIntegration
cd ELLMERIntegration

# Run the installation script
chmod +x scripts/install_dependencies.sh
./scripts/install_dependencies.sh
```

### Step 2: Manual Installation (if script fails)

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

## 🔧 Hardware Setup & Connection

### Step 3: Robot Arm Setup
```bash
# 1. Power on the ufactory850 robot arm
# 2. Connect robot to network (default IP: 192.168.1.241)
# 3. Verify connection:
ping 192.168.1.241

# 4. Test robot connection:
python3 -c "from robot_control.robot_controller import XArmRunner; runner = XArmRunner('192.168.1.241', sim=True); print('Robot connection test passed')"
```

### Step 4: Dynamixel Gripper Setup
```bash
# 1. Connect Dynamixel servo to USB-to-TTL converter
# 2. Connect to computer via USB

# 3. Set serial port permissions:
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyUSB0

# 4. Verify Dynamixel connection:
ls -la /dev/ttyUSB*
python3 -c "import dynamixel_sdk; print('Dynamixel SDK available')"

# 5. Configure gripper settings:
nano config/gripper_config.yaml
```

### Step 5: Camera Setup
```bash
# 1. Connect Intel RealSense D435 to USB 3.0 port
# 2. Verify camera connection:
lsusb | grep RealSense
realsense-viewer  # Test camera feed

# 3. Test camera integration:
python3 -c "import pyrealsense2 as rs; print('RealSense SDK available')"
```

### Step 6: Test Complete System
```bash
# 1. Test in simulation mode (no hardware required):
python3 robot_control/main.py --task "open gripper" --sim --dry-run

# 2. Test with real hardware:
python3 robot_control/main.py --task "move to home" --dry-run

# 3. Test gripper operations:
python3 robot_control/main.py --task "test gripper" --dry-run

# 4. Test complete pick-and-place:
python3 robot_control/main.py --task "pick up cup" --dry-run

# 5. Test Dynamixel installation:
./scripts/test_dynamixel_installation.sh

# 6. Setup Dynamixel gripper:
./scripts/setup_dynamixel.sh
```

## 🎯 Quick Start Guide

### Step 1: Environment Setup
```bash
# 1. Set your API key and robot IP
export GEMINI_API_KEY="your-gemini-api-key"
export XARM_IP="192.168.1.241"

# 2. Activate Python environment
source venv/bin/activate

# 3. Setup ROS2 environment (if using ROS2)
source /opt/ros/humble/setup.bash
```

### Step 2: Test System (No Hardware Required)
```bash
# 1. Test basic functionality in simulation
python3 robot_control/main.py --task "move to home" --sim --dry-run

# 2. Test gripper operations
python3 robot_control/main.py --task "open gripper" --sim --dry-run

# 3. Test LLM planning
python3 robot_control/task_planner/llm_planner.py --task "pick up the cup" --use-fallback

# 4. Test complete pick-and-place simulation
python3 robot_control/main.py --task "pick up cup and place on table" --sim --dry-run
```

### Step 3: Hardware Connection Test
```bash
# 1. Test robot connection (with hardware)
python3 robot_control/main.py --task "move to home" --dry-run

# 2. Test gripper with Dynamixel servo
python3 robot_control/main.py --task "open gripper" --dry-run

# 3. Test camera integration
python3 robot_control/vision_system/pose_recorder.py

# 4. Test complete system
python3 robot_control/main.py --task "pick up the red cup" --dry-run
```

### Step 4: Production Usage
```bash
# 1. Simple pick and place
python3 robot_control/main.py --task "pick up the red cup"

# 2. Interactive mode for testing
python3 robot_control/main.py --interactive

# 3. Loop mode for continuous operation
python3 robot_control/main.py --task "sort objects by color" --loop

# 4. Custom world configuration
python3 robot_control/main.py --task "pick up cup" --world config/robot/my_workspace.yaml
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
│   ├── quick_start.sh         # Quick test script
│   ├── setup_dynamixel.sh     # Dynamixel gripper setup
│   └── test_dynamixel_installation.sh # Dynamixel installation test
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

## 🔧 Dynamixel Gripper Configuration

### Supported Servo Models
The system supports various Dynamixel servo models for gripper control:
- **MX Series** (Protocol 2.0): MX-28, MX-64, MX-106
- **AX Series** (Protocol 1.0): AX-12, AX-18
- **Custom Servos**: Configurable via YAML settings

### Step 1: Install Dynamixel SDK
```bash
# Install Dynamixel SDK
pip install dynamixel-sdk

# Verify installation
python3 -c "import dynamixel_sdk; print('Dynamixel SDK installed successfully')"
```

### Step 2: Hardware Connection
```bash
# 1. Connect Dynamixel servo to USB-to-TTL converter
# 2. Connect converter to computer via USB
# 3. Set servo ID using Dynamixel Wizard or similar tool

# Check connection
ls -la /dev/ttyUSB*

# Set permissions
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyUSB0
```

### Step 3: Configure Gripper Settings
Edit `config/gripper_config.yaml` to match your servo setup:

```yaml
# Dynamixel servo configuration
servo:
  id: 1                    # Servo ID (usually 1)
  port: "/dev/ttyUSB0"     # Serial port
  baudrate: 57600          # Baudrate (common: 57600, 1000000)
  protocol: 2              # Protocol version (1 or 2)

# Position limits (servo units: 0-4095 for MX series)
positions:
  open: 4095               # Fully open position
  closed: 1024             # Fully closed position
  half: 2560               # Half open position

# Speed and torque limits
limits:
  default_speed: 100       # Speed (0-1023 for MX series)
  default_torque: 512      # Torque limit (0-1023 for MX series)
  max_speed: 200           # Maximum speed
  max_torque: 1023         # Maximum torque

# Control addresses (for MX-28, MX-64, MX-106 series)
addresses:
  torque_enable: 64        # Torque enable address
  goal_position: 116       # Goal position address (Protocol 2)
  present_position: 132    # Present position address (Protocol 2)
  moving_speed: 104        # Moving speed address (Protocol 2)
  torque_limit: 102        # Torque limit address (Protocol 2)
  present_load: 126        # Present load address (Protocol 2)
```

### Step 4: Test Configuration
```bash
# Test basic gripper operations
python3 robot_control/main.py --task "open gripper" --sim --dry-run
python3 robot_control/main.py --task "close gripper" --sim --dry-run

# Test gripper cycle
python3 robot_control/main.py --task "test gripper" --sim --dry-run

# Check gripper status
python3 -c "from robot_control.robot_controller import XArmRunner; runner = XArmRunner('192.168.1.241', sim=True); print(runner.get_gripper_status())"
```

### Step 5: Servo-Specific Configuration

#### MX-28/MX-64/MX-106 (Protocol 2.0)
```yaml
servo:
  protocol: 2
  baudrate: 57600

addresses:
  torque_enable: 64
  goal_position: 116
  present_position: 132
  moving_speed: 104
  torque_limit: 102
  present_load: 126
```

#### AX-12 (Protocol 1.0)
```yaml
servo:
  protocol: 1
  baudrate: 1000000

addresses:
  torque_enable: 24
  goal_position: 30
  present_position: 36
  moving_speed: 32
  torque_limit: 34
  present_load: 40
```

### Hardware Setup
1. **Connect Dynamixel servo** to USB-to-TTL converter
2. **Set servo ID** using Dynamixel Wizard or similar tool
3. **Configure serial port** permissions:
   ```bash
   sudo usermod -a -G dialout $USER
   sudo chmod 666 /dev/ttyUSB0
   ```
4. **Test connection**:
   ```bash
   python3 -c "import dynamixel_sdk as dxl; print('Dynamixel SDK available')"
   ```

### Testing Dynamixel Gripper
```bash
# Test basic gripper operations
python3 robot_control/main.py --task "open gripper" --dry-run
python3 robot_control/main.py --task "close gripper" --dry-run

# Test gripper cycle
python3 robot_control/main.py --task "test gripper" --dry-run

# Check gripper status
python3 -c "from robot_control.robot_controller import XArmRunner; runner = XArmRunner('192.168.1.241'); print(runner.get_gripper_status())"
```

### Troubleshooting Dynamixel
```bash
# Check if port exists
ls -la /dev/ttyUSB*

# Check port permissions
ls -la /dev/ttyUSB0

# Test serial communication
sudo apt install minicom
minicom -D /dev/ttyUSB0 -b 57600

# Check Dynamixel SDK installation
python3 -c "import dynamixel_sdk; print('SDK version:', dynamixel_sdk.__version__)"
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

# Test Dynamixel gripper specifically
python3 -c "from robot_control.robot_controller import XArmRunner; runner = XArmRunner('192.168.1.241'); print(runner.get_gripper_status())"
```

## 🐛 Troubleshooting Guide

### Step-by-Step Problem Solving

#### 1. Robot Connection Issues
```bash
# Step 1: Check network connectivity
ping 192.168.1.241

# Step 2: Verify robot power and network
# - Ensure robot is powered on
# - Check network cable connection
# - Verify robot IP in xArm Studio

# Step 3: Test robot connection
python3 -c "from robot_control.robot_controller import XArmRunner; runner = XArmRunner('192.168.1.241', sim=True); print('Connection test passed')"

# Step 4: Check robot IP configuration
# Edit robot IP if different from default:
export XARM_IP="your-robot-ip"
```

#### 2. Dynamixel Gripper Issues
```bash
# Step 1: Check USB connection
ls -la /dev/ttyUSB*

# Step 2: Set permissions
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyUSB0

# Step 3: Test Dynamixel SDK
python3 -c "import dynamixel_sdk; print('Dynamixel SDK available')"

# Step 4: Check gripper configuration
cat config/gripper_config.yaml

# Step 5: Test gripper connection
python3 -c "from robot_control.robot_controller import XArmRunner; runner = XArmRunner('192.168.1.241', sim=True); print(runner.get_gripper_status())"
```

#### 3. Camera Connection Issues
```bash
# Step 1: Check USB connection
lsusb | grep RealSense

# Step 2: Test RealSense viewer
realsense-viewer

# Step 3: Test camera SDK
python3 -c "import pyrealsense2 as rs; print('RealSense SDK available')"

# Step 4: Check camera permissions
sudo usermod -a -G video $USER
```

#### 4. ROS2 Issues
```bash
# Step 1: Source ROS2 environment
source /opt/ros/humble/setup.bash

# Step 2: Check ROS2 installation
ros2 --version

# Step 3: Build ROS workspace
cd ros_workspace
colcon build --packages-select ufactory_ellmer_msgs
cd ..

# Step 4: Test ROS2 integration
python3 -c "import rclpy; print('ROS2 available')"
```

#### 5. Python Import Issues
```bash
# Step 1: Activate virtual environment
source venv/bin/activate

# Step 2: Clear Python cache
find . -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null || true

# Step 3: Check Python path
echo $PYTHONPATH

# Step 4: Reinstall dependencies
pip install -r requirements.txt
```

#### 6. LLM API Issues
```bash
# Step 1: Set API key
export GEMINI_API_KEY="your-gemini-api-key"

# Step 2: Test API connection
python3 robot_control/task_planner/llm_planner.py --task "test" --use-fallback

# Step 3: Check API key format
echo $GEMINI_API_KEY
```

### Common Error Messages & Solutions

| Error | Solution |
|-------|----------|
| `connect socket failed` | Check robot IP and network connection |
| `No such file or directory: '/dev/ttyUSB0'` | Connect Dynamixel servo and set permissions |
| `No module named 'dynamixel_sdk'` | Install: `pip install dynamixel-sdk` |
| `No module named 'pyrealsense2'` | Install RealSense SDK |
| `No module named 'rclpy'` | Install ROS2 or run without ROS2 |
| `Failed to load action schema` | Check `config/llm/action_schema.md` exists |

### Getting Help
1. **Check logs**: Look for detailed error messages in console output
2. **Verify dependencies**: Ensure all required packages are installed
3. **Test hardware**: Verify all hardware connections
4. **Review configuration**: Check all configuration files
5. **Use simulation mode**: Test without hardware first

## 📚 How It Works

### 1. Task Planning
- User provides natural language task
- LLM (Gemini) generates structured action plan
- Plan includes movement, gripper, and vision actions
- Fallback planning available without API key

## 🎯 Complete Setup Checklist

### ✅ Pre-Installation
- [ ] Ubuntu 22.04 LTS installed
- [ ] Python 3.8+ available
- [ ] Internet connection for package downloads

### ✅ Software Installation
- [ ] Repository cloned: `git clone <repo-url> ELLMERIntegration`
- [ ] Dependencies installed: `./scripts/install_dependencies.sh`
- [ ] Python environment activated: `source venv/bin/activate`
- [ ] ROS2 environment sourced: `source /opt/ros/humble/setup.bash`

### ✅ Hardware Setup
- [ ] ufactory850 robot arm powered on and connected to network
- [ ] Dynamixel servo connected via USB-to-TTL converter
- [ ] Intel RealSense D435 camera connected to USB 3.0 port
- [ ] All USB permissions set correctly

### ✅ Configuration
- [ ] Robot IP configured (default: 192.168.1.241)
- [ ] Dynamixel gripper settings in `config/gripper_config.yaml`
- [ ] World model configured in `config/robot/world_model.yaml`
- [ ] Gemini API key set (optional): `export GEMINI_API_KEY="your-key"`

### ✅ Testing
- [ ] Simulation mode test: `python3 robot_control/main.py --task "open gripper" --sim --dry-run`
- [ ] Robot connection test: `python3 robot_control/main.py --task "move to home" --dry-run`
- [ ] Gripper test: `python3 robot_control/main.py --task "test gripper" --dry-run`
- [ ] Camera test: `python3 robot_control/vision_system/pose_recorder.py`
- [ ] Complete system test: `python3 robot_control/main.py --task "pick up cup" --dry-run`

### ✅ Production Ready
- [ ] All tests passing
- [ ] Safety limits configured
- [ ] Emergency stop accessible
- [ ] Workspace clear of obstacles

## 🚀 Quick Reference Commands

### Essential Commands
```bash
# Start system
python3 robot_control/main.py --task "your task here"

# Test without hardware
python3 robot_control/main.py --task "test task" --sim --dry-run

# Interactive mode
python3 robot_control/main.py --interactive

# Loop mode
python3 robot_control/main.py --task "continuous task" --loop
```

### Configuration Commands
```bash
# Set robot IP
export XARM_IP="192.168.1.241"

# Set API key
export GEMINI_API_KEY="your-api-key"

# Activate environment
source venv/bin/activate
source /opt/ros/humble/setup.bash
```

### Testing Commands
```bash
# Test robot connection
ping 192.168.1.241

# Test gripper
python3 -c "from robot_control.robot_controller import XArmRunner; runner = XArmRunner('192.168.1.241', sim=True); print(runner.get_gripper_status())"

# Test camera
realsense-viewer

# Test LLM planning
python3 robot_control/task_planner/llm_planner.py --task "test task" --use-fallback

# Test Dynamixel installation
./scripts/test_dynamixel_installation.sh

# Setup Dynamixel gripper
./scripts/setup_dynamixel.sh
```

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
