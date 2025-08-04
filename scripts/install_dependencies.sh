#!/bin/bash

# Ubuntu 22.04 Installation Script for Robot Control System
# This script installs all dependencies for real hardware testing

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Print functions
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running on Ubuntu 22.04
check_ubuntu_version() {
    if [[ ! -f /etc/os-release ]]; then
        print_error "This script is designed for Ubuntu 22.04"
        exit 1
    fi
    
    source /etc/os-release
    if [[ "$VERSION_ID" != "22.04" ]]; then
        print_warning "This script is designed for Ubuntu 22.04, but you're running $VERSION_ID"
        read -p "Continue anyway? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
    
    print_success "Ubuntu version check passed"
}

# Check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Install system dependencies
install_system_deps() {
    print_status "Installing system dependencies..."
    
    sudo apt update
    
    # Essential build tools
    sudo apt install -y build-essential cmake pkg-config
    
    # Python development
    sudo apt install -y python3-dev python3-pip python3-venv
    
    # Git and other utilities
    sudo apt install -y git curl wget
    
    # RealSense dependencies
    sudo apt install -y libusb-1.0-0-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
    
    # ROS2 dependencies
    sudo apt install -y software-properties-common
    
    print_success "System dependencies installed"
}

# Install ROS2 Humble
install_ros2_humble() {
    print_status "Installing ROS2 Humble..."
    
    # Add ROS2 repository
    sudo apt update && sudo apt install -y curl gnupg2 lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    # Install ROS2 Humble Desktop
    sudo apt update
    sudo apt install -y ros-humble-desktop
    
    # Install ROS2 development tools
    sudo apt install -y python3-colcon-common-extensions python3-rosdep
    
    # Initialize rosdep
    sudo rosdep init
    rosdep update
    
    print_success "ROS2 Humble installed"
}

# Install RealSense SDK
install_realsense() {
    print_status "Installing Intel RealSense SDK..."
    
    # Add RealSense repository
    sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
    sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"
    
    # Install RealSense packages
    sudo apt update
    sudo apt install -y librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg
    
    # Install Python wrapper
    pip3 install pyrealsense2
    
    print_success "RealSense SDK installed"
}

# Setup Python virtual environment
setup_python_env() {
    print_status "Setting up Python virtual environment..."
    
    # Create virtual environment
    python3 -m venv venv
    
    # Activate virtual environment
    source venv/bin/activate
    
    # Upgrade pip
    pip install --upgrade pip
    
    print_success "Python virtual environment created"
}

# Install Python dependencies
install_python_deps() {
    print_status "Installing Python dependencies..."
    
    # Activate virtual environment
    source venv/bin/activate
    
    # Install all Python dependencies
    pip install -r requirements.txt
    
    print_success "Python dependencies installed"
}

# Download YOLO model
download_yolo_model() {
    print_status "Downloading YOLO model..."
    
    if [[ ! -f "yolov8n.pt" ]]; then
        wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt
        print_success "YOLO model downloaded"
    else
        print_status "YOLO model already exists"
    fi
}

# Build ROS messages
build_ros_messages() {
    print_status "Building ROS messages..."
    
    if [ -d "ros_workspace" ]; then
        cd ros_workspace
        
        # Source ROS2
        source /opt/ros/humble/setup.bash
        
        # Build custom messages
        if command_exists colcon; then
            colcon build --packages-select ufactory_ellmer_msgs
            print_success "ROS messages built"
        else
            print_warning "colcon not found, skipping ROS message build"
        fi
        
        cd ..
    else
        print_warning "ros_workspace directory not found, skipping ROS message build"
    fi
}

# Setup environment scripts
setup_env_scripts() {
    print_status "Setting up environment scripts..."
    
    # Create setup script
    cat > setup_env.sh << 'EOF'
#!/bin/bash
# Source ROS2 and robot control environment

# Source ROS2
source /opt/ros/humble/setup.bash

# Source robot control workspace
if [ -d "ros_workspace" ]; then
    source ros_workspace/install/setup.bash
fi

# Activate Python virtual environment
if [ -d "venv" ]; then
    source venv/bin/activate
fi

# Set environment variables
export PYTHONPATH="${PYTHONPATH}:$(pwd)"
export GEMINI_API_KEY="${GEMINI_API_KEY:-}"
export XARM_IP="${XARM_IP:-192.168.1.241}"

echo "Environment setup complete!"
echo "ROS2 Humble: $(ros2 --version 2>/dev/null | head -1 || echo 'Not found')"
echo "Python: $(python3 --version)"
echo "XArm IP: $XARM_IP"
EOF
    
    chmod +x setup_env.sh
    
    # Create quick start script
    cat > quick_start.sh << 'EOF'
#!/bin/bash
# Quick start script for robot control system

# Setup environment
source setup_env.sh

# Check if robot is connected
echo "Checking robot connection..."
python3 -c "
import sys
sys.path.append('.')
from robot_control.robot_controller import XArmRunner

try:
    runner = XArmRunner()
    if runner.connect():
        print('✅ Robot connected successfully!')
        runner.disconnect()
    else:
        print('❌ Failed to connect to robot')
except Exception as e:
    print(f'❌ Error: {e}')
"

echo ""
echo "To start the robot control system:"
echo "  python3 robot_control/main.py --task 'your task description'"
echo ""
echo "For interactive mode:"
echo "  python3 robot_control/main.py --interactive"
EOF
    
    chmod +x quick_start.sh
    
    print_success "Environment scripts created"
}

# Main installation function
main() {
    print_status "Starting Ubuntu 22.04 installation for Robot Control System..."
    
    # Check Ubuntu version
    check_ubuntu_version
    
    # Install system dependencies
    install_system_deps
    
    # Install ROS2 Humble
    install_ros2_humble
    
    # Install RealSense SDK
    install_realsense
    
    # Setup Python environment
    setup_python_env
    
    # Install Python dependencies
    install_python_deps
    
    # Download YOLO model
    download_yolo_model
    
    # Build ROS messages
    build_ros_messages
    
    # Setup environment scripts
    setup_env_scripts
    
    print_success "Installation completed successfully!"
    echo ""
    echo "Next steps:"
    echo "1. Set your Gemini API key: export GEMINI_API_KEY='your-api-key'"
    echo "2. Set your robot IP: export XARM_IP='192.168.1.241'"
    echo "3. Setup environment: source setup_env.sh"
    echo "4. Test connection: ./quick_start.sh"
    echo "5. Run robot control: python3 robot_control/main.py --task 'pick up the cup'"
    echo ""
    echo "For more information, see README.md and docs/testing_guide.md"
}

# Run main function
main "$@" 