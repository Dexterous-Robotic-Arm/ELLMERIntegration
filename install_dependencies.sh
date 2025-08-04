#!/bin/bash

# Robot Control System - Complete Installation Script
# ===================================================
# This script installs all dependencies for the ufactory850 robot control system

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
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

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to check Python version
check_python_version() {
    if command_exists python3; then
        PYTHON_VERSION=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
        PYTHON_MAJOR=$(echo $PYTHON_VERSION | cut -d. -f1)
        PYTHON_MINOR=$(echo $PYTHON_VERSION | cut -d. -f2)
        
        if [ "$PYTHON_MAJOR" -eq 3 ] && [ "$PYTHON_MINOR" -ge 8 ]; then
            print_success "Python $PYTHON_VERSION found (✓)"
            return 0
        else
            print_error "Python 3.8+ required, found $PYTHON_VERSION"
            return 1
        fi
    else
        print_error "Python 3 not found"
        return 1
    fi
}

# Function to install system dependencies
install_system_deps() {
    print_status "Installing system dependencies..."
    
    if command_exists apt-get; then
        # Ubuntu/Debian
        sudo apt-get update
        sudo apt-get install -y \
            python3-pip \
            python3-venv \
            git \
            curl \
            wget \
            build-essential \
            cmake \
            libssl-dev \
            libffi-dev \
            libjpeg-dev \
            libpng-dev \
            libtiff-dev \
            libavcodec-dev \
            libavformat-dev \
            libswscale-dev \
            libv4l-dev \
            libxvidcore-dev \
            libx264-dev \
            libgtk-3-dev \
            libatlas-base-dev \
            gfortran \
            libhdf5-dev \
            libhdf5-serial-dev \
            libhdf5-103 \
            libqtgui4 \
            libqtwebkit4 \
            libqt4-test \
            python3-pyqt5 \
            libusb-1.0-0-dev \
            pkg-config
    elif command_exists yum; then
        # CentOS/RHEL/Fedora
        sudo yum update -y
        sudo yum install -y \
            python3-pip \
            python3-devel \
            git \
            curl \
            wget \
            gcc \
            gcc-c++ \
            cmake \
            openssl-devel \
            libffi-devel \
            libjpeg-devel \
            libpng-devel \
            libtiff-devel \
            ffmpeg-devel \
            gtk3-devel \
            atlas-devel \
            gfortran \
            hdf5-devel \
            qt5-qtbase-devel \
            qt5-qtwebkit-devel \
            libusb1-devel \
            pkgconfig
    elif command_exists brew; then
        # macOS
        brew update
        brew install \
            python@3.11 \
            cmake \
            openssl \
            libffi \
            jpeg \
            libpng \
            libtiff \
            ffmpeg \
            gtk+3 \
            openblas \
            gfortran \
            hdf5 \
            qt5 \
            libusb \
            pkg-config
    else
        print_warning "Unsupported package manager. Please install dependencies manually."
        return 1
    fi
    
    print_success "System dependencies installed"
}

# Function to install ROS2
install_ros2() {
    print_status "Installing ROS2..."
    
    if ! command_exists ros2; then
        if command_exists apt-get; then
            # Ubuntu/Debian
            print_status "Installing ROS2 Humble on Ubuntu..."
            
            # Add ROS2 repository
            sudo apt update && sudo apt install -y software-properties-common
            sudo add-apt-repository universe
            sudo apt update && sudo apt install -y curl
            sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
            echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
            
            # Install ROS2
            sudo apt update
            sudo apt install -y ros-humble-desktop
            
            # Install ROS2 development tools
            sudo apt install -y \
                ros-humble-ros-base \
                ros-humble-ros-core \
                ros-humble-rclpy \
                ros-humble-std-msgs \
                ros-humble-geometry-msgs \
                ros-humble-sensor-msgs \
                ros-humble-nav-msgs \
                ros-humble-tf2-ros \
                ros-humble-tf2-msgs \
                ros-humble-visualization-msgs \
                python3-colcon-common-extensions \
                python3-rosdep
            
            # Initialize rosdep
            sudo rosdep init
            rosdep update
            
        elif command_exists brew; then
            # macOS
            print_status "Installing ROS2 on macOS..."
            brew install ros2
        else
            print_warning "ROS2 installation not supported on this system. Please install manually."
            return 1
        fi
    else
        print_success "ROS2 already installed"
    fi
    
    print_success "ROS2 installation completed"
}

# Function to create virtual environment
create_venv() {
    print_status "Creating Python virtual environment..."
    
    if [ ! -d "venv" ]; then
        python3 -m venv venv
        print_success "Virtual environment created"
    else
        print_success "Virtual environment already exists"
    fi
    
    # Activate virtual environment
    source venv/bin/activate
    
    # Upgrade pip
    pip install --upgrade pip setuptools wheel
}

# Function to install Python dependencies
install_python_deps() {
    print_status "Installing Python dependencies..."
    
    # Activate virtual environment
    source venv/bin/activate
    
    # Install dependencies from requirements.txt
    if [ -f "requirements.txt" ]; then
        pip install -r requirements.txt
        print_success "Python dependencies installed from requirements.txt"
    else
        print_error "requirements.txt not found"
        return 1
    fi
}

# Function to download YOLO model
download_yolo_model() {
    print_status "Downloading YOLO model..."
    
    if [ ! -f "yolov8n.pt" ]; then
        wget -O yolov8n.pt https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt
        print_success "YOLO model downloaded"
    else
        print_success "YOLO model already exists"
    fi
}

# Function to build ROS messages
build_ros_messages() {
    print_status "Building ROS messages..."
    
    if [ -d "ros_xarm" ]; then
        cd ros_xarm
        
        # Source ROS2
        if [ -f "/opt/ros/humble/setup.bash" ]; then
            source /opt/ros/humble/setup.bash
        fi
        
        # Build custom messages
        if command_exists colcon; then
            colcon build --packages-select ufactory_ellmer_msgs
            print_success "ROS messages built"
        else
            print_warning "colcon not found, skipping ROS message build"
        fi
        
        cd ..
    else
        print_warning "ros_xarm directory not found, skipping ROS message build"
    fi
}

# Function to create environment setup script
create_setup_script() {
    print_status "Creating environment setup script..."
    
    cat > setup_env.sh << 'EOF'
#!/bin/bash
# Environment setup script for Robot Control System

# Activate virtual environment
if [ -d "venv" ]; then
    source venv/bin/activate
fi

# Source ROS2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Source ROS messages
if [ -f "ros_xarm/install/setup.bash" ]; then
    source ros_xarm/install/setup.bash
fi

# Set environment variables
export PYTHONPATH="${PYTHONPATH}:$(pwd)"
export WORLD_YAML="arm_config/world_model.yaml"
export ACTION_CONTRACT_MD="llm_planning_gemini/custom_gemini_examples/custom_gpt_action_schema.md"

echo "Environment setup complete!"
echo "Remember to set your GEMINI_API_KEY:"
echo "export GEMINI_API_KEY='your_api_key_here'"
EOF

    chmod +x setup_env.sh
    print_success "Environment setup script created"
}

# Function to create quick start script
create_quick_start() {
    print_status "Creating quick start script..."
    
    cat > quick_start.sh << 'EOF'
#!/bin/bash
# Quick start script for Robot Control System

# Setup environment
source setup_env.sh

# Check if API key is set
if [ -z "$GEMINI_API_KEY" ]; then
    echo "Warning: GEMINI_API_KEY not set"
    echo "Please set it with: export GEMINI_API_KEY='your_api_key_here'"
fi

# Start the system
echo "Starting Robot Control System..."
echo "Available commands:"
echo "  python main.py --task 'pick up the cup'"
echo "  python main.py --interactive"
echo "  python main.py --loop"
echo ""
echo "For help: python main.py --help"
EOF

    chmod +x quick_start.sh
    print_success "Quick start script created"
}

# Function to verify installation
verify_installation() {
    print_status "Verifying installation..."
    
    # Activate virtual environment
    source venv/bin/activate
    
    # Check Python packages
    python3 -c "
import sys
print('Python version:', sys.version)

# Check core dependencies
try:
    import xarm
    print('✓ xarm-python-sdk')
except ImportError:
    print('✗ xarm-python-sdk')

try:
    import google.generativeai
    print('✓ google-generativeai')
except ImportError:
    print('✗ google-generativeai')

try:
    import ultralytics
    print('✓ ultralytics')
except ImportError:
    print('✗ ultralytics')

try:
    import pyrealsense2
    print('✓ pyrealsense2')
except ImportError:
    print('✗ pyrealsense2')

try:
    import rclpy
    print('✓ rclpy')
except ImportError:
    print('✗ rclpy')

try:
    import yaml
    print('✓ pyyaml')
except ImportError:
    print('✗ pyyaml')

print('\\nInstallation verification complete!')
"
    
    # Check YOLO model
    if [ -f "yolov8n.pt" ]; then
        print_success "YOLO model found"
    else
        print_warning "YOLO model not found"
    fi
    
    # Check ROS messages
    if [ -f "ros_xarm/install/setup.bash" ]; then
        print_success "ROS messages built"
    else
        print_warning "ROS messages not built"
    fi
}

# Main installation function
main() {
    echo "=========================================="
    echo "Robot Control System - Installation Script"
    echo "=========================================="
    echo ""
    
    # Check Python version
    if ! check_python_version; then
        print_error "Python 3.8+ is required"
        exit 1
    fi
    
    # Install system dependencies
    install_system_deps
    
    # Install ROS2
    install_ros2
    
    # Create virtual environment
    create_venv
    
    # Install Python dependencies
    install_python_deps
    
    # Download YOLO model
    download_yolo_model
    
    # Build ROS messages
    build_ros_messages
    
    # Create setup scripts
    create_setup_script
    create_quick_start
    
    # Verify installation
    verify_installation
    
    echo ""
    echo "=========================================="
    print_success "Installation completed successfully!"
    echo "=========================================="
    echo ""
    echo "Next steps:"
    echo "1. Set your Gemini API key:"
    echo "   export GEMINI_API_KEY='your_api_key_here'"
    echo ""
    echo "2. Setup environment:"
    echo "   source setup_env.sh"
    echo ""
    echo "3. Quick start:"
    echo "   ./quick_start.sh"
    echo ""
    echo "4. Test the system:"
    echo "   python main.py --task 'move to home' --sim"
    echo ""
}

# Run main function
main "$@" 