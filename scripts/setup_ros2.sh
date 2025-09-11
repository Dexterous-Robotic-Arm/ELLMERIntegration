#!/bin/bash
# ROS2 Setup Script for April Tags Vision Package

echo "🤖 Setting up ROS2 for April Tags Vision"
echo "========================================="

# Check if ROS2 is installed
if ! command -v colcon &> /dev/null; then
    echo "❌ ROS2 not found. Installing ROS2 Humble..."
    echo "Please run the following commands:"
    echo ""
    echo "1. Install ROS2 Humble:"
    echo "   sudo apt update"
    echo "   sudo apt install software-properties-common"
    echo "   sudo add-apt-repository universe"
    echo "   sudo apt update && sudo apt install curl -y"
    echo "   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg"
    echo "   echo \"deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \$(. /etc/os-release && echo \$UBUNTU_CODENAME) main\" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null"
    echo "   sudo apt update"
    echo "   sudo apt upgrade -y"
    echo "   sudo apt install ros-humble-desktop python3-colcon-common-extensions python3-rosdep python3-argcomplete ros-humble-ament-cmake ros-humble-ament-cmake-auto -y"
    echo ""
    echo "2. Setup ROS2 environment:"
    echo "   echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc"
    echo "   source ~/.bashrc"
    echo ""
    echo "3. Install additional ROS2 packages:"
    echo "   sudo apt install ros-humble-realsense2-camera ros-humble-image-proc ros-humble-apriltag-ros -y"
    echo ""
    echo "4. Then run this script again:"
    echo "   bash scripts/setup_ros2.sh"
    exit 1
fi

echo "✅ ROS2 found. Setting up April Tags Vision package..."

# Navigate to ROS workspace
cd ros_workspace

# Install dependencies
echo "📦 Installing ROS2 dependencies..."
sudo apt update
sudo apt install ros-humble-realsense2-camera ros-humble-image-proc ros-humble-apriltag-ros python3-colcon-common-extensions python3-rosdep python3-argcomplete -y

# Install ament_cmake and other build tools
echo "🔧 Installing ROS2 build tools..."
sudo apt install ros-humble-ament-cmake ros-humble-ament-cmake-auto ros-humble-ament-cmake-gtest ros-humble-ament-cmake-pytest -y

# Build the package
echo "🏗️ Building April Tags Vision package..."
colcon build --packages-select april_tags_vision

if [ $? -eq 0 ]; then
    echo "✅ ROS2 package built successfully!"
    echo ""
    echo "🚀 To use the ROS2 package:"
    echo "1. Source the workspace:"
    echo "   cd ros_workspace"
    echo "   source install/setup.bash"
    echo ""
    echo "2. Launch the complete system:"
    echo "   ros2 launch april_tags_vision complete_system.launch.py"
    echo ""
    echo "3. Or launch individual components:"
    echo "   ros2 launch april_tags_vision realsense_camera.launch.py"
    echo "   ros2 launch april_tags_vision image_proc.launch.py"
    echo "   ros2 launch april_tags_vision april_tags_detection.launch.py"
else
    echo "❌ ROS2 package build failed!"
    echo "Check the error messages above and fix any issues."
    exit 1
fi

cd ..
echo "✅ ROS2 setup complete!"
