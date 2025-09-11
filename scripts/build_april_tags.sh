#!/bin/bash
# Build script for April Tags Vision ROS package

echo "🏗️ Building April Tags Vision ROS Package"
echo "=========================================="

# Check if we're in the right directory
if [ ! -d "src/april_tags_vision" ]; then
    echo "❌ Error: april_tags_vision package not found in src/"
    echo "   Please run this script from the ros_workspace directory"
    exit 1
fi

# Check if ROS 2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "⚠️ Warning: ROS 2 not sourced. Attempting to source..."
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        echo "✅ Sourced ROS 2 Humble"
    elif [ -f "/opt/ros/iron/setup.bash" ]; then
        source /opt/ros/iron/setup.bash
        echo "✅ Sourced ROS 2 Iron"
    else
        echo "❌ Error: ROS 2 not found. Please install ROS 2 first."
        exit 1
    fi
fi

# Install Python dependencies
echo "📦 Installing Python dependencies..."
pip install apriltag opencv-python numpy scipy

# Build the package
echo "🔨 Building ROS package..."
colcon build --packages-select april_tags_vision

# Check build result
if [ $? -eq 0 ]; then
    echo "✅ Build successful!"
    
    # Source the workspace
    source install/setup.bash
    echo "✅ Workspace sourced"
    
    # Test the package
    echo "🧪 Testing package..."
    ros2 pkg list | grep april_tags_vision
    if [ $? -eq 0 ]; then
        echo "✅ Package found in ROS 2"
    else
        echo "❌ Package not found in ROS 2"
    fi
    
    # Show available executables
    echo "📋 Available executables:"
    ros2 run april_tags_vision --help 2>/dev/null || echo "   (No help available)"
    
    echo ""
    echo "🎉 April Tags Vision package is ready!"
    echo ""
    echo "Usage:"
    echo "  ros2 launch april_tags_vision april_tags_detector.launch.py"
    echo "  ros2 run april_tags_vision test_april_tags.py"
    echo ""
    
else
    echo "❌ Build failed!"
    exit 1
fi
