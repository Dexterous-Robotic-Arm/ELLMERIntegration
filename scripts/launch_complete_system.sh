#!/bin/bash

# Complete Robot Control System Launch Script
# This script launches the camera, AprilTag detection, bridge, and robot control system

set -e  # Exit on any error

echo "🤖 Starting Complete Robot Control System..."
echo "=============================================="

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

# Check if ROS2 is available
if ! command -v ros2 &> /dev/null; then
    print_error "ROS2 not found. Please source ROS2 setup."
    exit 1
fi

# Source ROS2
print_status "Sourcing ROS2 Humble..."
source /opt/ros/humble/setup.bash

# Source workspace
WORKSPACE_PATH="/Users/eugenevorobiov/ELLMERIntegration/ros_workspace"
if [ -d "$WORKSPACE_PATH" ]; then
    print_status "Sourcing workspace..."
    source $WORKSPACE_PATH/install/setup.bash
else
    print_warning "Workspace not found at $WORKSPACE_PATH"
fi

# Function to launch camera system
launch_camera_system() {
    print_status "Launching camera system..."
    
    # Terminal 1: Camera Launch
    print_status "Starting camera launch (Terminal 1)..."
    gnome-terminal --title="Camera Launch" -- bash -c "
        source /opt/ros/humble/setup.bash
        ros2 launch realsense2_camera rs_launch.py \
        camera_name:=cam_hand \
        serial_no:=\"'153122078759'\" \
        camera_info_url:=file:///Users/eugenevorobiov/ELLMERIntegration/ros_workspace/src/april_tags_vision/config/cam_hand.yaml
        read -p 'Press Enter to close...'
    " &
    
    sleep 3
    
    # Terminal 2: Image Processing
    print_status "Starting image processing (Terminal 2)..."
    gnome-terminal --title="Image Processing" -- bash -c "
        source /opt/ros/humble/setup.bash
        source $WORKSPACE_PATH/install/setup.bash
        ros2 run image_proc image_proc --ros-args \
          -r image:=/camera/cam_hand/color/image_raw \
          -r camera_info:=/camera/cam_hand/color/camera_info \
          -r __ns:=/cam_hand \
          -p qos_overrides./camera/cam_hand/color/camera_info.reliability:=best_effort
        read -p 'Press Enter to close...'
    " &
    
    sleep 3
    
    # Terminal 3: AprilTag Detection
    print_status "Starting AprilTag detection (Terminal 3)..."
    gnome-terminal --title="AprilTag Detection" -- bash -c "
        source /opt/ros/humble/setup.bash
        source $WORKSPACE_PATH/install/setup.bash
        ros2 run apriltag_ros apriltag_node \
        --ros-args \
        -r image_rect:=/cam_hand/image_rect \
        -r /cam_hand/camera_info:=/camera/cam_hand/color/camera_info \
        -r detections:=/cam_hand/tag_detections \
        -p family:=36h11 \
        -p size:=0.064 \
        -p pose_estimation_method:=pnp \
        -p z_up:=true \
        -p approximate_sync:=true \
        -p publish_tf:=true \
        -p qos_overrides./cam_hand/camera_info.reliability:=best_effort \
        -p qos_overrides./cam_hand/image_rect.reliability:=best_effort \
        -p tag.ids:=\"[0]\" \
        -p tag.frames:=\"[tag36h11:0_hand]\"
        read -p 'Press Enter to close...'
    " &
    
    sleep 5
    print_success "Camera system launched!"
}

# Function to launch AprilTag bridge
launch_apriltag_bridge() {
    print_status "Launching AprilTag bridge (Terminal 4)..."
    gnome-terminal --title="AprilTag Bridge" -- bash -c "
        source /opt/ros/humble/setup.bash
        source $WORKSPACE_PATH/install/setup.bash
        cd /Users/eugenevorobiov/ELLMERIntegration
        python3 robot_control/vision_system/apriltag_bridge.py
        read -p 'Press Enter to close...'
    " &
    
    sleep 3
    print_success "AprilTag bridge launched!"
}

# Function to launch robot control system
launch_robot_control() {
    print_status "Launching robot control system (Terminal 5)..."
    gnome-terminal --title="Robot Control" -- bash -c "
        source /opt/ros/humble/setup.bash
        source $WORKSPACE_PATH/install/setup.bash
        cd /Users/eugenevorobiov/ELLMERIntegration
        python3 robot_control/main.py
        read -p 'Press Enter to close...'
    " &
    
    sleep 3
    print_success "Robot control system launched!"
}

# Function to show monitoring terminal
launch_monitoring() {
    print_status "Launching monitoring terminal (Terminal 6)..."
    gnome-terminal --title="System Monitor" -- bash -c "
        source /opt/ros/humble/setup.bash
        source $WORKSPACE_PATH/install/setup.bash
        
        echo '🔍 System Monitoring'
        echo '==================='
        echo ''
        echo 'Available topics:'
        ros2 topic list
        echo ''
        echo 'Press Ctrl+C to stop monitoring'
        echo ''
        
        # Monitor detected objects
        echo 'Monitoring detected objects...'
        ros2 topic echo /detected_objects --once
        
        # Keep terminal open
        read -p 'Press Enter to close...'
    " &
    
    print_success "Monitoring terminal launched!"
}

# Main execution
main() {
    print_status "Starting complete robot control system..."
    
    # Check if we're on macOS (no gnome-terminal)
    if [[ "$OSTYPE" == "darwin"* ]]; then
        print_warning "macOS detected. Please run each command manually in separate terminals:"
        echo ""
        echo "Terminal 1 - Camera Launch:"
        echo "source /opt/ros/humble/setup.bash"
        echo "ros2 launch realsense2_camera rs_launch.py camera_name:=cam_hand serial_no:=\"'153122078759'\" camera_info_url:=file:///Users/eugenevorobiov/ELLMERIntegration/ros_workspace/src/april_tags_vision/config/cam_hand.yaml"
        echo ""
        echo "Terminal 2 - Image Processing:"
        echo "source /opt/ros/humble/setup.bash"
        echo "source $WORKSPACE_PATH/install/setup.bash"
        echo "ros2 run image_proc image_proc --ros-args -r image:=/camera/cam_hand/color/image_raw -r camera_info:=/camera/cam_hand/color/camera_info -r __ns:=/cam_hand -p qos_overrides./camera/cam_hand/color/camera_info.reliability:=best_effort"
        echo ""
        echo "Terminal 3 - AprilTag Detection:"
        echo "source /opt/ros/humble/setup.bash"
        echo "source $WORKSPACE_PATH/install/setup.bash"
        echo "ros2 run apriltag_ros apriltag_node --ros-args -r image_rect:=/cam_hand/image_rect -r /cam_hand/camera_info:=/camera/cam_hand/color/camera_info -r detections:=/cam_hand/tag_detections -p family:=36h11 -p size:=0.064 -p pose_estimation_method:=pnp -p z_up:=true -p approximate_sync:=true -p publish_tf:=true -p qos_overrides./cam_hand/camera_info.reliability:=best_effort -p qos_overrides./cam_hand/image_rect.reliability:=best_effort -p tag.ids:=\"[0]\" -p tag.frames:=\"[tag36h11:0_hand]\""
        echo ""
        echo "Terminal 4 - AprilTag Bridge:"
        echo "source /opt/ros/humble/setup.bash"
        echo "source $WORKSPACE_PATH/install/setup.bash"
        echo "cd /Users/eugenevorobiov/ELLMERIntegration"
        echo "python3 robot_control/vision_system/apriltag_bridge.py"
        echo ""
        echo "Terminal 5 - Robot Control:"
        echo "source /opt/ros/humble/setup.bash"
        echo "source $WORKSPACE_PATH/install/setup.bash"
        echo "cd /Users/eugenevorobiov/ELLMERIntegration"
        echo "python3 robot_control/main.py"
        echo ""
        echo "Terminal 6 - Monitoring:"
        echo "source /opt/ros/humble/setup.bash"
        echo "source $WORKSPACE_PATH/install/setup.bash"
        echo "ros2 topic echo /detected_objects"
        echo ""
        return
    fi
    
    # Launch all components
    launch_camera_system
    launch_apriltag_bridge
    launch_robot_control
    launch_monitoring
    
    print_success "Complete system launched!"
    print_status "All terminals are now running. Check each terminal for status."
    print_status "Use the robot control terminal to send commands."
    print_status "Monitor detected objects in the monitoring terminal."
    
    echo ""
    echo "🎯 System Ready!"
    echo "==============="
    echo "• Camera system: Running"
    echo "• AprilTag detection: Running" 
    echo "• AprilTag bridge: Running"
    echo "• Robot control: Running"
    echo "• Monitoring: Running"
    echo ""
    echo "💡 Try commands like:"
    echo "   'approach the bottle'"
    echo "   'move to the cup'"
    echo "   'scan for objects'"
}

# Run main function
main
