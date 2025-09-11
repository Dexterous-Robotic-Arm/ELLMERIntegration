#!/bin/bash
# Complete build and run script for April Tags robot system

echo "🤖 Building and Setting Up April Tags Robot System"
echo "=================================================="

# 1. Install dependencies
echo "📦 Installing dependencies..."
pip install numpy scipy opencv-python apriltag xarm-python-sdk dynamixel-sdk google-generativeai openai chromadb sentence-transformers faiss-cpu langchain tiktoken pyrealsense2

# Optional ROS2 dependencies
echo "📦 Installing ROS2 dependencies (optional)..."
pip install rclpy std_msgs geometry_msgs sensor_msgs tf2_ros cv_bridge 2>/dev/null || echo "⚠️ ROS2 dependencies failed - ROS features will be limited"

# 2. Setup environment
echo "🔧 Setting up environment..."
export PYTHONPATH="${PYTHONPATH}:$(pwd)"
export GEMINI_API_KEY="${GEMINI_API_KEY:-}"
export XARM_IP="${XARM_IP:-192.168.1.241}"

# 3. Build ROS package
echo "🏗️ Building ROS package..."
cd ros_workspace

# Check if ROS2 is available
if command -v colcon &> /dev/null; then
    colcon build --packages-select april_tags_vision
    source install/setup.bash
    echo "✅ ROS package built successfully"
else
    echo "⚠️ ROS2 not available - skipping ROS package build"
fi

cd ..

# 4. Create environment setup script
echo "📝 Creating environment setup script..."
cat > scripts/setup_env.sh << 'EOF'
#!/bin/bash
export PYTHONPATH="${PYTHONPATH}:$(pwd)"
export GEMINI_API_KEY="${GEMINI_API_KEY:-}"
export XARM_IP="${XARM_IP:-192.168.1.241}"
echo "✅ Environment setup complete!"
echo "Python path: $PYTHONPATH"
echo "XArm IP: $XARM_IP"
echo "Gemini API Key: ${GEMINI_API_KEY:+SET}"
EOF

chmod +x scripts/setup_env.sh

echo "✅ Build and setup complete!"
echo ""
echo "🎯 System Configuration:"
echo "  - April Tags: TagStandard41h12, 100mm size"
echo "  - Object Mapping: 0=bottle, 1=book, 2=cup, etc."
echo "  - Robot Speed: Reduced for precision (50mm/s)"
echo "  - Collision Avoidance: Enabled"
echo "  - Continuous Vision: Enabled"
echo ""
echo "🚀 How to Run:"
echo ""
echo "1. Setup environment:"
echo "   source scripts/setup_env.sh"
echo ""
echo "2. Print April Tags:"
echo "   Download from: https://github.com/AprilRobotics/apriltag-imgs"
echo "   Use TagStandard41h12 family, 100mm size"
echo "   Attach to objects: 0=bottle, 1=book, 2=cup, etc."
echo ""
echo "3. Run the system:"
echo "   # Simulation mode (safe)"
echo "   python3 robot_control/main.py --task 'pick up the bottle' --sim --dry-run"
echo ""
echo "   # Real robot mode"
echo "   python3 robot_control/main.py --task 'pick up the bottle'"
echo ""
echo "   # Interactive mode"
echo "   python3 robot_control/main.py --interactive"
echo ""
echo "4. ROS Integration (if ROS2 available):"
echo "   # Terminal 1: Launch complete system"
echo "   cd ros_workspace"
echo "   source install/setup.bash"
echo "   ros2 launch april_tags_vision complete_system.launch.py"
echo ""
echo "   # Terminal 2: Run robot control"
echo "   source scripts/setup_env.sh"
echo "   python3 robot_control/main.py --task 'find the book in the pile and grab it'"
echo ""
echo "🎯 Example Tasks:"
echo "  - 'pick up the bottle'"
echo "  - 'find the book in the pile and grab it'"
echo "  - 'move the cup to the left'"
echo "  - 'organize the desk objects'"
echo ""
echo "💡 The system will:"
echo "  - Detect all April Tags with different IDs"
echo "  - Prioritize the object mentioned in the prompt"
echo "  - Use collision avoidance"
echo "  - Perform precise millimeter-level movements"
echo "  - Reason about object stacking and manipulation"
