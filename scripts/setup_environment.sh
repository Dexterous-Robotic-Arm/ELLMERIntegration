#!/bin/bash
# Simple setup script for April Tags robot control system

echo "🚀 Setting up April Tags Robot Control System"
echo "============================================="

# Install Python dependencies
echo "📦 Installing Python dependencies..."

# Check if pip is available
if ! command -v pip &> /dev/null; then
    echo "❌ pip not found. Installing pip..."
    sudo apt update
    sudo apt install python3-pip -y
fi

# Install Python packages
pip install numpy scipy opencv-python apriltag xarm-python-sdk dynamixel-sdk google-generativeai openai chromadb sentence-transformers faiss-cpu langchain tiktoken pyrealsense2

# Optional: Install ROS2 dependencies
echo "📦 Installing ROS2 dependencies (optional)..."
pip install rclpy std_msgs geometry_msgs sensor_msgs tf2_ros cv_bridge 2>/dev/null || echo "⚠️ ROS2 dependencies failed - ROS features will be limited"

# Setup environment
echo "🔧 Setting up environment..."
export PYTHONPATH="${PYTHONPATH}:$(pwd)"
export GEMINI_API_KEY="${GEMINI_API_KEY:-}"
export XARM_IP="${XARM_IP:-192.168.1.241}"

# Create environment script
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

echo "✅ Setup complete!"
echo ""
echo "Next steps:"
echo "1. Print April Tags from: https://github.com/AprilRobotics/apriltag-imgs"
echo "2. Use TagStandard41h12 family, 100mm size"
echo "3. Attach tag to water bottle"
echo "4. Setup environment: source scripts/setup_env.sh"
echo "5. Run system: python3 robot_control/main.py --task 'pick up the water bottle' --sim --dry-run"
echo ""
echo "For ROS2 package:"
echo "1. Run ROS2 setup: bash scripts/setup_ros2.sh"
echo "2. Or manually:"
echo "   cd ros_workspace"
echo "   colcon build --packages-select april_tags_vision"
echo "   source install/setup.bash"
