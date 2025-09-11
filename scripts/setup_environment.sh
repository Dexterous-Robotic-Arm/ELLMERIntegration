#!/bin/bash
# Simple setup script for April Tags robot control system

echo "🚀 Setting up April Tags Robot Control System"
echo "============================================="

# Install Python dependencies
echo "📦 Installing Python dependencies..."
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
cat > setup_env.sh << 'EOF'
#!/bin/bash
export PYTHONPATH="${PYTHONPATH}:$(pwd)"
export GEMINI_API_KEY="${GEMINI_API_KEY:-}"
export XARM_IP="${XARM_IP:-192.168.1.241}"
echo "✅ Environment setup complete!"
echo "Python path: $PYTHONPATH"
echo "XArm IP: $XARM_IP"
echo "Gemini API Key: ${GEMINI_API_KEY:+SET}"
EOF

chmod +x setup_env.sh

echo "✅ Setup complete!"
echo ""
echo "Next steps:"
echo "1. Print April Tags from: https://github.com/AprilRobotics/apriltag-imgs"
echo "2. Use TagStandard41h12 family, 50mm size"
echo "3. Attach tag to water bottle"
echo "4. Setup environment: source setup_env.sh"
echo "5. Run system: python3 robot_control/main.py --task 'pick up the water bottle' --sim --dry-run"
echo ""
echo "For ROS package:"
echo "1. cd ros_workspace"
echo "2. colcon build --packages-select april_tags_vision"
echo "3. source install/setup.bash"
