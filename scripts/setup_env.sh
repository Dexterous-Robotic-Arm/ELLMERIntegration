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
if [ -f "ros_workspace/install/setup.bash" ]; then
    source ros_workspace/install/setup.bash
fi

# Set environment variables
export PYTHONPATH="${PYTHONPATH}:$(pwd)"
export WORLD_YAML="config/robot/world_model.yaml"
export ACTION_CONTRACT_MD="config/llm/action_schema.md"

echo "Environment setup complete!"
echo "Remember to set your GEMINI_API_KEY:"
echo "export GEMINI_API_KEY='your_api_key_here'" 