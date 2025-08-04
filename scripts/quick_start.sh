#!/bin/bash
# Quick start script for Robot Control System

# Setup environment
source scripts/setup_env.sh

# Check if API key is set
if [ -z "$GEMINI_API_KEY" ]; then
    echo "Warning: GEMINI_API_KEY not set"
    echo "Please set it with: export GEMINI_API_KEY='your_api_key_here'"
fi

# Start the system
echo "Starting Robot Control System..."
echo "Available commands:"
echo "  python robot_control/main.py --task 'pick up the cup'"
echo "  python robot_control/main.py --interactive"
echo "  python robot_control/main.py --loop"
echo ""
echo "For help: python robot_control/main.py --help" 