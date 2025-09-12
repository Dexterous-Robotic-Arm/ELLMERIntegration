#!/bin/bash
export PYTHONPATH="${PYTHONPATH}:$(pwd)"
export GEMINI_API_KEY="${GEMINI_API_KEY:-}"
export XARM_IP="${XARM_IP:-192.168.1.241}"
echo "✅ Environment setup complete!"
echo "Python path: $PYTHONPATH"
echo "XArm IP: $XARM_IP"
echo "Gemini API Key: ${GEMINI_API_KEY:+SET}"
