# ELLMER Integration Project

This repository contains two main components:
1.  `ros_xarm`: A ROS 2 workspace for controlling the xArm robotic arm and related functionalities.
2.  `llm_planning_gemini`: A Python project for Large Language Model-based task planning using Google Gemini.

## Prerequisites

*   Ubuntu 22.04
*   ROS 2 Humble Hawksbill: [Follow the installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
*   Git
*   Python 3.10+ and `venv`

## Getting Started on Ubuntu

### 1. Clone the Repository

First, clone this repository to your local machine.

```bash
git clone https://github.com/Dexterous-Robotic-Arm/ELLMERIntegration.git
cd ELLMERIntegration
```

### 2. Set Up and Build the ROS 2 Workspace (`ros_xarm`)

The `ros_xarm` directory contains the ROS 2 packages. You need to build them using `colcon`.

```bash
# Navigate to the ROS workspace directory
cd ros_xarm

# Install dependencies using rosdep
# This command will check the package.xml files in the src directory
# and install any required ROS dependencies.
sudo apt-get update
rosdep update
rosdep install -i --from-path src -y --rosdistro humble

# Build the workspace
colcon build
```

### 3. Run the ROS 2 Nodes

After a successful build, you can run the ROS 2 nodes.

```bash
# Source the workspace overlay to make packages available
cd ros_xarm
source install/setup.bash

# Example: Launch the pose recorder
# (Replace with the actual launch file you need)
ros2 launch kortex_examples record_pose.launch.py
```

### 4. Set Up and Run the LLM Planning Project (`llm_planning_gemini`)

The `llm_planning_gemini` project is a standard Python application. It is recommended to run it in a virtual environment.

```bash
# Navigate to the llm_planning_gemini directory from the repo root
cd llm_planning_gemini

# Create and activate a Python virtual environment
python3 -m venv venv
source venv/bin/activate

# Install required Python packages
# NOTE: A requirements.txt file is not provided.
# You will need to install libraries like google-generativeai,
# and any others based on the imports in the scripts.
pip install google-generativeai "haystack-ai"

# Set up your environment variables
# Copy the example .env file and add your API key
cp .env.example .env
nano .env  # Add your GOOGLE_API_KEY

# Run an example script
python3 haystack_rag_pipelines/example1.py
