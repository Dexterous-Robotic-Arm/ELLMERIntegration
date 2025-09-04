# ELLMER Integration - Robot Control System

A comprehensive robot control system for the ufactory850 robotic arm with Intel RealSense D435 camera, designed for real-world autonomous tasks using **RAG-based LLM planning** and computer vision.

## 🤖 System Overview

This system enables autonomous robot control for complex tasks using:
- **RAG-based LLM Planning**: Google Gemini LLM with Retrieval-Augmented Generation for intelligent task planning and **automatic reprompting when confused**
- **Enhanced Computer Vision**: YOLO object detection with RealSense depth sensing and accurate 3D positioning
- **Direct Robot Control**: XArm Python SDK for precise robot manipulation
- **Safety First**: Comprehensive safety limits and emergency stop functionality
- **Modular Architecture**: Clean separation of planning, vision, execution, and movement logic
- **Self-Healing**: Automatic error detection, rescanning, and plan regeneration

### Key Features
- ✅ **Real Hardware Support**: ufactory850 robot arm + RealSense D435 camera
- ✅ **RAG-based LLM Planning**: Google Gemini integration with enhanced movement logic
- ✅ **Automatic Reprompting**: LLM automatically rescan and regenerate plans when confused
- ✅ **Enhanced Computer Vision**: YOLO + RealSense for accurate 3D object detection and positioning
- ✅ **Dynamixel Gripper Control**: Full servo-based gripper manipulation with force feedback
- ✅ **Safety Monitoring**: Real-time safety checks and limits
- ✅ **ROS2 Integration**: Optional ROS2 support for vision data
- ✅ **Simulation Mode**: Test without hardware connection
- ✅ **Enhanced Movement Logic**: Accurate 3D positioning using camera FOV and depth data
- ✅ **Universal Object Interaction**: Handles complex objects (kitchen appliances, etc.) without hardcoded logic

## 🏗️ Architecture

```
ELLMERIntegration/
├── README.md                           # This comprehensive documentation
├── requirements.txt                    # Python dependencies
├── config/                            # Configuration files
│   ├── robot/                         # Robot-specific configs
│   │   ├── safety_config.yaml        # Safety limits and constraints
│   │   └── world_model.yaml          # Named poses and world model
│   ├── vision/                        # Vision system configs
│   │   └── camera_config.yaml        # Camera calibration and settings
│   ├── gripper/                       # Gripper configs
│   │   ├── gripper_config.yaml       # General gripper settings
│   │   └── gripper_config_xl330.yaml # XL330-specific settings
│   ├── llm/                          # LLM configs
│   │   └── action_schema.md          # Action definitions for LLM
│   └── rag_config.yaml               # RAG system configuration
├── robot_control/                     # Main robot control system
│   ├── __init__.py
│   ├── main.py                       # Legacy main entry point (uses RAG system)
│   ├── rag_system/                   # RAG-based system (NEW)
│   │   ├── __init__.py
│   │   ├── rag_main.py              # RAG system main entry point
│   │   ├── planner/                 # Planning components
│   │   │   ├── __init__.py
│   │   │   ├── rag_planner.py       # RAG-based LLM planner (with reprompting)
│   │   │   └── movement_logic.py    # Enhanced movement logic
│   │   └── integration/             # Integration layer
│   │       ├── __init__.py
│   │       └── rag_integration.py   # RAG integration
│   ├── robot_controller/            # Robot control components
│   │   ├── __init__.py
│   │   ├── actions_xarm.py         # XArm robot actions
│   │   ├── executor.py             # Task execution
│   │   └── gripper.py              # Gripper control
│   ├── vision_system/              # Vision components
│   │   ├── __init__.py
│   │   └── pose_recorder.py        # Object detection and pose recording
│   └── utils/                      # Utility functions
│       ├── __init__.py
│       ├── config_manager.py       # Configuration management
│       └── logging_utils.py        # Logging utilities
├── scripts/                         # Utility scripts
│   ├── install_dependencies.sh     # Installation script
│   ├── setup_env.sh                # Environment setup
│   ├── quick_start.sh              # Quick start script
│   └── setup_dynamixel.sh          # Dynamixel gripper setup
├── tests/                          # Test files
│   ├── test_basic_structure.py     # Basic structure tests
│   ├── test_robot_control.py       # Robot control tests
│   ├── test_rag_system.py          # RAG system tests
│   └── debug_vision_complete.py    # Vision system tests
├── docs/                           # Documentation
│   └── testing_guide.md            # Testing guide
└── ros_workspace/                  # ROS2 workspace (optional)
    └── src/                        # ROS2 packages
```

## 🚀 Quick Start

### 1. Installation

```bash
# Clone the repository
git clone <your-repo-url> ELLMERIntegration
cd ELLMERIntegration

# Install dependencies
chmod +x scripts/install_dependencies.sh
./scripts/install_dependencies.sh

   # Setup environment
source scripts/setup_env.sh
```

### 2. Basic Usage

#### RAG-based System (Recommended)
```bash
# Start RAG-based system with automatic reprompting
python -m robot_control.rag_system.rag_main --task "pick up the red cup"

# Interactive mode
python -m robot_control.rag_system.rag_main --interactive

# Simulation mode
python -m robot_control.rag_system.rag_main --sim --task "test movement"
```

#### Legacy System (Uses RAG System Internally)
```bash
# Start legacy system (now uses RAG system internally)
python -m robot_control.main --task "pick up the red cup"

# Interactive mode
python -m robot_control.main --interactive
```

### 3. Configuration

Edit configuration files in `config/` directory:

```yaml
# config/robot/safety_config.yaml
safety_limits:
  max_velocity: 100.0
  max_acceleration: 50.0
  workspace_limits:
    x: [-500, 500]
    y: [-500, 500]
    z: [0, 500]
```

## 🎯 Key Components

### 1. RAG-based LLM Planner
- **Enhanced Movement Logic**: Accurate 3D positioning using camera FOV and depth data
- **Intelligent Planning**: LLM-based task planning with real-time feedback
- **Safety-Aware**: Automatic safety distance calculations and position validation
- **Automatic Reprompting**: When confused, automatically rescan and regenerate plans
- **Universal Object Interaction**: Handles complex objects without hardcoded logic
- **Error Recovery**: Up to 3 retries with improving context on each attempt

### 2. Vision System
- **Object Detection**: YOLO-based object detection with RealSense depth sensing
- **3D Positioning**: Accurate 3D position calculation using camera calibration
- **Real-time Updates**: Smooth position updates during movement
- **Scan Quality Assessment**: Automatic assessment of vision data quality

### 3. Robot Controller
- **XArm Integration**: Direct control of ufactory850 robot arm
- **Gripper Control**: Dynamixel servo-based gripper manipulation
- **Safety Monitoring**: Real-time safety checks and limits

### 4. Task Execution
- **Plan Execution**: Execute LLM-generated plans
- **Error Recovery**: Handle errors and adapt plans
- **Real-time Feedback**: Provide feedback to LLM for plan adaptation
- **Automatic Rescanning**: Rescan environment when confusion detected

## 🔄 RAG System Features

### Automatic Reprompting & Confusion Detection
The RAG system automatically detects when it's confused and takes corrective action:

```python
# Automatic retry loop with rescanning
retry_count = 0
while retry_count < self.max_retries:
    try:
        # Create RAG context with fresh data
        context = self._create_rag_context(task_description)
        
        # Generate plan
        plan = self._generate_plan_with_llm(context)
        
        # Execute plan
        results = self._execute_plan(plan)
        
        if results.get('success', False):
            return results  # Success!
        else:
            # Task failed, try again
            retry_count += 1
            
            # If confused, scan again
            if retry_count < self.max_retries:
                self._scan_area(duration=3.0)  # Rescan for better context
```

### Confusion Detection Triggers
- **Plan Execution Failure**: Any step in the plan fails
- **Vision Uncertainty**: Low confidence in object detection
- **Position Errors**: Invalid target positions
- **Gripper Issues**: Failed grasp attempts
- **Safety Violations**: Position outside safe limits

### Recovery Actions
1. **Rescan Environment**: `self._scan_area(duration=3.0)`
2. **Update Context**: Fresh vision data + current robot state
3. **Regenerate Plan**: New LLM prompt with updated context
4. **Retry Execution**: Execute new plan with better information

### Universal Object Interaction
The system handles complex objects (like kitchen appliances) through universal patterns:

```python
# Object interaction requirements analysis
interaction_requirements = {
    "needs_vision": True,
    "object_types": ["appliance", "door", "button"],
    "interaction_types": ["opening", "pressing", "grasping"],
    "safety_considerations": ["hot_surface", "moving_parts"],
    "complexity_level": "high"
}
```

## 🔧 Configuration

### Robot Configuration
```yaml
# config/robot/safety_config.yaml
robot:
  ip: "192.168.1.241"
  safety_limits:
    max_velocity: 100.0
    max_acceleration: 50.0
workspace_limits:
      x: [-500, 500]
      y: [-500, 500]
      z: [0, 500]
```

### Vision Configuration
```yaml
# config/vision/camera_config.yaml
camera:
  type: "realsense_d435"
  resolution: [640, 480]
  fps: 30
  depth_scale: 1000.0
```

### RAG System Configuration
```yaml
# config/rag_config.yaml
rag_planner:
  max_retries: 3
  scan_timeout: 10.0
  planning_timeout: 30.0
  
  llm:
    provider: "gemini"
    model: "gemini-2.5-pro"
    temperature: 0.1
    
  planning:
    enable_adaptive_planning: true
    enable_error_recovery: true
    enable_scanning_on_confusion: true
```

## 🧪 Testing

### Run Tests
```bash
# Run all tests
python -m pytest tests/

# Run specific test
python -m pytest tests/test_robot_control.py
```

### Test RAG System
```bash
# Test RAG system with reprompting
python tests/test_rag_system.py
```

## 📚 Documentation

- **Testing Guide**: `docs/testing_guide.md`
- **RAG System**: `robot_control/rag_system/` (see individual module docstrings)

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests
5. Submit a pull request

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 🆘 Support

For support and questions:
1. Check the documentation
2. Run tests to verify setup
3. Open an issue on GitHub

## 🎉 Acknowledgments

- ufactory850 robot arm
- Intel RealSense D435 camera
- Google Gemini LLM
- YOLO object detection
- ROS2 community
- ELIXIR Research Lab