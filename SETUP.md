# 🚀 ELLMERIntegration Setup Guide

Complete setup instructions for a fresh clone of the repository.

## 📋 Prerequisites

- **Python 3.8+** (Python 3.10+ recommended)
- **Ubuntu 22.04** (for full hardware support)
- **Git** installed

## ⚡ Quick Setup (5 minutes)

### 1. Clone & Navigate
```bash
git clone <your-repo-url>
cd ELLMERIntegration
```

### 2. Create Virtual Environment
```bash
# Create venv
python3 -m venv venv

# Activate venv (Linux/Mac)
source venv/bin/activate

# Or on Windows
# venv\Scripts\activate
```

### 3. Install Dependencies
```bash
# Install Python dependencies
pip install --upgrade pip
pip install -r requirements.txt
```

### 4. Set API Key
```bash
# Get free API key from: https://makersuite.google.com/app/apikey
export GEMINI_API_KEY='your_gemini_api_key_here'

# Or add to ~/.bashrc for persistence:
echo 'export GEMINI_API_KEY="your_gemini_api_key_here"' >> ~/.bashrc
source ~/.bashrc
```

### 5. Test Setup
```bash
# Test RAG system (no hardware needed)
python test_rag_system.py

# Test Gemini compatibility
python simple_prompt_test.py

# Test basic structure
python tests/test_basic_structure.py
```

## 🎯 Usage Examples

### Interactive Mode (RAG Testing)
```bash
# Start interactive mode
python robot_control/main.py --interactive

# Example commands:
robot> move toward bottle
robot> pick up the cup
robot> scan workspace
```

### Single Task Mode
```bash
# Run specific task
python robot_control/main.py --task "move toward bottle"
```

### Help
```bash
python robot_control/main.py --help
```

## 🔧 Advanced Setup (Hardware Support)

If you need full hardware support (robot arm, camera, ROS2):

### 1. Run Full Installation Script
```bash
# For Ubuntu 22.04 with hardware
chmod +x scripts/install_dependencies.sh
./scripts/install_dependencies.sh
```

This installs:
- ROS2 Humble
- RealSense SDK
- Robot drivers
- Additional system dependencies

### 2. Build ROS Workspace
```bash
# Build ROS2 packages
cd ros_workspace
colcon build
source install/setup.bash
cd ..
```

### 3. Setup Environment
```bash
# Use the setup script
source scripts/setup_env.sh

# Or manually:
export PYTHONPATH="${PYTHONPATH}:$(pwd)"
export WORLD_YAML="config/robot/world_model.yaml"
export ACTION_CONTRACT_MD="config/llm/action_schema.md"
```

## 📁 Project Structure

```
ELLMERIntegration/
├── robot_control/          # Main Python package
│   ├── main.py            # Entry point
│   ├── rag_system/        # RAG intelligence system
│   ├── robot_controller/  # Robot hardware control
│   └── vision_system/     # Camera and object detection
├── config/                # Configuration files
├── knowledge_base/        # RAG knowledge base
├── tests/                 # Test files
├── scripts/              # Setup scripts
├── requirements.txt      # Python dependencies
└── README.md            # Project documentation
```

## 🧪 Testing Your Setup

### 1. Basic Tests (No Hardware)
```bash
# Test RAG system
python test_rag_system.py --verbose

# Test prompt generation
python test_rag_prompts.py

# Test basic structure
python tests/test_basic_structure.py
```

### 2. RAG Intelligence Tests
```bash
# Interactive RAG testing
python robot_control/main.py --interactive

# Test specific RAG queries:
robot> move toward bottle        # Tests approach planning
robot> pick up fragile glass     # Tests safety planning  
robot> organize workspace        # Tests complex planning
```

### 3. Hardware Tests (If Available)
```bash
# Test robot connection
python tests/test_robot_control.py

# Test vision system
python tests/debug_vision_complete.py
```

## 🔑 API Key Setup Details

### Get Gemini API Key (Free)
1. Go to: https://makersuite.google.com/app/apikey
2. Click "Create API Key"
3. Copy the key (starts with 'AI...')

### Set Environment Variable
```bash
# Temporary (current session only)
export GEMINI_API_KEY='AIza...'

# Permanent (add to ~/.bashrc)
echo 'export GEMINI_API_KEY="AIza..."' >> ~/.bashrc
source ~/.bashrc

# Verify it's set
echo $GEMINI_API_KEY
```

### API Key Security ✅
- ✅ **Never hardcode** API keys in code
- ✅ **Use environment variables** only
- ✅ **Add to .gitignore** if using .env files
- ✅ **Regenerate keys** if accidentally committed

## 🐛 Troubleshooting

### Common Issues

#### "ModuleNotFoundError"
```bash
# Activate virtual environment
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Add project to Python path
export PYTHONPATH="${PYTHONPATH}:$(pwd)"
```

#### "GEMINI_API_KEY not set"
```bash
# Check if set
echo $GEMINI_API_KEY

# Set it
export GEMINI_API_KEY='your_key_here'

# Test
python simple_prompt_test.py
```

#### "ChromaDB not available"
```bash
# Install RAG dependencies
pip install chromadb sentence-transformers langchain
```

#### "ROS2 not available" (Hardware mode)
```bash
# Install ROS2 Humble (Ubuntu 22.04)
./scripts/install_dependencies.sh

# Source ROS2
source /opt/ros/humble/setup.bash
```

### Getting Help

1. **Check logs**: `logs/` directory contains detailed logs
2. **Run tests**: Use test files to isolate issues
3. **Check config**: Verify configuration files in `config/`
4. **Hardware issues**: Ensure proper connections and drivers

## 🎉 You're Ready!

After setup, you can:

- ✅ **Test RAG system** without any hardware
- ✅ **Run interactive mode** for planning tests
- ✅ **Use Gemini free tier** for AI planning
- ✅ **Develop and test** new RAG prompts
- ✅ **Connect hardware** when available

### Quick Start Commands:
```bash
# Activate environment
source venv/bin/activate

# Set API key  
export GEMINI_API_KEY='your_key'

# Start testing
python robot_control/main.py --interactive
```

**Happy coding! 🤖✨**
