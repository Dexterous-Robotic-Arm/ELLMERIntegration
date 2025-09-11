# Scripts Directory

This directory contains all shell scripts for the ELLMER Integration project.

## 📁 Scripts Overview

### **setup.sh** (Root Directory)
- **Purpose**: Main entry point for project setup
- **Usage**: `./setup.sh`
- **Description**: Calls all necessary setup scripts in the correct order

### **setup_environment.sh**
- **Purpose**: Set up Python environment and dependencies
- **Usage**: `source scripts/setup_environment.sh`
- **Description**: Installs core Python dependencies and sets up environment variables

### **build_and_run.sh**
- **Purpose**: Complete build and setup process
- **Usage**: `bash scripts/build_and_run.sh`
- **Description**: 
  - Installs all dependencies
  - Builds ROS packages
  - Creates environment setup scripts
  - Provides usage instructions

### **build_april_tags.sh**
- **Purpose**: Build April Tags ROS package specifically
- **Usage**: `bash scripts/build_april_tags.sh`
- **Description**: Builds the April Tags vision ROS package with colcon

## 🚀 Quick Start

```bash
# From project root
./setup.sh

# Or step by step
bash scripts/setup_environment.sh
bash scripts/build_and_run.sh
```

## 📋 Script Dependencies

- `setup.sh` → calls `setup_environment.sh` and `build_and_run.sh`
- `build_and_run.sh` → calls `setup_environment.sh` internally
- All scripts are independent and can be run separately

## 🔧 Environment Variables

Scripts set up these environment variables:
- `PYTHONPATH`: Python module search path
- `GEMINI_API_KEY`: Google Gemini API key (optional)
- `XARM_IP`: Robot IP address (default: 192.168.1.241)
