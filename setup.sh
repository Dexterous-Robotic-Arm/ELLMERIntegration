e to run #!/bin/bash
# Main setup script - calls organized scripts from scripts/ directory

echo "🤖 ELLMER Integration Setup"
echo "=========================="

# Check if scripts directory exists
if [ ! -d "scripts" ]; then
    echo "❌ Error: scripts/ directory not found"
    exit 1
fi

# Make scripts executable
chmod +x scripts/*.sh

echo "📦 Running environment setup..."
bash scripts/setup_environment.sh

echo "🏗️ Running build and setup..."
bash scripts/build_and_run.sh

echo "✅ Setup complete!"
echo ""
echo "🚀 Quick Start:"
echo "  source scripts/setup_environment.sh"
echo "  python3 robot_control/main.py --task 'pick up the bottle' --sim --dry-run"