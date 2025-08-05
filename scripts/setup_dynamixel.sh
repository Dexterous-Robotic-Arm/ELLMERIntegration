#!/bin/bash

# Dynamixel Gripper Setup Script
# This script helps configure Dynamixel servo for gripper control

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Print functions
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if Dynamixel SDK is installed
check_dynamixel_sdk() {
    print_status "Checking Dynamixel SDK installation..."
    
    if python3 -c "import dynamixel_sdk" 2>/dev/null; then
        print_success "Dynamixel SDK is installed"
        return 0
    else
        print_error "Dynamixel SDK not found"
        print_status "Installing Dynamixel SDK..."
        pip3 install dynamixel-sdk
        print_success "Dynamixel SDK installed"
        return 0
    fi
}

# Setup USB permissions
setup_usb_permissions() {
    print_status "Setting up USB permissions for Dynamixel..."
    
    # Add user to dialout group
    sudo usermod -a -G dialout $USER
    
    # Create udev rules for common USB-to-TTL converters
    sudo tee /etc/udev/rules.d/99-dynamixel.rules > /dev/null << 'EOF'
# Dynamixel USB-to-TTL converter rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE="0666", GROUP="dialout"
EOF
    
    # Reload udev rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    
    print_success "USB permissions configured"
    print_warning "You may need to log out and back in for group changes to take effect"
}

# Check USB devices
check_usb_devices() {
    print_status "Checking USB devices..."
    
    if ls /dev/ttyUSB* 1> /dev/null 2>&1; then
        print_success "USB devices found:"
        ls -la /dev/ttyUSB*
    else
        print_warning "No USB devices found"
        print_status "Please connect your Dynamixel servo via USB-to-TTL converter"
        return 1
    fi
}

# Test Dynamixel connection
test_dynamixel_connection() {
    print_status "Testing Dynamixel connection..."
    
    # Create a simple test script
    cat > test_dynamixel.py << 'EOF'
#!/usr/bin/env python3
"""
Simple Dynamixel connection test
"""
import sys
import time

try:
    import dynamixel_sdk as dxl
    
    # Test basic imports
    print("✅ Dynamixel SDK imported successfully")
    
    # Test port handler creation
    port_handler = dxl.PortHandler("/dev/ttyUSB0")
    print("✅ Port handler created")
    
    # Test packet handler creation
    packet_handler = dxl.Protocol2PacketHandler()
    print("✅ Packet handler created")
    
    # Try to open port
    if port_handler.openPort():
        print("✅ Port opened successfully")
        
        # Set baudrate
        if port_handler.setBaudRate(57600):
            print("✅ Baudrate set to 57600")
            
            # Try to ping servo ID 1
            result, error = packet_handler.ping(port_handler, 1)
            if result:
                print(f"✅ Servo ID 1 found (model: {error})")
            else:
                print("⚠️  Servo ID 1 not found (this is normal if no servo is connected)")
        else:
            print("❌ Failed to set baudrate")
        
        port_handler.closePort()
    else:
        print("❌ Failed to open port")
        
except ImportError as e:
    print(f"❌ Dynamixel SDK import error: {e}")
except Exception as e:
    print(f"❌ Connection test error: {e}")
EOF
    
    # Run the test
    python3 test_dynamixel.py
    
    # Clean up
    rm -f test_dynamixel.py
}

# Configure gripper settings
configure_gripper() {
    print_status "Configuring gripper settings..."
    
    # Check if config file exists
    if [ ! -f "config/gripper_config.yaml" ]; then
        print_error "Gripper config file not found: config/gripper_config.yaml"
        return 1
    fi
    
    print_status "Current gripper configuration:"
    cat config/gripper_config.yaml
    
    echo ""
    read -p "Do you want to edit the gripper configuration? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        if command -v nano >/dev/null 2>&1; then
            nano config/gripper_config.yaml
        elif command -v vim >/dev/null 2>&1; then
            vim config/gripper_config.yaml
        else
            print_warning "No text editor found. Please edit config/gripper_config.yaml manually"
        fi
    fi
}

# Test gripper integration
test_gripper_integration() {
    print_status "Testing gripper integration..."
    
    # Create a test script
    cat > test_gripper_integration.py << 'EOF'
#!/usr/bin/env python3
"""
Test Dynamixel gripper integration
"""
import sys
import os
sys.path.append('.')

try:
    from robot_control.robot_controller import XArmRunner
    
    print("Testing gripper integration...")
    
    # Create XArmRunner in simulation mode
    runner = XArmRunner('192.168.1.241', sim=True)
    
    # Get gripper status
    status = runner.get_gripper_status()
    print(f"Gripper status: {status}")
    
    # Test gripper methods
    print("Testing gripper methods...")
    
    # Test open gripper (simulation)
    print("Testing open_gripper...")
    result = runner.open_gripper()
    print(f"Open gripper result: {result}")
    
    # Test close gripper (simulation)
    print("Testing close_gripper...")
    result = runner.close_gripper()
    print(f"Close gripper result: {result}")
    
    print("✅ Gripper integration test completed")
    
except ImportError as e:
    print(f"❌ Import error: {e}")
except Exception as e:
    print(f"❌ Integration test error: {e}")
EOF
    
    # Run the test
    python3 test_gripper_integration.py
    
    # Clean up
    rm -f test_gripper_integration.py
}

# Main function
main() {
    print_status "Starting Dynamixel gripper setup..."
    
    # Check Dynamixel SDK
    check_dynamixel_sdk
    
    # Setup USB permissions
    setup_usb_permissions
    
    # Check USB devices
    if check_usb_devices; then
        # Test connection
        test_dynamixel_connection
    fi
    
    # Configure gripper
    configure_gripper
    
    # Test integration
    test_gripper_integration
    
    print_success "Dynamixel gripper setup completed!"
    echo ""
    echo "Next steps:"
    echo "1. Connect your Dynamixel servo to USB port"
    echo "2. Set servo ID using Dynamixel Wizard"
    echo "3. Test gripper: python3 robot_control/main.py --task 'test gripper' --sim --dry-run"
    echo "4. Test with hardware: python3 robot_control/main.py --task 'open gripper' --dry-run"
    echo ""
    echo "For more information, see README.md section 'Dynamixel Gripper Configuration'"
}

# Run main function
main "$@" 