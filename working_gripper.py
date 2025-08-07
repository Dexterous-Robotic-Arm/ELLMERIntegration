#!/usr/bin/env python3

from dynamixel_sdk import *
import time

# Constants - DO NOT MODIFY
MOTOR_ID = 1  # Using only ID 1
DEV = '/dev/ttyUSB0'  # Fixed port
BAUDRATE = 4000000
PROTOCOL = 2.0

# Control table addresses - DO NOT MODIFY
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_CURRENT_LIMIT = 102
ADDR_P_GAIN = 84
ADDR_D_GAIN = 80

# Position values - DO NOT MODIFY
OPEN_POSITION = 2048
CLOSED_POSITION = 1581
HALF_POSITION = 2287

def initialize_gripper():
    """Initialize the gripper with exact working parameters."""
    portHandler = PortHandler(DEV)
    packetHandler = PacketHandler(PROTOCOL)
    
    if not portHandler.openPort():
        raise RuntimeError("Failed to open port")
    
    if not portHandler.setBaudRate(BAUDRATE):
        raise RuntimeError("Failed to set baudrate")
    
    print("Setting up motor...")
    # Exact working configuration - DO NOT MODIFY
    packetHandler.write2ByteTxRx(portHandler, MOTOR_ID, ADDR_CURRENT_LIMIT, 200)
    packetHandler.write2ByteTxRx(portHandler, MOTOR_ID, ADDR_P_GAIN, 400)
    packetHandler.write2ByteTxRx(portHandler, MOTOR_ID, ADDR_D_GAIN, 100)
    packetHandler.write1ByteTxRx(portHandler, MOTOR_ID, ADDR_TORQUE_ENABLE, 1)
    
    return portHandler, packetHandler

def open_gripper(portHandler, packetHandler):
    """Open the gripper to exact working position."""
    print("Moving motor to open position (2048 units)...")
    packetHandler.write4ByteTxRx(portHandler, MOTOR_ID, ADDR_GOAL_POSITION, OPEN_POSITION)
    print(f"Motor {MOTOR_ID}: {OPEN_POSITION} units")
    time.sleep(0.1)
    print("Open position set!")

def close_gripper(portHandler, packetHandler):
    """Close the gripper to exact working position."""
    print("Moving motor to closed position (1581 units)...")
    packetHandler.write4ByteTxRx(portHandler, MOTOR_ID, ADDR_GOAL_POSITION, CLOSED_POSITION)
    print(f"Motor {MOTOR_ID}: {CLOSED_POSITION} units")
    time.sleep(0.1)
    print("Closed position set!")

def set_half_open(portHandler, packetHandler):
    """Set gripper to half-open position."""
    print("Moving motor to half-open position (2287 units)...")
    packetHandler.write4ByteTxRx(portHandler, MOTOR_ID, ADDR_GOAL_POSITION, HALF_POSITION)
    print(f"Motor {MOTOR_ID}: {HALF_POSITION} units")
    time.sleep(0.1)
    print("Half-open position set!")

def disable_motor(portHandler, packetHandler):
    """Disable the motor."""
    print("Disabling motor...")
    packetHandler.write1ByteTxRx(portHandler, MOTOR_ID, ADDR_TORQUE_ENABLE, 0)
    portHandler.closePort()
    print("Motor disabled and port closed!")

# Example usage:
if __name__ == "__main__":
    try:
        portHandler, packetHandler = initialize_gripper()
        open_gripper(portHandler, packetHandler)
        time.sleep(1)
        close_gripper(portHandler, packetHandler)
        time.sleep(1)
        open_gripper(portHandler, packetHandler)
    except KeyboardInterrupt:
        print("\nInterrupted!")
    finally:
        disable_motor(portHandler, packetHandler)
        print("Done!")
