#!/usr/bin/env python3
"""
XL330-M288 Gripper Controller - Simplified
Based on working dynamixel code
"""

import time
import yaml
import os
from typing import Optional, Dict, Any
from dataclasses import dataclass

try:
    from dynamixel_sdk import *
    DYNAMIXEL_AVAILABLE = True
except ImportError:
    DYNAMIXEL_AVAILABLE = False
    print("[Gripper] Dynamixel SDK not available - gripper disabled")

@dataclass
class XL330Config:
    """XL330-M288 specific configuration."""
    servo_id: int = 1
    servo_port: str = "/dev/ttyUSB0"
    servo_baudrate: int = 4000000
    servo_protocol: float = 2.0
    
    position_open: int = 1558    # 137 degrees
    position_closed: int = 1138  # 100 degrees
    
    addr_torque_enable: int = 64
    addr_goal_position: int = 116
    addr_current_limit: int = 102
    addr_p_gain: int = 84
    addr_d_gain: int = 80
    addr_present_position: int = 132

class XL330GripperController:
    """Simplified XL330-M288 Gripper Controller using working code pattern."""
    
    def __init__(self, config_file: str = "config/gripper_config_xl330.yaml"):
        self.config = self._load_config(config_file)
        self.enabled = False
        self.port_handler = None
        self.packet_handler = None
        
        if DYNAMIXEL_AVAILABLE:
            self._initialize_dynamixel()
        else:
            print("[Gripper] Dynamixel SDK not available - gripper disabled")
    
    def _load_config(self, config_file: str) -> XL330Config:
        """Load configuration from YAML file."""
        try:
            with open(config_file, 'r') as f:
                config_data = yaml.safe_load(f)
            
            return XL330Config(
                servo_id=config_data.get('servo_id', 1),
                servo_port=config_data.get('servo_port', '/dev/ttyUSB0'),
                servo_baudrate=config_data.get('servo_baudrate', 4000000),
                position_open=config_data.get('position_open', 1558),
                position_closed=config_data.get('position_closed', 1138)
            )
        except Exception as e:
            print(f"[Gripper] Failed to load config: {e}")
            return XL330Config()
    
    def _position_to_degrees(self, position: int) -> float:
        """Convert position value to degrees for XL330-M288."""
        return (position / 4095.0) * 360.0
    
    def _initialize_dynamixel(self) -> None:
        """Initialize XL330-M288 using the working code pattern."""
        print(f"[Gripper] Initializing XL330-M288...")
        
        try:
            self.port_handler = PortHandler(self.config.servo_port)
            self.packet_handler = PacketHandler(self.config.servo_protocol)
            
            if not self.port_handler.openPort():
                print(f"[Gripper] Failed to open port {self.config.servo_port}")
                return
            
            if not self.port_handler.setBaudRate(self.config.servo_baudrate):
                print(f"[Gripper] Failed to set baudrate {self.config.servo_baudrate}")
                self.port_handler.closePort()
                return
            
            print(f"[Gripper] Port opened successfully at {self.config.servo_baudrate} baud")
            
            print("[Gripper] Setting up motor...")
            self.packet_handler.write2ByteTxRx(self.port_handler, self.config.servo_id, self.config.addr_current_limit, 200)
            self.packet_handler.write2ByteTxRx(self.port_handler, self.config.servo_id, self.config.addr_p_gain, 400)
            self.packet_handler.write2ByteTxRx(self.port_handler, self.config.servo_id, self.config.addr_d_gain, 100)
            self.packet_handler.write1ByteTxRx(self.port_handler, self.config.servo_id, self.config.addr_torque_enable, 1)
            
            print("[Gripper] Motor setup complete")
            self.enabled = True
            print(f"[Gripper] XL330-M288 initialized successfully!")
            
        except Exception as e:
            print(f"[Gripper] Initialization error: {e}")
            if hasattr(self, 'port_handler') and self.port_handler:
                self.port_handler.closePort()
            self.enabled = False
    
    def open_gripper(self) -> bool:
        """Open the gripper (137° = 1558 units)."""
        if not self.enabled:
            print("[Gripper] Gripper not enabled")
            return False
        
        try:
            self.packet_handler.write4ByteTxRx(self.port_handler, self.config.servo_id, self.config.addr_goal_position, self.config.position_open)
            print(f"[Gripper] Open command sent: {self.config.position_open} units (137°)")
            return True
        except Exception as e:
            print(f"[Gripper] Open failed: {e}")
            return False
    
    def close_gripper(self) -> bool:
        """Close the gripper (100° = 1138 units)."""
        if not self.enabled:
            print("[Gripper] Gripper not enabled")
            return False
        
        try:
            self.packet_handler.write4ByteTxRx(self.port_handler, self.config.servo_id, self.config.addr_goal_position, self.config.position_closed)
            print(f"[Gripper] Close command sent: {self.config.position_closed} units (100°)")
            return True
        except Exception as e:
            print(f"[Gripper] Close failed: {e}")
            return False
    
    def set_gripper_position(self, position: int) -> bool:
        """Set gripper to specific position (raw units)."""
        if not self.enabled:
            print("[Gripper] Gripper not enabled")
            return False
        
        try:
            degrees = self._position_to_degrees(position)
            self.packet_handler.write4ByteTxRx(self.port_handler, self.config.servo_id, self.config.addr_goal_position, position)
            print(f"[Gripper] Position command sent: {position} units ({degrees:.1f}°)")
            return True
        except Exception as e:
            print(f"[Gripper] Set position failed: {e}")
            return False
    
    def set_gripper_degrees(self, degrees: float) -> bool:
        """Set gripper to specific angle in degrees."""
        if not self.enabled:
            print("[Gripper] Gripper not enabled")
            return False
        
        try:
            position = int((degrees / 360.0) * 4095)
            position = max(0, min(4095, position))
            self.packet_handler.write4ByteTxRx(self.port_handler, self.config.servo_id, self.config.addr_goal_position, position)
            print(f"[Gripper] Degrees command sent: {degrees}° ({position} units)")
            return True
        except Exception as e:
            print(f"[Gripper] Set degrees failed: {e}")
            return False
    
    def get_gripper_position(self) -> Optional[int]:
        """Get current gripper position in raw units."""
        if not self.enabled:
            print("[Gripper] Gripper not enabled")
            return None
        
        try:
            position, result, error = self.packet_handler.read4ByteTxRx(
                self.port_handler, self.config.servo_id, self.config.addr_present_position
            )
            if result != 0:
                print(f"[Gripper] Failed to read position: {error}")
                return None
            return position
        except Exception as e:
            print(f"[Gripper] Get position error: {e}")
            return None
    
    def get_gripper_degrees(self) -> Optional[float]:
        """Get current gripper position in degrees."""
        position = self.get_gripper_position()
        if position is not None:
            return self._position_to_degrees(position)
        return None
    
    def get_gripper_status(self) -> Dict[str, Any]:
        """Get basic gripper status."""
        if not self.enabled:
            return {
                'enabled': False,
                'position': None,
                'degrees': None,
                'error': 'Gripper not enabled'
            }
        
        try:
            position = self.get_gripper_position()
            degrees = self._position_to_degrees(position) if position is not None else None
            return {
                'enabled': True,
                'position': position,
                'degrees': degrees,
                'error': None
            }
        except Exception as e:
            return {
                'enabled': True,
                'position': None,
                'degrees': None,
                'error': str(e)
            }
    
    def disconnect(self) -> None:
        """Disconnect from gripper."""
        if self.enabled and self.port_handler:
            try:
                self.packet_handler.write1ByteTxRx(self.port_handler, self.config.servo_id, self.config.addr_torque_enable, 0)
                self.enabled = False
                print("[Gripper] Torque disabled")
            except Exception as e:
                print(f"[Gripper] Error disabling torque: {e}")
        
        if self.port_handler:
            self.port_handler.closePort()
            print("[Gripper] Disconnected")

def test_xl330_gripper():
    """Test XL330-M288 gripper functionality."""
    print("🔧 Testing XL330-M288 Gripper")
    print("=============================")
    
    gripper = XL330GripperController()
    
    if not gripper.enabled:
        print("❌ Gripper not enabled")
        return False
    
    try:
        status = gripper.get_gripper_status()
        print(f"📊 Initial status: {status}")
        
        print("🔓 Testing open...")
        if gripper.open_gripper():
            time.sleep(2)
            position = gripper.get_gripper_position()
            degrees = gripper.get_gripper_degrees()
            print(f"✅ Opened to position: {position} units ({degrees:.1f}°)")
        else:
            print("❌ Open failed")
            return False
        
        print("🔒 Testing close...")
        if gripper.close_gripper():
            time.sleep(2)
            position = gripper.get_gripper_position()
            degrees = gripper.get_gripper_degrees()
            print(f"✅ Closed to position: {position} units ({degrees:.1f}°)")
        else:
            print("❌ Close failed")
            return False
        
        print("🔧 Testing custom angle (120°)...")
        if gripper.set_gripper_degrees(120.0):
            time.sleep(2)
            position = gripper.get_gripper_position()
            degrees = gripper.get_gripper_degrees()
            print(f"✅ Set to position: {position} units ({degrees:.1f}°)")
        
        print("✅ All tests passed!")
        return True
        
    except Exception as e:
        print(f"❌ Test error: {e}")
        return False
    finally:
        gripper.disconnect()

if __name__ == "__main__":
    test_xl330_gripper()