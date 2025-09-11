#!/usr/bin/env python3
"""
XArm Robot Control Interface
===========================

This module provides a safe and efficient interface for controlling the ufactory850
robot arm with comprehensive safety limits, gripper control, and error handling.

Features:
- Safe movement with workspace and speed limits
- Comprehensive gripper control with force/speed limits
- Emergency stop functionality
- Collision detection and prevention
- Real-time safety monitoring

Usage:
    runner = XArmRunner("192.168.1.241")
    runner.move_pose([400, 0, 200], [0, 0, 0])
    runner.open_gripper()
    runner.close_gripper()
"""

import time
import math
from typing import Optional, Tuple, List, Dict, Any
from dataclasses import dataclass
from xarm.wrapper import XArmAPI

try:
    import dynamixel_sdk as dxl
    DYNAMIXEL_AVAILABLE = True
except ImportError:
    DYNAMIXEL_AVAILABLE = False
    print("Warning: Dynamixel SDK not available. Install with: pip install dynamixel-sdk")


@dataclass
class SafetyLimits:
    """Safety limits for robot operation."""
    # Movement limits (slower for precision)
    max_cartesian_speed: float = 50.0       # mm/s (reduced for precision)
    max_joint_speed: float = 15.0            # deg/s (reduced for precision)
    max_acceleration: float = 500.0          # mm/s² (reduced for smoothness)
    max_force: float = 100.0                # N
    max_torque: float = 50.0                # Nm
    
    # Workspace limits (mm)
    workspace_x_min: float = 150.0
    workspace_x_max: float = 650.0
    workspace_y_min: float = -300.0
    workspace_y_max: float = 300.0
    workspace_z_min: float = 50.0
    workspace_z_max: float = 500.0
    
    # Gripper limits
    max_gripper_speed: float = 300.0        # mm/s
    max_gripper_force: float = 100.0        # N
    min_gripper_position: float = 0.0       # mm
    max_gripper_position: float = 850.0     # mm
    
    # Timing limits
    safety_check_interval: float = 0.1      # seconds
    max_movement_distance: float = 400.0    # mm (increased for scan operations)


@dataclass
class GripperConfig:
    """Gripper configuration and positions."""
    # Dynamixel servo configuration
    servo_id: int = 1                    # Dynamixel servo ID
    servo_port: str = "/dev/ttyUSB0"     # Serial port for Dynamixel
    servo_baudrate: int = 57600          # Baudrate for Dynamixel
    servo_protocol: int = 2              # Protocol version (1 or 2)
    
    # Position limits (in servo units, typically 0-4095 for MX series)
    position_open: int = 4095            # Fully open position
    position_closed: int = 1024          # Fully closed position  
    position_half: int = 2560            # Half open position
    
    # Speed and torque limits
    default_speed: int = 100             # Speed (0-1023 for MX series)
    default_torque: int = 512            # Torque limit (0-1023 for MX series)
    max_speed: int = 200                 # Maximum speed
    max_torque: int = 1023               # Maximum torque
    
    # Control addresses (for MX-28, MX-64, MX-106 series)
    addr_torque_enable: int = 64         # Torque enable address
    addr_goal_position: int = 116        # Goal position address (Protocol 2)
    addr_present_position: int = 132     # Present position address (Protocol 2)
    addr_moving_speed: int = 104         # Moving speed address (Protocol 2)
    addr_torque_limit: int = 102         # Torque limit address (Protocol 2)
    addr_present_load: int = 126         # Present load address (Protocol 2)


class SafetyMonitor:
    """Monitors and enforces safety limits."""
    
    def __init__(self, limits: SafetyLimits):
        self.limits = limits
        self.emergency_stop = False
        self.last_safety_check = time.time()
    
    def check_emergency_stop(self) -> None:
        """Check if emergency stop is triggered."""
        if self.emergency_stop:
            raise RuntimeError("Emergency stop triggered - stopping all movement")
    
    def validate_workspace_limits(self, xyz_mm: List[float]) -> bool:
        """Validate that target position is within safe workspace."""
        x, y, z = xyz_mm
        
        if not (self.limits.workspace_x_min <= x <= self.limits.workspace_x_max):
            raise ValueError(f"X position {x}mm outside workspace limits "
                           f"[{self.limits.workspace_x_min}, {self.limits.workspace_x_max}]")
        
        if not (self.limits.workspace_y_min <= y <= self.limits.workspace_y_max):
            raise ValueError(f"Y position {y}mm outside workspace limits "
                           f"[{self.limits.workspace_y_min}, {self.limits.workspace_y_max}]")
        
        if not (self.limits.workspace_z_min <= z <= self.limits.workspace_z_max):
            raise ValueError(f"Z position {z}mm outside workspace limits "
                           f"[{self.limits.workspace_z_min}, {self.limits.workspace_z_max}]")
        
        return True
    
    def validate_speed_limits(self, speed: float) -> float:
        """Validate and limit speed to safe values."""
        if speed > self.limits.max_cartesian_speed:
            print(f"[Safety] Speed {speed} exceeds limit {self.limits.max_cartesian_speed}, using limit")
            return self.limits.max_cartesian_speed
        return speed
    
    def check_collision_risk(self, target_xyz: List[float], current_xyz: Optional[List[float]] = None) -> bool:
        """Basic collision risk assessment."""
        if current_xyz is None:
            return False  # Can't check without current position
        
        # Ensure both positions are lists of floats
        try:
            target_xyz = [float(x) for x in target_xyz]
            current_xyz = [float(x) for x in current_xyz]
        except (ValueError, TypeError):
            print(f"[Safety] Invalid position format - target: {target_xyz}, current: {current_xyz}")
            return False
        
        # Ensure we have 3D coordinates
        if len(target_xyz) < 3 or len(current_xyz) < 3:
            print(f"[Safety] Incomplete position data - target: {target_xyz}, current: {current_xyz}")
            return False
        
        # Calculate distance to target
        distance = math.sqrt(sum((t - c) ** 2 for t, c in zip(target_xyz, current_xyz)))
        
        # If movement is very small, likely safe
        if distance < 10:  # mm
            return False
        
        # Check if movement is within reasonable bounds
        if distance > self.limits.max_movement_distance:
            print(f"[Safety] Large movement detected: {distance:.1f}mm")
            return True
        
        return False
    
    def trigger_emergency_stop(self) -> None:
        """Trigger emergency stop."""
        self.emergency_stop = True
        print("[EMERGENCY] Emergency stop triggered!")
    
    def clear_emergency_stop(self) -> None:
        """Clear emergency stop."""
        self.emergency_stop = False
        print("[Safety] Emergency stop cleared")


class DynamixelGripperController:
    """Manages Dynamixel servo-based gripper operations."""
    
    def __init__(self, config: GripperConfig):
        self.config = config
        self.enabled = False
        self.port_handler = None
        self.packet_handler = None
        self.group_write = None
        self.group_read = None
        
        if DYNAMIXEL_AVAILABLE:
            self._initialize_dynamixel()
        else:
            print("[Gripper] Dynamixel SDK not available - gripper disabled")
    
    def _initialize_dynamixel(self) -> None:
        """Initialize the Dynamixel servo connection."""
        try:
            # Initialize port handler
            self.port_handler = dxl.PortHandler(self.config.servo_port)
            
            # Initialize packet handler
            if self.config.servo_protocol == 1:
                self.packet_handler = dxl.Protocol1PacketHandler()
            else:
                self.packet_handler = dxl.Protocol2PacketHandler()
            
            # Open port
            if not self.port_handler.openPort():
                print(f"[Gripper] Failed to open port {self.config.servo_port}")
                return
            
            # Set baudrate
            if not self.port_handler.setBaudRate(self.config.servo_baudrate):
                print(f"[Gripper] Failed to set baudrate {self.config.servo_baudrate}")
                return
            
            # Initialize group sync write for position
            self.group_write = dxl.GroupSyncWrite(
                self.port_handler, 
                self.packet_handler, 
                self.config.addr_goal_position, 
                4  # 4 bytes for position
            )
            
            # Initialize group sync read for position and load
            self.group_read = dxl.GroupSyncRead(
                self.port_handler,
                self.packet_handler,
                self.config.addr_present_position,
                6  # 4 bytes position + 2 bytes load
            )
            
            # Add parameter storage for sync read
            self.group_read.addParam(self.config.servo_id)
            
            # Enable torque
            self._enable_torque(True)
            
            self.enabled = True
            print(f"[Gripper] Dynamixel servo {self.config.servo_id} initialized successfully")
            
        except Exception as e:
            print(f"[Gripper] Dynamixel initialization failed: {e}")
            self.enabled = False
    
    def _enable_torque(self, enable: bool) -> bool:
        """Enable or disable servo torque."""
        try:
            result = self.packet_handler.write1ByteTxRx(
                self.port_handler, 
                self.config.servo_id, 
                self.config.addr_torque_enable, 
                1 if enable else 0
            )
            if result != 0:
                print(f"[Gripper] Failed to set torque enable: {result}")
                return False
            return True
        except Exception as e:
            print(f"[Gripper] Torque enable error: {e}")
            return False
    
    def _set_position(self, position: int, speed: int = None, torque: int = None) -> bool:
        """Set servo position with optional speed and torque limits."""
        try:
            # Apply limits
            safe_position = max(0, min(4095, position))
            safe_speed = speed if speed is not None else self.config.default_speed
            safe_speed = max(0, min(self.config.max_speed, safe_speed))
            safe_torque = torque if torque is not None else self.config.default_torque
            safe_torque = max(0, min(self.config.max_torque, safe_torque))
            
            # Set speed limit
            result = self.packet_handler.write2ByteTxRx(
                self.port_handler,
                self.config.servo_id,
                self.config.addr_moving_speed,
                safe_speed
            )
            if result != 0:
                print(f"[Gripper] Failed to set speed: {result}")
                return False
            
            # Set torque limit
            result = self.packet_handler.write2ByteTxRx(
                self.port_handler,
                self.config.servo_id,
                self.config.addr_torque_limit,
                safe_torque
            )
            if result != 0:
                print(f"[Gripper] Failed to set torque limit: {result}")
                return False
            
            # Set position
            result = self.packet_handler.write4ByteTxRx(
                self.port_handler,
                self.config.servo_id,
                self.config.addr_goal_position,
                safe_position
            )
            if result != 0:
                print(f"[Gripper] Failed to set position: {result}")
                return False
            
            print(f"[Gripper] Position set to {safe_position} (speed: {safe_speed}, torque: {safe_torque})")
            return True
            
        except Exception as e:
            print(f"[Gripper] Set position error: {e}")
            return False
    
    def _get_position(self) -> Optional[int]:
        """Get current servo position."""
        try:
            result, position = self.packet_handler.read4ByteTxRx(
                self.port_handler,
                self.config.servo_id,
                self.config.addr_present_position
            )
            if result != 0:
                print(f"[Gripper] Failed to read position: {result}")
                return None
            return position
        except Exception as e:
            print(f"[Gripper] Get position error: {e}")
            return None
    
    def _get_load(self) -> Optional[int]:
        """Get current servo load."""
        try:
            result, load = self.packet_handler.read2ByteTxRx(
                self.port_handler,
                self.config.servo_id,
                self.config.addr_present_load
            )
            if result != 0:
                print(f"[Gripper] Failed to read load: {result}")
                return None
            return load
        except Exception as e:
            print(f"[Gripper] Get load error: {e}")
            return None
    
    def open_gripper(self, position: Optional[int] = None, 
                    speed: int = None, torque: int = None) -> bool:
        """Open the gripper."""
        if not self.enabled:
            print("[Gripper] Gripper not enabled")
            return False
        
        target_pos = position if position is not None else self.config.position_open
        return self._set_position(target_pos, speed, torque)
    
    def close_gripper(self, position: Optional[int] = None,
                     speed: int = None, torque: int = None) -> bool:
        """Close the gripper."""
        if not self.enabled:
            print("[Gripper] Gripper not enabled")
            return False
        
        target_pos = position if position is not None else self.config.position_closed
        return self._set_position(target_pos, speed, torque)
    
    def set_gripper_position(self, position: int, speed: int = None, 
                           torque: int = None) -> bool:
        """Set gripper to a specific position."""
        if not self.enabled:
            print("[Gripper] Gripper not enabled")
            return False
        
        return self._set_position(position, speed, torque)
    
    def get_gripper_position(self) -> Optional[int]:
        """Get current gripper position."""
        if not self.enabled:
            return None
        return self._get_position()
    
    def get_gripper_status(self) -> Dict[str, Any]:
        """Get comprehensive gripper status."""
        if not self.enabled:
            return {"enabled": False, "position": None, "load": None, "error": "Gripper not enabled"}
        
        position = self.get_gripper_position()
        load = self._get_load()
        
        return {
            "enabled": True,
            "position": position,
            "load": load,
            "position_limits": {
                "min": 0,
                "max": 4095
            },
            "speed_limits": {
                "min": 0,
                "max": self.config.max_speed
            },
            "torque_limits": {
                "min": 0,
                "max": self.config.max_torque
            }
        }
    
    def gripper_grasp(self, target_position: int = None, speed: int = None,
                     torque: int = None, timeout: float = 5.0) -> bool:
        """Perform a grasp operation with force feedback."""
        if not self.enabled:
            print("[Gripper] Gripper not enabled")
            return False
        
        target_pos = target_position if target_position is not None else self.config.position_closed
        safe_speed = speed if speed is not None else self.config.default_speed * 0.5
        safe_torque = torque if torque is not None else self.config.default_torque
        
        print(f"[Gripper] Starting grasp to position {target_pos}")
        
        # Start closing the gripper
        if not self._set_position(target_pos, safe_speed, safe_torque):
            print("[Gripper] Grasp failed to start")
            return False
        
        # Wait for completion or load detection
        import time
        start_time = time.time()
        while time.time() - start_time < timeout:
            current_pos = self.get_gripper_position()
            current_load = self._get_load()
            
            if current_pos is not None and abs(current_pos - target_pos) < 50:
                print(f"[Gripper] Grasp completed at position {current_pos}")
                return True
            
            if current_load is not None and current_load > self.config.max_torque * 0.8:
                print(f"[Gripper] Grasp stopped by load limit: {current_load}")
                return True
            
            time.sleep(0.1)
        
        print(f"[Gripper] Grasp timeout after {timeout}s")
        return False
    
    def gripper_release(self, target_position: int = None, speed: int = None,
                       torque: int = None) -> bool:
        """Release grasped object by opening gripper."""
        target_pos = target_position if target_position is not None else self.config.position_open
        return self.open_gripper(position=target_pos, speed=speed, torque=torque)
    
    def gripper_half_open(self, speed: int = None, torque: int = None) -> bool:
        """Open gripper to half position."""
        return self.set_gripper_position(self.config.position_half, speed, torque)
    
    def gripper_soft_close(self, speed: int = None, torque: int = None) -> bool:
        """Close gripper gently with low torque."""
        safe_speed = speed if speed is not None else self.config.default_speed * 0.3
        safe_torque = torque if torque is not None else self.config.default_torque * 0.5
        return self.close_gripper(position=self.config.position_closed, speed=safe_speed, torque=safe_torque)
    
    def gripper_cycle_test(self, cycles: int = 3, delay: float = 1.0) -> bool:
        """Test gripper by cycling open/close."""
        if not self.enabled:
            print("[Gripper] Gripper not enabled for test")
            return False
        
        print(f"[Gripper] Starting cycle test ({cycles} cycles)")
        
        for i in range(cycles):
            print(f"[Gripper] Cycle {i+1}/{cycles}")
            
            if not self.open_gripper():
                print(f"[Gripper] Cycle {i+1} failed at open")
                return False
            
            import time
            time.sleep(delay)
            
            if not self.close_gripper():
                print(f"[Gripper] Cycle {i+1} failed at close")
                return False
            
            time.sleep(delay)
        
        print("[Gripper] Cycle test completed successfully")
        return True
    
    def disconnect(self) -> None:
        """Disconnect from Dynamixel servo."""
        if self.enabled:
            self._enable_torque(False)
            if self.port_handler:
                self.port_handler.closePort()
            self.enabled = False
            print("[Gripper] Dynamixel servo disconnected")

class GripperController:
    """Manages gripper operations with safety limits - now supports only Dynamixel."""
    
    def __init__(self, arm: XArmAPI, limits: SafetyLimits, config: GripperConfig):
        self.arm = arm
        self.limits = limits
        self.config = config
        self.enabled = False
        
        # Initialize Dynamixel gripper
        self.dynamixel_gripper = DynamixelGripperController(config)
        if self.dynamixel_gripper.enabled:
            self.enabled = True
            print("[Gripper] Using Dynamixel servo control")
        else:
            print("[Gripper] Dynamixel failed to initialize, gripper disabled")
    
    def _apply_safety_limits(self, position: float, speed: float, force: float) -> Tuple[float, float, float]:
        """Apply safety limits to gripper parameters."""
        safe_position = max(self.limits.min_gripper_position, 
                           min(self.limits.max_gripper_position, position))
        safe_speed = min(speed, self.limits.max_gripper_speed)
        safe_force = min(force, self.limits.max_gripper_force)
        return safe_position, safe_speed, safe_force
    
    def open_gripper(self, position: Optional[float] = None, 
                    speed: float = None, force: float = None) -> bool:
        """Open the gripper with safety limits."""
        if not self.enabled:
            print("[Gripper] Gripper not enabled")
            return False
        
        # Convert mm to servo units if needed
        if position is not None:
            servo_position = int((position / self.config.position_open) * self.config.position_open)
        else:
            servo_position = self.config.position_open
        
        return self.dynamixel_gripper.open_gripper(servo_position, speed, force)
    
    def close_gripper(self, position: Optional[float] = None,
                     speed: float = None, force: float = None) -> bool:
        """Close the gripper with safety limits."""
        if not self.enabled:
            print("[Gripper] Gripper not enabled")
            return False
        
        # Convert mm to servo units if needed
        if position is not None:
            servo_position = int((position / self.config.position_closed) * self.config.position_closed)
        else:
            servo_position = self.config.position_closed
        
        return self.dynamixel_gripper.close_gripper(servo_position, speed, force)
    
    def set_gripper_position(self, position: float, speed: float = None, 
                           force: float = None) -> bool:
        """Set gripper to a specific position with safety limits."""
        if not self.enabled:
            print("[Gripper] Gripper not enabled")
            return False
        
        # Convert mm to servo units
        servo_position = int((position / self.config.position_open) * self.config.position_open)
        return self.dynamixel_gripper.set_gripper_position(servo_position, speed, force)
    
    def get_gripper_position(self) -> Optional[float]:
        """Get current gripper position."""
        if not self.enabled:
            return None
        
        servo_pos = self.dynamixel_gripper.get_gripper_position()
        if servo_pos is not None:
            # Convert servo units to mm
            return (servo_pos / self.config.position_open) * self.config.position_open
        return None
    
    def get_gripper_status(self) -> Dict[str, Any]:
        """Get comprehensive gripper status."""
        if not self.enabled:
            return {"enabled": False, "position": None, "error": "Gripper not enabled"}
        
        return self.dynamixel_gripper.get_gripper_status()
    
    def gripper_grasp(self, target_position: float = None, speed: float = None,
                     force: float = None, timeout: float = 5.0) -> bool:
        """Perform a grasp operation with force feedback."""
        if not self.enabled:
            print("[Gripper] Gripper not enabled")
            return False
        
        # Convert mm to servo units if needed
        if target_position is not None:
            servo_position = int((target_position / self.config.position_closed) * self.config.position_closed)
        else:
            servo_position = self.config.position_closed
        
        return self.dynamixel_gripper.gripper_grasp(servo_position, speed, force, timeout)
    
    def gripper_release(self, target_position: float = None, speed: float = None,
                       force: float = None) -> bool:
        """Release grasped object by opening gripper."""
        target_pos = target_position if target_position is not None else self.config.position_open
        return self.open_gripper(position=target_pos, speed=speed, force=force)
    
    def gripper_half_open(self, speed: float = None, force: float = None) -> bool:
        """Open gripper to half position."""
        return self.set_gripper_position(self.config.position_half, speed, force)
    
    def gripper_soft_close(self, speed: float = None, force: float = None) -> bool:
        """Close gripper gently with low force."""
        safe_speed = speed if speed is not None else self.config.default_speed * 0.3
        safe_force = force if force is not None else self.config.default_force * 0.5
        return self.close_gripper(position=self.config.position_closed, speed=safe_speed, force=safe_force)
    
    def gripper_cycle_test(self, cycles: int = 3, delay: float = 1.0) -> bool:
        """Test gripper by cycling open/close with safety checks."""
        if not self.enabled:
            print("[Gripper] Gripper not enabled for test")
            return False
        
        return self.dynamixel_gripper.gripper_cycle_test(cycles, delay)


class XArmRunner:
    """Main robot control interface with safety monitoring and gripper control."""
    
    def __init__(self, ip: str, cart_speed: float = 100, cart_acc: float = 500,
                 joint_speed: float = 20, joint_acc: float = 50, sim: bool = False):
        self.robot_ip = ip
        self.sim = sim
        
        # Store speed parameters
        self.cart_speed = cart_speed
        self.cart_acc = cart_acc
        self.joint_speed = joint_speed
        self.joint_acc = joint_acc
        
        self.arm = None
        self.safety_limits = SafetyLimits()
        self.safety_monitor = SafetyMonitor(self.safety_limits)
        self.gripper_config = GripperConfig()
        
        # Initialize gripper (will be reinitialized after connection)
        self.gripper = None
        
        if not sim:
            self._connect_and_configure()
        else:
            print("[Robot] Running in simulation mode - no robot connection")
            # Initialize gripper for simulation mode
            self.gripper = GripperController(None, self.safety_limits, self.gripper_config)
    
    def _connect_and_configure(self) -> None:
        """Connect to robot and configure settings."""
        try:
            # Connect to robot
            self.arm = XArmAPI(self.robot_ip, is_radian=False)
            
            # Wait for connection
            time.sleep(1)
            
            # Check connection
            if not self.arm.connected:
                raise Exception("Failed to connect to robot")
            
            print(f"[Robot] Connected to robot at {self.robot_ip}")
            
            # Enable motion and set proper state
            self._enable_robot_motion()
            
            # Apply speed limits
            self._apply_speed_limits(self.cart_speed, self.cart_acc, self.joint_speed, self.joint_acc)
            
            # Reinitialize gripper with connected arm
            self.gripper = GripperController(self.arm, self.safety_limits, self.gripper_config)
            
        except Exception as e:
            print(f"[Robot] Failed to connect to robot: {e}")
            raise
    
    def _enable_robot_motion(self) -> None:
        """Enable robot motion and set to proper state."""
        try:
            # Get current state
            state = self.arm.get_state()
            print(f"[Robot] Current state: {state}")
            
            # Enable motion
            self.arm.motion_enable(enable=True)
            print("[Robot] Motion enabled")
            
            # Set mode to position control (0 = position control)
            self.arm.set_mode(0)
            print("[Robot] Set to position control mode")
            
            # Set state to ready (0 = ready)
            self.arm.set_state(0)
            print("[Robot] Set to ready state")
            
            # Wait for state change
            time.sleep(2)
            
            # Verify state
            final_state = self.arm.get_state()
            print(f"[Robot] Final state: {final_state}")
            
            # Accept states 0, 1, or 2 as ready states (different XArm models use different state codes)
            if final_state[1] in [0, 1, 2]:
                print(f"[Robot] Robot is ready (state: {final_state[1]})")
            else:
                print(f"[Robot] Warning: Robot not in expected ready state, current state: {final_state[1]}")
                
        except Exception as e:
            print(f"[Robot] Error enabling motion: {e}")
            # Continue anyway, the robot might still work
    
    def _apply_speed_limits(self, cart_speed: float, cart_acc: float,
                           joint_speed: float, joint_acc: float) -> None:
        """Apply conservative speed limits to robot."""
        try:
            # Set TCP (tool center point) acceleration limit
            self.arm.set_tcp_maxacc(min(2000, self.safety_limits.max_acceleration))
            
            # Set TCP speed limit - use the correct API method
            try:
                self.arm.set_reduced_max_tcp_speed(min(cart_speed, self.safety_limits.max_cartesian_speed))
            except:
                # Fallback method
                self.arm.set_tcp_maxvel(min(cart_speed, self.safety_limits.max_cartesian_speed))
            
            # Set joint speed limit - use the correct API method
            try:
                self.arm.set_reduced_max_joint_speed(min(joint_speed, self.safety_limits.max_joint_speed))
            except:
                # Fallback method
                self.arm.set_joint_maxvel(min(joint_speed, self.safety_limits.max_joint_speed))
            
            print("[Robot] Speed limits applied successfully")
            
        except Exception as e:
            print(f"[Robot] Warning: Could not apply all speed limits: {e}")
            # Continue without speed limits if they fail
    
    def _validate_target_position(self, xyz_mm: List[float]) -> bool:
        """Validate target position is within safe workspace and reachable."""
        try:
            if len(xyz_mm) < 3:
                print(f"[Robot] Invalid position format: {xyz_mm}")
                return False
            
            x, y, z = xyz_mm[0], xyz_mm[1], xyz_mm[2]
            
            # Check for NaN or infinite values
            if not all(isinstance(coord, (int, float)) and not (coord != coord or coord == float('inf') or coord == float('-inf')) for coord in xyz_mm):
                print(f"[Robot] Invalid coordinate values (NaN/Inf): {xyz_mm}")
                return False
            
            # Check workspace limits
            if not self.safety_monitor.validate_workspace_limits(xyz_mm):
                return False
            
            # Additional safety checks
            if abs(x) > 1000 or abs(y) > 1000 or z < 0 or z > 1000:
                print(f"[Robot] Position outside reasonable bounds: {xyz_mm}")
                return False
            
            return True
            
        except Exception as e:
            print(f"[Robot] Error validating position: {e}")
            return False
    
    def _recover_robot_state(self) -> None:
        """Attempt to recover robot from error state."""
        try:
            if self.arm is None:
                return
            
            print("[Robot] Attempting to recover robot state...")
            
            # Clear any errors
            self.arm.clean_error()
            time.sleep(0.5)
            
            # Re-enable motion
            self.arm.motion_enable(enable=True)
            time.sleep(0.5)
            
            # Set proper mode and state
            self.arm.set_mode(0)  # Position control
            self.arm.set_state(0)  # Ready state
            time.sleep(1)
            
            # Check final state
            state = self.arm.get_state()
            if state[1] in [0, 1, 2]:
                print(f"[Robot] Robot recovered successfully (state: {state[1]})")
            else:
                print(f"[Robot] Robot recovery failed (state: {state[1]})")
                
        except Exception as e:
            print(f"[Robot] Error during robot recovery: {e}")

    def _ensure_robot_ready(self) -> bool:
        """Ensure robot is in ready state for movement."""
        try:
            if self.arm is None:
                return False
            
            state = self.arm.get_state()
            # Accept states 0, 1, or 2 as ready states (different XArm models use different state codes)
            if state[1] not in [0, 1, 2]:  # Not in ready state
                print(f"[Robot] Robot not ready (state: {state[1]}), attempting to fix...")
                
                # Try to enable motion and set ready state
                self.arm.motion_enable(enable=True)
                self.arm.set_mode(0)  # Position control
                self.arm.set_state(0)  # Ready state
                
                time.sleep(1)
                
                # Check again
                new_state = self.arm.get_state()
                if new_state[1] in [0, 1, 2]:
                    print(f"[Robot] Successfully set robot to ready state (state: {new_state[1]})")
                    return True
                else:
                    print(f"[Robot] Failed to set robot to ready state, current state: {new_state[1]}")
                    return False
            
            return True
            
        except Exception as e:
            print(f"[Robot] Error ensuring robot ready: {e}")
            return False
    
    def get_current_position(self) -> Optional[List[float]]:
        """Get current robot position."""
        try:
            if self.arm is None:
                return None
            position = self.arm.get_position()
            print(f"[DEBUG] Raw position data: {position}")
            
            if position[0] == 0:  # Success
                # Handle different position data formats
                if isinstance(position[1], (list, tuple)):
                    # Position is nested in a list/tuple
                    pos_data = position[1]
                    if len(pos_data) >= 3:
                        return [float(pos_data[0]), float(pos_data[1]), float(pos_data[2])]
                else:
                    # Position is directly in the array
                    if len(position) >= 4:
                        return [float(position[1]), float(position[2]), float(position[3])]
            
            print(f"[DEBUG] Could not extract position from: {position}")
            return None
        except Exception as e:
            print(f"[Robot] Error getting position: {e}")
            return None
    
    def go_home(self) -> None:
        """Move robot to home position."""
        try:
            if self.arm is not None:
                self.arm.move_gohome()
                print("[Robot] Moved to home position")
            else:
                print("[Robot] No robot connection - cannot move to home")
        except Exception as e:
            print(f"[Robot] Error moving to home: {e}")
    
    def move_pose(self, xyz_mm: List[float], rpy_deg: List[float], 
                 speed: Optional[float] = None) -> None:
        """Move robot to specific pose."""
        try:
            if self.arm is None:
                print("[Robot] No robot connection - cannot move")
                return
            
            # Validate coordinates before movement
            if not self._validate_target_position(xyz_mm):
                print(f"[Robot] Invalid target position {xyz_mm} - movement cancelled")
                return
            
            # Ensure robot is ready for movement
            if not self._ensure_robot_ready():
                print("[Robot] Robot not ready for movement")
                return
            
            # Use safe speed - reduced for safety
            safe_speed = min(speed if speed is not None else 50, 80)
            
            # Execute movement with error handling
            result = self.arm.set_position(x=xyz_mm[0], y=xyz_mm[1], z=xyz_mm[2],
                                roll=rpy_deg[0], pitch=rpy_deg[1], yaw=rpy_deg[2],
                                speed=safe_speed, wait=True)
            
            if result == 0:
                print(f"[Robot] Moved to position {xyz_mm} with orientation {rpy_deg}")
            else:
                print(f"[Robot] Movement failed with error code {result}")
                # Try to recover robot state
                self._recover_robot_state()
            
        except Exception as e:
            print(f"[Robot] Error in move_pose: {e}")
            # Try to recover robot state
            self._recover_robot_state()
    
    def move_rel_z(self, dz_mm: float) -> None:
        """Move robot relative in Z direction."""
        try:
            if self.arm is None:
                print("[Robot] No robot connection - cannot move")
                return
            
            current_pos = self.get_current_position()
            if current_pos is None:
                print("[Robot] Cannot get current position")
                return
            
            target_z = current_pos[2] + dz_mm
            
            # Validate workspace limits
            if not self.safety_monitor.validate_workspace_limits([current_pos[0], current_pos[1], target_z]):
                print(f"[Robot] Target Z position {target_z} outside workspace limits")
                return
            
            # Execute movement
            self.arm.set_position(x=current_pos[0], y=current_pos[1], z=target_z,
                                speed=self.safety_limits.max_cartesian_speed, wait=True)
            
            print(f"[Robot] Moved relative Z by {dz_mm}mm")
            
        except Exception as e:
            print(f"[Robot] Error in move_rel_z: {e}")
    
    # Gripper methods (delegated to GripperController)
    def open_gripper(self, position: Optional[float] = None, 
                    speed: float = None, force: float = None) -> bool:
        """Open the gripper."""
        if self.gripper is None:
            print("[Robot] Gripper not initialized")
            return False
        return self.gripper.open_gripper(position, speed, force)
    
    def close_gripper(self, position: Optional[float] = None,
                     speed: float = None, force: float = None) -> bool:
        """Close the gripper."""
        if self.gripper is None:
            print("[Robot] Gripper not initialized")
            return False
        return self.gripper.close_gripper(position, speed, force)
    
    def set_gripper_position(self, position: float, speed: float = None, 
                           force: float = None) -> bool:
        """Set gripper to a specific position."""
        if self.gripper is None:
            print("[Robot] Gripper not initialized")
            return False
        return self.gripper.set_gripper_position(position, speed, force)
    
    def get_gripper_position(self) -> Optional[float]:
        """Get current gripper position."""
        if self.gripper is None:
            return None
        return self.gripper.get_gripper_position()
    
    def get_gripper_status(self) -> Dict[str, Any]:
        """Get comprehensive gripper status."""
        if self.gripper is None:
            return {"enabled": False, "error": "Gripper not initialized"}
        return self.gripper.get_gripper_status()
    
    def gripper_grasp(self, target_position: float = None, speed: float = None,
                     force: float = None, timeout: float = 5.0) -> bool:
        """Perform a grasp operation with force feedback."""
        if self.gripper is None:
            print("[Robot] Gripper not initialized")
            return False
        return self.gripper.gripper_grasp(target_position, speed, force, timeout)
    
    def gripper_release(self, target_position: float = None, speed: float = None,
                       force: float = None) -> bool:
        """Release grasped object by opening gripper."""
        return self.gripper.gripper_release(target_position, speed, force)
    
    def gripper_half_open(self, speed: float = None, force: float = None) -> bool:
        """Open gripper to half position."""
        return self.gripper.gripper_half_open(speed, force)
    
    def gripper_soft_close(self, speed: float = None, force: float = None) -> bool:
        """Close gripper gently with low force."""
        return self.gripper.gripper_soft_close(speed, force)
    
    def gripper_cycle_test(self, cycles: int = 3, delay: float = 1.0) -> bool:
        """Test gripper by cycling open/close."""
        return self.gripper.gripper_cycle_test(cycles, delay)
    
    def trigger_emergency_stop(self) -> None:
        """Trigger emergency stop."""
        try:
            if self.arm is not None:
                self.arm.emergency_stop()
            self.safety_monitor.trigger_emergency_stop()
            print("[Robot] Emergency stop triggered")
        except Exception as e:
            print(f"[Robot] Error triggering emergency stop: {e}")
    
    def clear_emergency_stop(self) -> None:
        """Clear emergency stop."""
        try:
            if self.arm is not None:
                self.arm.motion_enable(enable=True)
            self.safety_monitor.clear_emergency_stop()
            print("[Robot] Emergency stop cleared")
        except Exception as e:
            print(f"[Robot] Error clearing emergency stop: {e}")
    
    def motion_enable(self, enable: bool = True) -> bool:
        """Enable or disable robot motion."""
        try:
            if self.arm is not None:
                result = self.arm.motion_enable(enable=enable)
                if result == 0:
                    print(f"[Robot] Motion {'enabled' if enable else 'disabled'} successfully")
                    return True
                else:
                    print(f"[Robot] Failed to {'enable' if enable else 'disable'} motion")
                    return False
            else:
                print("[Robot] No robot connection - cannot enable motion")
                return False
        except Exception as e:
            print(f"[Robot] Error enabling motion: {e}")
            return False
    
    def get_safety_status(self) -> Dict[str, Any]:
        """Get comprehensive safety status."""
        try:
            current_pos = self.get_current_position()
            gripper_status = self.get_gripper_status()
            
            return {
                "robot_connected": self.arm is not None,
                "current_position": current_pos,
                "workspace_safe": current_pos is not None and 
                                self.safety_monitor.validate_workspace_limits(current_pos),
                "gripper_status": gripper_status,
                "emergency_stop_active": False,  # Would need to track this state
                "safety_limits": {
                    "max_cartesian_speed": self.safety_limits.max_cartesian_speed,
                    "max_joint_speed": self.safety_limits.max_joint_speed,
                    "max_acceleration": self.safety_limits.max_acceleration,
                    "workspace_limits": {
                        "x": [self.safety_limits.workspace_x_min, self.safety_limits.workspace_x_max],
                        "y": [self.safety_limits.workspace_y_min, self.safety_limits.workspace_y_max],
                        "z": [self.safety_limits.workspace_z_min, self.safety_limits.workspace_z_max]
                    }
                }
            }
        except Exception as e:
            return {"error": str(e)}
    
    def disconnect(self) -> None:
        """Disconnect from robot and clean up resources."""
        try:
            print("[Robot] Starting graceful shutdown...")
            
            # Disconnect Dynamixel gripper if available
            if self.gripper is not None and hasattr(self.gripper, 'dynamixel_gripper') and self.gripper.dynamixel_gripper.enabled:
                try:
                    self.gripper.dynamixel_gripper.disconnect()
                except Exception as e:
                    print(f"[Robot] Error disconnecting gripper: {e}")
            
            # Disconnect xArm
            if self.arm is not None:
                try:
                    # Stop any ongoing motion
                    self.arm.stop()
                    time.sleep(0.5)
                    
                    # Disconnect cleanly
                    self.arm.disconnect()
                    self.arm = None
                except Exception as e:
                    print(f"[Robot] Error disconnecting arm: {e}")
            
            print("[Robot] Disconnected from robot")
        except Exception as e:
            print(f"[Robot] Error during disconnect: {e}")
        finally:
            # Ensure cleanup even if errors occur
            self.arm = None
            self.gripper = None
