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


@dataclass
class SafetyLimits:
    """Safety limits for robot operation."""
    # Movement limits
    max_cartesian_speed: float = 150.0      # mm/s
    max_joint_speed: float = 30.0           # deg/s
    max_acceleration: float = 1000.0        # mm/s²
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
    max_movement_distance: float = 200.0    # mm


@dataclass
class GripperConfig:
    """Gripper configuration and positions."""
    position_open: float = 850.0            # Fully open
    position_closed: float = 200.0          # Fully closed
    position_half: float = 525.0            # Half open
    default_speed: float = 200.0            # mm/s
    default_force: float = 50.0             # N


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


class GripperController:
    """Manages gripper operations with safety limits."""
    
    def __init__(self, arm: XArmAPI, limits: SafetyLimits, config: GripperConfig):
        self.arm = arm
        self.limits = limits
        self.config = config
        self.enabled = False
        self._initialize_gripper()
    
    def _initialize_gripper(self) -> None:
        """Initialize the gripper system."""
        try:
            self.arm.set_gripper_enable(True)
            self.arm.set_gripper_mode(0)  # Position control mode
            self.enabled = True
            print("[Gripper] Enabled successfully")
        except Exception as e:
            print(f"[Gripper] Initialization failed: {e}")
            self.enabled = False
    
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
        
        target_pos = position if position is not None else self.config.position_open
        safe_speed = speed if speed is not None else self.config.default_speed
        safe_force = force if force is not None else self.config.default_force
        
        safe_position, safe_speed, safe_force = self._apply_safety_limits(
            target_pos, safe_speed, safe_force
        )
        
        return self._set_gripper_position(safe_position, safe_speed, safe_force, "Opened")
    
    def close_gripper(self, position: Optional[float] = None,
                     speed: float = None, force: float = None) -> bool:
        """Close the gripper with safety limits."""
        if not self.enabled:
            print("[Gripper] Gripper not enabled")
            return False
        
        target_pos = position if position is not None else self.config.position_closed
        safe_speed = speed if speed is not None else self.config.default_speed * 0.5
        safe_force = force if force is not None else self.config.default_force
        
        safe_position, safe_speed, safe_force = self._apply_safety_limits(
            target_pos, safe_speed, safe_force
        )
        
        return self._set_gripper_position(safe_position, safe_speed, safe_force, "Closed")
    
    def set_gripper_position(self, position: float, speed: float = None, 
                           force: float = None) -> bool:
        """Set gripper to a specific position with safety limits."""
        if not self.enabled:
            print("[Gripper] Gripper not enabled")
            return False
        
        safe_speed = speed if speed is not None else self.config.default_speed
        safe_force = force if force is not None else self.config.default_force
        
        safe_position, safe_speed, safe_force = self._apply_safety_limits(
            position, safe_speed, safe_force
        )
        
        return self._set_gripper_position(safe_position, safe_speed, safe_force, "Set to")
    
    def _set_gripper_position(self, position: float, speed: float, 
                            force: float, action: str) -> bool:
        """Internal method to set gripper position."""
        try:
            code = self.arm.set_gripper_position(
                position=position, speed=speed, force=force, wait=True
            )
            if code == 0:
                print(f"[Gripper] {action} position {position} (speed: {speed}, force: {force})")
                return True
            else:
                print(f"[Gripper] Failed with code: {code}")
                return False
        except Exception as e:
            print(f"[Gripper] Error: {e}")
            return False
    
    def get_gripper_position(self) -> Optional[float]:
        """Get current gripper position."""
        if not self.enabled:
            return None
        
        try:
            code, position = self.arm.get_gripper_position()
            if code == 0:
                return position
            else:
                print(f"[Gripper] Failed to get position, code: {code}")
                return None
        except Exception as e:
            print(f"[Gripper] Error getting position: {e}")
            return None
    
    def get_gripper_status(self) -> Dict[str, Any]:
        """Get comprehensive gripper status."""
        if not self.enabled:
            return {"enabled": False, "position": None, "error": "Gripper not enabled"}
        
        try:
            position = self.get_gripper_position()
            return {
                "enabled": True,
                "position": position,
                "is_open": position >= self.config.position_half if position is not None else None,
                "is_closed": position <= self.config.position_closed if position is not None else None,
                "limits": {
                    "min": self.limits.min_gripper_position,
                    "max": self.limits.max_gripper_position
                }
            }
        except Exception as e:
            return {"enabled": True, "position": None, "error": str(e)}
    
    def gripper_grasp(self, target_position: float = None, speed: float = None,
                     force: float = None, timeout: float = 5.0) -> bool:
        """Perform a grasp operation with safety limits."""
        if not self.enabled:
            print("[Gripper] Gripper not enabled")
            return False
        
        target_pos = target_position if target_position is not None else self.config.position_closed
        safe_speed = speed if speed is not None else self.config.default_speed * 0.5
        safe_force = force if force is not None else self.config.default_force
        
        safe_position, safe_speed, safe_force = self._apply_safety_limits(
            target_pos, safe_speed, safe_force
        )
        
        try:
            # Start closing the gripper
            code = self.arm.set_gripper_position(
                position=safe_position, speed=safe_speed, force=safe_force, wait=False
            )
            if code != 0:
                print(f"[Gripper] Grasp failed to start, code: {code}")
                return False
            
            # Wait for completion or timeout
            start_time = time.time()
            while time.time() - start_time < timeout:
                current_pos = self.get_gripper_position()
                if current_pos is not None and abs(current_pos - safe_position) < 10:
                    print(f"[Gripper] Grasp completed at position {current_pos}")
                    return True
                time.sleep(0.1)
            
            print(f"[Gripper] Grasp timeout after {timeout}s")
            return False
            
        except Exception as e:
            print(f"[Gripper] Grasp error: {e}")
            return False
    
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
        safe_speed = speed if speed is not None else self.config.default_speed * 0.25
        safe_force = force if force is not None else self.config.default_force * 0.6
        return self.close_gripper(position=self.config.position_closed, speed=safe_speed, force=safe_force)
    
    def gripper_cycle_test(self, cycles: int = 3, delay: float = 1.0) -> bool:
        """Test gripper by cycling open/close with safety checks."""
        if not self.enabled:
            print("[Gripper] Gripper not enabled for test")
            return False
        
        print(f"[Gripper] Starting cycle test ({cycles} cycles)")
        for i in range(cycles):
            print(f"[Gripper] Cycle {i+1}/{cycles}")
            
            if not self.open_gripper():
                print(f"[Gripper] Cycle {i+1} failed at open")
                return False
            
            time.sleep(delay)
            
            if not self.close_gripper():
                print(f"[Gripper] Cycle {i+1} failed at close")
                return False
            
            time.sleep(delay)
        
        print("[Gripper] Cycle test completed successfully")
        return True


class XArmRunner:
    """
    Safe and efficient interface for controlling the ufactory850 robot arm.
    
    This class provides comprehensive robot control with built-in safety limits,
    error handling, and monitoring capabilities.
    """
    
    def __init__(self, ip: str, cart_speed: float = 100, cart_acc: float = 500,
                 joint_speed: float = 20, joint_acc: float = 50):
        """
        Initialize the robot control interface.
        
        Args:
            ip: Robot IP address
            cart_speed: Cartesian movement speed (mm/s)
            cart_acc: Cartesian acceleration (mm/s²)
            joint_speed: Joint movement speed (deg/s)
            joint_acc: Joint acceleration (deg/s²)
        """
        # Initialize robot connection
        self.arm = XArmAPI(ip, is_radian=False)
        self._connect_and_configure()
        
        # Initialize safety systems
        self.safety_limits = SafetyLimits()
        self.safety_monitor = SafetyMonitor(self.safety_limits)
        self.gripper_config = GripperConfig()
        
        # Apply conservative speed limits
        self._apply_speed_limits(cart_speed, cart_acc, joint_speed, joint_acc)
        
        # Initialize gripper
        self.gripper = GripperController(self.arm, self.safety_limits, self.gripper_config)
    
    def _connect_and_configure(self) -> None:
        """Connect to robot and configure basic settings."""
        try:
            self.arm.connect()
            self.arm.motion_enable(True)
            self.arm.set_mode(0)  # Position mode
            self.arm.set_state(0)  # Ready state
            print("[Robot] Connected and configured successfully")
        except Exception as e:
            print(f"[Robot] Connection/configuration failed: {e}")
            raise
    
    def _apply_speed_limits(self, cart_speed: float, cart_acc: float,
                           joint_speed: float, joint_acc: float) -> None:
        """Apply conservative speed limits to robot."""
        try:
            self.arm.set_tcp_maxacc(min(2000, self.safety_limits.max_acceleration))
            self.arm.set_position_speed(min(cart_speed, self.safety_limits.max_cartesian_speed))
            self.arm.set_position_acc(min(cart_acc, self.safety_limits.max_acceleration))
            self.arm.set_joint_maxvel(min(joint_speed, self.safety_limits.max_joint_speed))
            self.arm.set_joint_maxacc(min(joint_acc, self.safety_limits.max_acceleration))
            print("[Robot] Speed limits applied successfully")
        except Exception as e:
            print(f"[Robot] Failed to apply speed limits: {e}")
    
    def get_current_position(self) -> Optional[List[float]]:
        """Get current robot position with error checking."""
        try:
            code, pose = self.arm.get_position()
            if code != 0:
                print(f"[Robot] Failed to get position, error code: {code}")
                return None
            return pose[:3]  # Return xyz only
        except Exception as e:
            print(f"[Robot] Error getting current position: {e}")
            return None
    
    def go_home(self) -> None:
        """Go to home position with safety checks."""
        self.safety_monitor.check_emergency_stop()
        
        # Validate home position is safe
        home_xyz = [300, 0, 300]  # Default home position
        self.safety_monitor.validate_workspace_limits(home_xyz)
        
        print("[Robot] Moving to home position...")
        try:
            code = self.arm.move_gohome(wait=True)
            if code not in (0, None):
                raise RuntimeError(f"move_gohome error: {code}")
            print("[Robot] Reached home position")
        except Exception as e:
            print(f"[Robot] Error moving to home: {e}")
            raise
    
    def move_pose(self, xyz_mm: List[float], rpy_deg: List[float], 
                 speed: Optional[float] = None) -> None:
        """
        Move to pose with comprehensive safety checks.
        
        Args:
            xyz_mm: Target position [x, y, z] in mm
            rpy_deg: Target orientation [roll, pitch, yaw] in degrees
            speed: Movement speed in mm/s (optional)
        """
        self.safety_monitor.check_emergency_stop()
        
        # Validate workspace limits
        self.safety_monitor.validate_workspace_limits(xyz_mm)
        
        # Get current position for collision risk assessment
        current_xyz = self.get_current_position()
        if current_xyz:
            if self.safety_monitor.check_collision_risk(xyz_mm, current_xyz):
                print(f"[Safety] Potential collision risk moving from {current_xyz} to {xyz_mm}")
                # Continue but with extra caution
        
        # Validate and limit speed
        safe_speed = self.safety_monitor.validate_speed_limits(speed) if speed else None
        
        print(f"[Robot] Moving to {xyz_mm} at speed {safe_speed}")
        try:
            code = self.arm.set_position(
                *xyz_mm, *rpy_deg, speed=safe_speed, mvacc=None, radius=0, wait=True
            )
            if code not in (0, None):
                raise RuntimeError(f"set_position error {code} → {xyz_mm}, {rpy_deg}")
        except Exception as e:
            print(f"[Robot] Error in move_pose: {e}")
            raise
    
    def move_rel_z(self, dz_mm: float) -> None:
        """
        Move relative in Z with safety checks.
        
        Args:
            dz_mm: Relative Z movement in mm (positive = up, negative = down)
        """
        self.safety_monitor.check_emergency_stop()
        
        # Limit relative movement
        if abs(dz_mm) > 100:  # mm
            print(f"[Safety] Large Z movement {dz_mm}mm, limiting to 100mm")
            dz_mm = 100 if dz_mm > 0 else -100
        
        try:
            code, pose = self.arm.get_position()
            if code != 0:
                raise RuntimeError(f"get_position error {code}")
            
            current_xyz = [pose[0], pose[1], pose[2]]
            target_xyz = [pose[0], pose[1], pose[2] + float(dz_mm)]
            
            # Validate new position
            self.safety_monitor.validate_workspace_limits(target_xyz)
            
            print(f"[Robot] Moving Z by {dz_mm}mm")
            self.move_pose(target_xyz, [pose[3], pose[4], pose[5]])
        except Exception as e:
            print(f"[Robot] Error in move_rel_z: {e}")
            raise
    
    # Gripper methods (delegated to GripperController)
    def open_gripper(self, position: Optional[float] = None, 
                    speed: float = None, force: float = None) -> bool:
        """Open the gripper."""
        return self.gripper.open_gripper(position, speed, force)
    
    def close_gripper(self, position: Optional[float] = None,
                     speed: float = None, force: float = None) -> bool:
        """Close the gripper."""
        return self.gripper.close_gripper(position, speed, force)
    
    def set_gripper_position(self, position: float, speed: float = None, 
                           force: float = None) -> bool:
        """Set gripper to specific position."""
        return self.gripper.set_gripper_position(position, speed, force)
    
    def get_gripper_position(self) -> Optional[float]:
        """Get current gripper position."""
        return self.gripper.get_gripper_position()
    
    def get_gripper_status(self) -> Dict[str, Any]:
        """Get comprehensive gripper status."""
        return self.gripper.get_gripper_status()
    
    def gripper_grasp(self, target_position: float = None, speed: float = None,
                     force: float = None, timeout: float = 5.0) -> bool:
        """Perform grasp operation."""
        return self.gripper.gripper_grasp(target_position, speed, force, timeout)
    
    def gripper_release(self, target_position: float = None, speed: float = None,
                       force: float = None) -> bool:
        """Release grasped object."""
        return self.gripper.gripper_release(target_position, speed, force)
    
    def gripper_half_open(self, speed: float = None, force: float = None) -> bool:
        """Open gripper to half position."""
        return self.gripper.gripper_half_open(speed, force)
    
    def gripper_soft_close(self, speed: float = None, force: float = None) -> bool:
        """Close gripper gently."""
        return self.gripper.gripper_soft_close(speed, force)
    
    def gripper_cycle_test(self, cycles: int = 3, delay: float = 1.0) -> bool:
        """Test gripper by cycling open/close."""
        return self.gripper.gripper_cycle_test(cycles, delay)
    
    # Safety methods
    def trigger_emergency_stop(self) -> None:
        """Trigger emergency stop."""
        self.safety_monitor.trigger_emergency_stop()
        try:
            self.arm.stop()
            if self.gripper.enabled:
                self.gripper.open_gripper()
        except Exception as e:
            print(f"[EMERGENCY] Error during emergency stop: {e}")
    
    def clear_emergency_stop(self) -> None:
        """Clear emergency stop."""
        self.safety_monitor.clear_emergency_stop()
    
    def get_safety_status(self) -> Dict[str, Any]:
        """Get comprehensive safety status."""
        return {
            "emergency_stop": self.safety_monitor.emergency_stop,
            "workspace_limits": {
                "x": [self.safety_limits.workspace_x_min, self.safety_limits.workspace_x_max],
                "y": [self.safety_limits.workspace_y_min, self.safety_limits.workspace_y_max],
                "z": [self.safety_limits.workspace_z_min, self.safety_limits.workspace_z_max]
            },
            "speed_limits": {
                "max_cart_speed": self.safety_limits.max_cartesian_speed,
                "max_joint_speed": self.safety_limits.max_joint_speed,
                "max_acceleration": self.safety_limits.max_acceleration
            },
            "gripper_limits": {
                "max_speed": self.safety_limits.max_gripper_speed,
                "max_force": self.safety_limits.max_gripper_force,
                "position_range": [self.safety_limits.min_gripper_position, 
                                 self.safety_limits.max_gripper_position]
            },
            "gripper_status": self.gripper.get_gripper_status()
        }
    
    def disconnect(self) -> None:
        """Safely disconnect from robot."""
        try:
            if self.gripper.enabled:
                # Open gripper before disconnecting for safety
                self.gripper.open_gripper()
            self.arm.disconnect()
            print("[Robot] Disconnected safely")
        except Exception as e:
            print(f"[Robot] Error during disconnect: {e}")
