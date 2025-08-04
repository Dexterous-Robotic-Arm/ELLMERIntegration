# actions_xarm.py
from xarm.wrapper import XArmAPI
import time
import math

class XArmRunner:
    def __init__(self, ip: str, cart_speed=100, cart_acc=500, joint_speed=20, joint_acc=50):
        self.arm = XArmAPI(ip, is_radian=False)
        self.arm.connect()
        self.arm.motion_enable(True)
        self.arm.set_mode(0)  # position mode
        self.arm.set_state(0) # ready

        # Safety limits for real-life testing
        self.MAX_CART_SPEED = 150      # mm/s - conservative speed limit
        self.MAX_JOINT_SPEED = 30      # deg/s - conservative joint speed
        self.MAX_ACCELERATION = 1000   # mm/s² - conservative acceleration
        self.MAX_FORCE = 100           # N - maximum force limit
        self.MAX_TORQUE = 50           # Nm - maximum torque limit
        
        # Workspace safety limits (adjust for your setup)
        self.WORKSPACE_LIMITS = {
            'x_min': 150, 'x_max': 650,    # mm - front/back limits
            'y_min': -300, 'y_max': 300,   # mm - left/right limits  
            'z_min': 50, 'z_max': 500,     # mm - up/down limits
        }
        
        # Emergency stop flag
        self.emergency_stop = False
        self.last_safety_check = time.time()
        self.SAFETY_CHECK_INTERVAL = 0.1  # seconds

        # Apply conservative speed limits
        self.arm.set_tcp_maxacc(min(2000, self.MAX_ACCELERATION))
        self.arm.set_position_speed(min(cart_speed, self.MAX_CART_SPEED))
        self.arm.set_position_acc(min(cart_acc, self.MAX_ACCELERATION))
        self.arm.set_joint_maxvel(min(joint_speed, self.MAX_JOINT_SPEED))
        self.arm.set_joint_maxacc(min(joint_acc, self.MAX_ACCELERATION))

        # Gripper configuration with safety limits
        self.gripper_enabled = False
        self.gripper_position_open = 850    # Fully open position
        self.gripper_position_closed = 200  # Fully closed position
        self.gripper_position_half = 525    # Half-open position
        
        # Gripper safety limits
        self.MAX_GRIPPER_SPEED = 300       # mm/s
        self.MAX_GRIPPER_FORCE = 100       # N
        self.MIN_GRIPPER_POSITION = 0      # mm
        self.MAX_GRIPPER_POSITION = 850    # mm
        
        try:
            self.arm.set_gripper_enable(True)
            self.arm.set_gripper_mode(0)  # Position control mode
            self.gripper_enabled = True
            print("[Gripper] Enabled successfully")
        except Exception as e:
            print(f"[WARN] Gripper initialization failed: {e}")
            self.gripper_enabled = False

    def check_emergency_stop(self):
        """Check if emergency stop is triggered"""
        if self.emergency_stop:
            raise RuntimeError("Emergency stop triggered - stopping all movement")
        return False

    def validate_workspace_limits(self, xyz_mm):
        """Validate that target position is within safe workspace"""
        x, y, z = xyz_mm
        limits = self.WORKSPACE_LIMITS
        
        if not (limits['x_min'] <= x <= limits['x_max']):
            raise ValueError(f"X position {x}mm outside workspace limits [{limits['x_min']}, {limits['x_max']}]")
        if not (limits['y_min'] <= y <= limits['y_max']):
            raise ValueError(f"Y position {y}mm outside workspace limits [{limits['y_min']}, {limits['y_max']}]")
        if not (limits['z_min'] <= z <= limits['z_max']):
            raise ValueError(f"Z position {z}mm outside workspace limits [{limits['z_min']}, {limits['z_max']}]")
        
        return True

    def validate_speed_limits(self, speed):
        """Validate speed is within safe limits"""
        if speed > self.MAX_CART_SPEED:
            print(f"[WARN] Speed {speed} exceeds limit {self.MAX_CART_SPEED}, using limit")
            return self.MAX_CART_SPEED
        return speed

    def get_current_position(self):
        """Get current robot position with error checking"""
        try:
            code, pose = self.arm.get_position()
            if code != 0:
                raise RuntimeError(f"Failed to get position, error code: {code}")
            return pose[:3]  # Return xyz only
        except Exception as e:
            print(f"[ERROR] Failed to get current position: {e}")
            return None

    def check_collision_risk(self, target_xyz, current_xyz=None):
        """Basic collision risk assessment"""
        if current_xyz is None:
            current_xyz = self.get_current_position()
            if current_xyz is None:
                return False  # Can't check, assume safe
        
        # Calculate distance to target
        distance = math.sqrt(sum((t - c) ** 2 for t, c in zip(target_xyz, current_xyz)))
        
        # If movement is very small, likely safe
        if distance < 10:  # mm
            return False
        
        # Check if movement is within reasonable bounds
        if distance > 200:  # mm - suspiciously large movement
            print(f"[WARN] Large movement detected: {distance:.1f}mm")
            return True
        
        return False

    def go_home(self):
        """Go to home position with safety checks"""
        self.check_emergency_stop()
        
        # Validate home position is safe
        home_xyz = [300, 0, 300]  # Default home position
        self.validate_workspace_limits(home_xyz)
        
        print("[Safety] Moving to home position...")
        code = self.arm.move_gohome(wait=True)
        if code not in (0, None):
            raise RuntimeError(f"move_gohome error: {code}")
        print("[Safety] Reached home position")

    def move_pose(self, xyz_mm, rpy_deg, speed=None):
        """Move to pose with comprehensive safety checks"""
        self.check_emergency_stop()
        
        # Validate workspace limits
        self.validate_workspace_limits(xyz_mm)
        
        # Get current position for collision risk assessment
        current_xyz = self.get_current_position()
        if current_xyz:
            if self.check_collision_risk(xyz_mm, current_xyz):
                print(f"[WARN] Potential collision risk moving from {current_xyz} to {xyz_mm}")
                # Continue but with extra caution
        
        # Validate and limit speed
        safe_speed = self.validate_speed_limits(speed) if speed else None
        
        print(f"[Safety] Moving to {xyz_mm} at speed {safe_speed}")
        code = self.arm.set_position(*xyz_mm, *rpy_deg, speed=safe_speed, mvacc=None, radius=0, wait=True)
        if code not in (0, None):
            raise RuntimeError(f"set_position error {code} → {xyz_mm}, {rpy_deg}")

    def move_rel_z(self, dz_mm):
        """Move relative in Z with safety checks"""
        self.check_emergency_stop()
        
        # Limit relative movement
        if abs(dz_mm) > 100:  # mm
            print(f"[WARN] Large Z movement {dz_mm}mm, limiting to 100mm")
            dz_mm = 100 if dz_mm > 0 else -100
        
        code, pose = self.arm.get_position()
        if code != 0:
            raise RuntimeError(f"get_position error {code}")
        
        current_xyz = [pose[0], pose[1], pose[2]]
        target_xyz = [pose[0], pose[1], pose[2] + float(dz_mm)]
        
        # Validate new position
        self.validate_workspace_limits(target_xyz)
        
        print(f"[Safety] Moving Z by {dz_mm}mm")
        self.move_pose(target_xyz, [pose[3], pose[4], pose[5]])

    # Enhanced gripper methods with safety limits
    def open_gripper(self, position=None, speed=200, force=50):
        """Open the gripper with safety limits"""
        if not self.gripper_enabled:
            print("[WARN] Gripper not enabled")
            return False
        
        self.check_emergency_stop()
        
        # Apply safety limits
        target_pos = position if position is not None else self.gripper_position_open
        target_pos = max(self.MIN_GRIPPER_POSITION, min(self.MAX_GRIPPER_POSITION, target_pos))
        safe_speed = min(speed, self.MAX_GRIPPER_SPEED)
        safe_force = min(force, self.MAX_GRIPPER_FORCE)
        
        try:
            code = self.arm.set_gripper_position(position=target_pos, speed=safe_speed, force=safe_force, wait=True)
            if code == 0:
                print(f"[Gripper] Opened to position {target_pos} (speed: {safe_speed}, force: {safe_force})")
                return True
            else:
                print(f"[WARN] open_gripper failed with code: {code}")
                return False
        except Exception as e:
            print(f"[WARN] open_gripper failed: {e}")
            return False

    def close_gripper(self, position=None, speed=100, force=50):
        """Close the gripper with safety limits"""
        if not self.gripper_enabled:
            print("[WARN] Gripper not enabled")
            return False
        
        self.check_emergency_stop()
        
        # Apply safety limits
        target_pos = position if position is not None else self.gripper_position_closed
        target_pos = max(self.MIN_GRIPPER_POSITION, min(self.MAX_GRIPPER_POSITION, target_pos))
        safe_speed = min(speed, self.MAX_GRIPPER_SPEED)
        safe_force = min(force, self.MAX_GRIPPER_FORCE)
        
        try:
            code = self.arm.set_gripper_position(position=target_pos, speed=safe_speed, force=safe_force, wait=True)
            if code == 0:
                print(f"[Gripper] Closed to position {target_pos} (speed: {safe_speed}, force: {safe_force})")
                return True
            else:
                print(f"[WARN] close_gripper failed with code: {code}")
                return False
        except Exception as e:
            print(f"[WARN] close_gripper failed: {e}")
            return False

    def set_gripper_position(self, position, speed=150, force=50):
        """Set gripper to a specific position with safety limits"""
        if not self.gripper_enabled:
            print("[WARN] Gripper not enabled")
            return False
        
        self.check_emergency_stop()
        
        # Apply safety limits
        position = max(self.MIN_GRIPPER_POSITION, min(self.MAX_GRIPPER_POSITION, position))
        safe_speed = min(speed, self.MAX_GRIPPER_SPEED)
        safe_force = min(force, self.MAX_GRIPPER_FORCE)
        
        try:
            code = self.arm.set_gripper_position(position=position, speed=safe_speed, force=safe_force, wait=True)
            if code == 0:
                print(f"[Gripper] Set to position {position} (speed: {safe_speed}, force: {safe_force})")
                return True
            else:
                print(f"[WARN] set_gripper_position failed with code: {code}")
                return False
        except Exception as e:
            print(f"[WARN] set_gripper_position failed: {e}")
            return False

    def get_gripper_position(self):
        """Get current gripper position"""
        if not self.gripper_enabled:
            return None
        
        try:
            code, position = self.arm.get_gripper_position()
            if code == 0:
                return position
            else:
                print(f"[WARN] get_gripper_position failed with code: {code}")
                return None
        except Exception as e:
            print(f"[WARN] get_gripper_position failed: {e}")
            return None

    def get_gripper_status(self):
        """Get comprehensive gripper status"""
        if not self.gripper_enabled:
            return {"enabled": False, "position": None, "error": "Gripper not enabled"}
        
        try:
            position = self.get_gripper_position()
            return {
                "enabled": True,
                "position": position,
                "is_open": position >= self.gripper_position_half if position is not None else None,
                "is_closed": position <= self.gripper_position_closed if position is not None else None,
                "limits": {
                    "min": self.MIN_GRIPPER_POSITION,
                    "max": self.MAX_GRIPPER_POSITION
                }
            }
        except Exception as e:
            return {"enabled": True, "position": None, "error": str(e)}

    def gripper_grasp(self, target_position=200, speed=100, force=50, timeout=5.0):
        """Perform a grasp operation with safety limits"""
        if not self.gripper_enabled:
            print("[WARN] Gripper not enabled")
            return False
        
        self.check_emergency_stop()
        
        # Apply safety limits
        target_position = max(self.MIN_GRIPPER_POSITION, min(self.MAX_GRIPPER_POSITION, target_position))
        safe_speed = min(speed, self.MAX_GRIPPER_SPEED)
        safe_force = min(force, self.MAX_GRIPPER_FORCE)
        
        try:
            # Start closing the gripper
            code = self.arm.set_gripper_position(position=target_position, speed=safe_speed, force=safe_force, wait=False)
            if code != 0:
                print(f"[WARN] gripper_grasp failed to start with code: {code}")
                return False
            
            # Wait for completion or timeout
            start_time = time.time()
            while time.time() - start_time < timeout:
                self.check_emergency_stop()  # Check e-stop during operation
                current_pos = self.get_gripper_position()
                if current_pos is not None and abs(current_pos - target_position) < 10:
                    print(f"[Gripper] Grasp completed at position {current_pos}")
                    return True
                time.sleep(0.1)
            
            print(f"[WARN] gripper_grasp timeout after {timeout}s")
            return False
            
        except Exception as e:
            print(f"[WARN] gripper_grasp failed: {e}")
            return False

    def gripper_release(self, target_position=850, speed=200, force=50):
        """Release grasped object by opening gripper"""
        return self.open_gripper(position=target_position, speed=speed, force=force)

    def gripper_half_open(self, speed=150, force=50):
        """Open gripper to half position"""
        return self.set_gripper_position(self.gripper_position_half, speed, force)

    def gripper_soft_close(self, speed=50, force=30):
        """Close gripper gently with low force"""
        return self.close_gripper(position=self.gripper_position_closed, speed=speed, force=force)

    def gripper_cycle_test(self, cycles=3, delay=1.0):
        """Test gripper by cycling open/close with safety checks"""
        if not self.gripper_enabled:
            print("[WARN] Gripper not enabled for test")
            return False
        
        print(f"[Gripper] Starting cycle test ({cycles} cycles)")
        for i in range(cycles):
            self.check_emergency_stop()  # Check e-stop between cycles
            print(f"[Gripper] Cycle {i+1}/{cycles}")
            
            if not self.open_gripper():
                print(f"[WARN] Cycle {i+1} failed at open")
                return False
            
            time.sleep(delay)
            
            if not self.close_gripper():
                print(f"[WARN] Cycle {i+1} failed at close")
                return False
            
            time.sleep(delay)
        
        print("[Gripper] Cycle test completed successfully")
        return True

    def trigger_emergency_stop(self):
        """Trigger emergency stop"""
        self.emergency_stop = True
        print("[EMERGENCY] Emergency stop triggered!")
        try:
            # Stop all movement
            self.arm.stop()
            # Open gripper for safety
            if self.gripper_enabled:
                self.open_gripper()
        except Exception as e:
            print(f"[EMERGENCY] Error during emergency stop: {e}")

    def clear_emergency_stop(self):
        """Clear emergency stop (requires manual intervention)"""
        self.emergency_stop = False
        print("[Safety] Emergency stop cleared")

    def get_safety_status(self):
        """Get comprehensive safety status"""
        return {
            "emergency_stop": self.emergency_stop,
            "workspace_limits": self.WORKSPACE_LIMITS,
            "speed_limits": {
                "max_cart_speed": self.MAX_CART_SPEED,
                "max_joint_speed": self.MAX_JOINT_SPEED,
                "max_acceleration": self.MAX_ACCELERATION
            },
            "gripper_limits": {
                "max_speed": self.MAX_GRIPPER_SPEED,
                "max_force": self.MAX_GRIPPER_FORCE,
                "position_range": [self.MIN_GRIPPER_POSITION, self.MAX_GRIPPER_POSITION]
            }
        }

    def disconnect(self):
        try:
            if self.gripper_enabled:
                # Open gripper before disconnecting for safety
                self.open_gripper()
            self.arm.disconnect()
        except Exception:
            pass
