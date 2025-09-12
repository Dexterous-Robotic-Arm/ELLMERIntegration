# executor.py
import time, threading
import json
import logging
import subprocess
import sys
import os
import numpy as np
from typing import List, Dict, Any, Optional

# Initialize fallback values BEFORE try-except
ROS2_AVAILABLE = False
Node = object  # Fallback for when ROS2 is not available
StringMsg = str  # Fallback for when ROS2 is not available

# Optional ROS2 imports - only import if available
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String as StringMsg
    ROS2_AVAILABLE = True
except ImportError:
    print("Warning: ROS2 not available, running in simulation mode")

from .actions_xarm import XArmRunner

class CameraToRobotTransformer:
    """Handles coordinate transformation from camera to robot base frame."""
    
    def __init__(self, camera_offset_mm: List[float] = [0, 0, 22.5]):
        """
        Initialize transformer with camera offset from TCP.
        
        Args:
            camera_offset_mm: [x, y, z] offset from TCP to camera optical center (mm)
        """
        self.camera_offset = np.array(camera_offset_mm)
        
        # Transformation matrix from camera frame to robot TCP frame
        # Camera: X=left/right, Y=up/down, Z=forward/backward
        # Robot:  X=forward/backward, Y=left/right, Z=up/down
        self.camera_to_tcp_matrix = np.array([
            [0,  0,  1],   # Camera Z (forward/backward) -> Robot X (forward/backward)
            [1,  0,  0],   # Camera X (left/right) -> Robot Y (left/right)
            [0,  1,  0]    # Camera Y (up/down) -> Robot Z (up/down)
        ])
    
    def transform_camera_to_robot_base(self, 
                                    camera_coords: List[float], 
                                    robot_tcp_position: List[float]) -> List[float]:
        """
        Transform camera coordinates to robot base coordinates.
        
        Args:
            camera_coords: [x, y, z] in camera frame (mm)
            robot_tcp_position: Current robot TCP position [x, y, z] (mm)
            
        Returns:
            robot_base_coords: [x, y, z] in robot base frame (mm)
        """
        # Convert to numpy arrays
        cam_coords = np.array(camera_coords)
        tcp_pos = np.array(robot_tcp_position)
        
        print(f"[TRANSFORM DEBUG] Input camera coords: {camera_coords}")
        print(f"[TRANSFORM DEBUG] Input robot TCP: {robot_tcp_position}")
        
        # Step 1: Transform camera coordinates to TCP-relative coordinates
        tcp_relative = self.camera_to_tcp_matrix @ cam_coords
        print(f"[TRANSFORM DEBUG] After matrix transform: {tcp_relative}")
        
        # Step 2: Subtract camera offset from TCP (we want TCP to move to object position)
        # Camera offset represents where camera is relative to TCP, so we subtract it
        tcp_relative -= self.camera_offset
        print(f"[TRANSFORM DEBUG] After subtracting camera offset {self.camera_offset}: {tcp_relative}")
        
        # Step 3: Add current TCP position to get robot base coordinates
        robot_base_coords = tcp_pos + tcp_relative
        print(f"[TRANSFORM DEBUG] Final robot base coords: {robot_base_coords}")
        
        # Safety check: Ensure coordinates are within reasonable bounds
        if (abs(robot_base_coords[0]) > 1000 or abs(robot_base_coords[1]) > 1000 or 
            robot_base_coords[2] < 0 or robot_base_coords[2] > 1000):
            print(f"[TRANSFORM WARNING] Coordinates seem out of bounds: {robot_base_coords}")
            print(f"[TRANSFORM WARNING] This might indicate incorrect transformation!")
        
        return robot_base_coords.tolist()

DEFAULT_HOVER_MM = 80
DEFAULT_PICK_RPY = [0, 90, 0]  # Tilt camera up more to see objects
# Camera is mounted on the end effector; distance from TCP to camera optical center (mm)
# If you recalibrate the mount, update this value accordingly.
CAMERA_TO_TCP_OFFSET_MM = [0, 0, 22.5]  # [x, y, z] offset from TCP to camera optical center

# Safety limits for real-life testing
MAX_EXECUTION_TIME = 300  # seconds - maximum time for any plan
MAX_STEPS_PER_PLAN = 50   # maximum number of steps in a plan
SAFETY_CHECK_INTERVAL = 0.5  # seconds between safety checks

# Named poses (edit to your workspace). Units: mm/deg
# Updated for camera-mounted robot - camera tilted up more to see objects
WORLD_POSES = {
    "home":             {"xyz_mm": [300,   0, 300], "rpy_deg": [0, 90, 0]},
    "bin_drop":         {"xyz_mm": [250, 250, 120], "rpy_deg": [0, 90, 0]},
    "table_pick_blue":  {"xyz_mm": [400,-150,  45], "rpy_deg": [0, 90, 0]},
    "staging_area":     {"xyz_mm": [450, -150, 150], "rpy_deg": [0, 90, 0]},
    "scan_center":      {"xyz_mm": [400,   0, 250], "rpy_deg": [0, 90, 0]},
}

class ObjectIndex(Node if ROS2_AVAILABLE else object):
    """Caches latest pose per label (mm) from /detected_objects."""
    # Class-level storage to persist across instances
    _global_latest_mm = {}
    _global_lock = threading.Lock()
    _movement_in_progress = False  # Flag to freeze updates during movement
    
    def __init__(self):
        self._lock = threading.Lock()
        # Use class-level storage for persistence
        self.latest_mm = ObjectIndex._global_latest_mm
        
        # Only setup ROS2 subscription if available
        if ROS2_AVAILABLE:
            super().__init__('object_index')
            self.sub = self.create_subscription(StringMsg, '/detected_objects', self._on_msg, 10)
            print("[ObjectIndex] ROS2 subscription created for /detected_objects")
        else:
            self.sub = None
            print("[ObjectIndex] Running without ROS2 - no subscription")

    def _normalize_object_name(self, class_name: str) -> str:
        """Normalize object names to match robot expectations."""
        # Simple approach: just look for object names in the string
        if "bottle" in class_name.lower():
            return "bottle"
        if "cup" in class_name.lower():
            return "cup"
        if "book" in class_name.lower():
            return "book"
        if "pen" in class_name.lower():
            return "pen"
        if "phone" in class_name.lower():
            return "phone"
            
        return class_name
    
    @classmethod
    def set_movement_in_progress(cls, in_progress: bool):
        """Set flag to freeze coordinate updates during movement."""
        cls._movement_in_progress = in_progress
        if in_progress:
            print("[ObjectIndex] Freezing coordinate updates during movement")
        else:
            print("[ObjectIndex] Resuming coordinate updates")
    
    def _on_msg(self, msg):
        try:
            # Skip updates if movement is in progress (prevents chasing moving targets)
            if self._movement_in_progress:
                return
                
            if hasattr(msg, 'data'):
                data = json.loads(msg.data)
            else:
                data = msg  # Handle direct dict input
            units = data.get("units", "m")
            k = 1000.0 if units == "m" else 1.0
            items = data.get("items", [])
            print(f"[ObjectIndex] Received message with {len(items)} items: {items}")
            with self._global_lock:
                for it in items:
                    lab = it.get("class")
                    pos = it.get("pos", [0,0,0])
                    if lab and isinstance(pos, list) and len(pos) == 3:
                        # Normalize object name
                        normalized_lab = self._normalize_object_name(lab)
                        # Store camera coordinates directly - transformation will be done during movement
                        # Camera frame: X=left/right, Y=up/down, Z=forward/backward
                        self.latest_mm[normalized_lab] = [float(pos[0])*k, float(pos[1])*k, float(pos[2])*k]
                        print(f"[ObjectIndex] Updated {normalized_lab} (from {lab}) camera position: {self.latest_mm[normalized_lab]}")
        except Exception as e:
            print(f"[ObjectIndex] Error processing message: {e}")
            pass

    def wait_for(self, label: str, timeout=5.0):
        t0 = time.time()
        print(f"[ObjectIndex] Waiting for '{label}' for {timeout}s...")
        print(f"[ObjectIndex] Current cached objects: {list(self.latest_mm.keys())}")
        
        # Check if we already have the object cached
        with self._global_lock:
            p = self.latest_mm.get(label)
            if p is not None:
                print(f"[ObjectIndex] Found '{label}' in cache at position: {p}")
                return p
        
        # Wait for new detections
        while time.time() - t0 < timeout:
            with self._global_lock:
                p = self.latest_mm.get(label)
            if p is not None:
                print(f"[ObjectIndex] Found '{label}' at position: {p}")
                return p
            time.sleep(0.05)
        print(f"[ObjectIndex] Timeout waiting for '{label}'. Available objects: {list(self.latest_mm.keys())}")
        raise TimeoutError(f"Object '{label}' not seen on /detected_objects within {timeout}s")

class TaskExecutor:
    def __init__(self, arm_ip: str, world_yaml: str = None, sim: bool = False, dry_run: bool = False, runner: XArmRunner = None):
        # Use provided runner or create new one
        if runner is not None:
            self.runner = runner
            print("[TaskExecutor] Using provided XArmRunner instance")
        else:
            self.runner = XArmRunner(arm_ip, sim=sim)
            print("[TaskExecutor] Created new XArmRunner instance")
        
        self.world = WORLD_POSES
        self.hover_mm = DEFAULT_HOVER_MM
        self.pick_rpy = DEFAULT_PICK_RPY
        self.sim = sim
        self.dry_run = dry_run
        
        # Initialize coordinate transformer
        self.coordinate_transformer = CameraToRobotTransformer(CAMERA_TO_TCP_OFFSET_MM)
        
        # Constant J5 position for all movements (except home)
        # J5 at 90 degrees = camera pointing up for scanning
        self.constant_j5_rpy = [0, 90, 0]  # Roll=0, Pitch=90, Yaw=0
        
        # Optionally get current J5 position and use it as constant
        if not self.sim and not self.dry_run:
            try:
                current_joints = self.runner.arm.get_servo_angle()
                if current_joints[0] == 0:
                    current_j5 = current_joints[1][4]  # J5 is index 4
                    print(f"[J5] Current J5 position: {current_j5} degrees")
                    # Convert J5 angle to RPY (simplified conversion)
                    if abs(current_j5 - 90) < 10:  # If J5 is close to 90 degrees
                        self.constant_j5_rpy = [0, 90, 0]
                        print(f"[J5] Using J5=90° (camera pointing up) as constant position")
                    else:
                        print(f"[J5] Using default J5=90° as constant position")
            except Exception as e:
                print(f"[J5] Could not get current J5 position: {e}")
                print(f"[J5] Using default J5=90° as constant position")
        
        # Safety monitoring
        self.execution_start_time = None
        self.step_count = 0
        self.error_count = 0
        self.max_errors = 3  # Stop after this many consecutive errors

        # Initialize vision system if needed
        self.vision_process = None
        self._start_vision_system()

        # Load world poses from YAML if provided
        if world_yaml:
            try:
                import yaml
                with open(world_yaml, 'r') as f:
                    world_data = yaml.safe_load(f)
                    poses = world_data.get('poses', {})
                    for name, pose_data in poses.items():
                        self.world[name] = {
                            "xyz_mm": pose_data.get("xyz_mm", [0, 0, 0]),
                            "rpy_deg": pose_data.get("rpy_deg", [180, 0, 0])
                        }
            except Exception as e:
                print(f"[WARN] Failed to load world YAML {world_yaml}: {e}")

        # Initialize ROS2 only if available
        if ROS2_AVAILABLE:
            rclpy.init(args=None)
            self.obj_index = ObjectIndex()
            self._spin_thread = threading.Thread(target=rclpy.spin, args=(self.obj_index,), daemon=True)
            self._spin_thread.start()
            
            # Wait for vision system to initialize
            if self.vision_process:
                print("[Vision] Waiting for vision system to initialize...")
                time.sleep(3)
            
            # Note: Vision system handled by main.py launching apriltag_bridge.py
            print("[Vision] Vision system ready - camera handled by apriltag_bridge.py")
        else:
            self.obj_index = ObjectIndex()
            self._spin_thread = None
    
    def _is_coordinate_safe(self, coordinates: list) -> bool:
        """Validate if coordinates are within safe robot workspace."""
        if len(coordinates) < 3:
            return False
        
        x, y, z = coordinates[0], coordinates[1], coordinates[2]
        
        # Define safe workspace limits (in mm)
        # These are typical xArm workspace limits - adjust for your robot
        safe_limits = {
            'x_min': 200,   # Minimum reach in X
            'x_max': 700,   # Maximum reach in X  
            'y_min': -500,  # Minimum reach in Y
            'y_max': 500,   # Maximum reach in Y
            'z_min': 50,    # Minimum height
            'z_max': 600    # Maximum height
        }
        
        # Check if coordinates are within safe workspace
        if not (safe_limits['x_min'] <= x <= safe_limits['x_max']):
            print(f"[SAFETY] X coordinate {x:.1f}mm outside safe range [{safe_limits['x_min']}, {safe_limits['x_max']}]")
            return False
            
        if not (safe_limits['y_min'] <= y <= safe_limits['y_max']):
            print(f"[SAFETY] Y coordinate {y:.1f}mm outside safe range [{safe_limits['y_min']}, {safe_limits['y_max']}]")
            return False
            
        if not (safe_limits['z_min'] <= z <= safe_limits['z_max']):
            print(f"[SAFETY] Z coordinate {z:.1f}mm outside safe range [{safe_limits['z_min']}, {safe_limits['z_max']}]")
            return False
        
        print(f"[SAFETY] Coordinates [{x:.1f}, {y:.1f}, {z:.1f}]mm are within safe workspace")
        return True

    def _start_vision_system(self):
        """Start the vision system for object detection."""
        try:
            # Vision system is now handled by main.py launching apriltag_bridge.py
            # No need to start pose_recorder.py here anymore
            print("[Vision] Vision system handled by main.py (apriltag_bridge.py)")
            self.vision_recorder = None
            
            # Vision system as a separate process for ROS2 publishing
            # DISABLED: main.py now handles vision system via apriltag_bridge.py
            print("[Vision] Skipping vision process - handled by main.py")
            self.vision_process = None
            
            # Vision system now handled by main.py
        except Exception as e:
            print(f"[Vision] Error starting vision system: {e}")
            self.vision_process = None
            self.vision_recorder = None

    def _named(self, name: str):
        if name not in self.world:
            raise KeyError(f"Named pose '{name}' not in WORLD_POSES.")
        p = self.world[name]
        return p["xyz_mm"], p["rpy_deg"]
    
    def _perform_hierarchical_scan(self, 
                                 target_objects: List[str] = [],
                                 sweep_mm: float = 300,
                                 steps: int = 5,
                                 pause_sec: float = 1.0,
                                 interrupt_on_detection: bool = True) -> List[str]:
        """
        Perform hierarchical scanning: horizontal → arc → overhead.
        
        Args:
            target_objects: List of specific objects to look for (empty = any object)
            sweep_mm: Sweep distance in mm
            steps: Number of scan positions per pattern
            pause_sec: Pause time at each position
            interrupt_on_detection: Whether to stop when objects are found
            
        Returns:
            List of found object labels (empty if none found)
        """
        print(f"[HIERARCHICAL_SCAN] Starting hierarchical scan for targets: {target_objects if target_objects else 'any'}")
        
        # Get current position for reference
        current_pos = self.runner.get_current_position()
        print(f"[HIERARCHICAL_SCAN] Current position: {current_pos}")
        
        # Define scan parameters
        scan_x = 400.0  # Center of X workspace
        base_scan_z = 250.0  # Base height for horizontal scan
        
        if current_pos is not None and len(current_pos) >= 3:
            if 100 <= current_pos[2] <= 400:
                base_scan_z = current_pos[2]
        
        # Calculate sweep range
        y_min = max(-300, -sweep_mm / 2)
        y_max = min(300, sweep_mm / 2)
        
        # Ensure minimum steps
        if steps < 2:
            steps = 2
        
        # 1. HORIZONTAL SCAN
        print(f"[HIERARCHICAL_SCAN] 🔍 Phase 1: Horizontal scan at Z={base_scan_z}")
        found_objects = self._perform_horizontal_scan(
            scan_x, base_scan_z, y_min, y_max, steps, pause_sec, 
            target_objects, interrupt_on_detection
        )
        
        if found_objects and interrupt_on_detection:
            print(f"[HIERARCHICAL_SCAN] ✅ Horizontal scan found targets: {found_objects}")
            return found_objects
        
        # 2. ARC SCAN
        print(f"[HIERARCHICAL_SCAN] 🔍 Phase 2: Arc scan")
        found_objects = self._perform_arc_scan(
            scan_x, base_scan_z, y_min, y_max, steps, pause_sec,
            target_objects, interrupt_on_detection
        )
        
        if found_objects and interrupt_on_detection:
            print(f"[HIERARCHICAL_SCAN] ✅ Arc scan found targets: {found_objects}")
            return found_objects
        
        # 3. OVERHEAD SCAN
        print(f"[HIERARCHICAL_SCAN] 🔍 Phase 3: Overhead scan")
        found_objects = self._perform_overhead_scan(
            scan_x, base_scan_z, y_min, y_max, steps, pause_sec,
            target_objects, interrupt_on_detection
        )
        
        if found_objects:
            print(f"[HIERARCHICAL_SCAN] ✅ Overhead scan found targets: {found_objects}")
        else:
            print(f"[HIERARCHICAL_SCAN] ❌ All scan phases completed - no targets found")
        
        return found_objects
    
    def _perform_horizontal_scan(self, scan_x: float, scan_z: float, y_min: float, y_max: float,
                                steps: int, pause_sec: float, target_objects: List[str],
                                interrupt_on_detection: bool) -> List[str]:
        """Perform horizontal sweep scan."""
        print(f"[HORIZONTAL_SCAN] Sweep range: Y={y_min:.1f} to {y_max:.1f}")
        
        # Move to scan center first
        scan_center = [scan_x, 0, scan_z]
        print(f"[HORIZONTAL_SCAN] Moving to scan center: {scan_center}")
        
        try:
            self.runner.move_pose(scan_center, self.constant_j5_rpy)
            print(f"[HORIZONTAL_SCAN] Arrived at scan center")
        except Exception as e:
            print(f"[HORIZONTAL_SCAN] Failed to move to scan center: {e}")
            return []
        
        # Perform horizontal sweep
        for j in range(steps):
            # Calculate target position
            if steps == 1:
                y_target = y_min
            else:
                y_target = y_min + (y_max - y_min) * j / (steps - 1)
            
            target = [scan_x, y_target, scan_z]
            print(f"[HORIZONTAL_SCAN] Moving to position {j+1}/{steps}: {target}")
            
            try:
                self.runner.move_pose(target, self.constant_j5_rpy)
                print(f"[HORIZONTAL_SCAN] Pausing {pause_sec}s for detection...")
                
                # Check for objects during pause
                found_targets = self._check_for_objects_during_pause(
                    pause_sec, target_objects, interrupt_on_detection
                )
                
                if found_targets and interrupt_on_detection:
                    print(f"[HORIZONTAL_SCAN] 🎯 TARGETS DETECTED: {found_targets}")
                    return found_targets
                    
            except Exception as e:
                print(f"[HORIZONTAL_SCAN] Error at position {j+1}: {e}")
                continue
        
        print(f"[HORIZONTAL_SCAN] ✅ Horizontal scan completed - no targets found")
        return []
    
    def _perform_arc_scan(self, scan_x: float, base_scan_z: float, y_min: float, y_max: float,
                          steps: int, pause_sec: float, target_objects: List[str],
                          interrupt_on_detection: bool) -> List[str]:
        """Perform arc pattern scan with varying heights."""
        print(f"[ARC_SCAN] Performing arc pattern with {steps} positions")
        
        # Move to arc start position
        arc_start = [scan_x, y_min, base_scan_z]
        print(f"[ARC_SCAN] Moving to arc start: {arc_start}")
        
        try:
            self.runner.move_pose(arc_start, self.constant_j5_rpy)
        except Exception as e:
            print(f"[ARC_SCAN] Failed to move to arc start: {e}")
            return []
        
        # Perform arc sweep with varying Z heights
        for j in range(steps):
            # Calculate Y position
            if steps == 1:
                y_target = y_min
            else:
                y_target = y_min + (y_max - y_min) * j / (steps - 1)
            
            # Calculate Z position (arc pattern: lower at edges, higher in center)
            arc_height_range = 50.0  # 50mm height variation
            if steps == 1:
                z_offset = 0
            else:
                # Create arc: low at edges, high in center
                progress = j / (steps - 1)  # 0 to 1
                arc_factor = 4 * progress * (1 - progress)  # Parabolic arc: 0 at edges, 1 at center
                z_offset = arc_height_range * arc_factor
            
            target_z = base_scan_z + z_offset
            target = [scan_x, y_target, target_z]
            
            print(f"[ARC_SCAN] Moving to arc position {j+1}/{steps}: {target} (Z offset: {z_offset:.1f})")
            
            try:
                self.runner.move_pose(target, self.constant_j5_rpy)
                print(f"[ARC_SCAN] Pausing {pause_sec}s for detection...")
                
                # Check for objects during pause
                found_targets = self._check_for_objects_during_pause(
                    pause_sec, target_objects, interrupt_on_detection
                )
                
                if found_targets and interrupt_on_detection:
                    print(f"[ARC_SCAN] 🎯 TARGETS DETECTED: {found_targets}")
                    return found_targets
                    
            except Exception as e:
                print(f"[ARC_SCAN] Error at position {j+1}: {e}")
                continue
        
        print(f"[ARC_SCAN] ✅ Arc scan completed - no targets found")
        return []
    
    def _perform_overhead_scan(self, scan_x: float, base_scan_z: float, y_min: float, y_max: float,
                              steps: int, pause_sec: float, target_objects: List[str],
                              interrupt_on_detection: bool) -> List[str]:
        """Perform overhead scan from higher position."""
        print(f"[OVERHEAD_SCAN] Performing overhead scan with {steps} positions")
        
        # Use higher Z position for overhead view
        overhead_z = min(400, base_scan_z + 100)  # 100mm higher, but not exceeding limits
        
        # Move to overhead center first
        overhead_center = [scan_x, 0, overhead_z]
        print(f"[OVERHEAD_SCAN] Moving to overhead center: {overhead_center}")
        
        try:
            self.runner.move_pose(overhead_center, self.constant_j5_rpy)
        except Exception as e:
            print(f"[OVERHEAD_SCAN] Failed to move to overhead center: {e}")
            return []
        
        # Perform overhead sweep
        for j in range(steps):
            # Calculate target position
            if steps == 1:
                y_target = y_min
            else:
                y_target = y_min + (y_max - y_min) * j / (steps - 1)
            
            target = [scan_x, y_target, overhead_z]
            print(f"[OVERHEAD_SCAN] Moving to overhead position {j+1}/{steps}: {target}")
            
            try:
                self.runner.move_pose(target, self.constant_j5_rpy)
                print(f"[OVERHEAD_SCAN] Pausing {pause_sec}s for detection...")
                
                # Check for objects during pause
                found_targets = self._check_for_objects_during_pause(
                    pause_sec, target_objects, interrupt_on_detection
                )
                
                if found_targets and interrupt_on_detection:
                    print(f"[OVERHEAD_SCAN] 🎯 TARGETS DETECTED: {found_targets}")
                    return found_targets
                    
            except Exception as e:
                print(f"[OVERHEAD_SCAN] Error at position {j+1}: {e}")
                continue
        
        print(f"[OVERHEAD_SCAN] ✅ Overhead scan completed - no targets found")
        return []
    
    def _check_for_objects_during_pause(self, pause_sec: float, target_objects: List[str],
                                      interrupt_on_detection: bool) -> List[str]:
        """Check for objects during pause period."""
        if not interrupt_on_detection:
            time.sleep(pause_sec)
            return []
        
        # Check for objects during pause with shorter intervals
        detection_interval = 0.2  # Check every 200ms
        elapsed = 0.0
        
        while elapsed < pause_sec:
            time.sleep(detection_interval)
            elapsed += detection_interval
            
            # Check if target objects are detected
            with self.obj_index._global_lock:
                detected_objects = list(self.obj_index.latest_mm.keys())
            
            # Check if we found any target objects (or any objects if no specific targets)
            found_targets = []
            if target_objects:
                # Look for specific target objects
                found_targets = [obj for obj in target_objects if obj in detected_objects]
            else:
                # Any detected object is a target
                found_targets = detected_objects
            
            if found_targets:
                if target_objects:
                    print(f"[OBJECT_CHECK] 🎯 TARGET OBJECTS DETECTED: {found_targets}")
                else:
                    print(f"[OBJECT_CHECK] 🎯 OBJECTS DETECTED: {found_targets}")
                return found_targets
        
        return []
    
    def _verify_target_coordinates(self, target: List[float]) -> bool:
        """Verify target coordinates are valid (safety checks disabled)."""
        try:
            if len(target) < 3:
                print(f"[VERIFY] Invalid coordinate format: {target}")
                return False
            
            x, y, z = target[0], target[1], target[2]
            
            # Check for NaN or infinite values only
            if not all(isinstance(coord, (int, float)) and not (coord != coord or coord == float('inf') or coord == float('-inf')) for coord in target):
                print(f"[VERIFY] Invalid coordinate values (NaN/Inf): {target}")
                return False
            
            # SAFETY CHECKS DISABLED - Allow robot to move to any coordinates
            print(f"[VERIFY] Coordinates validated (safety checks disabled): {target}")
            return True
            
        except Exception as e:
            print(f"[VERIFY] Error validating coordinates: {e}")
            return False
    
    def _execute_movement_with_verification(self, target: List[float], rpy: List[float], speed: float) -> bool:
        """Execute movement with proper state synchronization and verification."""
        try:
            # Freeze coordinate updates during movement
            ObjectIndex.set_movement_in_progress(True)
            
            # Execute movement
            self.runner.move_pose(target, rpy, speed)
            
            # Wait for movement to complete
            time.sleep(0.5)
            
            # Verify movement completed successfully
            current_pos = self.runner.get_current_position()
            if current_pos is None:
                print(f"[VERIFY] Cannot verify movement - no position data")
                return False
            
            # Check if robot is close to target (within 10mm tolerance)
            distance = ((current_pos[0] - target[0])**2 + 
                       (current_pos[1] - target[1])**2 + 
                       (current_pos[2] - target[2])**2)**0.5
            
            if distance > 10.0:  # 10mm tolerance
                print(f"[VERIFY] Movement verification failed - distance: {distance:.1f}mm")
                print(f"[VERIFY] Target: {target}, Current: {current_pos}")
                return False
            
            print(f"[VERIFY] Movement verified - distance: {distance:.1f}mm")
            return True
            
        except Exception as e:
            print(f"[VERIFY] Movement execution error: {e}")
            return False
        finally:
            # Always resume coordinate updates
            ObjectIndex.set_movement_in_progress(False)
    
    def _verify_step_completion(self, action: str, step: dict, step_num: int) -> bool:
        """Verify that a step completed successfully before proceeding."""
        try:
            if action in ("MOVE_TO_NAMED", "MOVE_TO_POSE"):
                # For named poses, verify we're at the expected location
                if action == "MOVE_TO_NAMED":
                    name = step.get("name")
                    if name in self.world:
                        expected_xyz, expected_rpy = self._named(name)
                        current_pos = self.runner.get_current_position()
                        if current_pos is None:
                            return False
                        
                        # Check if we're close to expected position
                        distance = ((current_pos[0] - expected_xyz[0])**2 + 
                                   (current_pos[1] - expected_xyz[1])**2 + 
                                   (current_pos[2] - expected_xyz[2])**2)**0.5
                        
                        if distance > 20.0:  # 20mm tolerance for named poses
                            print(f"[VERIFY] Step {step_num} verification failed - distance to {name}: {distance:.1f}mm")
                            return False
                        
                        print(f"[VERIFY] Step {step_num} verified - at {name} (distance: {distance:.1f}mm)")
                        return True
                
            elif action == "RETREAT_Z":
                # For Z retreat, verify we moved in Z direction
                dz_mm = float(step.get("dz_mm", 0))
                if abs(dz_mm) > 5:  # Only verify for significant Z movements
                    current_pos = self.runner.get_current_position()
                    if current_pos is None:
                        return False
                    
                    # Simple verification - robot should be at reasonable height
                    if current_pos[2] < 50 or current_pos[2] > 800:
                        print(f"[VERIFY] Step {step_num} verification failed - Z position {current_pos[2]:.1f}mm outside reasonable range")
                        return False
                    
                    print(f"[VERIFY] Step {step_num} verified - Z position: {current_pos[2]:.1f}mm")
                    return True
            
            # For APPROACH_OBJECT and MOVE_TO_OBJECT, verification is handled in _execute_movement_with_verification
            elif action in ("APPROACH_OBJECT", "MOVE_TO_OBJECT"):
                return True  # Already verified in movement method
            
            return True  # Default to success for other actions
            
        except Exception as e:
            print(f"[VERIFY] Step {step_num} verification error: {e}")
            return False

    def _validate_plan_safety(self, plan: dict):
        """Validate plan for safety concerns"""
        steps = plan.get("steps", [])
        
        # Check plan length
        if len(steps) > MAX_STEPS_PER_PLAN:
            raise ValueError(f"Plan has {len(steps)} steps, maximum allowed is {MAX_STEPS_PER_PLAN}")
        
        # Check for dangerous patterns
        dangerous_actions = []
        for i, step in enumerate(steps):
            action = step.get("action", "")
            
            # Check for large movements
            if action in ("MOVE_TO_POSE", "MOVE_TO_OBJECT", "APPROACH_OBJECT"):
                if "pose" in step:
                    pose = step["pose"]
                    xyz = pose.get("xyz_mm", [0, 0, 0])
                    # Check if movement is reasonable
                    if any(abs(x) > 500 for x in xyz):
                        dangerous_actions.append(f"Step {i+1}: Large movement to {xyz}")
            
            # Check for high-speed movements
            if "speed" in step and step["speed"] > 200:
                dangerous_actions.append(f"Step {i+1}: High speed {step['speed']}")
        
        if dangerous_actions:
            print("[WARN] Potentially dangerous actions detected:")
            for action in dangerous_actions:
                print(f"  - {action}")
            if not self.sim:
                print("[WARN] Consider reviewing plan before execution")

    def _check_execution_timeout(self):
        """Check if execution has exceeded maximum time"""
        if self.execution_start_time and time.time() - self.execution_start_time > MAX_EXECUTION_TIME:
            raise RuntimeError(f"Execution timeout after {MAX_EXECUTION_TIME} seconds")

    def _check_safety_status(self):
        """Periodic safety status check"""
        try:
            safety_status = self.runner.get_safety_status()
            if safety_status["emergency_stop"]:
                raise RuntimeError("Emergency stop is active")
        except Exception as e:
            print(f"[WARN] Safety check failed: {e}")

    def execute(self, plan: dict):
        print(f"[Plan] Goal: {plan.get('goal')}")
        
        # Validate plan safety
        self._validate_plan_safety(plan)
        
        # Check gripper status before starting
        gripper_status = self.runner.get_gripper_status()
        print(f"[Plan] Gripper status: {gripper_status}")
        
        # Initialize execution monitoring
        self.execution_start_time = time.time()
        self.step_count = 0
        self.error_count = 0
        
        # Note: Removed automatic go_home() to allow camera-mounted robot to stay in current position
        
        steps_list = plan.get("steps", [])

        for i, step in enumerate(steps_list, 1):
            self.step_count = i
            
            # Safety checks
            self._check_execution_timeout()
            if i % 5 == 0:  # Check safety every 5 steps
                self._check_safety_status()
            
            act = step.get("action")
            
            # Fix parameter format if AI wrapped parameters in "parameters" object
            if "parameters" in step and isinstance(step["parameters"], dict):
                # Flatten nested parameters to top level
                for key, value in step["parameters"].items():
                    if key not in step:  # Don't overwrite existing top-level keys
                        step[key] = value
                print(f"[Plan] Fixed nested parameters for step {i}")
            
            print(f"[Plan] Step {i}/{len(steps_list)}: {act} {step}")

            try:
                if act == "MOVE_TO_NAMED":
                    # If first step is 'home' but a target object is already detected
                    # (and appears later in the plan), skip going home to begin approach immediately.
                    if i == 1:
                        # Look ahead for first APPROACH_OBJECT/MOVE_TO_OBJECT label
                        future_label = None
                        for future in steps_list[i:]:
                            fa = future.get("action")
                            if fa in ("APPROACH_OBJECT", "MOVE_TO_OBJECT"):
                                labels = future.get("labels", [])
                                label = future.get("label")
                                if label and not labels:
                                    labels = [label]
                                if labels:
                                    future_label = labels[0]
                                break
                        if future_label and not self.dry_run:
                            try:
                                # Short, non-blocking check
                                _ = self.obj_index.wait_for(future_label, timeout=0.05)
                                print(f"[Plan] Skipping initial MOVE_TO_NAMED because '{future_label}' is already detected")
                                continue
                            except Exception:
                                pass

                    xyz, rpy = self._named(step["name"])
                    if not self.dry_run:
                        self.runner.move_pose(xyz, rpy)

                elif act == "APPROACH_NAMED":
                    hover = float(step.get("hover_mm", self.hover_mm))
                    xyz, rpy = self._named(step["name"])
                    target_xyz = [xyz[0], xyz[1], xyz[2] + hover]
                    if not self.dry_run:
                        self.runner.move_pose(target_xyz, rpy)

                elif act == "MOVE_TO_POSE":
                    pose = step["pose"]
                    if not self.dry_run:
                        # Check if this is a home movement - allow original orientation
                        pose_name = step.get("pose_name", "").lower()
                        if "home" in pose_name:
                            # Use original pose orientation for home
                            rpy = pose["rpy_deg"]
                            print(f"[MOVE] Moving to home with original orientation: {rpy}")
                        else:
                            # Use constant J5 position for all other movements
                            rpy = self.constant_j5_rpy
                            print(f"[MOVE] Moving to pose with constant J5 position: {rpy}")
                        self.runner.move_pose(pose["xyz_mm"], rpy)

                elif act == "RETREAT_Z":
                    dz_mm = float(step["dz_mm"])
                    # Limit retreat distance for safety
                    if abs(dz_mm) > 150:
                        print(f"[WARN] Large retreat {dz_mm}mm, limiting to 150mm")
                        dz_mm = 150 if dz_mm > 0 else -150
                    if not self.dry_run:
                        self.runner.move_rel_z(dz_mm)

                # Enhanced gripper actions with safety
                elif act == "OPEN_GRIPPER":
                    g = step.get("gripper", {})
                    position = g.get("position")
                    speed = min(g.get("speed", 200), 300)  # Limit speed
                    force = min(g.get("force", 50), 100)   # Limit force
                    if not self.dry_run:
                        success = self.runner.open_gripper(position, speed, force)
                        if not success:
                            self.error_count += 1
                            print(f"[WARN] Step {i} failed: OPEN_GRIPPER")

                elif act == "CLOSE_GRIPPER":
                    g = step.get("gripper", {})
                    position = g.get("position")
                    speed = min(g.get("speed", 100), 200)  # Limit speed
                    force = min(g.get("force", 50), 100)   # Limit force
                    if not self.dry_run:
                        success = self.runner.close_gripper(position, speed, force)
                        if not success:
                            self.error_count += 1
                            print(f"[WARN] Step {i} failed: CLOSE_GRIPPER")

                elif act == "SET_GRIPPER_POSITION":
                    position = step.get("position", 525)
                    speed = min(step.get("speed", 150), 300)  # Limit speed
                    force = min(step.get("force", 50), 100)   # Limit force
                    if not self.dry_run:
                        success = self.runner.set_gripper_position(position, speed, force)
                        if not success:
                            self.error_count += 1
                            print(f"[WARN] Step {i} failed: SET_GRIPPER_POSITION")

                elif act == "GRIPPER_GRASP":
                    target_position = step.get("target_position", 200)
                    speed = min(step.get("speed", 100), 200)  # Limit speed
                    force = min(step.get("force", 50), 100)   # Limit force
                    timeout = min(step.get("timeout", 5.0), 10.0)  # Limit timeout
                    if not self.dry_run:
                        success = self.runner.gripper_grasp(target_position, speed, force, timeout)
                        if not success:
                            self.error_count += 1
                            print(f"[WARN] Step {i} failed: GRIPPER_GRASP")

                elif act == "GRIPPER_RELEASE":
                    target_position = step.get("target_position", 850)
                    speed = min(step.get("speed", 200), 300)  # Limit speed
                    force = min(step.get("force", 50), 100)   # Limit force
                    if not self.dry_run:
                        success = self.runner.gripper_release(target_position, speed, force)
                        if not success:
                            self.error_count += 1
                            print(f"[WARN] Step {i} failed: GRIPPER_RELEASE")

                elif act == "GRIPPER_HALF_OPEN":
                    speed = min(step.get("speed", 150), 300)  # Limit speed
                    force = min(step.get("force", 50), 100)   # Limit force
                    if not self.dry_run:
                        success = self.runner.gripper_half_open(speed, force)
                        if not success:
                            self.error_count += 1
                            print(f"[WARN] Step {i} failed: GRIPPER_HALF_OPEN")

                elif act == "GRIPPER_SOFT_CLOSE":
                    speed = min(step.get("speed", 50), 100)   # Limit speed
                    force = min(step.get("force", 30), 50)    # Limit force
                    if not self.dry_run:
                        success = self.runner.gripper_soft_close(speed, force)
                        if not success:
                            self.error_count += 1
                            print(f"[WARN] Step {i} failed: GRIPPER_SOFT_CLOSE")

                elif act == "GRIPPER_TEST":
                    cycles = min(step.get("cycles", 3), 5)    # Limit cycles
                    delay = min(step.get("delay", 1.0), 2.0)  # Limit delay
                    if not self.dry_run:
                        success = self.runner.gripper_cycle_test(cycles, delay)
                        if not success:
                            self.error_count += 1
                            print(f"[WARN] Step {i} failed: GRIPPER_TEST")

                elif act in ("APPROACH_OBJECT", "MOVE_TO_OBJECT"):
                    # USE CORRECTED APPROACH WITH PROPER COORDINATE TRANSFORMATION
                    labels = step.get("labels", [])
                    label = step.get("label")
                    if label and not labels:
                        labels = [label]
                    
                    target_label = labels[0]
                    
                    if not self.dry_run:
                        # Get current robot TCP position
                        robot_tcp_position = self.runner.get_current_position()
                        if robot_tcp_position is None:
                            print(f"[ERROR] Could not get robot TCP position for {target_label}")
                            continue
                        
                        # Get camera coordinates from object index
                        with self.obj_index._global_lock:
                            camera_coordinates = self.obj_index.latest_mm.get(target_label)
                        
                        if camera_coordinates is None:
                            print(f"[ERROR] Object '{target_label}' not detected")
                            continue
                        
                        print(f"[CORRECTED] Found {target_label} at camera coords: {camera_coordinates}")
                        print(f"[CORRECTED] Current robot TCP: {robot_tcp_position}")
                        
                        # Transform camera coordinates to robot base coordinates
                        robot_base_coords = self.coordinate_transformer.transform_camera_to_robot_base(
                            camera_coordinates, 
                            robot_tcp_position
                        )
                        
                        print(f"[CORRECTED] Transformed to robot base: {robot_base_coords}")
                        
                        # Safety check: Ensure coordinates are within reasonable bounds
                        x, y, z = robot_base_coords[0], robot_base_coords[1], robot_base_coords[2]
                        if (abs(x) > 1000 or abs(y) > 1000 or z < 0 or z > 1000):
                            print(f"[CORRECTED ERROR] Coordinates out of bounds: X={x:.1f}, Y={y:.1f}, Z={z:.1f}")
                            print(f"[CORRECTED ERROR] Skipping movement to prevent damage!")
                            continue
                        
                        # Move robot to transformed coordinates
                        speed = step.get("speed", 300)
                        result = self.runner.arm.set_position(
                            x=robot_base_coords[0], 
                            y=robot_base_coords[1], 
                            z=robot_base_coords[2],
                            roll=0, pitch=90, yaw=0,  # Fixed orientation
                            speed=speed, 
                            wait=True
                        )
                        
                        if result == 0:
                            print(f"[SUCCESS] Corrected approach to {target_label} completed")
                            else:
                            print(f"[ERROR] Corrected approach to {target_label} failed with code {result}")
                                        else:
                        print(f"[DRY RUN] Would approach {target_label} with corrected coordinate transformation")

                elif act == "SCAN_FOR_OBJECTS" or act == "SCAN_AREA":
                    # Hierarchical scanning: horizontal → arc → overhead
                    pattern = step.get("pattern", "hierarchical")  # Default to hierarchical
                    sweep_mm = float(step.get("sweep_mm", 300))
                    steps = int(step.get("steps", 5))
                    pause_sec = float(step.get("pause_sec", 1.0))
                    interrupt_on_detection = step.get("interrupt_on_detection", True)
                    target_objects = step.get("target_objects", [])
                    
                    print(f"[SCAN] Starting hierarchical scan: pattern={pattern}, sweep={sweep_mm}mm, steps={steps}, pause={pause_sec}s")
                    
                    if not self.dry_run:
                        # Perform hierarchical scanning
                        found_objects = self._perform_hierarchical_scan(
                            target_objects=target_objects,
                            sweep_mm=sweep_mm,
                            steps=steps,
                            pause_sec=pause_sec,
                            interrupt_on_detection=interrupt_on_detection
                        )
                        
                        if found_objects:
                            print(f"[SCAN] ✅ Hierarchical scan successful - found objects: {found_objects}")
                        else:
                            print(f"[SCAN] ❌ Hierarchical scan completed - no target objects found")
                    else:
                        # Dry run
                        print(f"[SCAN] DRY RUN: Would perform hierarchical scan:")
                        print(f"[SCAN] DRY RUN:   1. Horizontal sweep: {steps} positions over {sweep_mm}mm")
                        print(f"[SCAN] DRY RUN:   2. Arc sweep: {steps} positions in arc pattern")
                        print(f"[SCAN] DRY RUN:   3. Overhead sweep: {steps} positions from above")

                elif act == "SLEEP":
                    sleep_time = float(step["seconds"])
                    # Limit sleep time for safety
                    if sleep_time > 30:
                        print(f"[WARN] Long sleep {sleep_time}s, limiting to 30s")
                        sleep_time = 30
                    if not self.dry_run:
                        time.sleep(sleep_time)

                else:
                    raise ValueError(f"Unsupported action: {act}")

                # STEP VERIFICATION DISABLED - Allow robot to continue regardless
                
                # Reset error count on successful step
                self.error_count = 0

            except Exception as e:
                self.error_count += 1
                print(f"[ERROR] Step {i} failed: {e}")
                
                # Stop execution if too many consecutive errors
                if self.error_count >= self.max_errors:
                    print(f"[ERROR] Stopping execution after {self.max_errors} consecutive errors")
                    break
                
                if not self.sim:
                    print("[ERROR] Stopping execution due to error")
                    break

        # Final safety check
        execution_time = time.time() - self.execution_start_time if self.execution_start_time else 0
        print(f"[Plan] Done. Execution time: {execution_time:.1f}s, Steps completed: {self.step_count}")

    def get_current_position(self):
        """Get current robot position."""
        try:
            if self.runner:
                return self.runner.get_current_position()
            return None
        except Exception as e:
            print(f"[TaskExecutor] Error getting current position: {e}")
            return None

    def get_execution_stats(self):
        """Get execution statistics"""
        execution_time = time.time() - self.execution_start_time if self.execution_start_time else 0
        return {
            "execution_time": execution_time,
            "steps_completed": self.step_count,
            "error_count": self.error_count,
            "safety_status": self.runner.get_safety_status()
        }

    def shutdown(self):
        try:
            if ROS2_AVAILABLE:
                rclpy.shutdown()
        except Exception:
            pass
        
        # Stop vision system if running
        if self.vision_process:
            print("[Vision] Stopping vision system...")
            try:
                self.vision_process.terminate()
                # Use a shorter timeout and force kill if needed
                try:
                    self.vision_process.wait(timeout=2)
                    print("[Vision] Vision system stopped gracefully")
                except:
                    print("[Vision] Force killing vision system...")
                    self.vision_process.kill()
                    self.vision_process.wait(timeout=1)
                    print("[Vision] Vision system force stopped")
            except Exception as e:
                print(f"[Vision] Error stopping vision system: {e}")
        
        self.runner.disconnect()
