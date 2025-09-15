import time, threading
import json
import logging
import subprocess
import sys
import os
import numpy as np
from typing import List, Dict, Any, Optional

ROS2_AVAILABLE = False
Node = object
StringMsg = str

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
    
    def __init__(self, camera_offset_mm: List[float] = [0, 0, -150]):
        self.camera_offset = np.array(camera_offset_mm)
        self.camera_to_tcp_matrix = np.array([
            [0,  0,  1],
            [1,  0,  0],
            [0,  1,  0]
        ])

    def transform_camera_to_robot_base_with_rpy(self, 
                                          camera_coords: List[float], 
                                          robot_tcp_position: List[float],
                                          tcp_rpy_deg: List[float]) -> List[float]:
        """Transform using rotation matrices with proper RPY transformation."""
        tcp_pos = np.array(robot_tcp_position)
        cam_coords = np.array(camera_coords)
        
        print(f"[TRANSFORM RPY] ========== COORDINATE TRANSFORMATION DEBUG ==========")
        print(f"[TRANSFORM RPY] Input camera coords: {camera_coords}")
        print(f"[TRANSFORM RPY] Input robot TCP: {robot_tcp_position}")
        print(f"[TRANSFORM RPY] Input TCP RPY: {tcp_rpy_deg}")
        print(f"[TRANSFORM RPY] Camera offset: {self.camera_offset}")
        
        roll_rad = np.radians(tcp_rpy_deg[0])
        pitch_rad = np.radians(tcp_rpy_deg[1]) 
        yaw_rad = np.radians(tcp_rpy_deg[2])
        
        print(f"[TRANSFORM RPY] RPY in radians: [{roll_rad:.4f}, {pitch_rad:.4f}, {yaw_rad:.4f}]")
        
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(roll_rad), -np.sin(roll_rad)],
            [0, np.sin(roll_rad), np.cos(roll_rad)]
        ])
        
        R_y = np.array([
            [np.cos(pitch_rad), 0, np.sin(pitch_rad)],
            [0, 1, 0],
            [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]
        ])
        
        R_z = np.array([
            [np.cos(yaw_rad), -np.sin(yaw_rad), 0],
            [np.sin(yaw_rad), np.cos(yaw_rad), 0],
            [0, 0, 1]
        ])
        
        print(f"[TRANSFORM RPY] R_x (Roll) matrix:\n{R_x}")
        print(f"[TRANSFORM RPY] R_y (Pitch) matrix:\n{R_y}")
        print(f"[TRANSFORM RPY] R_z (Yaw) matrix:\n{R_z}")
        
        R_cw = R_z @ R_y @ R_x
        print(f"[TRANSFORM RPY] Combined rotation matrix R_cw:\n{R_cw}")
        
        rotated_cam_coords = R_cw @ cam_coords
        print(f"[TRANSFORM RPY] Camera coords after rotation: {rotated_cam_coords}")
        
        robot_base_coords = tcp_pos + rotated_cam_coords
        print(f"[TRANSFORM RPY] Final robot base coords: {robot_base_coords}")
        
        distance_from_tcp = np.linalg.norm(rotated_cam_coords)
        print(f"[TRANSFORM RPY] Distance from TCP to target: {distance_from_tcp:.2f}mm")
        print(f"[TRANSFORM RPY] ================================================")
        
        return robot_base_coords.tolist()

DEFAULT_HOVER_MM = 80
DEFAULT_PICK_RPY = [0, 90, 0]
CAMERA_TO_TCP_OFFSET_MM = [0, 0, -150]

MAX_EXECUTION_TIME = 300
MAX_STEPS_PER_PLAN = 50
SAFETY_CHECK_INTERVAL = 0.5

WORLD_POSES = {
    "home":             {"xyz_mm": [300,   0, 300], "rpy_deg": [0, 90, 0]},
    "bin_drop":         {"xyz_mm": [250, 250, 120], "rpy_deg": [0, 90, 0]},
    "table_pick_blue":  {"xyz_mm": [400,-150,  45], "rpy_deg": [0, 90, 0]},
    "staging_area":     {"xyz_mm": [450, -150, 150], "rpy_deg": [0, 90, 0]},
    "scan_center":      {"xyz_mm": [400,   0, 250], "rpy_deg": [0, 90, 0]},
}

class ObjectIndex(Node if ROS2_AVAILABLE else object):
    """Caches latest pose per label from /detected_objects."""
    _global_latest_mm = {}
    _global_lock = threading.Lock()
    _movement_in_progress = False
    
    def __init__(self):
        self._lock = threading.Lock()
        self.latest_mm = ObjectIndex._global_latest_mm
        
        if ROS2_AVAILABLE:
            super().__init__('object_index')
            self.sub = self.create_subscription(StringMsg, '/detected_objects', self._on_msg, 10)
            print("[ObjectIndex] ROS2 subscription created for /detected_objects")
        else:
            self.sub = None
            print("[ObjectIndex] Running without ROS2")

    def _normalize_object_name(self, class_name: str) -> str:
        """Normalize object names to match robot expectations."""
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
    
    def _on_msg(self, msg):
        try:
            if self._movement_in_progress:
                return
                
            if hasattr(msg, 'data'):
                data = json.loads(msg.data)
            else:
                data = msg
            units = data.get("units", "m")
            k = 1000.0 if units == "m" else 1.0
            items = data.get("items", [])
            
            with self._global_lock:
                for it in items:
                    lab = it.get("class")
                    pos = it.get("pos", [0,0,0])
                    if lab and isinstance(pos, list) and len(pos) == 3:
                        normalized_lab = self._normalize_object_name(lab)
                        self.latest_mm[normalized_lab] = [float(pos[0])*k, float(pos[1])*k, float(pos[2])*k]
                        print(f"[ObjectIndex] Updated {normalized_lab}: {self.latest_mm[normalized_lab]}")
        except Exception as e:
            print(f"[ObjectIndex] Error processing message: {e}")

    def wait_for(self, label: str, timeout=5.0):
        t0 = time.time()
        print(f"[ObjectIndex] Waiting for '{label}' for {timeout}s...")
        
        with self._global_lock:
            p = self.latest_mm.get(label)
            if p is not None:
                print(f"[ObjectIndex] Found '{label}' in cache: {p}")
                return p
        
        while time.time() - t0 < timeout:
            with self._global_lock:
                p = self.latest_mm.get(label)
            if p is not None:
                print(f"[ObjectIndex] Found '{label}': {p}")
                return p
            time.sleep(0.05)
        
        print(f"[ObjectIndex] Timeout for '{label}'. Available: {list(self.latest_mm.keys())}")
        raise TimeoutError(f"Object '{label}' not seen within {timeout}s")

class TaskExecutor:
    def __init__(self, arm_ip: str, world_yaml: str = None, sim: bool = False, dry_run: bool = False, runner: XArmRunner = None):
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
        
        self.coordinate_transformer = CameraToRobotTransformer(CAMERA_TO_TCP_OFFSET_MM)
        self.constant_j5_rpy = [0, 90, 0]
        
        self.execution_start_time = None
        self.step_count = 0
        self.error_count = 0
        self.max_errors = 3
        
        self.detection_state = {
            'last_detection_rpy': None,
            'last_target_coords': None,
            'last_robot_coords': None
        }

        self.vision_process = None
        self._start_vision_system()

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

        if ROS2_AVAILABLE:
            rclpy.init(args=None)
            self.obj_index = ObjectIndex()
            self._spin_thread = threading.Thread(target=rclpy.spin, args=(self.obj_index,), daemon=True)
            self._spin_thread.start()
            
            if self.vision_process:
                print("[Vision] Waiting for vision system to initialize...")
                time.sleep(3)
            
            print("[Vision] Vision system ready")
        else:
            self.obj_index = ObjectIndex()
            self._spin_thread = None

    def _start_vision_system(self):
        """Start the vision system for object detection."""
        try:
            print("[Vision] Vision system handled by main.py (apriltag_bridge.py)")
            self.vision_recorder = None
            self.vision_process = None
        except Exception as e:
            print(f"[Vision] Error starting vision system: {e}")
            self.vision_process = None
            self.vision_recorder = None

    def _named(self, name: str):
        if name not in self.world:
            raise KeyError(f"Named pose '{name}' not in WORLD_POSES.")
        p = self.world[name]
        return p["xyz_mm"], p["rpy_deg"]
    
    def _perform_arc_scan(self, target_objects: List[str] = [], interrupt_on_detection: bool = True) -> List[str]:
        """Perform arc pattern scan using hardcoded joint positions."""
        print(f"[ARC_SCAN] ========== STARTING ARC SCAN DEBUG ==========")
        print(f"[ARC_SCAN] Target objects: {target_objects if target_objects else 'any object'}")
        print(f"[ARC_SCAN] Interrupt on detection: {interrupt_on_detection}")
        
        left_arc_joints = [46, -57, -74, -74, -90.4, -18.4]
        center_joints = [-0.7, 3.3, 1.1, -180.5, -90.2, -2]
        right_arc_joints = [-53, -57, -64, -281, -87, 0]
        
        scan_positions = [
            ("LEFT", left_arc_joints),
            ("CENTER", center_joints), 
            ("RIGHT", right_arc_joints)
        ]
        
        print(f"[ARC_SCAN] Scan sequence: {[pos[0] for pos in scan_positions]}")
        
        try:
            for position_name, joint_angles in scan_positions:
                print(f"[ARC_SCAN] ========== {position_name} POSITION ==========")
                print(f"[ARC_SCAN] Target joint angles: {joint_angles}")
                
                if not self.dry_run:
                    current_joints = self.runner.arm.get_servo_angle()
                    if current_joints[0] == 0:
                        print(f"[ARC_SCAN] Current joint angles: {current_joints[1]}")
                    else:
                        print(f"[ARC_SCAN] Warning: Could not get current joints, error: {current_joints[0]}")
                    
                    print(f"[ARC_SCAN] Moving to {position_name} position...")
                    result = self.runner.arm.set_servo_angle(angle=joint_angles, speed=25, wait=True)
                    
                    if result != 0:
                        print(f"[ARC_SCAN] ERROR: Failed to move to {position_name}, error code: {result}")
                        continue
                    
                    time.sleep(0.5)
                    
                    final_joints = self.runner.arm.get_servo_angle()
                    if final_joints[0] == 0:
                        print(f"[ARC_SCAN] Final joint angles: {final_joints[1]}")
                        joint_diff = [abs(final_joints[1][i] - joint_angles[i]) for i in range(6)]
                        print(f"[ARC_SCAN] Joint angle differences: {joint_diff}")
                    
                    tcp_position, tcp_rpy = self.runner.get_current_position_and_rpy()
                    if tcp_position and tcp_rpy:
                        print(f"[ARC_SCAN] TCP Position: {tcp_position}")
                        print(f"[ARC_SCAN] TCP RPY: {tcp_rpy}")
                    else:
                        print(f"[ARC_SCAN] Warning: Could not get TCP position/RPY")
                    
                    print(f"[ARC_SCAN] Scanning for objects at {position_name} position...")
                    
                    found_targets = self._check_for_objects_during_pause(
                        2.0, target_objects, interrupt_on_detection
                    )
                    
                    if found_targets and interrupt_on_detection:
                        print(f"[ARC_SCAN] ========== TARGETS DETECTED ==========")
                        print(f"[ARC_SCAN] Found targets: {found_targets} at {position_name} position")
                        print(f"[ARC_SCAN] IMMEDIATELY CALLING APPROACH FUNCTION")
                        
                        # CRITICAL FIX: Store detection state BEFORE calling approach
                        tcp_position, tcp_rpy = self.runner.get_current_position_and_rpy()
                        if tcp_position and tcp_rpy:
                            self.detection_state['last_detection_rpy'] = tcp_rpy.copy()
                            print(f"[ARC_SCAN] Stored detection RPY: {tcp_rpy}")
                            print(f"[ARC_SCAN] Stored detection TCP: {tcp_position}")
                        else:
                            print(f"[ARC_SCAN] ERROR: Could not store detection state")
                            continue
                        
                        # CRITICAL FIX: Call approach and check result properly
                        print(f"[ARC_SCAN] About to call approach function for {found_targets[0]}")
                        approach_success = self._approach_detected_object_from_arc(found_targets[0])
                        print(f"[ARC_SCAN] Approach function returned: {approach_success}")
                        
                        if approach_success:
                            print(f"[ARC_SCAN] SUCCESS: Approached {found_targets[0]} from {position_name}")
                            print(f"[ARC_SCAN] RETURNING FOUND TARGETS - NO MORE MOVEMENT")
                            return found_targets
                        else:
                            print(f"[ARC_SCAN] WARNING: Approach failed, but continuing with found targets")
                            print(f"[ARC_SCAN] RETURNING FOUND TARGETS ANYWAY")
                            return found_targets
                else:
                    print(f"[ARC_SCAN] DRY RUN: Would move to {position_name} position")
            
            print(f"[ARC_SCAN] ========== SCAN COMPLETE ==========")
            print(f"[ARC_SCAN] No targets found, returning to center position")
            if not self.dry_run:
                self.runner.arm.set_servo_angle(angle=center_joints, speed=30, wait=True)
            
            return []
            
        except Exception as e:
            print(f"[ARC_SCAN] ERROR: Exception during arc scan: {e}")
            try:
                if not self.dry_run:
                    print(f"[ARC_SCAN] Emergency return to center position")
                    self.runner.arm.set_servo_angle(angle=center_joints, speed=20, wait=True)
            except Exception as recovery_error:
                print(f"[ARC_SCAN] ERROR: Failed to recover: {recovery_error}")
            return []
    
    def _check_for_objects_during_pause(self, pause_sec: float, target_objects: List[str], interrupt_on_detection: bool) -> List[str]:
        """Check for objects during pause period."""
        print(f"[OBJECT_CHECK] ========== OBJECT DETECTION DEBUG ==========")
        print(f"[OBJECT_CHECK] Pause duration: {pause_sec}s")
        print(f"[OBJECT_CHECK] Target objects: {target_objects if target_objects else 'any'}")
        print(f"[OBJECT_CHECK] Interrupt on detection: {interrupt_on_detection}")
        
        if not interrupt_on_detection:
            print(f"[OBJECT_CHECK] No interruption mode - sleeping {pause_sec}s")
            time.sleep(pause_sec)
            return []
        
        detection_interval = 0.2
        elapsed = 0.0
        check_count = 0
        
        while elapsed < pause_sec:
            time.sleep(detection_interval)
            elapsed += detection_interval
            check_count += 1
            
            with self.obj_index._global_lock:
                detected_objects = list(self.obj_index.latest_mm.keys())
                object_positions = self.obj_index.latest_mm.copy()
            
            print(f"[OBJECT_CHECK] Check #{check_count} at {elapsed:.1f}s:")
            print(f"[OBJECT_CHECK]   Detected objects: {detected_objects}")
            
            if object_positions:
                for obj_name, obj_pos in object_positions.items():
                    print(f"[OBJECT_CHECK]   {obj_name}: [{obj_pos[0]:.1f}, {obj_pos[1]:.1f}, {obj_pos[2]:.1f}]mm")
            
            found_targets = []
            if target_objects:
                found_targets = [obj for obj in target_objects if obj in detected_objects]
                if found_targets:
                    print(f"[OBJECT_CHECK] SPECIFIC TARGETS FOUND: {found_targets}")
            else:
                found_targets = detected_objects
                if found_targets:
                    print(f"[OBJECT_CHECK] ANY OBJECTS FOUND: {found_targets}")
            
            if found_targets:
                print(f"[OBJECT_CHECK] SUCCESS: Interrupting scan - targets detected")
                print(f"[OBJECT_CHECK] ==========================================")
                return found_targets
        
        print(f"[OBJECT_CHECK] Scan timeout - no targets found after {check_count} checks")
        print(f"[OBJECT_CHECK] ==========================================")
        return []
    
    def _approach_detected_object_from_arc(self, target_label: str) -> bool:
        """Approach object using dynamic detection orientation - FIXED VERSION."""
        try:
            print(f"[APPROACH] ========== APPROACH FUNCTION START ==========")
            print(f"[APPROACH] Target label: {target_label}")
            print(f"[APPROACH] This is the ONLY approach function being called")
            
            # Get current robot TCP position AND RPY values
            tcp_position, tcp_rpy = self.runner.get_current_position_and_rpy()
            if tcp_position is None or tcp_rpy is None:
                print(f"[APPROACH] ERROR: Could not get current position/orientation")
                print(f"[APPROACH] TCP Position: {tcp_position}")
                print(f"[APPROACH] TCP RPY: {tcp_rpy}")
                return False
            
            print(f"[APPROACH] Current TCP position: {tcp_position}")
            print(f"[APPROACH] Current TCP RPY: {tcp_rpy}")
            
            # Get object coordinates from vision system
            camera_coordinates = self.obj_index.wait_for(target_label, timeout=2.0)
            if camera_coordinates is None:
                print(f"[APPROACH] ERROR: Object '{target_label}' not detected")
                with self.obj_index._global_lock:
                    available_objects = list(self.obj_index.latest_mm.keys())
                print(f"[APPROACH] Available objects: {available_objects}")
                return False
            
            print(f"[APPROACH] Camera coordinates: {camera_coordinates}")
            print(f"[APPROACH] Camera frame: X={camera_coordinates[0]:.2f}mm (left/right)")
            print(f"[APPROACH] Camera frame: Y={camera_coordinates[1]:.2f}mm (up/down)")
            print(f"[APPROACH] Camera frame: Z={camera_coordinates[2]:.2f}mm (forward/back)")
            
            # Transform camera coordinates to robot base coordinates using RPY transformation
            print(f"[APPROACH] Calling coordinate transformation...")
            robot_base_coords = self.coordinate_transformer.transform_camera_to_robot_base_with_rpy(
                camera_coordinates, tcp_position, tcp_rpy
            )
            
            print(f"[APPROACH] Transformed robot base coords: {robot_base_coords}")
            print(f"[APPROACH] Robot frame: X={robot_base_coords[0]:.2f}mm (forward/back)")
            print(f"[APPROACH] Robot frame: Y={robot_base_coords[1]:.2f}mm (left/right)")
            print(f"[APPROACH] Robot frame: Z={robot_base_coords[2]:.2f}mm (up/down)")
            
            # Store detection state for later use
            self.detection_state['last_target_coords'] = camera_coordinates.copy()
            self.detection_state['last_robot_coords'] = robot_base_coords.copy()
            self.detection_state['last_detection_rpy'] = tcp_rpy.copy()
            
            print(f"[APPROACH] Stored detection state:")
            print(f"[APPROACH]   Camera coords: {self.detection_state['last_target_coords']}")
            print(f"[APPROACH]   Robot coords: {self.detection_state['last_robot_coords']}")
            print(f"[APPROACH]   Detection RPY: {self.detection_state['last_detection_rpy']}")
            
            # Safety check coordinates
            x, y, z = robot_base_coords[0], robot_base_coords[1], robot_base_coords[2]
            if abs(x) > 1000 or abs(y) > 1000 or z < 0 or z > 1000:
                print(f"[APPROACH] ERROR: Coordinates out of bounds")
                print(f"[APPROACH] X: {x:.1f}mm (limit: ±1000mm)")
                print(f"[APPROACH] Y: {y:.1f}mm (limit: ±1000mm)")
                print(f"[APPROACH] Z: {z:.1f}mm (limit: 0-1000mm)")
                return False
            
            # Calculate approach position with offsets
            hover_height = 80.0
            y_offset = -40.0
            x_offset = -20.0
            
            approach_coords = [
                robot_base_coords[0] + x_offset,
                robot_base_coords[1] + y_offset, 
                robot_base_coords[2] + hover_height
            ]
            
            print(f"[APPROACH] Approach calculation:")
            print(f"[APPROACH]   Base target: {robot_base_coords}")
            print(f"[APPROACH]   X offset: {x_offset}mm")
            print(f"[APPROACH]   Y offset: {y_offset}mm")
            print(f"[APPROACH]   Z hover: +{hover_height}mm")
            print(f"[APPROACH]   Final approach coords: {approach_coords}")
            
            print(f"[APPROACH] Movement parameters:")
            print(f"[APPROACH]   Target position: [{approach_coords[0]:.2f}, {approach_coords[1]:.2f}, {approach_coords[2]:.2f}]")
            print(f"[APPROACH]   Target orientation: [{tcp_rpy[0]:.2f}, {tcp_rpy[1]:.2f}, {tcp_rpy[2]:.2f}]")
            print(f"[APPROACH]   Speed: 150mm/s")
            
            print(f"[APPROACH] EXECUTING ROBOT MOVEMENT NOW...")
            print(f"[APPROACH] Calling self.runner.arm.set_position()")
            
            # CRITICAL FIX: Use the runner's arm directly to avoid any wrapper issues
            result = self.runner.arm.set_position(
                x=approach_coords[0], 
                y=approach_coords[1], 
                z=approach_coords[2],
                roll=tcp_rpy[0], 
                pitch=tcp_rpy[1], 
                yaw=tcp_rpy[2],
                speed=150,
                wait=True
            )
            
            print(f"[APPROACH] Robot movement result: {result}")
            
            if result == 0:
                # Verify the movement worked
                final_tcp = self.runner.get_current_position()
                if final_tcp:
                    movement_error = [
                        abs(final_tcp[0] - approach_coords[0]),
                        abs(final_tcp[1] - approach_coords[1]),
                        abs(final_tcp[2] - approach_coords[2])
                    ]
                    total_error = sum(movement_error)
                    print(f"[APPROACH] Final TCP position: {final_tcp}")
                    print(f"[APPROACH] Movement error: {movement_error} (total: {total_error:.2f}mm)")
                
                print(f"[APPROACH] SUCCESS: Approached {target_label}")
                print(f"[APPROACH] =======================================")
                return True
            else:
                print(f"[APPROACH] ERROR: Movement failed with code {result}")
                print(f"[APPROACH] Robot did not move to approach coordinates")
                print(f"[APPROACH] =======================================")
                return False
                
        except Exception as e:
            print(f"[APPROACH] ERROR: Exception during approach: {e}")
            print(f"[APPROACH] =======================================")
            return False

    def _is_coordinate_safe(self, coordinates: list) -> bool:
        """Validate coordinates are within safe workspace."""
        if len(coordinates) < 3:
            return False
        
        x, y, z = coordinates[0], coordinates[1], coordinates[2]
        
        safe_limits = {
            'x_min': 200, 'x_max': 850,
            'y_min': -500, 'y_max': 500,
            'z_min': 50, 'z_max': 600
        }
        
        if not (safe_limits['x_min'] <= x <= safe_limits['x_max']):
            print(f"[SAFETY] X coordinate {x:.1f}mm outside safe range")
            return False
        if not (safe_limits['y_min'] <= y <= safe_limits['y_max']):
            print(f"[SAFETY] Y coordinate {y:.1f}mm outside safe range")
            return False
        if not (safe_limits['z_min'] <= z <= safe_limits['z_max']):
            print(f"[SAFETY] Z coordinate {z:.1f}mm outside safe range")
            return False
        
        print(f"[SAFETY] Coordinates safe: {coordinates}")
        return True

    def _validate_plan_safety(self, plan: dict):
        """Validate plan for safety concerns"""
        steps = plan.get("steps", [])
        
        if len(steps) > MAX_STEPS_PER_PLAN:
            raise ValueError(f"Plan has {len(steps)} steps, maximum allowed is {MAX_STEPS_PER_PLAN}")
        
        dangerous_actions = []
        for i, step in enumerate(steps):
            action = step.get("action", "")
            
            if action in ("MOVE_TO_POSE", "MOVE_TO_OBJECT", "APPROACH_OBJECT"):
                if "pose" in step:
                    pose = step["pose"]
                    xyz = pose.get("xyz_mm", [0, 0, 0])
                    if any(abs(x) > 500 for x in xyz):
                        dangerous_actions.append(f"Step {i+1}: Large movement to {xyz}")
            
            if "speed" in step and step["speed"] > 200:
                dangerous_actions.append(f"Step {i+1}: High speed {step['speed']}")
        
        if dangerous_actions:
            print("[WARN] Potentially dangerous actions detected:")
            for action in dangerous_actions:
                print(f"  - {action}")

    def execute(self, plan: dict):
        print(f"[Plan] Goal: {plan.get('goal')}")
        
        self._validate_plan_safety(plan)
        
        gripper_status = self.runner.get_gripper_status()
        print(f"[Plan] Gripper status: {gripper_status}")
        
        self.execution_start_time = time.time()
        self.step_count = 0
        self.error_count = 0
        
        steps_list = plan.get("steps", [])

        for i, step in enumerate(steps_list, 1):
            self.step_count = i
            
            act = step.get("action")
            
            if "parameters" in step and isinstance(step["parameters"], dict):
                for key, value in step["parameters"].items():
                    if key not in step:
                        step[key] = value
            
            print(f"[Plan] Step {i}/{len(steps_list)}: {act} {step}")

            try:
                if act == "MOVE_TO_NAMED":
                    if i == 1:
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
                                _ = self.obj_index.wait_for(future_label, timeout=0.05)
                                print(f"[Plan] Skipping initial MOVE_TO_NAMED - '{future_label}' already detected")
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
                        pose_name = step.get("pose_name", "").lower()
                        if "home" in pose_name:
                            rpy = pose["rpy_deg"]
                        else:
                            rpy = self.constant_j5_rpy
                        self.runner.move_pose(pose["xyz_mm"], rpy)

                elif act == "RETREAT_Z":
                    dz_mm = float(step["dz_mm"])
                    if abs(dz_mm) > 150:
                        dz_mm = 150 if dz_mm > 0 else -150
                    if not self.dry_run:
                        self.runner.move_rel_z(dz_mm)

                elif act == "OPEN_GRIPPER":
                    g = step.get("gripper", {})
                    position = g.get("position")
                    speed = min(g.get("speed", 200), 300)
                    force = min(g.get("force", 50), 100)
                    if not self.dry_run:
                        success = self.runner.open_gripper(position, speed, force)
                        if not success:
                            self.error_count += 1

                elif act == "CLOSE_GRIPPER":
                    g = step.get("gripper", {})
                    position = g.get("position")
                    speed = min(g.get("speed", 100), 200)
                    force = min(g.get("force", 50), 100)
                    if not self.dry_run:
                        success = self.runner.close_gripper(position, speed, force)
                        if not success:
                            self.error_count += 1

                elif act == "SET_GRIPPER_POSITION":
                    position = step.get("position", 525)
                    speed = min(step.get("speed", 150), 300)
                    force = min(step.get("force", 50), 100)
                    if not self.dry_run:
                        success = self.runner.set_gripper_position(position, speed, force)
                        if not success:
                            self.error_count += 1

                elif act == "GRIPPER_GRASP":
                    target_position = step.get("target_position", 200)
                    speed = min(step.get("speed", 100), 200)
                    force = min(step.get("force", 50), 100)
                    timeout = min(step.get("timeout", 5.0), 10.0)
                    if not self.dry_run:
                        success = self.runner.gripper_grasp(target_position, speed, force, timeout)
                        if not success:
                            self.error_count += 1

                elif act == "GRIPPER_RELEASE":
                    target_position = step.get("target_position", 850)
                    speed = min(step.get("speed", 200), 300)
                    force = min(step.get("force", 50), 100)
                    if not self.dry_run:
                        success = self.runner.gripper_release(target_position, speed, force)
                        if not success:
                            self.error_count += 1

                elif act == "GRIPPER_HALF_OPEN":
                    speed = min(step.get("speed", 150), 300)
                    force = min(step.get("force", 50), 100)
                    if not self.dry_run:
                        success = self.runner.gripper_half_open(speed, force)
                        if not success:
                            self.error_count += 1

                elif act == "GRIPPER_SOFT_CLOSE":
                    speed = min(step.get("speed", 50), 100)
                    force = min(step.get("force", 30), 50)
                    if not self.dry_run:
                        success = self.runner.gripper_soft_close(speed, force)
                        if not success:
                            self.error_count += 1

                elif act == "GRIPPER_TEST":
                    cycles = min(step.get("cycles", 3), 5)
                    delay = min(step.get("delay", 1.0), 2.0)
                    if not self.dry_run:
                        success = self.runner.gripper_cycle_test(cycles, delay)
                        if not success:
                            self.error_count += 1

                elif act in ("APPROACH_OBJECT", "MOVE_TO_OBJECT"):
                    labels = step.get("labels", [])
                    label = step.get("label")
                    if label and not labels:
                        labels = [label]
                    
                    target_label = labels[0]
                    approach_distance = step.get("approach_distance_mm", 30)
                    
                    print(f"[PLAN_APPROACH] ========== PLAN APPROACH DEBUG ==========")
                    print(f"[PLAN_APPROACH] Target label: {target_label}")
                    print(f"[PLAN_APPROACH] Approach distance: {approach_distance}mm")
                    print(f"[PLAN_APPROACH] This is a PLAN-LEVEL approach, different from scan approach")
                    
                    if not self.dry_run:
                        if self.detection_state['last_detection_rpy'] and self.detection_state['last_robot_coords']:
                            print(f"[PLAN_APPROACH] Using stored detection state from scan")
                            robot_base_coords = self.detection_state['last_robot_coords'].copy()
                            detection_rpy = self.detection_state['last_detection_rpy'].copy()
                            
                            print(f"[PLAN_APPROACH] Stored robot coords: {robot_base_coords}")
                            print(f"[PLAN_APPROACH] Stored detection RPY: {detection_rpy}")
                            
                            approach_coords = [
                                robot_base_coords[0],
                                robot_base_coords[1] - 40, 
                                robot_base_coords[2] + approach_distance
                            ]
                            
                            print(f"[PLAN_APPROACH] Using stored detection RPY: {detection_rpy}")
                        else:
                            print(f"[PLAN_APPROACH] No stored detection state, calculating fresh")
                            tcp_position, tcp_rpy = self.runner.get_current_position_and_rpy()
                            if tcp_position is None or tcp_rpy is None:
                                print(f"[PLAN_APPROACH] ERROR: Could not get current position/RPY")
                                continue
                            
                            with self.obj_index._global_lock:
                                target_coords = self.obj_index.latest_mm.get(target_label)
                            
                            if not target_coords:
                                print(f"[PLAN_APPROACH] ERROR: Could not get coordinates for {target_label}")
                                continue
                                
                            robot_base_coords = self.coordinate_transformer.transform_camera_to_robot_base_with_rpy(
                                target_coords, tcp_position, tcp_rpy
                            )
                            
                            approach_coords = [
                                robot_base_coords[0],
                                robot_base_coords[1] - 40, 
                                robot_base_coords[2] + approach_distance
                            ]
                            detection_rpy = tcp_rpy
                        
                        print(f"[PLAN_APPROACH] Final approach coords: {approach_coords}")
                        print(f"[PLAN_APPROACH] Using RPY: {detection_rpy}")
                        
                        if self._is_coordinate_safe(approach_coords):
                            try:
                                print(f"[PLAN_APPROACH] Executing robot movement...")
                                result = self.runner.arm.set_position(
                                    x=approach_coords[0],
                                    y=approach_coords[1], 
                                    z=approach_coords[2],
                                    roll=detection_rpy[0], 
                                    pitch=detection_rpy[1], 
                                    yaw=detection_rpy[2],
                                    speed=step.get("speed", 100),
                                    wait=True
                                )
                                
                                print(f"[PLAN_APPROACH] Movement result: {result}")
                                
                                if result == 0:
                                    final_pos = self.runner.get_current_position()
                                    if final_pos:
                                        error = [abs(final_pos[i] - approach_coords[i]) for i in range(3)]
                                        print(f"[PLAN_APPROACH] Final position: {final_pos}")
                                        print(f"[PLAN_APPROACH] Position error: {error}")
                                    print(f"[PLAN_APPROACH] SUCCESS: Approached {target_label}")
                                else:
                                    print(f"[PLAN_APPROACH] ERROR: Movement failed with code {result}")
                                    
                            except Exception as e:
                                print(f"[PLAN_APPROACH] ERROR: Exception during movement: {e}")
                        else:
                            print(f"[PLAN_APPROACH] ERROR: Coordinates failed safety check")
                    
                    print(f"[PLAN_APPROACH] ==========================================")

                elif act == "SCAN_FOR_OBJECTS" or act == "SCAN_AREA":
                    target_objects = step.get("target_objects", [])
                    interrupt_on_detection = step.get("interrupt_on_detection", True)
                    
                    print(f"[SCAN] Starting arc scan for objects: {target_objects if target_objects else 'any'}")
                    
                    if not self.dry_run:
                        found_objects = self._perform_arc_scan(
                            target_objects=target_objects,
                            interrupt_on_detection=interrupt_on_detection
                        )
                        
                        if found_objects:
                            print(f"[SCAN] Arc scan successful - found: {found_objects}")
                        else:
                            print(f"[SCAN] Arc scan completed - no target objects found")
                    else:
                        print(f"[SCAN] DRY RUN: Would perform arc scan")

                elif act == "SLEEP":
                    sleep_time = float(step["seconds"])
                    if sleep_time > 30:
                        sleep_time = 30
                    if not self.dry_run:
                        time.sleep(sleep_time)

                else:
                    raise ValueError(f"Unsupported action: {act}")

                self.error_count = 0

            except Exception as e:
                self.error_count += 1
                print(f"[ERROR] Step {i} failed: {e}")
                
                if self.error_count >= self.max_errors:
                    print(f"[ERROR] Stopping after {self.max_errors} consecutive errors")
                    break
                
                if not self.sim:
                    print("[ERROR] Stopping execution due to error")
                    break

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
        
        if self.vision_process:
            print("[Vision] Stopping vision system...")
            try:
                self.vision_process.terminate()
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