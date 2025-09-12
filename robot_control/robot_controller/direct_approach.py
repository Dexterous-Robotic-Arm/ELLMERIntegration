#!/usr/bin/env python3
"""
Direct Robot Approach with Proper Coordinate Transformation
===========================================================

Corrected approach function that properly transforms camera coordinates to robot base coordinates.
Uses mathematical transformation matrices for accurate coordinate conversion.

Usage:
    from robot_control.robot_controller.direct_approach import corrected_direct_approach
    corrected_direct_approach(robot_arm, camera_coords, robot_tcp_pos)
"""

import numpy as np
from typing import List, Optional

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
        
        # Step 1: Transform camera coordinates to TCP-relative coordinates
        tcp_relative = self.camera_to_tcp_matrix @ cam_coords
        
        # Step 2: Subtract camera offset from TCP (we want TCP to move to object position)
        # Camera offset represents where camera is relative to TCP, so we subtract it
        tcp_relative -= self.camera_offset
        
        # Step 3: Add current TCP position to get robot base coordinates
        robot_base_coords = tcp_pos + tcp_relative
        
        return robot_base_coords.tolist()

def corrected_direct_approach(robot_arm, camera_coordinates, robot_tcp_position, speed=300, camera_offset_mm=[0, 0, 22.5]):
    """
    Move robot to camera-detected coordinates with proper transformation.
    
    Args:
        robot_arm: XArmAPI instance (already connected)
        camera_coordinates: [x, y, z] in camera frame (mm)
        robot_tcp_position: Current robot TCP position [x, y, z] (mm)
        speed: Movement speed in mm/s (default 300)
        camera_offset_mm: Camera offset from TCP [x, y, z] (mm)
    
    Returns:
        bool: True if command sent successfully, False otherwise
    """
    try:
        if robot_arm is None:
            print("[CORRECTED] No robot connection")
            return False
            
        if len(camera_coordinates) < 3:
            print("[CORRECTED] Invalid camera coordinates")
            return False
            
        # Create coordinate transformer
        transformer = CameraToRobotTransformer(camera_offset_mm)
        
        # Transform camera coordinates to robot base coordinates
        robot_base_coords = transformer.transform_camera_to_robot_base(
            camera_coordinates, 
            robot_tcp_position
        )
        
        x, y, z = robot_base_coords[0], robot_base_coords[1], robot_base_coords[2]
        
        print(f"[CORRECTED] Camera coords: {camera_coordinates}")
        print(f"[CORRECTED] Robot TCP: {robot_tcp_position}")
        print(f"[CORRECTED] Transformed to robot base: X={x:.1f}, Y={y:.1f}, Z={z:.1f}mm")
        print(f"[CORRECTED] Moving at {speed}mm/s")
        
        # Move robot to transformed coordinates
        result = robot_arm.set_position(
            x=x, y=y, z=z,
            roll=0, pitch=90, yaw=0,  # Fixed orientation
            speed=speed, 
            wait=True
        )
        
        if result == 0:
            print(f"[CORRECTED] SUCCESS: Robot moved to [{x:.1f}, {y:.1f}, {z:.1f}]")
            return True
        else:
            print(f"[CORRECTED] FAILED: Movement failed with code {result}")
            return False
            
    except Exception as e:
        print(f"[CORRECTED] ERROR: {e}")
        return False

def direct_approach(robot_arm, coordinates, speed=300):
    """
    Move robot directly to coordinates - no safety, no checks, just move.
    
    Args:
        robot_arm: XArmAPI instance (already connected)
        coordinates: [x, y, z] in mm
        speed: Movement speed in mm/s (default 300)
    
    Returns:
        bool: True if command sent successfully, False otherwise
    """
    try:
        if robot_arm is None:
            print("[DIRECT] No robot connection")
            return False
            
        if len(coordinates) < 3:
            print("[DIRECT] Invalid coordinates")
            return False
            
        x, y, z = coordinates[0], coordinates[1], coordinates[2]
        
        print(f"[DIRECT] Moving to: X={x:.1f}, Y={y:.1f}, Z={z:.1f}mm at {speed}mm/s")
        
        # Direct robot movement - no safety, no checks
        result = robot_arm.set_position(
            x=x, y=y, z=z,
            roll=0, pitch=90, yaw=0,  # Fixed orientation
            speed=speed, 
            wait=True
        )
        
        if result == 0:
            print(f"[DIRECT] SUCCESS: Robot moved to [{x:.1f}, {y:.1f}, {z:.1f}]")
            return True
        else:
            print(f"[DIRECT] FAILED: Movement failed with code {result}")
            return False
            
    except Exception as e:
        print(f"[DIRECT] ERROR: {e}")
        return False


def corrected_approach_detected_object(robot_arm, object_index, object_name, speed=300, camera_offset_mm=[0, 0, 22.5]):
    """
    Approach a detected object with proper coordinate transformation.
    
    Args:
        robot_arm: XArmAPI instance (already connected)
        object_index: ObjectIndex instance with latest coordinates
        object_name: Name of object to approach (e.g., "bottle")
        speed: Movement speed in mm/s (default 300)
        camera_offset_mm: Camera offset from TCP [x, y, z] (mm)
        
    Returns:
        bool: True if approach successful, False otherwise
    """
    try:
        # Get current robot TCP position
        position = robot_arm.get_position()
        if position[0] == 0:  # Success
            if isinstance(position[1], (list, tuple)):
                robot_tcp_position = [float(position[1][0]), float(position[1][1]), float(position[1][2])]
            else:
                robot_tcp_position = [float(position[1]), float(position[2]), float(position[3])]
        else:
            print("[CORRECTED] Could not get robot TCP position")
            return False
        
        if robot_tcp_position is None:
            print("[CORRECTED] Could not get robot TCP position")
            return False
        
        # Get latest camera coordinates from object index
        with object_index._global_lock:
            camera_coordinates = object_index.latest_mm.get(object_name)
            
        if camera_coordinates is None:
            print(f"[CORRECTED] Object '{object_name}' not detected")
            return False
            
        print(f"[CORRECTED] Found {object_name} at camera coords: {camera_coordinates}")
        print(f"[CORRECTED] Current robot TCP: {robot_tcp_position}")
        
        # Move robot to object with proper transformation
        return corrected_direct_approach(
            robot_arm, 
            camera_coordinates, 
            robot_tcp_position, 
            speed, 
            camera_offset_mm
        )
        
    except Exception as e:
        print(f"[CORRECTED] ERROR approaching {object_name}: {e}")
        return False

def approach_detected_object(robot_arm, object_index, object_name, speed=300):
    """
    Approach a detected object directly - no safety, just move.
    DEPRECATED: Use corrected_approach_detected_object for proper coordinate transformation.
    
    Args:
        robot_arm: XArmAPI instance (already connected)
        object_index: ObjectIndex instance with latest coordinates
        object_name: Name of object to approach (e.g., "bottle")
        speed: Movement speed in mm/s (default 300)
        
    Returns:
        bool: True if approach successful, False otherwise
    """
    try:
        # Get latest coordinates from object index
        with object_index._global_lock:
            coordinates = object_index.latest_mm.get(object_name)
            
        if coordinates is None:
            print(f"[DIRECT] Object '{object_name}' not detected")
            return False
            
        print(f"[DIRECT] Found {object_name} at: {coordinates}")
        
        # Move directly to object coordinates
        return direct_approach(robot_arm, coordinates, speed)
        
    except Exception as e:
        print(f"[DIRECT] ERROR approaching {object_name}: {e}")
        return False
