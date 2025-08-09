#!/usr/bin/env python3
"""
Enhanced Movement Logic for RAG System

This module integrates the working movement logic from detect_and_move.py
to provide accurate object detection and movement capabilities.
"""

import math
import time
import logging
from typing import Dict, Any, List, Optional, Tuple
from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass
class CameraConfig:
    """Camera configuration for movement calculations."""
    fov_horizontal: float = 87.0  # degrees horizontal FOV
    fov_vertical: float = 58.0    # degrees vertical FOV
    image_width: int = 640
    image_height: int = 480
    depth_offset: float = 170.0   # mm - distance to move closer to object


@dataclass
class ObjectDetection:
    """Object detection data."""
    class_name: str
    confidence: float
    pixel_center: Tuple[int, int]  # (x, y) pixel coordinates
    depth: float  # depth in meters
    bbox: Tuple[int, int, int, int]  # (x1, y1, x2, y2)
    timestamp: float = None

    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = time.time()


class MovementLogic:
    """
    Enhanced movement logic that integrates working movement calculations
    from detect_and_move.py.
    """
    
    def __init__(self, camera_config: Optional[CameraConfig] = None):
        """
        Initialize movement logic.
        
        Args:
            camera_config: Camera configuration for calculations
        """
        self.camera_config = camera_config or CameraConfig()
        logger.info("MovementLogic initialized with camera config: %s", self.camera_config)
    
    def calculate_object_position(
        self, 
        detection: ObjectDetection, 
        robot_position: List[float]
    ) -> List[float]:
        """
        Calculate 3D object position from detection data.
        
        Args:
            detection: Object detection data
            robot_position: Current robot position [x, y, z] in mm
            
        Returns:
            Target position [x, y, z] in mm
        """
        # Extract detection data
        pixel_x, pixel_y = detection.pixel_center
        depth = detection.depth
        
        # Calculate angular offsets from pixel coordinates
        angle_y = (pixel_x - self.camera_config.image_width/2) * (
            self.camera_config.fov_horizontal / self.camera_config.image_width
        )  # Yaw angle
        angle_z = (pixel_y - self.camera_config.image_height/2) * (
            self.camera_config.fov_vertical / self.camera_config.image_height
        )  # Pitch angle
        
        # Convert to radians
        angle_y_rad = math.radians(angle_y)
        angle_z_rad = math.radians(angle_z)
        
        # Calculate 3D position of object relative to camera
        object_x = depth * 1000  # Convert to mm
        object_y = depth * 1000 * math.tan(angle_y_rad)
        object_z = depth * 1000 * math.tan(angle_z_rad)
        
        # Apply calibration offset (compensate for camera mounting)
        # object_y = object_y + 50  # Move 5cm right to compensate for left offset
        
        # Move closer to the object (subtract depth offset)
        object_x = object_x - self.camera_config.depth_offset
        
        # Calculate absolute target position
        target_x = robot_position[0] + object_x
        target_y = robot_position[1] + object_y
        target_z = robot_position[2] + object_z
        
        logger.info(
            "Calculated object position: object_relative=[%.1f, %.1f, %.1f], "
            "target_absolute=[%.1f, %.1f, %.1f]",
            object_x, object_y, object_z, target_x, target_y, target_z
        )
        
        return [target_x, target_y, target_z]
    
    def create_movement_sequence(
        self, 
        target_position: List[float], 
        object_name: str,
        approach_distance: float = 300.0
    ) -> List[Dict[str, Any]]:
        """
        Create a movement sequence for approaching an object.
        
        Args:
            target_position: Target position [x, y, z] in mm
            object_name: Name of the object
            approach_distance: Distance to move down for approach (mm)
            
        Returns:
            List of movement steps
        """
        sequence = [
            {
                "action": "OPEN_GRIPPER",
                "description": f"Open gripper before approaching {object_name}"
            },
            {
                "action": "MOVE_TO_POSE",
                "pose": {
                    "xyz_mm": target_position,
                    "rpy_deg": [0, 90, 0]
                },
                "description": f"Move to {object_name} position"
            },
            {
                "action": "SLEEP",
                "seconds": 1.0,
                "description": "Wait for movement to complete"
            },
            {
                "action": "MOVE_DOWN",
                "distance_mm": approach_distance,
                "description": f"Move down {approach_distance}mm to approach {object_name}"
            },
            {
                "action": "SLEEP",
                "seconds": 1.0,
                "description": "Wait for down movement"
            },
            {
                "action": "CLOSE_GRIPPER",
                "description": f"Close gripper to grasp {object_name}"
            },
            {
                "action": "SLEEP",
                "seconds": 2.0,
                "description": "Wait for gripper to close"
            }
        ]
        
        logger.info("Created movement sequence for %s with %d steps", object_name, len(sequence))
        return sequence
    
    def create_approach_sequence(
        self, 
        target_position: List[float], 
        object_name: str,
        hover_distance: float = 100.0
    ) -> List[Dict[str, Any]]:
        """
        Create an approach sequence for moving towards an object.
        
        Args:
            target_position: Target position [x, y, z] in mm
            object_name: Name of the object
            hover_distance: Hover distance above target (mm)
            
        Returns:
            List of approach steps
        """
        # Calculate hover position (above the target)
        hover_position = target_position.copy()
        hover_position[2] += hover_distance
        
        sequence = [
            {
                "action": "MOVE_TO_POSE",
                "pose": {
                    "xyz_mm": hover_position,
                    "rpy_deg": [0, 90, 0]
                },
                "description": f"Move to hover position above {object_name}"
            },
            {
                "action": "SLEEP",
                "seconds": 1.0,
                "description": "Wait for hover movement"
            },
            {
                "action": "MOVE_TO_POSE",
                "pose": {
                    "xyz_mm": target_position,
                    "rpy_deg": [0, 90, 0]
                },
                "description": f"Move to {object_name} position"
            },
            {
                "action": "SLEEP",
                "seconds": 1.0,
                "description": "Wait for final movement"
            }
        ]
        
        logger.info("Created approach sequence for %s with %d steps", object_name, len(sequence))
        return sequence
    
    def create_gripper_sequence(
        self, 
        object_name: str,
        grasp_position: int = 200,
        release_position: int = 850
    ) -> List[Dict[str, Any]]:
        """
        Create a gripper sequence for grasping an object.
        
        Args:
            object_name: Name of the object
            grasp_position: Gripper position for grasping
            release_position: Gripper position for releasing
            
        Returns:
            List of gripper steps
        """
        sequence = [
            {
                "action": "OPEN_GRIPPER",
                "gripper": {"position": release_position},
                "description": f"Open gripper to prepare for {object_name}"
            },
            {
                "action": "SLEEP",
                "seconds": 1.0,
                "description": "Wait for gripper to open"
            },
            {
                "action": "CLOSE_GRIPPER",
                "gripper": {"position": grasp_position},
                "description": f"Close gripper to grasp {object_name}"
            },
            {
                "action": "SLEEP",
                "seconds": 2.0,
                "description": "Wait for gripper to close"
            }
        ]
        
        logger.info("Created gripper sequence for %s with %d steps", object_name, len(sequence))
        return sequence
    
    def create_safety_sequence(
        self, 
        object_name: str,
        safety_distance: float = 200.0
    ) -> List[Dict[str, Any]]:
        """
        Create a safety sequence for handling dangerous objects.
        
        Args:
            object_name: Name of the object
            safety_distance: Safety distance to maintain (mm)
            
        Returns:
            List of safety steps
        """
        sequence = [
            {
                "action": "MOVE_UP",
                "distance_mm": safety_distance,
                "description": f"Move up {safety_distance}mm for safety with {object_name}"
            },
            {
                "action": "SLEEP",
                "seconds": 1.0,
                "description": "Wait for safety movement"
            }
        ]
        
        logger.info("Created safety sequence for %s with %d steps", object_name, len(sequence))
        return sequence
    
    def validate_target_position(
        self, 
        target_position: List[float], 
        robot_position: List[float],
        workspace_limits: Optional[Dict[str, List[float]]] = None
    ) -> Tuple[bool, str]:
        """
        Validate target position is within safe limits.
        
        Args:
            target_position: Target position [x, y, z] in mm
            robot_position: Current robot position [x, y, z] in mm
            workspace_limits: Workspace limits {"x": [min, max], "y": [min, max], "z": [min, max]}
            
        Returns:
            (is_valid, error_message)
        """
        if workspace_limits:
            x_limits = workspace_limits.get("x", [-500, 500])
            y_limits = workspace_limits.get("y", [-500, 500])
            z_limits = workspace_limits.get("z", [0, 500])
            
            if not (x_limits[0] <= target_position[0] <= x_limits[1]):
                return False, f"Target X position {target_position[0]} outside limits {x_limits}"
            if not (y_limits[0] <= target_position[1] <= y_limits[1]):
                return False, f"Target Y position {target_position[1]} outside limits {y_limits}"
            if not (z_limits[0] <= target_position[2] <= z_limits[1]):
                return False, f"Target Z position {target_position[2]} outside limits {z_limits}"
        
        # Check distance from current position
        distance = math.sqrt(
            (target_position[0] - robot_position[0])**2 +
            (target_position[1] - robot_position[1])**2 +
            (target_position[2] - robot_position[2])**2
        )
        
        if distance > 1000:  # 1 meter limit
            return False, f"Target position too far: {distance:.1f}mm"
        
        return True, "Position is valid"
    
    def update_target_position(
        self, 
        current_detection: ObjectDetection,
        robot_position: List[float],
        last_target_position: Optional[List[float]] = None
    ) -> List[float]:
        """
        Update target position based on new detection data.
        
        Args:
            current_detection: Current object detection
            robot_position: Current robot position
            last_target_position: Last known target position
            
        Returns:
            Updated target position
        """
        new_target_position = self.calculate_object_position(current_detection, robot_position)
        
        # If we have a last position, smooth the update
        if last_target_position:
            # Simple smoothing - average with last position
            smoothed_position = [
                (new_target_position[i] + last_target_position[i]) / 2
                for i in range(3)
            ]
            logger.info(
                "Updated target position: last=[%.1f, %.1f, %.1f], "
                "new=[%.1f, %.1f, %.1f], smoothed=[%.1f, %.1f, %.1f]",
                *last_target_position, *new_target_position, *smoothed_position
            )
            return smoothed_position
        
        return new_target_position
