#!/usr/bin/env python3
"""
Utility Functions - Pure Python Implementation
==============================================

Utility functions for April Tags vision system.
No ROS2 dependencies - pure Python package.
"""

from typing import Dict, List, Tuple, Optional
import numpy as np

class ObjectMapper:
    """Maps April Tag IDs to object names."""
    
    def __init__(self):
        """Initialize object mapper."""
        self.mapping = {
            0: "bottle",
            1: "book", 
            2: "cup",
            3: "pen",
            4: "phone",
            5: "laptop",
            6: "notebook",
            7: "stapler",
            8: "keyboard",
            9: "mouse",
            10: "calculator"
        }
    
    def get_object_name(self, tag_id: int) -> str:
        """Get object name for tag ID."""
        return self.mapping.get(tag_id, f"unknown_object_{tag_id}")
    
    def get_tag_id(self, object_name: str) -> Optional[int]:
        """Get tag ID for object name."""
        for tag_id, name in self.mapping.items():
            if name == object_name:
                return tag_id
        return None
    
    def get_all_objects(self) -> Dict[int, str]:
        """Get all object mappings."""
        return self.mapping.copy()
    
    def add_object(self, tag_id: int, object_name: str):
        """Add new object mapping."""
        self.mapping[tag_id] = object_name
    
    def remove_object(self, tag_id: int):
        """Remove object mapping."""
        if tag_id in self.mapping:
            del self.mapping[tag_id]

class CoordinateStabilizer:
    """Stabilizes coordinates over time."""
    
    def __init__(self, max_samples: int = 10, stability_threshold: float = 5.0):
        """Initialize coordinate stabilizer."""
        self.max_samples = max_samples
        self.stability_threshold = stability_threshold
        self.coordinate_history = {}
    
    def add_coordinate(self, object_name: str, position: Tuple[float, float, float], confidence: float) -> Tuple[float, float, float]:
        """Add coordinate and return stabilized position."""
        if object_name not in self.coordinate_history:
            self.coordinate_history[object_name] = []
        
        # Add new coordinate
        self.coordinate_history[object_name].append({
            'position': position,
            'confidence': confidence,
            'timestamp': time.time()
        })
        
        # Keep only recent samples
        if len(self.coordinate_history[object_name]) > self.max_samples:
            self.coordinate_history[object_name] = self.coordinate_history[object_name][-self.max_samples:]
        
        # Calculate stabilized position
        return self._calculate_stabilized_position(object_name)
    
    def _calculate_stabilized_position(self, object_name: str) -> Tuple[float, float, float]:
        """Calculate stabilized position."""
        if object_name not in self.coordinate_history:
            return (0, 0, 0)
        
        history = self.coordinate_history[object_name]
        if not history:
            return (0, 0, 0)
        
        # Weight by confidence
        total_weight = sum(entry['confidence'] for entry in history)
        if total_weight == 0:
            return (0, 0, 0)
        
        # Calculate weighted average
        x = sum(entry['position'][0] * entry['confidence'] for entry in history) / total_weight
        y = sum(entry['position'][1] * entry['confidence'] for entry in history) / total_weight
        z = sum(entry['position'][2] * entry['confidence'] for entry in history) / total_weight
        
        return (x, y, z)
    
    def is_stable(self, object_name: str) -> bool:
        """Check if coordinates are stable."""
        if object_name not in self.coordinate_history:
            return False
        
        history = self.coordinate_history[object_name]
        if len(history) < 3:
            return False
        
        # Check if recent positions are within threshold
        recent_positions = [entry['position'] for entry in history[-3:]]
        
        for i in range(1, len(recent_positions)):
            pos1 = recent_positions[i-1]
            pos2 = recent_positions[i]
            
            distance = np.sqrt(sum((a - b) ** 2 for a, b in zip(pos1, pos2)))
            if distance > self.stability_threshold:
                return False
        
        return True

def transform_to_robot_frame(camera_position: Tuple[float, float, float], 
                            camera_offset: Tuple[float, float, float] = (0, 0, 0.0225)) -> Tuple[float, float, float]:
    """Transform camera coordinates to robot base frame."""
    x, y, z = camera_position
    offset_x, offset_y, offset_z = camera_offset
    
    # Simple transformation (adjust based on your robot setup)
    robot_x = x + offset_x
    robot_y = y + offset_y  
    robot_z = z + offset_z
    
    return (robot_x, robot_y, robot_z)

def filter_by_depth(position_3d: Tuple[float, float, float], 
                   min_depth: float = 0.1, max_depth: float = 2.0) -> bool:
    """Filter positions by depth range."""
    x, y, z = position_3d
    return min_depth <= z <= max_depth

def calculate_distance(pos1: Tuple[float, float, float], pos2: Tuple[float, float, float]) -> float:
    """Calculate Euclidean distance between two 3D points."""
    return np.sqrt(sum((a - b) ** 2 for a, b in zip(pos1, pos2)))
