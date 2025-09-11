#!/usr/bin/env python3
"""
Camera Manager - Pure Python Implementation
==========================================

RealSense camera management for April Tags detection.
No ROS2 dependencies - pure Python package.
"""

import numpy as np
import cv2
import time
from typing import Optional, Tuple, List, Dict, Any
from dataclasses import dataclass

try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False
    print("Warning: RealSense SDK not available - install with: pip install pyrealsense2")

@dataclass
class CameraConfig:
    """Camera configuration."""
    width: int = 640
    height: int = 480
    fps: int = 30
    color_format: str = "bgr8"
    depth_format: str = "z16"

class CameraManager:
    """RealSense camera manager."""
    
    def __init__(self, config: Optional[CameraConfig] = None):
        """Initialize camera manager."""
        self.config = config or CameraConfig()
        self.pipeline = None
        self.camera_matrix = None
        self.dist_coeffs = None
        
        if REALSENSE_AVAILABLE:
            self._initialize_camera()
        else:
            print("⚠️ RealSense not available - running in simulation mode")
    
    def _initialize_camera(self):
        """Initialize RealSense camera."""
        try:
            # Create pipeline
            self.pipeline = rs.pipeline()
            
            # Create config
            config = rs.config()
            config.enable_stream(rs.stream.color, self.config.width, self.config.height, rs.format.bgr8, self.config.fps)
            config.enable_stream(rs.stream.depth, self.config.width, self.config.height, rs.format.z16, self.config.fps)
            
            # Start pipeline
            self.pipeline.start(config)
            
            # Get camera intrinsics
            profile = self.pipeline.get_active_profile()
            color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
            intrinsics = color_profile.get_intrinsics()
            
            # Create camera matrix
            self.camera_matrix = np.array([
                [intrinsics.fx, 0, intrinsics.ppx],
                [0, intrinsics.fy, intrinsics.ppy],
                [0, 0, 1]
            ], dtype=np.float32)
            
            # Create distortion coefficients
            self.dist_coeffs = np.array(intrinsics.coeffs, dtype=np.float32)
            
            print("📷 RealSense camera initialized")
            print(f"   Resolution: {self.config.width}x{self.config.height}")
            print(f"   FPS: {self.config.fps}")
            print(f"   Camera matrix: {self.camera_matrix}")
            
        except Exception as e:
            print(f"❌ Failed to initialize RealSense camera: {e}")
            self.pipeline = None
    
    def get_frames(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Get color and depth frames."""
        if not self.pipeline:
            return None, None
        
        try:
            # Wait for frames
            frames = self.pipeline.wait_for_frames()
            
            # Get color frame
            color_frame = frames.get_color_frame()
            if not color_frame:
                return None, None
            
            # Get depth frame
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                return None, None
            
            # Convert to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            return color_image, depth_image
            
        except Exception as e:
            print(f"⚠️ Failed to get frames: {e}")
            return None, None
    
    def get_depth_at_point(self, x: int, y: int, depth_image: np.ndarray) -> Optional[float]:
        """Get depth value at specific pixel."""
        if depth_image is None:
            return None
        
        try:
            depth_value = depth_image[y, x]
            return depth_value / 1000.0  # Convert to meters
        except IndexError:
            return None
    
    def pixel_to_3d(self, x: int, y: int, depth: float) -> Optional[Tuple[float, float, float]]:
        """Convert pixel coordinates to 3D coordinates."""
        if self.camera_matrix is None or depth <= 0:
            return None
        
        try:
            # Get camera intrinsics
            fx = self.camera_matrix[0, 0]
            fy = self.camera_matrix[1, 1]
            cx = self.camera_matrix[0, 2]
            cy = self.camera_matrix[1, 2]
            
            # Convert to 3D coordinates
            x_3d = (x - cx) * depth / fx
            y_3d = (y - cy) * depth / fy
            z_3d = depth
            
            return (x_3d, y_3d, z_3d)
            
        except Exception as e:
            print(f"⚠️ Failed to convert pixel to 3D: {e}")
            return None
    
    def stop(self):
        """Stop camera pipeline."""
        if self.pipeline:
            self.pipeline.stop()
            print("📷 Camera stopped")
    
    def is_available(self) -> bool:
        """Check if camera is available."""
        return self.pipeline is not None
