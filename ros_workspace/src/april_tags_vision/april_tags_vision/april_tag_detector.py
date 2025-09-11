#!/usr/bin/env python3
"""
April Tags Detector - ROS2 Implementation
========================================

April Tags detection using TagStandard41h12 family with 100mm size.
ROS2 package implementation.
"""

import numpy as np
import cv2
import time
from typing import List, Dict, Any, Optional, Tuple
from dataclasses import dataclass

try:
    import apriltag
    APRILTAG_AVAILABLE = True
except ImportError:
    APRILTAG_AVAILABLE = False
    print("Warning: apriltag not available - install with: pip install apriltag")

try:
    from scipy.spatial.transform import Rotation as R
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    print("Warning: SciPy not available")

# April Tags configuration
TAG_FAMILY = "tagStandard41h12"
TAG_SIZE_MM = 100.0  # Size of April Tag in millimeters (100mm for desk objects)
CONFIDENCE_THRESHOLD = 0.8  # Minimum confidence for tag detection

@dataclass
class TagDetection:
    """April Tag detection result."""
    tag_id: int
    center: Tuple[float, float]  # (x, y) in pixels
    confidence: float
    corners: List[Tuple[float, float]]  # 4 corners in pixels
    pose_3d: Optional[np.ndarray] = None  # 3D pose matrix
    position_3d: Optional[Tuple[float, float, float]] = None  # (x, y, z) in mm

class AprilTagDetector:
    """April Tags detector for robot vision system."""
    
    def __init__(self, tag_size_mm: float = TAG_SIZE_MM, confidence_threshold: float = CONFIDENCE_THRESHOLD):
        """Initialize April Tags detector."""
        self.tag_size_mm = tag_size_mm
        self.confidence_threshold = confidence_threshold
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Initialize April Tags detector
        if APRILTAG_AVAILABLE:
            self.detector = apriltag.Detector(families=TAG_FAMILY)
            print(f"🏷️ April Tags detector initialized (TagStandard41h12, {tag_size_mm}mm)")
        else:
            self.detector = None
            print("⚠️ April Tags detector not available")
    
    def set_camera_calibration(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray):
        """Set camera calibration parameters."""
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        print("📷 Camera calibration set")
    
    def get_camera_matrix_from_realsense(self, depth_sensor) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Get camera matrix from RealSense depth sensor."""
        try:
            import pyrealsense2 as rs
            
            # Get intrinsics from depth sensor
            intrinsics = depth_sensor.get_intrinsics()
            
            # Create camera matrix
            camera_matrix = np.array([
                [intrinsics.fx, 0, intrinsics.ppx],
                [0, intrinsics.fy, intrinsics.ppy],
                [0, 0, 1]
            ], dtype=np.float32)
            
            # Create distortion coefficients
            dist_coeffs = np.array(intrinsics.coeffs, dtype=np.float32)
            
            return camera_matrix, dist_coeffs
            
        except Exception as e:
            print(f"⚠️ Failed to get camera matrix from RealSense: {e}")
            return None, None
    
    def detect_tags(self, image: np.ndarray) -> List[TagDetection]:
        """Detect April Tags in image."""
        if not self.detector:
            return []
        
        # Convert to grayscale if needed
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        
        # Detect tags
        detections = self.detector.detect(gray)
        
        results = []
        for detection in detections:
            # Check confidence
            if detection.decision_margin < self.confidence_threshold:
                continue
            
            # Get tag ID
            tag_id = detection.tag_id
            
            # Get center point
            center = detection.center
            
            # Get corners
            corners = detection.corners.tolist()
            
            # Calculate 3D pose if camera calibration is available
            pose_3d = None
            position_3d = None
            
            if self.camera_matrix is not None and SCIPY_AVAILABLE:
                try:
                    # Convert corners to numpy array
                    corners_3d = np.array(corners, dtype=np.float32)
                    
                    # Define 3D points for tag corners (in mm)
                    tag_size = self.tag_size_mm / 1000.0  # Convert to meters
                    object_points = np.array([
                        [-tag_size/2, -tag_size/2, 0],
                        [tag_size/2, -tag_size/2, 0],
                        [tag_size/2, tag_size/2, 0],
                        [-tag_size/2, tag_size/2, 0]
                    ], dtype=np.float32)
                    
                    # Solve PnP
                    success, rvec, tvec = cv2.solvePnP(
                        object_points, corners_3d, 
                        self.camera_matrix, self.dist_coeffs
                    )
                    
                    if success:
                        # Convert rotation vector to rotation matrix
                        rotation_matrix, _ = cv2.Rodrigues(rvec)
                        
                        # Create 4x4 transformation matrix
                        pose_3d = np.eye(4)
                        pose_3d[:3, :3] = rotation_matrix
                        pose_3d[:3, 3] = tvec.flatten()
                        
                        # Get position in mm
                        position_3d = (tvec[0][0] * 1000, tvec[1][0] * 1000, tvec[2][0] * 1000)
                        
                except Exception as e:
                    print(f"⚠️ Failed to calculate 3D pose for tag {tag_id}: {e}")
            
            # Create detection result
            detection_result = TagDetection(
                tag_id=tag_id,
                center=center,
                confidence=detection.decision_margin,
                corners=corners,
                pose_3d=pose_3d,
                position_3d=position_3d
            )
            
            results.append(detection_result)
        
        return results
    
    def draw_detections(self, image: np.ndarray, detections: List[TagDetection]) -> np.ndarray:
        """Draw detections on image."""
        result_image = image.copy()
        
        for detection in detections:
            # Draw tag ID
            cv2.putText(result_image, f"ID: {detection.tag_id}", 
                        (int(detection.center[0]), int(detection.center[1]) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Draw confidence
            cv2.putText(result_image, f"Conf: {detection.confidence:.2f}", 
                        (int(detection.center[0]), int(detection.center[1]) + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            # Draw corners
            corners = np.array(detection.corners, dtype=np.int32)
            cv2.polylines(result_image, [corners], True, (0, 255, 0), 2)
            
            # Draw center
            cv2.circle(result_image, (int(detection.center[0]), int(detection.center[1])), 5, (0, 0, 255), -1)
        
        return result_image