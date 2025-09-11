#!/usr/bin/env python3
"""
april_tag_detector.py
April Tags detection module for robot vision system.
Replaces YOLOv8 with April Tags for object detection using TagStandard41h12.
Note: YOLO has been completely disabled. This system now uses April Tags exclusively.
"""

import numpy as np
import cv2
import time
import logging
from typing import List, Dict, Any, Optional, Tuple

# Optional imports - only import if available
try:
    import apriltag
    APRILTAG_AVAILABLE = True
except ImportError:
    APRILTAG_AVAILABLE = False
    print("Warning: apriltag not available - install with: pip install apriltag")

try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False
    print("Warning: RealSense SDK not available")

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

class AprilTagDetector:
    """April Tags detector for robot vision system."""
    
    def __init__(self, tag_size_mm: float = TAG_SIZE_MM, confidence_threshold: float = CONFIDENCE_THRESHOLD):
        """
        Initialize April Tags detector.
        
        Args:
            tag_size_mm: Physical size of April Tags in millimeters
            confidence_threshold: Minimum confidence for tag detection (0.0-1.0)
        """
        self.tag_size_mm = tag_size_mm
        self.confidence_threshold = confidence_threshold
        self.detector = None
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Initialize April Tags detector
        if APRILTAG_AVAILABLE:
            try:
                # Create detector for TagStandard41h12 family
                self.detector = apriltag.Detector(families=TAG_FAMILY)
                print(f"✅ April Tags detector initialized for {TAG_FAMILY}")
            except Exception as e:
                print(f"❌ Failed to initialize April Tags detector: {e}")
                self.detector = None
        else:
            print("❌ April Tags not available - install with: pip install apriltag")
    
    def set_camera_calibration(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray):
        """
        Set camera calibration parameters for 3D pose estimation.
        
        Args:
            camera_matrix: 3x3 camera intrinsic matrix
            dist_coeffs: Distortion coefficients
        """
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        print("✅ Camera calibration parameters set")
    
    def detect_tags(self, image: np.ndarray) -> List[Dict[str, Any]]:
        """
        Detect April Tags in the given image.
        
        Args:
            image: Input image (BGR format)
            
        Returns:
            List of detected tags with their properties
        """
        if not self.detector:
            print("❌ April Tags detector not available")
            return []
        
        # Convert BGR to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Detect tags
        try:
            detections = self.detector.detect(gray)
        except Exception as e:
            print(f"❌ April Tags detection failed: {e}")
            return []
        
        detected_tags = []
        
        for detection in detections:
            # Filter by confidence
            if detection.decision_margin < self.confidence_threshold * 100:  # decision_margin is typically 0-100
                continue
            
            # Calculate tag center
            center = detection.center
            center_x, center_y = int(center[0]), int(center[1])
            
            # Get tag corners
            corners = detection.corners.astype(int)
            
            # Calculate tag properties
            tag_info = {
                "tag_id": detection.tag_id,
                "center": [center_x, center_y],
                "corners": corners.tolist(),
                "decision_margin": detection.decision_margin,
                "confidence": min(detection.decision_margin / 100.0, 1.0),  # Normalize to 0-1
                "hamming": detection.hamming,
                "goodness": detection.goodness,
                "tag_family": TAG_FAMILY,
                "tag_size_mm": self.tag_size_mm
            }
            
            # Estimate 3D pose if camera calibration is available
            if self.camera_matrix is not None and self.dist_coeffs is not None:
                pose = self._estimate_pose(detection)
                if pose is not None:
                    tag_info["pose_3d"] = pose
            
            detected_tags.append(tag_info)
        
        return detected_tags
    
    def _estimate_pose(self, detection) -> Optional[Dict[str, Any]]:
        """
        Estimate 3D pose of the April Tag.
        
        Args:
            detection: April Tag detection object
            
        Returns:
            Dictionary with 3D pose information or None if estimation fails
        """
        if self.camera_matrix is None or self.dist_coeffs is None:
            return None
        
        try:
            # Define object points for the tag (in tag coordinate system)
            # Tag is centered at origin, lying in XY plane
            tag_half_size = self.tag_size_mm / 2000.0  # Convert mm to meters, half size
            object_points = np.array([
                [-tag_half_size, -tag_half_size, 0],
                [ tag_half_size, -tag_half_size, 0],
                [ tag_half_size,  tag_half_size, 0],
                [-tag_half_size,  tag_half_size, 0]
            ], dtype=np.float32)
            
            # Image points (corners of the tag in image)
            image_points = detection.corners.astype(np.float32)
            
            # Solve PnP to get pose
            success, rvec, tvec = cv2.solvePnP(
                object_points, 
                image_points, 
                self.camera_matrix, 
                self.dist_coeffs
            )
            
            if not success:
                return None
            
            # Convert rotation vector to rotation matrix
            if SCIPY_AVAILABLE:
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                
                # Convert to Euler angles for easier interpretation
                r = R.from_matrix(rotation_matrix)
                euler_angles = r.as_euler('xyz', degrees=True)
                
                pose = {
                    "translation": tvec.flatten().tolist(),  # [x, y, z] in meters
                    "rotation_matrix": rotation_matrix.tolist(),
                    "euler_angles": euler_angles.tolist(),  # [roll, pitch, yaw] in degrees
                    "distance": np.linalg.norm(tvec),  # Distance from camera
                    "success": True
                }
            else:
                pose = {
                    "translation": tvec.flatten().tolist(),
                    "rotation_vector": rvec.flatten().tolist(),
                    "distance": np.linalg.norm(tvec),
                    "success": True
                }
            
            return pose
            
        except Exception as e:
            print(f"❌ Pose estimation failed: {e}")
            return None
    
    def draw_detections(self, image: np.ndarray, detections: List[Dict[str, Any]]) -> np.ndarray:
        """
        Draw April Tag detections on the image.
        
        Args:
            image: Input image
            detections: List of detected tags
            
        Returns:
            Image with drawn detections
        """
        result_image = image.copy()
        
        for tag in detections:
            # Draw tag outline
            corners = np.array(tag["corners"], dtype=np.int32)
            cv2.polylines(result_image, [corners], True, (0, 255, 0), 2)
            
            # Draw center point
            center = tag["center"]
            cv2.circle(result_image, tuple(center), 5, (0, 0, 255), -1)
            
            # Draw tag ID and confidence
            tag_id = tag["tag_id"]
            confidence = tag["confidence"]
            text = f"ID:{tag_id} ({confidence:.2f})"
            
            # Position text above the tag
            text_pos = (center[0] - 30, center[1] - 10)
            cv2.putText(result_image, text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Draw distance if available
            if "pose_3d" in tag and tag["pose_3d"]["success"]:
                distance = tag["pose_3d"]["distance"]
                dist_text = f"Dist: {distance:.2f}m"
                dist_pos = (center[0] - 30, center[1] + 20)
                cv2.putText(result_image, dist_text, dist_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        
        return result_image
    
    def get_camera_matrix_from_realsense(self, depth_frame) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Extract camera matrix and distortion coefficients from RealSense depth frame.
        
        Args:
            depth_frame: RealSense depth frame
            
        Returns:
            Tuple of (camera_matrix, dist_coeffs) or (None, None) if extraction fails
        """
        if not REALSENSE_AVAILABLE:
            return None, None
        
        try:
            # Get intrinsics from depth frame
            intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
            
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
            print(f"❌ Failed to extract camera calibration from RealSense: {e}")
            return None, None


def main():
    """Test the April Tags detector."""
    print("🧪 Testing April Tags Detector")
    print("=" * 50)
    
    # Check dependencies
    if not APRILTAG_AVAILABLE:
        print("❌ April Tags not available - install with: pip install apriltag")
        return
    
    if not REALSENSE_AVAILABLE:
        print("❌ RealSense not available - using test image")
        # Create a test image with a simple pattern
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(test_image, "April Tags Test Image", (200, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(test_image, "No RealSense camera available", (180, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128, 128, 128), 1)
    else:
        # Try to get image from RealSense
        try:
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            pipeline.start(config)
            
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            test_image = np.asanyarray(color_frame.get_data())
            
            pipeline.stop()
            print("✅ Captured image from RealSense")
        except Exception as e:
            print(f"❌ Failed to capture from RealSense: {e}")
            # Create test image
            test_image = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(test_image, "RealSense Error - Test Image", (180, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    
    # Initialize detector
    detector = AprilTagDetector()
    
    # Detect tags
    print("🔍 Detecting April Tags...")
    detections = detector.detect_tags(test_image)
    
    print(f"📊 Found {len(detections)} April Tags")
    
    for i, tag in enumerate(detections):
        print(f"  Tag {i+1}: ID={tag['tag_id']}, Center=({tag['center'][0]}, {tag['center'][1]}), Confidence={tag['confidence']:.3f}")
        if "pose_3d" in tag and tag["pose_3d"]["success"]:
            pose = tag["pose_3d"]
            print(f"    3D Pose: Translation={pose['translation']}, Distance={pose['distance']:.3f}m")
    
    # Draw detections
    result_image = detector.draw_detections(test_image, detections)
    
    # Show result
    cv2.imshow("April Tags Detection", result_image)
    print("Press any key to close...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    print("✅ April Tags detector test completed")


if __name__ == "__main__":
    main()
