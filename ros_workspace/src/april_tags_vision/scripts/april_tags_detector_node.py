#!/usr/bin/env python3
"""
April Tags Detector ROS Node
Publishes April Tag detections to ROS topics.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import cv2
import time
import json
from typing import List, Dict, Any

# ROS message imports
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Point2D, Pose, Quaternion
from april_tags_vision.msg import AprilTagDetection, AprilTagDetectionArray

# CV Bridge for image conversion
from cv_bridge import CvBridge

# April Tags detector
from april_tags_vision.april_tag_detector import AprilTagDetector

# Optional imports
try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False
    print("Warning: RealSense SDK not available")

try:
    from xarm.wrapper import XArmAPI
    XARM_AVAILABLE = True
except ImportError:
    XARM_AVAILABLE = False
    print("Warning: XArm SDK not available")

try:
    from scipy.spatial.transform import Rotation as R
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    print("Warning: SciPy not available")

class AprilTagsDetectorNode(Node):
    """ROS Node for April Tags detection."""
    
    def __init__(self):
        super().__init__('april_tags_detector')
        
        # Parameters
        self.declare_parameter('tag_size_mm', 50.0)
        self.declare_parameter('confidence_threshold', 0.8)
        self.declare_parameter('camera_frame_id', 'camera_link')
        self.declare_parameter('robot_frame_id', 'base_link')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('enable_3d_pose', True)
        self.declare_parameter('enable_robot_coords', True)
        self.declare_parameter('show_debug_image', False)
        
        # Get parameters
        self.tag_size_mm = self.get_parameter('tag_size_mm').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.camera_frame_id = self.get_parameter('camera_frame_id').value
        self.robot_frame_id = self.get_parameter('robot_frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.enable_3d_pose = self.get_parameter('enable_3d_pose').value
        self.enable_robot_coords = self.get_parameter('enable_robot_coords').value
        self.show_debug_image = self.get_parameter('show_debug_image').value
        
        # Initialize April Tags detector
        self.detector = AprilTagDetector(
            tag_size_mm=self.tag_size_mm,
            confidence_threshold=self.confidence_threshold
        )
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Camera calibration
        self.camera_info = None
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Robot connection (optional)
        self.robot = None
        if XARM_AVAILABLE and self.enable_robot_coords:
            try:
                robot_ip = self.declare_parameter('robot_ip', '192.168.1.241').value
                self.robot = XArmAPI(robot_ip)
                self.robot.connect()
                self.robot.motion_enable(True)
                self.robot.set_mode(0)
                self.robot.set_state(0)
                self.get_logger().info("✅ Connected to robot")
            except Exception as e:
                self.get_logger().warn(f"⚠️ Failed to connect to robot: {e}")
                self.robot = None
        
        # RealSense pipeline (optional)
        self.pipeline = None
        if REALSENSE_AVAILABLE:
            try:
                self.pipeline = rs.pipeline()
                config = rs.config()
                config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
                self.pipeline.start(config)
                self.get_logger().info("✅ RealSense camera connected")
            except Exception as e:
                self.get_logger().warn(f"⚠️ RealSense camera not available: {e}")
                self.pipeline = None
        
        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.detections_pub = self.create_publisher(
            AprilTagDetectionArray,
            '/april_tags/detections',
            qos_profile
        )
        
        self.debug_image_pub = self.create_publisher(
            Image,
            '/april_tags/debug_image',
            qos_profile
        )
        
        # Legacy publisher for compatibility
        self.legacy_pub = self.create_publisher(
            String,
            '/detected_objects',
            qos_profile
        )
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            qos_profile
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            qos_profile
        )
        
        # Timer for periodic detection
        self.timer = self.create_timer(1.0 / self.publish_rate, self.detection_timer_callback)
        
        self.get_logger().info("🏷️ April Tags Detector Node started")
        self.get_logger().info(f"   Tag size: {self.tag_size_mm}mm")
        self.get_logger().info(f"   Confidence threshold: {self.confidence_threshold}")
        self.get_logger().info(f"   Publish rate: {self.publish_rate}Hz")
        self.get_logger().info(f"   3D pose estimation: {self.enable_3d_pose}")
        self.get_logger().info(f"   Robot coordinates: {self.enable_robot_coords}")
    
    def camera_info_callback(self, msg: CameraInfo):
        """Callback for camera info to set calibration parameters."""
        if self.camera_info is None:
            self.camera_info = msg
            
            # Extract camera matrix and distortion coefficients
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            
            # Set calibration in detector
            self.detector.set_camera_calibration(self.camera_matrix, self.dist_coeffs)
            
            self.get_logger().info("✅ Camera calibration parameters set")
    
    def image_callback(self, msg: Image):
        """Callback for image messages."""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Process the image
            self.process_image(cv_image, msg.header)
            
        except Exception as e:
            self.get_logger().error(f"❌ Error processing image: {e}")
    
    def detection_timer_callback(self):
        """Timer callback for periodic detection using RealSense."""
        if self.pipeline is None:
            return
        
        try:
            # Get frames from RealSense
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                return
            
            # Convert to OpenCV format
            color_image = np.asanyarray(color_frame.get_data())
            
            # Set camera calibration if not already set
            if self.camera_matrix is None:
                camera_matrix, dist_coeffs = self.detector.get_camera_matrix_from_realsense(depth_frame)
                if camera_matrix is not None:
                    self.detector.set_camera_calibration(camera_matrix, dist_coeffs)
                    self.camera_matrix = camera_matrix
                    self.dist_coeffs = dist_coeffs
            
            # Create header
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.camera_frame_id
            
            # Process the image
            self.process_image(color_image, header)
            
        except Exception as e:
            self.get_logger().error(f"❌ Error in timer callback: {e}")
    
    def process_image(self, image: np.ndarray, header: Header):
        """Process image and publish detections."""
        start_time = time.time()
        
        # Detect April Tags
        tag_detections = self.detector.detect_tags(image)
        
        # Convert to ROS messages
        ros_detections = []
        legacy_detections = []
        
        for tag in tag_detections:
            # Create AprilTagDetection message
            detection_msg = self.create_detection_message(tag, header)
            ros_detections.append(detection_msg)
            
            # Create legacy format for compatibility
            legacy_detection = self.create_legacy_detection(tag)
            legacy_detections.append(legacy_detection)
        
        # Create detection array message
        detection_array_msg = AprilTagDetectionArray()
        detection_array_msg.header = header
        detection_array_msg.detections = ros_detections
        detection_array_msg.num_detections = len(ros_detections)
        detection_array_msg.processing_time_ms = (time.time() - start_time) * 1000
        detection_array_msg.camera_frame_id = self.camera_frame_id
        detection_array_msg.robot_frame_id = self.robot_frame_id
        
        # Publish detections
        self.detections_pub.publish(detection_array_msg)
        
        # Publish legacy format
        if legacy_detections:
            legacy_payload = {
                "t": time.time(),
                "units": "mm",
                "items": legacy_detections
            }
            self.legacy_pub.publish(String(data=json.dumps(legacy_payload)))
        
        # Publish debug image if enabled
        if self.show_debug_image:
            debug_image = self.detector.draw_detections(image, tag_detections)
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                debug_msg.header = header
                self.debug_image_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().error(f"❌ Error publishing debug image: {e}")
        
        # Log detection results
        if tag_detections:
            self.get_logger().info(f"🏷️ Detected {len(tag_detections)} April Tags")
            for tag in tag_detections:
                self.get_logger().info(f"   Tag ID: {tag['tag_id']}, Confidence: {tag['confidence']:.3f}")
    
    def create_detection_message(self, tag: Dict[str, Any], header: Header) -> AprilTagDetection:
        """Create AprilTagDetection ROS message from tag data."""
        detection = AprilTagDetection()
        detection.header = header
        
        # Tag identification
        detection.tag_id = tag["tag_id"]
        detection.tag_family = tag["tag_family"]
        detection.tag_size_mm = tag["tag_size_mm"]
        
        # Detection properties
        detection.confidence = tag["confidence"]
        detection.decision_margin = tag["decision_margin"]
        detection.hamming = tag["hamming"]
        detection.goodness = tag["goodness"]
        
        # 2D image coordinates
        center = tag["center"]
        detection.center = Point2D(x=float(center[0]), y=float(center[1]))
        
        # Corners
        for corner in tag["corners"]:
            detection.corners.append(Point2D(x=float(corner[0]), y=float(corner[1])))
        
        # 3D pose information
        if "pose_3d" in tag and tag["pose_3d"] and self.enable_3d_pose:
            pose_3d = tag["pose_3d"]
            detection.pose_3d = Pose()
            detection.pose_3d.position = Point(
                x=pose_3d["translation"][0],
                y=pose_3d["translation"][1],
                z=pose_3d["translation"][2]
            )
            detection.distance = pose_3d["distance"]
            detection.pose_valid = True
        else:
            detection.pose_valid = False
            detection.distance = 0.0
        
        # Robot coordinates (if available)
        if self.robot and self.enable_robot_coords:
            robot_coords = self.get_robot_coordinates(tag)
            if robot_coords:
                detection.position_robot = Point(
                    x=robot_coords["position"][0],
                    y=robot_coords["position"][1],
                    z=robot_coords["position"][2]
                )
                detection.orientation_robot = Quaternion(
                    x=robot_coords["orientation"][0],
                    y=robot_coords["orientation"][1],
                    z=robot_coords["orientation"][2],
                    w=robot_coords["orientation"][3]
                )
                detection.robot_coords_valid = True
            else:
                detection.robot_coords_valid = False
        else:
            detection.robot_coords_valid = False
        
        return detection
    
    def create_legacy_detection(self, tag: Dict[str, Any]) -> Dict[str, Any]:
        """Create legacy detection format for compatibility."""
        legacy_detection = {
            "class": f"april_tag_{tag['tag_id']}",
            "tag_id": tag["tag_id"],
            "pos": [0.0, 0.0, 0.0],  # Will be filled by robot coordinates if available
            "conf": tag["confidence"],
            "pixel_center": tag["center"],
            "image_size": [640, 480],  # Default image size
            "tag_family": tag["tag_family"]
        }
        
        # Add robot coordinates if available
        if self.robot and self.enable_robot_coords:
            robot_coords = self.get_robot_coordinates(tag)
            if robot_coords:
                legacy_detection["pos"] = robot_coords["position"]
        
        return legacy_detection
    
    def get_robot_coordinates(self, tag: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Get robot coordinates for a tag (placeholder implementation)."""
        # This is a simplified implementation
        # In a real system, you would:
        # 1. Get robot pose
        # 2. Transform camera coordinates to robot base frame
        # 3. Apply camera-to-robot transformation
        
        if not self.robot:
            return None
        
        try:
            # Get robot pose
            code, pose = self.robot.get_position()
            if code != 0:
                return None
            
            # Simplified transformation (you would implement proper coordinate transformation here)
            # For now, return the tag center as robot coordinates
            center = tag["center"]
            robot_coords = {
                "position": [float(center[0]), float(center[1]), 0.0],  # Simplified
                "orientation": [0.0, 0.0, 0.0, 1.0]  # Identity quaternion
            }
            
            return robot_coords
            
        except Exception as e:
            self.get_logger().error(f"❌ Error getting robot coordinates: {e}")
            return None
    
    def destroy_node(self):
        """Cleanup on node destruction."""
        if self.pipeline:
            self.pipeline.stop()
        if self.robot:
            self.robot.disconnect()
        super().destroy_node()


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    try:
        node = AprilTagsDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
