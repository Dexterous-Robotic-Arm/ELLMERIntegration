#!/usr/bin/env python3
"""
April Tags Detector ROS2 Node
==============================

ROS2 node for April Tags detection with RealSense camera.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import time

from april_tags_vision.msg import AprilTagDetection, AprilTagDetectionArray
from april_tags_vision.april_tag_detector import AprilTagDetector

class AprilTagsDetectorNode(Node):
    """April Tags detector ROS2 node."""
    
    def __init__(self):
        super().__init__('april_tags_detector')
        
        # Parameters
        self.declare_parameter('tag_size_mm', 100.0)
        self.declare_parameter('confidence_threshold', 0.8)
        self.declare_parameter('camera_frame_id', 'camera_link')
        self.declare_parameter('robot_frame_id', 'base_link')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('enable_3d_pose', True)
        self.declare_parameter('enable_robot_coords', True)
        self.declare_parameter('show_debug_image', True)
        
        # Get parameters
        self.tag_size_mm = self.get_parameter('tag_size_mm').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.camera_frame_id = self.get_parameter('camera_frame_id').value
        self.robot_frame_id = self.get_parameter('robot_frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.enable_3d_pose = self.get_parameter('enable_3d_pose').value
        self.enable_robot_coords = self.get_parameter('enable_robot_coords').value
        self.show_debug_image = self.get_parameter('show_debug_image').value
        
        # Initialize detector
        self.detector = AprilTagDetector(
            tag_size_mm=self.tag_size_mm,
            confidence_threshold=self.confidence_threshold
        )
        
        # CV bridge
        self.bridge = CvBridge()
        
        # Object mapping
        self.object_mapping = {
            0: "bottle", 1: "book", 2: "cup", 3: "pen", 4: "phone",
            5: "laptop", 6: "notebook", 7: "stapler", 8: "keyboard",
            9: "mouse", 10: "calculator"
        }
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Publishers
        self.detections_pub = self.create_publisher(
            AprilTagDetectionArray,
            '/april_tags/detections',
            10
        )
        
        if self.show_debug_image:
            self.debug_image_pub = self.create_publisher(
                Image,
                '/april_tags/debug_image',
                10
            )
        
        # Timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_detections)
        
        # Storage
        self.latest_image = None
        self.latest_camera_info = None
        self.latest_detections = []
        
        self.get_logger().info('April Tags detector node started')
        self.get_logger().info(f'Tag size: {self.tag_size_mm}mm')
        self.get_logger().info(f'Confidence threshold: {self.confidence_threshold}')
    
    def image_callback(self, msg):
        """Handle incoming image messages."""
        try:
            # Convert ROS image to OpenCV
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Detect tags
            detections = self.detector.detect_tags(self.latest_image)
            
            # Convert to ROS messages
            self.latest_detections = []
            for detection in detections:
                ros_detection = AprilTagDetection()
                ros_detection.tag_id = detection.tag_id
                ros_detection.object_name = self.object_mapping.get(detection.tag_id, f"unknown_object_{detection.tag_id}")
                ros_detection.center.x = detection.center[0]
                ros_detection.center.y = detection.center[1]
                ros_detection.center.z = 0.0
                ros_detection.confidence = detection.confidence
                ros_detection.tag_family = "tagStandard41h12"
                ros_detection.tag_size_mm = self.tag_size_mm
                ros_detection.frame_id = self.camera_frame_id
                ros_detection.has_3d_pose = detection.position_3d is not None
                
                if detection.position_3d:
                    ros_detection.position_3d.x = detection.position_3d[0]
                    ros_detection.position_3d.y = detection.position_3d[1]
                    ros_detection.position_3d.z = detection.position_3d[2]
                
                self.latest_detections.append(ros_detection)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def camera_info_callback(self, msg):
        """Handle camera info messages."""
        try:
            self.latest_camera_info = msg
            
            # Set camera calibration
            camera_matrix = np.array([
                [msg.k[0], msg.k[1], msg.k[2]],
                [msg.k[3], msg.k[4], msg.k[5]],
                [msg.k[6], msg.k[7], msg.k[8]]
            ])
            
            dist_coeffs = np.array(msg.d)
            
            self.detector.set_camera_calibration(camera_matrix, dist_coeffs)
            
        except Exception as e:
            self.get_logger().error(f'Error processing camera info: {e}')
    
    def publish_detections(self):
        """Publish detection results."""
        if not self.latest_detections:
            return
        
        # Create detection array message
        detection_array = AprilTagDetectionArray()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = self.camera_frame_id
        detection_array.detections = self.latest_detections
        detection_array.num_detections = len(self.latest_detections)
        detection_array.processing_time_ms = 0.0  # Could calculate actual processing time
        
        # Publish detections
        self.detections_pub.publish(detection_array)
        
        # Publish debug image if enabled
        if self.show_debug_image and self.latest_image is not None:
            try:
                # Draw detections on image
                debug_image = self.detector.draw_detections(self.latest_image, [
                    self.detector.TagDetection(
                        tag_id=d.tag_id,
                        center=(d.center.x, d.center.y),
                        confidence=d.confidence,
                        corners=[(0, 0), (0, 0), (0, 0), (0, 0)]  # Simplified for debug
                    ) for d in self.latest_detections
                ])
                
                # Convert to ROS image and publish
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
                debug_msg.header.stamp = self.get_clock().now().to_msg()
                debug_msg.header.frame_id = self.camera_frame_id
                self.debug_image_pub.publish(debug_msg)
                
            except Exception as e:
                self.get_logger().error(f'Error publishing debug image: {e}')

def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    node = AprilTagsDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
