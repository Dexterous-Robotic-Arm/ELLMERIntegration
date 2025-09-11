#!/usr/bin/env python3
"""
Test script for April Tags ROS package.
Tests the April Tags detector with ROS integration.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import cv2
import time
from typing import List

# ROS message imports
from std_msgs.msg import String
from sensor_msgs.msg import Image
from april_tags_vision.msg import AprilTagDetection, AprilTagDetectionArray

# CV Bridge for image conversion
from cv_bridge import CvBridge

class AprilTagsTestNode(Node):
    """Test node for April Tags detection."""
    
    def __init__(self):
        super().__init__('april_tags_test')
        
        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Subscribers
        self.detections_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/april_tags/detections',
            self.detections_callback,
            qos_profile
        )
        
        self.debug_image_sub = self.create_subscription(
            Image,
            '/april_tags/debug_image',
            self.debug_image_callback,
            qos_profile
        )
        
        self.legacy_sub = self.create_subscription(
            String,
            '/detected_objects',
            self.legacy_callback,
            qos_profile
        )
        
        # Statistics
        self.detection_count = 0
        self.last_detection_time = 0
        
        self.get_logger().info("🧪 April Tags Test Node started")
        self.get_logger().info("   Listening for detections on /april_tags/detections")
        self.get_logger().info("   Listening for debug images on /april_tags/debug_image")
        self.get_logger().info("   Listening for legacy format on /detected_objects")
    
    def detections_callback(self, msg: AprilTagDetectionArray):
        """Callback for April Tag detections."""
        self.detection_count += 1
        self.last_detection_time = time.time()
        
        self.get_logger().info(f"🏷️ Detection #{self.detection_count}: {msg.num_detections} tags")
        self.get_logger().info(f"   Processing time: {msg.processing_time_ms:.2f}ms")
        self.get_logger().info(f"   Camera frame: {msg.camera_frame_id}")
        self.get_logger().info(f"   Robot frame: {msg.robot_frame_id}")
        
        for i, detection in enumerate(msg.detections):
            self.get_logger().info(f"   Tag {i+1}: ID={detection.tag_id}, "
                                 f"Confidence={detection.confidence:.3f}, "
                                 f"Family={detection.tag_family}")
            
            if detection.pose_valid:
                pose = detection.pose_3d
                self.get_logger().info(f"     3D Pose: [{pose.position.x:.3f}, "
                                     f"{pose.position.y:.3f}, {pose.position.z:.3f}]")
                self.get_logger().info(f"     Distance: {detection.distance:.3f}m")
            
            if detection.robot_coords_valid:
                pos = detection.position_robot
                self.get_logger().info(f"     Robot Coords: [{pos.x:.1f}, "
                                     f"{pos.y:.1f}, {pos.z:.1f}]mm")
    
    def debug_image_callback(self, msg: Image):
        """Callback for debug images."""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Display image
            cv2.imshow("April Tags Debug Image", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"❌ Error processing debug image: {e}")
    
    def legacy_callback(self, msg: String):
        """Callback for legacy detection format."""
        try:
            import json
            data = json.loads(msg.data)
            
            self.get_logger().info(f"📦 Legacy format: {len(data.get('items', []))} objects")
            
            for item in data.get('items', []):
                self.get_logger().info(f"   {item.get('class', 'unknown')}: "
                                     f"pos={item.get('pos', [0,0,0])}, "
                                     f"conf={item.get('conf', 0.0):.3f}")
                
        except Exception as e:
            self.get_logger().error(f"❌ Error processing legacy message: {e}")
    
    def print_statistics(self):
        """Print detection statistics."""
        current_time = time.time()
        time_since_last = current_time - self.last_detection_time if self.last_detection_time > 0 else 0
        
        self.get_logger().info("📊 Detection Statistics:")
        self.get_logger().info(f"   Total detections: {self.detection_count}")
        self.get_logger().info(f"   Time since last detection: {time_since_last:.1f}s")
        
        if self.detection_count > 0:
            self.get_logger().info("✅ April Tags detection is working!")
        else:
            self.get_logger().warn("⚠️ No detections received yet")


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    try:
        node = AprilTagsTestNode()
        
        # Print statistics every 10 seconds
        timer = node.create_timer(10.0, node.print_statistics)
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
