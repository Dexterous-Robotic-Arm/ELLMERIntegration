#!/usr/bin/env python3
"""
Test script for April Tags Vision ROS2 package
"""

import rclpy
from rclpy.node import Node
from april_tags_vision.msg import AprilTagDetectionArray

class AprilTagsTestNode(Node):
    """Test node for April Tags detections."""
    
    def __init__(self):
        super().__init__('april_tags_test')
        
        # Subscribe to detections
        self.detections_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/april_tags/detections',
            self.detections_callback,
            10
        )
        
        self.get_logger().info('April Tags test node started')
        self.get_logger().info('Listening for detections on /april_tags/detections')
    
    def detections_callback(self, msg):
        """Handle detection messages."""
        self.get_logger().info(f'Received {msg.num_detections} detections')
        
        for detection in msg.detections:
            self.get_logger().info(
                f'Tag {detection.tag_id} ({detection.object_name}): '
                f'Center: [{detection.center.x:.1f}, {detection.center.y:.1f}] '
                f'Confidence: {detection.confidence:.2f}'
            )
            
            if detection.has_3d_pose:
                self.get_logger().info(
                    f'  3D Position: [{detection.position_3d.x:.1f}, '
                    f'{detection.position_3d.y:.1f}, {detection.position_3d.z:.1f}]mm'
                )

def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    node = AprilTagsTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
