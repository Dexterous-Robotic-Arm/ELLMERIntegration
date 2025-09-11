#!/usr/bin/env python3
"""
AprilTag Bridge Node
Converts AprilTag detections from ROS2 topics to robot control format.
This bridges the gap between AprilTag detection and robot movement commands.
"""

import json
import time
import numpy as np
from typing import Dict, List, Any, Optional

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from apriltag_msgs.msg import AprilTagDetectionArray
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformListener, Buffer
    import tf2_ros
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("Warning: ROS2 packages not available")

class AprilTagBridge(Node if ROS2_AVAILABLE else object):
    """
    Bridge between AprilTag detection and robot control system.
    
    IMPORTANT: Camera is mounted on the robot's end effector!
    - AprilTag coordinates are detected in camera frame
    - TF tree automatically transforms to robot base frame
    - Robot moves to coordinates in its own base frame
    
    Subscribes to:
    - /cam_hand/tag_detections (AprilTag detections)
    - /tf (transform data)
    
    Publishes to:
    - /detected_objects (robot control format)
    """
    
    def __init__(self):
        if ROS2_AVAILABLE:
            super().__init__('apriltag_bridge')
            
            # Publishers
            self.detection_pub = self.create_publisher(String, '/detected_objects', 10)
            
            # Subscribers
            self.tag_sub = self.create_subscription(
                AprilTagDetectionArray,
                '/cam_hand/tag_detections',
                self.tag_callback,
                10
            )
            
            # TF buffer and listener
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            
            # Object mapping (AprilTag ID -> object name)
            self.object_mapping = {
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
            
            # Coordinate history for stabilization
            self.coordinate_history = {}
            self.max_history_size = 5
            
            self.get_logger().info("AprilTag Bridge initialized")
            print("[AprilTag Bridge] Node initialized and ready")
            
        else:
            print("[AprilTag Bridge] Running without ROS2 - simulation mode")
    
    def tag_callback(self, msg: AprilTagDetectionArray):
        """Process AprilTag detections and convert to robot control format."""
        if not ROS2_AVAILABLE:
            return
            
        detected_objects = []
        
        for detection in msg.detections:
            try:
                # Get tag ID and basic info
                tag_id = detection.id
                object_name = self.object_mapping.get(tag_id, f"unknown_object_{tag_id}")
                class_name = object_name  # Use simple object name for robot control
                
                # Get 3D position from TF
                position_3d = self._get_tag_position_3d(tag_id)
                
                if position_3d is not None:
                    # Apply coordinate stabilization
                    stabilized_pos = self._stabilize_coordinates(class_name, position_3d)
                    
                    # Create detection object in robot control format
                    detection_obj = {
                        "class": class_name,
                        "tag_id": tag_id,
                        "pos": stabilized_pos,  # [x, y, z] in mm
                        "conf": 0.95,  # High confidence for AprilTags
                        "object_type": "april_tag",
                        "object_name": object_name,
                        "timestamp": time.time()
                    }
                    
                    detected_objects.append(detection_obj)
                    
                    self.get_logger().info(
                        f"Detected {class_name} (AprilTag ID: {tag_id}) at {stabilized_pos}mm"
                    )
                    
            except Exception as e:
                self.get_logger().error(f"Error processing detection: {e}")
                continue
        
        # Publish detected objects
        if detected_objects:
            self._publish_detections(detected_objects)
    
    def _get_tag_position_3d(self, tag_id: int) -> Optional[List[float]]:
        """Get 3D position of AprilTag from TF tree in robot base frame."""
        try:
            # Since camera is on end effector, we need to get the transform from robot base to tag
            # This gives us the tag's position in the robot's base frame
            source_frame = "cam_hand_color_optical_frame"
            target_frame = f"tag36h11:{tag_id}_hand"
            
            # Debug: Print available frames
            self.get_logger().info(f"Looking for transform from {source_frame} to {target_frame}")
            
            # Try to get the transform
            transform = self.tf_buffer.lookup_transform(
                source_frame,
                target_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Extract position (convert from meters to mm)
            pos = transform.transform.translation
            position_mm = [
                pos.x * 1000.0,
                pos.y * 1000.0, 
                pos.z * 1000.0
            ]
            
            # Debug: Print the coordinate transformation
            self.get_logger().info(f"AprilTag {tag_id} in robot base frame: X={position_mm[0]:.1f}, Y={position_mm[1]:.1f}, Z={position_mm[2]:.1f}mm")
            
            return position_mm
            
        except Exception as e:
            # Try alternative frame names
            try:
                target_frame = f"tag36h11:{tag_id}"
                transform = self.tf_buffer.lookup_transform(
                    source_frame,  # Still base_link
                    target_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                
                pos = transform.transform.translation
                position_mm = [
                    pos.x * 1000.0,
                    pos.y * 1000.0,
                    pos.z * 1000.0
                ]
                
                return position_mm
                
            except Exception as e2:
                self.get_logger().debug(f"Could not get TF for tag {tag_id}: {e2}")
                return None
    
    def _stabilize_coordinates(self, class_name: str, position: List[float]) -> List[float]:
        """Apply coordinate stabilization to reduce noise."""
        if class_name not in self.coordinate_history:
            self.coordinate_history[class_name] = []
        
        # Add new position to history
        self.coordinate_history[class_name].append(position.copy())
        
        # Keep only recent positions
        if len(self.coordinate_history[class_name]) > self.max_history_size:
            self.coordinate_history[class_name].pop(0)
        
        # Calculate average position
        if len(self.coordinate_history[class_name]) >= 2:
            positions = np.array(self.coordinate_history[class_name])
            avg_position = np.mean(positions, axis=0)
            return avg_position.tolist()
        else:
            return position
    
    def _publish_detections(self, detected_objects: List[Dict[str, Any]]):
        """Publish detected objects in robot control format."""
        payload = {
            "t": time.time(),
            "units": "mm",
            "items": detected_objects
        }
        
        msg = String(data=json.dumps(payload))
        self.detection_pub.publish(msg)
        
        self.get_logger().info(f"Published {len(detected_objects)} detected objects")

def main(args=None):
    """Main function to run the AprilTag bridge."""
    if not ROS2_AVAILABLE:
        print("ROS2 not available. Please install ROS2 packages.")
        return
    
    rclpy.init(args=args)
    
    try:
        bridge = AprilTagBridge()
        print("[AprilTag Bridge] Starting bridge node...")
        print("[AprilTag Bridge] Listening for AprilTag detections...")
        print("[AprilTag Bridge] Publishing to /detected_objects")
        
        rclpy.spin(bridge)
        
    except KeyboardInterrupt:
        print("\n[AprilTag Bridge] Shutting down...")
    finally:
        if ROS2_AVAILABLE:
            rclpy.shutdown()

if __name__ == '__main__':
    main()
