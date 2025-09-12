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

#realsense camera detection code added
import pyrealsense2 as rs

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

# add depthobstacledetector with realsense depth
# ADD THIS CLASS BEFORE THE AprilTagBridge CLASS
class DepthObstacleDetector:
    """Detects obstacles using RealSense depth data."""
    
    def __init__(self):
        self.pipeline = None
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            self.pipeline.start(config)
            print("[DepthObstacle] RealSense depth pipeline started")
        except Exception as e:
            print(f"[DepthObstacle] Failed to start RealSense depth pipeline: {e}")
            self.pipeline = None
    
    def get_obstacle_points(self, min_distance_m=0.15, max_distance_m=1.5, grid_size=40):
        """Sample depth image and return obstacle points in camera frame."""
        if not self.pipeline:
            return []
            
        try:
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            
            if not depth_frame:
                return []
            
            obstacles = []
            width = depth_frame.get_width()
            height = depth_frame.get_height()
            
            # Get camera intrinsics
            depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
            
            obstacle_count = 0
            
            for x in range(grid_size, width - grid_size, grid_size):
                for y in range(grid_size, height - grid_size, grid_size):
                    distance = depth_frame.get_distance(x, y)
                    
                    if min_distance_m < distance < max_distance_m:
                        # Convert pixel to 3D point in camera frame
                        point_3d = rs.rs2_deproject_pixel_to_point(
                            depth_intrinsics, [x, y], distance
                        )
                        
                        # Convert to mm and format for robot control
                        obstacle_point = {
                            "class": "obstacle",
                            "pos": [point_3d[0] * 1000, point_3d[1] * 1000, point_3d[2] * 1000],
                            "conf": 0.8,
                            "object_type": "depth_obstacle",
                            "object_name": f"obstacle_{obstacle_count}",
                            "distance": distance,
                            "pixel_coords": [x, y],
                            "timestamp": time.time()
                        }
                        
                        obstacles.append(obstacle_point)
                        obstacle_count += 1
                        
                        if obstacle_count > 50:
                            break
                if obstacle_count > 50:
                    break
            
            return obstacles
            
        except Exception as e:
            print(f"[DepthObstacle] Error getting obstacle points: {e}")
            return []
    
    def cleanup(self):
        """Stop the RealSense pipeline."""
        if self.pipeline:
            try:
                self.pipeline.stop()
                print("[DepthObstacle] RealSense pipeline stopped")
            except Exception as e:
                print(f"[DepthObstacle] Error stopping pipeline: {e}")


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
    
    # def __init__(self):
    #     if ROS2_AVAILABLE:
    #         super().__init__('apriltag_bridge')
            
    #         # Publishers
    #         self.detection_pub = self.create_publisher(String, '/detected_objects', 10)
            
    #         # Subscribers
    #         self.tag_sub = self.create_subscription(
    #             AprilTagDetectionArray,
    #             '/cam_hand/tag_detections',
    #             self.tag_callback,
    #             10
    #         )
            
    #         # TF buffer and listener
    #         self.tf_buffer = Buffer()
    #         self.tf_listener = TransformListener(self.tf_buffer, self)
            
    #         # Parameters
    #         # Base frame in which to publish/tag poses. If your robot base is available in TF,
    #         # set this to that frame (e.g., 'base_link', 'xarm_base_link'). Otherwise, it will
    #         # default to the camera optical frame so values match your TF echo.
    #         self.declare_parameter('robot_base_frame', 'cam_hand_color_optical_frame')
    #         self.robot_base_frame = self.get_parameter('robot_base_frame').get_parameter_value().string_value

    #         # Object mapping (AprilTag ID -> object name)
    #         self.object_mapping = {
    #             0: "bottle",
    #             1: "book", 
    #             2: "cup",
    #             3: "pen",
    #             4: "phone",
    #             5: "laptop",
    #             6: "notebook",
    #             7: "stapler",
    #             8: "keyboard",
    #             9: "mouse",
    #             10: "calculator"
    #         }
            
    #         # Coordinate history for stabilization
    #         self.coordinate_history = {}
    #         self.max_history_size = 10  # Increased for better stabilization
            
    #         self.get_logger().info("AprilTag Bridge initialized")
    #         print("[AprilTag Bridge] Node initialized and ready")
            
    #     else:
    #         print("[AprilTag Bridge] Running without ROS2 - simulation mode")
    
    # REPLACE THE __init__ METHOD IN YOUR AprilTagBridge CLASS WITH THIS:
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
            
            # Parameters
            self.declare_parameter('robot_base_frame', 'cam_hand_color_optical_frame')
            self.robot_base_frame = self.get_parameter('robot_base_frame').get_parameter_value().string_value

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
            self.max_history_size = 10
            
            # ADD THIS: Initialize depth obstacle detector
            self.depth_detector = DepthObstacleDetector()
            
            # ADD THIS: Timer for depth obstacle detection
            self.depth_timer = self.create_timer(0.5, self.publish_depth_obstacles)  # 2 Hz
            
            self.get_logger().info("AprilTag Bridge with Depth Obstacles initialized")
            print("[AprilTag Bridge] Node initialized with depth obstacle detection")
            
        else:
            print("[AprilTag Bridge] Running without ROS2 - simulation mode")

    def publish_depth_obstacles(self):
        """Publish depth-based obstacles separately from AprilTags."""
        if not self.depth_detector or not self.depth_detector.pipeline:
            return
        
        try:
            # Get obstacle points from depth data
            obstacles = self.depth_detector.get_obstacle_points()
            
            if obstacles:
                # Publish obstacles
                payload = {
                    "t": time.time(),
                    "units": "mm", 
                    "items": obstacles
                }
                
                msg = String(data=json.dumps(payload))
                self.detection_pub.publish(msg)
                
                self.get_logger().debug(f"Published {len(obstacles)} depth obstacles")
                
        except Exception as e:
            self.get_logger().error(f"Error publishing depth obstacles: {e}")

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
                        f"[AprilTag Bridge] Detected {class_name} (AprilTag ID: {tag_id}) at {stabilized_pos}mm"
                    )
                    
            except Exception as e:
                self.get_logger().error(f"Error processing detection: {e}")
                continue
        
        # Publish detected objects
        if detected_objects:
            self._publish_detections(detected_objects)
    
    def _get_tag_position_3d(self, tag_id: int) -> Optional[List[float]]:
        """Get 3D position of AprilTag from TF tree in robot base frame."""
        # Try different frame name combinations (keeping the ones that produce correct coordinates)
        frame_combinations = [
            (self.robot_base_frame, f"tag36h11:{tag_id}"),
            ("cam_hand_color_optical_frame", f"tag36h11:{tag_id}")
        ]
        
        for source_frame, target_frame in frame_combinations:
            try:
                # Debug: Print frame lookup attempt
                self.get_logger().debug(f"Trying transform from {source_frame} to {target_frame}")
                
                # Try to get the transform
                transform = self.tf_buffer.lookup_transform(
                    source_frame,
                    target_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                
                # Extract position (convert from meters to mm)
                pos = transform.transform.translation
                
                # Debug: Print raw TF coordinates
                self.get_logger().debug(f"Raw TF coordinates for tag {tag_id}: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}m")
                
                # Store camera coordinates directly - transformation will be done during movement
                # Camera frame: X=left/right, Y=up/down, Z=forward/backward
                # Store as [x, y, z] in camera coordinate system
                position_mm = [
                    pos.x * 1000.0,  # Camera X (left/right)
                    pos.y * 1000.0,  # Camera Y (up/down)  
                    pos.z * 1000.0   # Camera Z (forward/backward)
                ]
                
                # Validate coordinates (basic sanity check)
                x, y, z = position_mm[0], position_mm[1], position_mm[2]
                
                # Check for reasonable coordinate ranges (camera frame)
                if abs(x) > 2000 or abs(y) > 2000 or abs(z) > 2000:  # 2m range check
                    self.get_logger().warning(f"[AprilTag Bridge] REJECTING coordinates out of range: X={x:.1f}, Y={y:.1f}, Z={z:.1f}mm")
                    continue
                
                # Debug: Print the coordinate transformation
                self.get_logger().info(f"[AprilTag Bridge] Tag {tag_id} transform SUCCESS: {source_frame}->{target_frame}")
                self.get_logger().info(f"[AprilTag Bridge] Camera coordinates: X={position_mm[0]:.1f}, Y={position_mm[1]:.1f}, Z={position_mm[2]:.1f}mm")
                
                return position_mm
                
            except Exception as e:
                self.get_logger().debug(f"Transform failed {source_frame}->{target_frame}: {e}")
                continue
        
        # If all frame combinations failed
        self.get_logger().warning(f"Could not get TF transform for tag {tag_id} with any frame combination")
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
        
        # Calculate average position with outlier filtering
        if len(self.coordinate_history[class_name]) >= 3:
            positions = np.array(self.coordinate_history[class_name])
            
            # Remove outliers (positions that are too far from the median)
            median_pos = np.median(positions, axis=0)
            distances = np.linalg.norm(positions - median_pos, axis=1)
            threshold = np.percentile(distances, 75)  # Keep positions within 75th percentile
            
            # Filter out outliers
            valid_indices = distances <= threshold
            if np.sum(valid_indices) >= 2:  # Need at least 2 valid positions
                filtered_positions = positions[valid_indices]
                avg_position = np.mean(filtered_positions, axis=0)
                return avg_position.tolist()
        
        # Fallback to simple average if not enough data
        if len(self.coordinate_history[class_name]) >= 2:
            positions = np.array(self.coordinate_history[class_name])
            avg_position = np.mean(positions, axis=0)
            return avg_position.tolist()
        else:
            return position
    
    # def _publish_detections(self, detected_objects: List[Dict[str, Any]]):
    #     """Publish detected objects in robot control format."""
    #     payload = {
    #         "t": time.time(),
    #         "units": "mm",
    #         "items": detected_objects
    #     }
        
    #     msg = String(data=json.dumps(payload))
    #     self.detection_pub.publish(msg)
        
    #     self.get_logger().info(f"Published {len(detected_objects)} detected objects")
    def _publish_detections(self, detected_objects: List[Dict[str, Any]]):
        """Publish detected objects in robot control format."""
        
        # Get current depth obstacles
        depth_obstacles = []
        if self.depth_detector and self.depth_detector.pipeline:
            try:
                depth_obstacles = self.depth_detector.get_obstacle_points()
            except Exception as e:
                self.get_logger().warning(f"Could not get depth obstacles: {e}")
        
        # Combine AprilTag objects with depth obstacles
        all_objects = detected_objects + depth_obstacles
        
        payload = {
            "t": time.time(),
            "units": "mm",
            "items": all_objects
        }
        
        msg = String(data=json.dumps(payload))
        self.detection_pub.publish(msg)
        
        if depth_obstacles:
            self.get_logger().info(f"Published {len(detected_objects)} AprilTags + {len(depth_obstacles)} depth obstacles")
        else:
            self.get_logger().info(f"Published {len(detected_objects)} detected objects")


    # def publish_depth_obstacles(self):
    #     """Publish depth-based obstacles separately from AprilTags."""
    #     if not self.depth_detector or not self.depth_detector.pipeline:
    #         return
        
    #     try:
    #         # Get obstacle points from depth data
    #         obstacles = self.depth_detector.get_obstacle_points()
            
    #         if obstacles:
    #             # Publish obstacles
    #             payload = {
    #                 "t": time.time(),
    #                 "units": "mm", 
    #                 "items": obstacles
    #             }
                
    #             msg = String(data=json.dumps(payload))
    #             self.detection_pub.publish(msg)
                
    #             self.get_logger().debug(f"Published {len(obstacles)} depth obstacles")
                
    #     except Exception as e:
    #         self.get_logger().error(f"Error publishing depth obstacles: {e}")

    def cleanup(self):
        """Cleanup resources."""
        if hasattr(self, 'depth_detector') and self.depth_detector:
            self.depth_detector.cleanup()

# def main(args=None):
#     """Main function to run the AprilTag bridge."""
#     if not ROS2_AVAILABLE:
#         print("ROS2 not available. Please install ROS2 packages.")
#         return
    
#     rclpy.init(args=args)
    
#     try:
#         bridge = AprilTagBridge()
#         print("[AprilTag Bridge] Starting bridge node...")
#         print("[AprilTag Bridge] Listening for AprilTag detections...")
#         print("[AprilTag Bridge] Publishing to /detected_objects")
        
#         rclpy.spin(bridge)
        
#     except KeyboardInterrupt:
#         print("\n[AprilTag Bridge] Shutting down...")
#     finally:
#         if ROS2_AVAILABLE:
#             rclpy.shutdown()

def main(args=None):
    """Main function to run the AprilTag bridge."""
    if not ROS2_AVAILABLE:
        print("ROS2 not available. Please install ROS2 packages.")
        return
    
    rclpy.init(args=args)
    
    bridge = None
    try:
        bridge = AprilTagBridge()
        print("[AprilTag Bridge] Starting bridge node with depth obstacles...")
        print("[AprilTag Bridge] Listening for AprilTag detections...")
        print("[AprilTag Bridge] Publishing to /detected_objects")
        
        rclpy.spin(bridge)
        
    except KeyboardInterrupt:
        print("\n[AprilTag Bridge] Shutting down...")
    finally:
        if bridge:
            bridge.cleanup()
        if ROS2_AVAILABLE:
            rclpy.shutdown()

if __name__ == '__main__':
    main()
