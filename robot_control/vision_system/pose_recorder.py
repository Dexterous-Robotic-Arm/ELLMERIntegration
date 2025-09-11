#!/usr/bin/env python3
"""
pose_recorder.py
Publishes April Tags+RealSense detections (meters, base frame) to /detected_objects.
Also logs joints/EEF to CSV. No planning/motion here.
"""

import json
import time
import os
import csv
import numpy as np
import logging

# Optional imports - only import if available
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("Warning: ROS2 not available, running in simulation mode")

try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False
    print("Warning: RealSense SDK not available")

# YOLO DISABLED - Using April Tags instead
# Note: YOLO code has been removed. This system now uses April Tags (TagStandard41h12)
# for object detection. Do not re-enable YOLO without explicit approval.

try:
    from april_tags_vision.april_tag_detector import AprilTagDetector
    APRILTAG_AVAILABLE = True
except ImportError:
    APRILTAG_AVAILABLE = False
    print("Warning: April Tags not available - build ROS2 package with: colcon build --packages-select april_tags_vision")

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

CAMERA_OFFSET_MM = [0.0, 0.0, 22.5]            # camera vs flange along tool Z
CAMERA_OFFSET_M  = [x/1000.0 for x in CAMERA_OFFSET_MM]
CONF_MIN = 0.35

# Coordinate stabilization parameters (optimized for speed)
COORDINATE_AVERAGING_SAMPLES = 2  # Reduced from 3 to 2 for speed
COORDINATE_STABILITY_THRESHOLD = 80.0  # Increased from 50 to 80 for faster convergence

def get_transform_matrix(tx, ty, tz, rpy_xyz_rad):
    if not SCIPY_AVAILABLE:
        print("Warning: SciPy not available, using identity matrix")
        return np.eye(4)
    rot = R.from_euler("xyz", rpy_xyz_rad).as_matrix()
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = [tx, ty, tz]
    return T

def pose_to_transform(pose):
    # pose: (code, [x(mm),y(mm),z(mm),roll,pitch,yaw] deg)
    if not SCIPY_AVAILABLE:
        print("Warning: SciPy not available, using identity matrix")
        return np.eye(4)
    vals = pose[1]
    x,y,z,roll,pitch,yaw = vals if len(vals)==6 else (*vals,0,0,0)
    x/=1000.0; y/=1000.0; z/=1000.0
    rpy = np.deg2rad([roll,pitch,yaw])
    rot = R.from_euler("xyz", rpy).as_matrix()
    T = np.eye(4)
    T[:3,:3] = rot
    T[:3,3]  = [x,y,z]
    return T

class PoseRecorder(Node if ROS2_AVAILABLE else object):
    def __init__(self, standalone=True):
        # Initialize ROS2 node if available and in standalone mode
        if ROS2_AVAILABLE and standalone:
            try:
                super().__init__('pose_recorder')
                self.det_pub = self.create_publisher(String, '/detected_objects', 10)
                self.standalone = True
            except Exception as e:
                print(f"ROS2 node creation failed: {e}")
                self.det_pub = None
                self.standalone = False
        else:
            self.det_pub = None
            self.standalone = False
            if not standalone:
                print("Running in integrated mode - no ROS2 publisher")
            else:
                print("Running in simulation mode - no ROS2 publisher")

        # xArm - only if available
        if XARM_AVAILABLE:
            self.arm = XArmAPI(os.environ.get("XARM_IP", "192.168.1.241"))
            self.arm.connect()
            self.arm.motion_enable(True)
            self.arm.set_mode(0)
            self.arm.set_state(0)
            # Don't move to home - stay in current position for camera-mounted robot
            print("Vision system connected to robot - staying in current position")
        else:
            self.arm = None
            print("Running in simulation mode - no XArm connection")

        # RealSense - only if available and in standalone mode
        if REALSENSE_AVAILABLE and standalone:
            try:
                self.pipeline = rs.pipeline()
                cfg = rs.config()
                cfg.enable_stream(rs.stream.color, 640,480,rs.format.bgr8,30)
                cfg.enable_stream(rs.stream.depth, 640,480,rs.format.z16,30)
                self.pipeline.start(cfg)
                print("✅ RealSense camera connected")
            except Exception as e:
                print(f"❌ RealSense camera failed: {e}")
                self.pipeline = None
        else:
            self.pipeline = None
            if not standalone:
                print("Running in integrated mode - camera handled by ROS2 publisher")
            else:
                print("Running in simulation mode - no RealSense camera")

        # April Tags detector - only if available
        if APRILTAG_AVAILABLE:
            self.april_detector = AprilTagDetector()
            print("🏷️ April Tags detector initialized")
        else:
            self.april_detector = None
            print("Running in simulation mode - no April Tags detector")

        # CSV logging
        self.csv_path = os.path.expanduser("~/arm_pose_log.csv")
        new_file = not os.path.exists(self.csv_path)
        self.csv_file = open(self.csv_path, "a", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        if new_file:
            self.csv_writer.writerow([
                "timestamp","joint1","joint2","joint3","joint4","joint5","joint6","eef_x","eef_y","eef_z"
            ])

        if self.standalone:
            self.timer = self.create_timer(1.0, self.scan_and_publish)
            self.get_logger().info(f"PoseRecorder running → logging to {self.csv_path}")
        else:
            print(f"PoseRecorder initialized in integrated mode → logging to {self.csv_path}")
        
        # Coordinate stabilization
        self.coordinate_history = {}  # Store recent detections per object class

    def _add_to_coordinate_history(self, class_name, position, confidence):
        """Add detection to coordinate history and return averaged position if stable."""
        if class_name not in self.coordinate_history:
            self.coordinate_history[class_name] = []
        
        # Add new detection
        self.coordinate_history[class_name].append({
            'pos': position,
            'conf': confidence,
            'timestamp': time.time()
        })
        
        # Keep only recent detections (reduced time window for speed)
        current_time = time.time()
        self.coordinate_history[class_name] = [
            det for det in self.coordinate_history[class_name]
            if current_time - det['timestamp'] < 2.0  # Keep last 2 seconds (reduced from 5)
        ]
        
        # Need at least 2 detections to average
        if len(self.coordinate_history[class_name]) < 2:
            return position
        
        # Calculate average position
        positions = [det['pos'] for det in self.coordinate_history[class_name]]
        avg_pos = [
            sum(pos[i] for pos in positions) / len(positions)
            for i in range(3)
        ]
        
        # Check if positions are stable (low variation)
        max_variation = 0
        for i in range(3):
            values = [pos[i] for pos in positions]
            variation = max(values) - min(values)
            max_variation = max(max_variation, variation)
        
        if max_variation <= COORDINATE_STABILITY_THRESHOLD:
            print(f"[Vision] Stable coordinates for {class_name}: avg={avg_pos}, variation={max_variation:.1f}mm")
            return avg_pos
        else:
            print(f"[Vision] Unstable coordinates for {class_name}: variation={max_variation:.1f}mm > {COORDINATE_STABILITY_THRESHOLD}mm, using latest")
            return position

    def scan_and_publish(self):
        if not self.pipeline:
            if not self.standalone:
                print("[Vision] No camera pipeline in integrated mode - skipping detection")
            else:
                print("[Vision] No camera pipeline - skipping detection")
            return
            
        frames = self.pipeline.wait_for_frames()
        c = frames.get_color_frame(); d = frames.get_depth_frame()
        if not c or not d:
            if self.standalone:
                self.get_logger().warn("No camera frame, skipping.")
            else:
                print("[Vision] No camera frame, skipping.")
            return

        color = np.asanyarray(c.get_data())
        intrin = d.profile.as_video_stream_profile().intrinsics

        # Set camera calibration for April Tags detector
        if self.april_detector and self.april_detector.camera_matrix is None:
            camera_matrix, dist_coeffs = self.april_detector.get_camera_matrix_from_realsense(d)
            if camera_matrix is not None:
                self.april_detector.set_camera_calibration(camera_matrix, dist_coeffs)

        # Detect April Tags
        detected = []
        if self.april_detector:
            tag_detections = self.april_detector.detect_tags(color)
            
            for tag in tag_detections:
                tag_id = tag["tag_id"]
                conf = tag["confidence"]
                if conf < CONF_MIN:
                    continue
                
                # Get tag center
                cx, cy = tag["center"]
                
                # Get depth at the center of the tag
                depth_m = d.get_distance(cx, cy)
                if depth_m <= 0.0:
                    continue

                code, tcp_pose = self.arm.get_position()
                if code != 0:
                    continue
                
                # T_f2c = get_transform_matrix(*CAMERA_OFFSET_M, [0,0,0])  # REMOVED - NO CAMERA OFFSETS
                T_f2c = get_transform_matrix(0.0, 0.0, 0.0, [0,0,0])  # NO OFFSETS - DIRECT POSITIONING
                T_b2f = pose_to_transform((code, tcp_pose))
                T_b2c = T_b2f @ T_f2c

                pt_cam = rs.rs2_deproject_pixel_to_point(intrin, [cx,cy], depth_m)  # meters
                pt_base_h = T_b2c @ np.array([*pt_cam,1.0]).reshape(4,1)
                px,py,pz = pt_base_h[:3,0].tolist()
                # Convert from meters to millimeters for robot coordinates
                px_mm, py_mm, pz_mm = px * 1000.0, py * 1000.0, pz * 1000.0
                
                # Debug: Check for obviously wrong coordinates
                if abs(px_mm) > 5000 or abs(py_mm) > 5000 or abs(pz_mm) > 5000:
                    print(f"[Vision] WARNING: Suspicious coordinates detected!")
                    print(f"[Vision]   Camera point: {pt_cam}")
                    print(f"[Vision]   Robot pose: {tcp_pose}")
                    print(f"[Vision]   Depth: {depth_m:.3f}m")
                    print(f"[Vision]   Final coordinates: [{px_mm:.1f}, {py_mm:.1f}, {pz_mm:.1f}]mm")
                    print(f"[Vision]   Skipping this detection due to invalid coordinates")
                    continue
                
                # Apply coordinate stabilization
                raw_position = [px_mm, py_mm, pz_mm]
                
                # Map tag ID to object name
                object_mapping = {
                    0: "bottle", 1: "book", 2: "cup", 3: "pen", 4: "phone",
                    5: "laptop", 6: "notebook", 7: "stapler", 8: "keyboard",
                    9: "mouse", 10: "calculator"
                }
                object_name = object_mapping.get(tag_id, f"unknown_object_{tag_id}")
                class_name = f"april_tag_{tag_id}_{object_name}"
                
                stabilized_position = self._add_to_coordinate_history(class_name, raw_position, conf)
                
                print(f"[Vision] April Tag {tag_id} raw: [{px_mm:.1f}, {py_mm:.1f}, {pz_mm:.1f}]mm, stabilized: [{stabilized_position[0]:.1f}, {stabilized_position[1]:.1f}, {stabilized_position[2]:.1f}]mm (conf: {conf:.3f})")
                
                # Add pixel coordinates for visual servoing
                detected.append({
                    "class": class_name,
                    "tag_id": tag_id,
                    "pos": stabilized_position, 
                    "conf": conf,
                    "pixel_center": [cx, cy],  # Center of April Tag in pixels
                    "image_size": [color.shape[1], color.shape[0]],  # [width, height]
                    "tag_family": tag["tag_family"],
                    "pose_3d": tag.get("pose_3d", None)  # Include 3D pose if available
                })

        payload = {"t": time.time(), "units": "mm", "items": detected}
        if self.det_pub:
            self.det_pub.publish(String(data=json.dumps(payload)))
        else:
            print(f"[Vision] Detected objects: {detected}")

        # log joints/EEF
        code, pose = self.arm.get_position()
        joints = self.arm.get_servo_angle()[1]
        x_m,y_m,z_m = [pose[i]/1000.0 for i in range(3)]
        row = [time.time(), *[float(j) for j in joints], x_m, y_m, z_m]
        self.csv_writer.writerow(row); self.csv_file.flush()

        if self.standalone:
            self.get_logger().info(f"objs={len(detected)}  eef=[{x_m:.3f},{y_m:.3f},{z_m:.3f}]")
        else:
            print(f"[Vision] objs={len(detected)}  eef=[{x_m:.3f},{y_m:.3f},{z_m:.3f}]")

    def destroy_node(self):
        self.csv_file.close()
        if self.pipeline:
            self.pipeline.stop()
        if hasattr(self, 'arm') and self.arm:
            self.arm.move_gohome(wait=True)
            self.arm.disconnect()
        if self.standalone:
            super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PoseRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()
