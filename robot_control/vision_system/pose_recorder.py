#!/usr/bin/env python3
"""
pose_recorder.py
Publishes YOLO+RealSense detections (meters, base frame) to /detected_objects.
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

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("Warning: YOLO not available")

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

        # YOLO - only if available
        if YOLO_AVAILABLE:
            self.model = YOLO("yolov8n.pt")
            print("🖥️ YOLO model loaded - using CPU for inference")
        else:
            self.model = None
            print("Running in simulation mode - no YOLO model")

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

        # Try GPU first, fallback to CPU if CUDA issues
        try:
            results = self.model(color, device='cuda')[0]
        except Exception:
            # Fallback to CPU if GPU fails
            results = self.model(color, device='cpu')[0]
        detected = []
        for box in results.boxes:
            cls_id = int(box.cls[0]); conf = float(box.conf[0])
            if conf < CONF_MIN: 
                continue
            x1,y1,x2,y2 = [int(v) for v in box.xyxy[0]]
            cx,cy = (x1+x2)//2, (y1+y2)//2
            depth_m = d.get_distance(cx,cy)
            if depth_m <= 0.0: 
                continue

            code, tcp_pose = self.arm.get_position()
            if code != 0: 
                continue
            T_f2c = get_transform_matrix(*CAMERA_OFFSET_M, [0,0,0])
            T_b2f = pose_to_transform((code, tcp_pose))
            T_b2c = T_b2f @ T_f2c

            pt_cam = rs.rs2_deproject_pixel_to_point(intrin, [cx,cy], depth_m)  # meters
            pt_base_h = T_b2c @ np.array([*pt_cam,1.0]).reshape(4,1)
            px,py,pz = pt_base_h[:3,0].tolist()
            detected.append({"class": self.model.names[cls_id], "pos":[px,py,pz], "conf":conf})

        payload = {"t": time.time(), "units": "m", "items": detected}
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
