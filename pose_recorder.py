#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from xarm.wrapper import XArmAPI
import pyrealsense2 as rs
from ultralytics import YOLO
import numpy as np
import cv2
import time
import threading
from scipy.spatial.transform import Rotation as R
import csv
import os

CAMERA_OFFSET_MM = [0.0, 0.0, 22.5]
CAMERA_OFFSET_M  = [x/1000.0 for x in CAMERA_OFFSET_MM]

def get_transform_matrix(tx, ty, tz, rpy):
    rot = R.from_euler("xyz", rpy).as_matrix()
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = [tx, ty, tz]
    return T

def pose_to_transform(pose):
    # pose: (code, [x,y,z,roll,pitch,yaw])
    vals = pose[1]
    x,y,z,roll,pitch,yaw = vals if len(vals)==6 else (*vals,0,0,0)
    x/=1000; y/=1000; z/=1000
    rpy = np.deg2rad([roll,pitch,yaw])
    rot = R.from_euler("xyz", rpy).as_matrix()
    T = np.eye(4)
    T[:3,:3] = rot
    T[:3,3]  = [x,y,z]
    return T

class PoseRecorder(Node):
    def __init__(self):
        super().__init__('pose_recorder')

        # — XArm setup —
        self.arm = XArmAPI("192.168.1.241")
        self.arm.connect()
        self.arm.motion_enable(True)
        self.arm.set_mode(0)
        self.arm.set_state(0)
        self.arm.move_gohome(wait=True)

        # — RealSense setup —
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640,480,rs.format.bgr8,30)
        cfg.enable_stream(rs.stream.depth, 640,480,rs.format.z16, 30)
        self.pipeline.start(cfg)

        # — YOLO model —
        self.model = YOLO("yolov8n.pt")

        # — CSV logging setup —
        self.csv_path = os.path.expanduser("~/arm_pose_log.csv")
        new_file = not os.path.exists(self.csv_path)
        self.csv_file = open(self.csv_path, "a", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        if new_file:
            self.csv_writer.writerow([
                "timestamp",
                "joint1","joint2","joint3","joint4","joint5","joint6",
                "eef_x","eef_y","eef_z",
            ])

        # — Timer for periodic scan/record —
        self.timer = self.create_timer(1.0, self.scan_and_record)
        self.get_logger().info(f"PoseRecorder running → logging to {self.csv_path}")

    def scan_and_record(self):
        frames = self.pipeline.wait_for_frames()
        c = frames.get_color_frame(); d = frames.get_depth_frame()
        if not c or not d:
            self.get_logger().warn("No camera frame, skipping.")
            return

        color_image = np.asanyarray(c.get_data())
        intrin = d.profile.as_video_stream_profile().intrinsics

        # YOLO detect (inline for simplicity)
        results = self.model(color_image)[0]
        detected = []
        for box in results.boxes:
            cls_id = int(box.cls[0]); conf = float(box.conf[0])
            x1,y1,x2,y2 = [int(v) for v in box.xyxy[0]]
            cx,cy = (x1+x2)//2, (y1+y2)//2
            depth_m = d.get_distance(cx,cy)
            if depth_m == 0: continue
            # compute base-frame point
            code, tcp_pose = self.arm.get_position()
            T_f2c = get_transform_matrix(*CAMERA_OFFSET_M, [0,0,0])
            T_b2f = pose_to_transform((code, tcp_pose))
            T_b2c = T_b2f @ T_f2c

            pt_cam = rs.rs2_deproject_pixel_to_point(intrin, [cx,cy], depth_m)
            pt_base_h = T_b2c @ np.array([*pt_cam,1.0]).reshape(4,1)
            px,py,pz = pt_base_h[:3,0].tolist()
            detected.append({"class": self.model.names[cls_id], "pos":[px,py,pz], "conf":conf})

        # Record robot pose & joints
        code, pose = self.arm.get_position()      # [x(mm),y,z, r,p,y]
        joints = self.arm.get_servo_angle()[1]    # [j1..j6]
        x_m,y_m,z_m = [pose[i]/1000.0 for i in range(3)]
        row = [ time.time(), *[float(j) for j in joints], x_m, y_m, z_m ]
        self.csv_writer.writerow(row)
        self.csv_file.flush()

        self.get_logger().info(
            f"Logged joints={joints}  eef=[{x_m:.3f},{y_m:.3f},{z_m:.3f}]  objs={len(detected)}"
        )

    def destroy_node(self):
        self.csv_file.close()
        self.pipeline.stop()
        self.arm.move_gohome(wait=True)
        self.arm.disconnect()
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
