#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import csv, os

try:
    from xarm import XArmAPI
    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False

class PoseRecorder(Node):
    def __init__(self):
        super().__init__('pose_recorder')

        # CSV file setup
        self.csv_path = os.path.expanduser('~/arm_pose_log.csv')
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, 'w') as f:
                w = csv.writer(f)
                w.writerow([
                    'timestamp',
                    'j1','j2','j3','j4','j5','j6',
                    'x','y','z','roll','pitch','yaw'
                ])

        # State
        self.latest_joints = [0.0]*6
        self.use_fk = False

        # Try to connect SDK
        if SDK_AVAILABLE:
            arm_ip = os.environ.get('XARM_IP','192.168.1.241')
            try:
                self.arm = XArmAPI(arm_ip)
                self.arm.connect()
                self.arm.motion_enable(True)
                self.arm.set_mode(0)
                self.arm.set_state(0)
                self.use_fk = True
                self.get_logger().info(f'Connected to xArm at {arm_ip}')
            except Exception as e:
                self.get_logger().warn(f'Could not connect xArmSDK ({e}); falling back to joints only')
        else:
            self.get_logger().warn('xArmSDK not installed; falling back to joints only')

        # Subscription for feedback
        self.create_subscription(
            Float32MultiArray,
            '/ufactory/joint_feedback',
            self.joints_cb,
            10
        )

        # 1 Hz timer
        self.create_timer(1.0, self.record_pose)
        self.get_logger().info(f'🖊️ PoseRecorder initialized, logging to {self.csv_path}')

    def joints_cb(self, msg):
        if len(msg.data) >= 6:
            self.latest_joints = list(msg.data[:6])

    def record_pose(self):
        ts = self.get_clock().now().to_msg().sec

        # default pose
        fk = [0.0]*6
        if self.use_fk:
            ret, pose = self.arm.get_position()
            if ret == 0:
                fk = pose
            else:
                self.get_logger().warn(f'FK error code {ret}; logging zeros')

        row = [ts] + self.latest_joints + fk
        with open(self.csv_path, 'a') as f:
            csv.writer(f).writerow(row)

        self.get_logger().info(f'Logged t={ts} joints={self.latest_joints} fk={fk}')

    def destroy_node(self):
        if self.use_fk:
            try: self.arm.disconnect()
            except: pass
        return super().destroy_node()

def main():
    rclpy.init()
    node = PoseRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
