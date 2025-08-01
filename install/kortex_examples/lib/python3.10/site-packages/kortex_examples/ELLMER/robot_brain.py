#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String, Float32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ufactory_ellmer_msgs.msg import TargetCoordinatesArray
from kortex_examples.ELLMER.llm_interface import call_llm

class RobotBrain(Node):
    def __init__(self):
        super().__init__('robot_brain')

        # — State —
        self.latest_joints = [0.0]*6
        self.objects = []  # always set to a list

        # — Trajectory publisher —
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/ufactory/joint_trajectory_controller/joint_trajectory',
            10
        )

        # — Joint feedback subscriber —
        self.create_subscription(
            Float32MultiArray,
            '/ufactory/joint_feedback',
            self.joints_cb,
            10
        )

        # — User command subscriber —
        self.create_subscription(
            String,
            '/user_command',
            self.command_cb,
            10
        )

        # — Object‐position subscriber with transient‐local QoS —
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(
            TargetCoordinatesArray,
            '/target_coordinates',
            self.targets_cb,
            qos_profile=qos
        )

        # — Periodically re‐print what we have —
        self.create_timer(1.0, self._debug_objects)

        self.get_logger().info('RobotBrain ready, waiting for /user_command and /target_coordinates')

    def joints_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 6:
            self.latest_joints = list(msg.data[:6])

    def targets_cb(self, msg: TargetCoordinatesArray):
        self.objects = [
            [t.position.x, t.position.y, t.position.z]
            for t in msg.targets
        ]
        self.get_logger().info(f"[targets_cb] got {len(self.objects)} object(s): {self.objects}")

    def _debug_objects(self):
        # this will print every second so you can watch self.objects change
        self.get_logger().info(f"[DEBUG] current objects list: {self.objects}")

    def command_cb(self, msg: String):
        # ... your existing planning & publishing logic ...
        pass

def main(args=None):
    rclpy.init(args=args)
    node = RobotBrain()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
