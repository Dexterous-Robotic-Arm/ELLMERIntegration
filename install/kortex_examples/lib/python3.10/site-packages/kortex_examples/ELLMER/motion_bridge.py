#!/usr/bin/env python3
# src/kortex_examples/ELLMER/motion_bridge.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ufactory_ellmer_msgs.msg import TargetCoordinatesArray
from moveit_msgs.srv import GetPositionIK
from rclpy.duration import Duration

class MotionBridge(Node):

    def __init__(self):
        super().__init__('motion_bridge')
        self.get_logger().info('🛠 MotionBridge with MoveIt IK ready')

        # 1) create IK client
        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('IK service /compute_ik not available!')
            raise RuntimeError('compute_ik missing')

        # 2) publisher for joint trajectories
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/ufactory/joint_trajectory_controller/joint_trajectory',
            10
        )

        # 3) subscribe to your labeled Cartesian targets
        self.create_subscription(
            TargetCoordinatesArray,
            '/target_coordinates',
            self.targets_cb,
            10
        )
        self.targets = []

    def targets_cb(self, msg: TargetCoordinatesArray):
        # cache all PoseStamped waypoints
        self.targets = []
        for t in msg.targets:
            ps = PoseStamped()
            ps.header.frame_id = 'base_link'    # or your planning frame
            ps.pose.position.x = t.position.x
            ps.pose.position.y = t.position.y
            ps.pose.position.z = t.position.z
            # convert roll/pitch/yaw → quaternion:
            from tf_transformations import quaternion_from_euler
            q = quaternion_from_euler(t.roll, t.pitch, t.yaw)
            ps.pose.orientation.x = q[0]
            ps.pose.orientation.y = q[1]
            ps.pose.orientation.z = q[2]
            ps.pose.orientation.w = q[3]
            self.targets.append(ps)

        # once we have targets, immediately plan & publish
        self.plan_and_publish()

    def plan_and_publish(self):
        traj = JointTrajectory()
        traj.joint_names = [f'joint{i}' for i in range(1,7)]
        time_acc = 0.0

        for idx, target in enumerate(self.targets):
            # build the service request
            req = GetPositionIK.Request()
            req.ik_request.group_name = 'ufactory_arm'       # your MoveIt group
            req.ik_request.pose_stamped = target
            req.ik_request.timeout.sec = 2
            # call IK
            future = self.ik_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            if not future.result() or future.result().error_code.val != future.result().error_code.SUCCESS:
                self.get_logger().error(f'IK failed for waypoint {idx}')
                continue

            solution = future.result().solution.joint_state.position
            # take first 6 joints:
            joints = list(solution[:6])

            # append to trajectory
            pt = JointTrajectoryPoint()
            pt.positions = joints
            time_acc += 1.0  # 1s per waypoint
            pt.time_from_start = Duration(seconds=int(time_acc)).to_msg()
            traj.points.append(pt)

        if traj.points:
            self.traj_pub.publish(traj)
            self.get_logger().info(f'✅ Published {len(traj.points)}-point traj via MoveIt IK')
        else:
            self.get_logger().warn('⚠️ No IK solutions—trajectory empty')

def main(args=None):
    rclpy.init(args=args)
    node = MotionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
