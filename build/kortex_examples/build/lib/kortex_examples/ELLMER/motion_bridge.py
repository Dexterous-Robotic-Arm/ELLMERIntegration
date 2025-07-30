#!/usr/bin/env python3
"""
motion_bridge.py  ·  Step‑1/2 glue between the ELLMER “brain” layer
(TargetCoordinatesArray messages) and the UFactory xArm 850 hardware.

✦  Subscribes :  /target_coordinates   (ufactory_ellmer_msgs/TargetCoordinatesArray)
✦  Services   :  /compute_ik (moveit_msgs/srv/GetPositionIK)        [default]
                 /ufactory/uf850/ik (xarm_msgs/srv/PositionIK)      [optional]
✦  Publishes  :  /joint_trajectory_controller/joint_trajectory
                 (trajectory_msgs/JointTrajectory)

All topic / service names can be overridden by ROS 2 parameters.
"""

from __future__ import annotations
import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ufactory_ellmer_msgs.msg import TargetCoordinatesArray

# MoveIt 2 GetPositionIK
from moveit_msgs.srv import GetPositionIK

# -----------------------------------------------------------------------------
#  Helpers
# -----------------------------------------------------------------------------
def tc_to_pose(tc) -> PoseStamped:
    """Convert TargetCoordinates to a stamped Pose for IK."""
    p = PoseStamped()
    p.header.frame_id = "base_link"       # same as xArm MoveIt SRDF
    p.pose.position = tc.position
    # roll‑pitch‑yaw → quaternion
    cr, sr = math.cos(tc.roll / 2), math.sin(tc.roll / 2)
    cp, sp = math.cos(tc.pitch / 2), math.sin(tc.pitch / 2)
    cy, sy = math.cos(tc.yaw / 2), math.sin(tc.yaw / 2)
    p.pose.orientation.w = cr * cp * cy + sr * sp * sy
    p.pose.orientation.x = sr * cp * cy - cr * sp * sy
    p.pose.orientation.y = cr * sp * cy + sr * cp * sy
    p.pose.orientation.z = cr * cp * sy - sr * sp * cy
    return p


# -----------------------------------------------------------------------------
#  Main node
# -----------------------------------------------------------------------------
class MotionBridge(Node):
    def __init__(self):
        super().__init__("motion_bridge")

        # ---------------- Parameters (override in launch file) ---------------
        self.declare_parameter("group_name",               "xarm")
        self.declare_parameter("ik_service_name",          "/compute_ik")
        self.declare_parameter("traj_topic",
                               "/joint_trajectory_controller/joint_trajectory")
        self.declare_parameter("move_time", 4.0)     # seconds per waypoint

        # ---------------- ROS entities ---------------------------------------
        self._traj_pub = self.create_publisher(
            JointTrajectory,
            self.get_parameter("traj_topic").get_parameter_value().string_value,
            10,
        )
        self.create_subscription(
            TargetCoordinatesArray, "/target_coordinates",
            self.cb_target_coordinates, 10
        )

        # IK client
        srv_name = self.get_parameter("ik_service_name")\
                        .get_parameter_value().string_value
        self._ik_client = self.create_client(GetPositionIK, srv_name)
        self.get_logger().info(
            f"🛠  MotionBridge ready —  IK service: {srv_name}, "
            f"traj_topic: {self._traj_pub.topic}"
        )

    # -------------------------------------------------------------------------
    #  Callback
    # -------------------------------------------------------------------------
    def cb_target_coordinates(self, msg: TargetCoordinatesArray):
        if not msg.targets:
            self.get_logger().warn("Received empty TargetCoordinatesArray")
            return

        # Build single JointTrajectory that concatenates all way‑points
        traj = JointTrajectory()
        traj.joint_names = [f"joint{i}" for i in range(1, 7)]
        t_from_start = 0.0

        for i, tc in enumerate(msg.targets):
            jt_point = self._ik_to_trajectory_point(tc)
            if jt_point is None:
                self.get_logger().error(f"IK failed for waypoint #{i}")
                return
            t_from_start += self.get_parameter("move_time").value
            jt_point.time_from_start = Duration(seconds=t_from_start).to_msg()
            traj.points.append(jt_point)

        self._traj_pub.publish(traj)
        self.get_logger().info(
            f"Sent trajectory with {len(traj.points)} segment(s) "
            f"(T_total={t_from_start:.1f}s)"
        )

    # -------------------------------------------------------------------------
    #  IK helper
    # -------------------------------------------------------------------------
    def _ik_to_trajectory_point(self, tc) -> JointTrajectoryPoint | None:
        pose = tc_to_pose(tc)
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.get_parameter("group_name").value
        req.ik_request.pose_stamped = pose
        req.ik_request.timeout = Duration(seconds=0.2).to_msg()

        if not self._ik_client.service_is_ready():
            self.get_logger().warn("Waiting for IK service...")
            if not self._ik_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("IK service not available")
                return None

        future = self._ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.3)
        if future.result() is None or not future.result().solution.joint_state.name:
            return None

        # Fill JointTrajectoryPoint
        pt = JointTrajectoryPoint()
        ordered = traj_joint_order(traj_names=pt.positions or
                                   [f"joint{i}" for i in range(1, 7)],
                                   ik_state=future.result().solution.joint_state)
        pt.positions = ordered
        return pt


# -------------------------------------------------------------------------
#  Utility
# -------------------------------------------------------------------------
def traj_joint_order(traj_names: List[str], ik_state) -> List[float]:
    """Re‑orders IK joint array to match trajectory joint_names order."""
    name_to_pos = dict(zip(ik_state.name, ik_state.position))
    return [name_to_pos[n] for n in traj_names]


# -------------------------------------------------------------------------
#  Boiler‑plate
# -------------------------------------------------------------------------
def main():
    rclpy.init()
    node = MotionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
