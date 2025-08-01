#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration

class IKClient(Node):
    def __init__(self):
        super().__init__('ik_client')
        self.cli = self.create_client(GetPositionIK, '/compute_ik')
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('IK service /compute_ik not available!')
            sys.exit(1)

    def call_ik(self, group, x, y, z, ox, oy, oz, ow):
        # build request
        req = GetPositionIK.Request()
        req.ik_request.group_name = group
        # current robot state (home)
        req.ik_request.robot_state.joint_state = JointState(
            header=Header(),
            name=[f'joint{i}' for i in range(1,7)],
            position=[0.0]*6
        )
        # target link (end‐effector)
        req.ik_request.ik_link_name = 'link_eef'
        # desired pose
        ps = PoseStamped()
        ps.header.frame_id = 'world'
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.x = ox
        ps.pose.orientation.y = oy
        ps.pose.orientation.z = oz
        ps.pose.orientation.w = ow
        req.ik_request.pose_stamped = ps
        req.ik_request.timeout = Duration(sec=1)
        # call
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    if len(sys.argv) != 9:
        print(f"Usage: {sys.argv[0]} GROUP X Y Z OX OY OZ OW")
        sys.exit(1)
    _, group, x, y, z, ox, oy, oz, ow = sys.argv
    rclpy.init()
    client = IKClient()
    resp = client.call_ik(group,
                          float(x),
                          float(y),
                          float(z),
                          float(ox),
                          float(oy),
                          float(oz),
                          float(ow))
    if resp.error_code.val == 1:
        js = resp.solution.joint_state
        print("IK SUCCESS →", list(zip(js.name, js.position)))
    else:
        print("IK ERROR", resp.error_code.val)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()