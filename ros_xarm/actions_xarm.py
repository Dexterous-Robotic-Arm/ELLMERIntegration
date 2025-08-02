# actions_xarm.py
from xarm.wrapper import XArmAPI

class XArmRunner:
    def __init__(self, ip: str, cart_speed=100, cart_acc=500, joint_speed=20, joint_acc=50):
        self.arm = XArmAPI(ip, is_radian=False)
        self.arm.connect()
        self.arm.motion_enable(True)
        self.arm.set_mode(0)  # position mode
        self.arm.set_state(0) # ready

        self.arm.set_tcp_maxacc(2000)
        self.arm.set_position_speed(cart_speed)
        self.arm.set_position_acc(cart_acc)
        self.arm.set_joint_maxvel(joint_speed)
        self.arm.set_joint_maxacc(joint_acc)

        try:
            self.arm.set_gripper_enable(True)
            self.arm.set_gripper_mode(0)
        except Exception:
            pass

    def go_home(self):
        code = self.arm.move_gohome(wait=True)
        if code not in (0, None):
            raise RuntimeError(f"move_gohome error: {code}")

    def move_pose(self, xyz_mm, rpy_deg):
        code = self.arm.set_position(*xyz_mm, *rpy_deg, speed=None, mvacc=None, radius=0, wait=True)
        if code not in (0, None):
            raise RuntimeError(f"set_position error {code} → {xyz_mm}, {rpy_deg}")

    def move_rel_z(self, dz_mm):
        code, pose = self.arm.get_position()
        if code != 0:
            raise RuntimeError(f"get_position error {code}")
        xyz = [pose[0], pose[1], pose[2] + float(dz_mm)]
        rpy = [pose[3], pose[4], pose[5]]
        self.move_pose(xyz, rpy)

    def open_gripper(self, position=850, speed=200, force=50):
        try:
            self.arm.set_gripper_position(position=position, speed=speed, force=force, wait=True)
        except Exception as e:
            print(f"[WARN] open_gripper failed: {e}")

    def close_gripper(self, position=200, speed=100, force=50):
        try:
            self.arm.set_gripper_position(position=position, speed=speed, force=force, wait=True)
        except Exception as e:
            print(f"[WARN] close_gripper failed: {e}")

    def disconnect(self):
        try:
            self.arm.disconnect()
        except Exception:
            pass
