# executor.py
import time, threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String as StringMsg
import json

from actions_xarm import XArmRunner

DEFAULT_HOVER_MM = 80
DEFAULT_PICK_RPY = [180, 0, 0]

# Named poses (edit to your workspace). Units: mm/deg
WORLD_POSES = {
    "home":             {"xyz_mm": [300,   0, 300], "rpy_deg": [180, 0, 0]},
    "bin_drop":         {"xyz_mm": [250, 250, 120], "rpy_deg": [180, 0, 0]},
    "table_pick_blue":  {"xyz_mm": [400,-150,  45], "rpy_deg": [180, 0, 0]},
}

class ObjectIndex(Node):
    """Caches latest pose per label (mm) from /detected_objects."""
    def __init__(self):
        super().__init__('object_index')
        self._lock = threading.Lock()
        self.latest_mm = {}  # label -> [x_mm, y_mm, z_mm]
        self.sub = self.create_subscription(StringMsg, '/detected_objects', self._on_msg, 10)

    def _on_msg(self, msg: StringMsg):
        try:
            data = json.loads(msg.data)
            units = data.get("units", "m")
            k = 1000.0 if units == "m" else 1.0
            items = data.get("items", [])
            with self._lock:
                for it in items:
                    lab = it.get("class")
                    pos = it.get("pos", [0,0,0])
                    if lab and isinstance(pos, list) and len(pos) == 3:
                        self.latest_mm[lab] = [float(pos[0])*k, float(pos[1])*k, float(pos[2])*k]
        except Exception:
            pass

    def wait_for(self, label: str, timeout=5.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self._lock:
                p = self.latest_mm.get(label)
            if p is not None:
                return p
            time.sleep(0.05)
        raise TimeoutError(f"Object '{label}' not seen on /detected_objects within {timeout}s")

class Executor:
    def __init__(self, arm_ip: str):
        self.runner = XArmRunner(arm_ip)
        self.world = WORLD_POSES
        self.hover_mm = DEFAULT_HOVER_MM
        self.pick_rpy = DEFAULT_PICK_RPY

        rclpy.init(args=None)
        self.obj_index = ObjectIndex()
        self._spin_thread = threading.Thread(target=rclpy.spin, args=(self.obj_index,), daemon=True)
        self._spin_thread.start()

    def _named(self, name: str):
        if name not in self.world:
            raise KeyError(f"Named pose '{name}' not in WORLD_POSES.")
        p = self.world[name]
        return p["xyz_mm"], p["rpy_deg"]

    def execute(self, plan: dict):
        print(f"[Plan] Goal: {plan.get('goal')}")
        self.runner.go_home()
        steps = plan.get("steps", [])

        for i, step in enumerate(steps, 1):
            act = step.get("action")
            print(f"[Plan] Step {i}/{len(steps)}: {act} {step}")

            if act == "MOVE_TO_NAMED":
                xyz, rpy = self._named(step["name"]); self.runner.move_pose(xyz, rpy)

            elif act == "APPROACH_NAMED":
                hover = float(step.get("hover_mm", self.hover_mm))
                xyz, rpy = self._named(step["name"])
                self.runner.move_pose([xyz[0], xyz[1], xyz[2] + hover], rpy)

            elif act == "MOVE_TO_POSE":
                pose = step["pose"]; self.runner.move_pose(pose["xyz_mm"], pose["rpy_deg"])

            elif act == "RETREAT_Z":
                self.runner.move_rel_z(float(step["dz_mm"]))

            elif act == "OPEN_GRIPPER":
                g = step.get("gripper", {})
                self.runner.open_gripper(g.get("position", 850), g.get("speed", 200), g.get("force", 50))

            elif act == "CLOSE_GRIPPER":
                g = step.get("gripper", {})
                self.runner.close_gripper(g.get("position", 200), g.get("speed", 100), g.get("force", 50))

            elif act in ("APPROACH_OBJECT", "MOVE_TO_OBJECT"):
                label = step["label"]
                hover = float(step.get("hover_mm", self.hover_mm))
                offset = step.get("offset_mm", [0.0, 0.0, 0.0])
                obj = self.obj_index.wait_for(label, timeout=step.get("timeout_sec", 5.0))  # [x_mm,y_mm,z_mm]
                target = [
                    obj[0] + float(offset[0]),
                    obj[1] + float(offset[1]),
                    obj[2] + float(offset[2]),
                ]
                rpy = self.pick_rpy
                if act == "APPROACH_OBJECT":
                    target[2] += hover
                self.runner.move_pose(target, rpy)

            elif act == "SLEEP":
                time.sleep(float(step["seconds"]))

            else:
                raise ValueError(f"Unsupported action: {act}")

        print("[Plan] Done.")

    def shutdown(self):
        try:
            rclpy.shutdown()
        except Exception:
            pass
        self.runner.disconnect()
