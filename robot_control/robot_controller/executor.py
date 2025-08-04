# executor.py
import time, threading
import json
import logging

# Optional ROS2 imports - only import if available
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String as StringMsg
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("Warning: ROS2 not available, running in simulation mode")

from .actions_xarm import XArmRunner

DEFAULT_HOVER_MM = 80
DEFAULT_PICK_RPY = [180, 0, 0]

# Safety limits for real-life testing
MAX_EXECUTION_TIME = 300  # seconds - maximum time for any plan
MAX_STEPS_PER_PLAN = 50   # maximum number of steps in a plan
SAFETY_CHECK_INTERVAL = 0.5  # seconds between safety checks

# Named poses (edit to your workspace). Units: mm/deg
WORLD_POSES = {
    "home":             {"xyz_mm": [300,   0, 300], "rpy_deg": [180, 0, 0]},
    "bin_drop":         {"xyz_mm": [250, 250, 120], "rpy_deg": [180, 0, 0]},
    "table_pick_blue":  {"xyz_mm": [400,-150,  45], "rpy_deg": [180, 0, 0]},
}

class ObjectIndex:
    """Caches latest pose per label (mm) from /detected_objects."""
    def __init__(self):
        self._lock = threading.Lock()
        self.latest_mm = {}  # label -> [x_mm, y_mm, z_mm]
        
        # Only setup ROS2 subscription if available
        if ROS2_AVAILABLE:
            super().__init__('object_index')
            self.sub = self.create_subscription(StringMsg, '/detected_objects', self._on_msg, 10)
        else:
            self.sub = None

    def _on_msg(self, msg):
        try:
            if hasattr(msg, 'data'):
                data = json.loads(msg.data)
            else:
                data = msg  # Handle direct dict input
            units = data.get("units", "m")
            k = 1000.0 if units == "m" else 1.0
            items = data.get("items", [])
            with self._lock:
                for it in items:
                    lab = it.get("class")
                    pos = it.get("pos", [0,0,0])
                    if lab and isinstance(pos, list) and len(pos) == 3:
                        self.latest_mm[lab] = [float(pos[0])*k, float(pos[1])*k, float(pos[2])*k]
        except Exception as e:
            print(f"Error processing message: {e}")
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

class TaskExecutor:
    def __init__(self, arm_ip: str, world_yaml: str = None, sim: bool = False, dry_run: bool = False):
        self.runner = XArmRunner(arm_ip)
        self.world = WORLD_POSES
        self.hover_mm = DEFAULT_HOVER_MM
        self.pick_rpy = DEFAULT_PICK_RPY
        self.sim = sim
        self.dry_run = dry_run
        
        # Safety monitoring
        self.execution_start_time = None
        self.step_count = 0
        self.error_count = 0
        self.max_errors = 3  # Stop after this many consecutive errors

        # Load world poses from YAML if provided
        if world_yaml:
            try:
                import yaml
                with open(world_yaml, 'r') as f:
                    world_data = yaml.safe_load(f)
                    poses = world_data.get('poses', {})
                    for name, pose_data in poses.items():
                        self.world[name] = {
                            "xyz_mm": pose_data.get("xyz_mm", [0, 0, 0]),
                            "rpy_deg": pose_data.get("rpy_deg", [180, 0, 0])
                        }
            except Exception as e:
                print(f"[WARN] Failed to load world YAML {world_yaml}: {e}")

        # Initialize ROS2 only if available
        if ROS2_AVAILABLE:
            rclpy.init(args=None)
            self.obj_index = ObjectIndex()
            self._spin_thread = threading.Thread(target=rclpy.spin, args=(self.obj_index,), daemon=True)
            self._spin_thread.start()
        else:
            self.obj_index = ObjectIndex()
            self._spin_thread = None

    def _named(self, name: str):
        if name not in self.world:
            raise KeyError(f"Named pose '{name}' not in WORLD_POSES.")
        p = self.world[name]
        return p["xyz_mm"], p["rpy_deg"]

    def _validate_plan_safety(self, plan: dict):
        """Validate plan for safety concerns"""
        steps = plan.get("steps", [])
        
        # Check plan length
        if len(steps) > MAX_STEPS_PER_PLAN:
            raise ValueError(f"Plan has {len(steps)} steps, maximum allowed is {MAX_STEPS_PER_PLAN}")
        
        # Check for dangerous patterns
        dangerous_actions = []
        for i, step in enumerate(steps):
            action = step.get("action", "")
            
            # Check for large movements
            if action in ("MOVE_TO_POSE", "MOVE_TO_OBJECT", "APPROACH_OBJECT"):
                if "pose" in step:
                    pose = step["pose"]
                    xyz = pose.get("xyz_mm", [0, 0, 0])
                    # Check if movement is reasonable
                    if any(abs(x) > 500 for x in xyz):
                        dangerous_actions.append(f"Step {i+1}: Large movement to {xyz}")
            
            # Check for high-speed movements
            if "speed" in step and step["speed"] > 200:
                dangerous_actions.append(f"Step {i+1}: High speed {step['speed']}")
        
        if dangerous_actions:
            print("[WARN] Potentially dangerous actions detected:")
            for action in dangerous_actions:
                print(f"  - {action}")
            if not self.sim:
                print("[WARN] Consider reviewing plan before execution")

    def _check_execution_timeout(self):
        """Check if execution has exceeded maximum time"""
        if self.execution_start_time and time.time() - self.execution_start_time > MAX_EXECUTION_TIME:
            raise RuntimeError(f"Execution timeout after {MAX_EXECUTION_TIME} seconds")

    def _check_safety_status(self):
        """Periodic safety status check"""
        try:
            safety_status = self.runner.get_safety_status()
            if safety_status["emergency_stop"]:
                raise RuntimeError("Emergency stop is active")
        except Exception as e:
            print(f"[WARN] Safety check failed: {e}")

    def execute(self, plan: dict):
        print(f"[Plan] Goal: {plan.get('goal')}")
        
        # Validate plan safety
        self._validate_plan_safety(plan)
        
        # Check gripper status before starting
        gripper_status = self.runner.get_gripper_status()
        print(f"[Plan] Gripper status: {gripper_status}")
        
        # Initialize execution monitoring
        self.execution_start_time = time.time()
        self.step_count = 0
        self.error_count = 0
        
        if not self.dry_run:
            self.runner.go_home()
        
        steps = plan.get("steps", [])

        for i, step in enumerate(steps, 1):
            self.step_count = i
            
            # Safety checks
            self._check_execution_timeout()
            if i % 5 == 0:  # Check safety every 5 steps
                self._check_safety_status()
            
            act = step.get("action")
            print(f"[Plan] Step {i}/{len(steps)}: {act} {step}")

            try:
                if act == "MOVE_TO_NAMED":
                    xyz, rpy = self._named(step["name"])
                    if not self.dry_run:
                        self.runner.move_pose(xyz, rpy)

                elif act == "APPROACH_NAMED":
                    hover = float(step.get("hover_mm", self.hover_mm))
                    xyz, rpy = self._named(step["name"])
                    target_xyz = [xyz[0], xyz[1], xyz[2] + hover]
                    if not self.dry_run:
                        self.runner.move_pose(target_xyz, rpy)

                elif act == "MOVE_TO_POSE":
                    pose = step["pose"]
                    if not self.dry_run:
                        self.runner.move_pose(pose["xyz_mm"], pose["rpy_deg"])

                elif act == "RETREAT_Z":
                    dz_mm = float(step["dz_mm"])
                    # Limit retreat distance for safety
                    if abs(dz_mm) > 150:
                        print(f"[WARN] Large retreat {dz_mm}mm, limiting to 150mm")
                        dz_mm = 150 if dz_mm > 0 else -150
                    if not self.dry_run:
                        self.runner.move_rel_z(dz_mm)

                # Enhanced gripper actions with safety
                elif act == "OPEN_GRIPPER":
                    g = step.get("gripper", {})
                    position = g.get("position")
                    speed = min(g.get("speed", 200), 300)  # Limit speed
                    force = min(g.get("force", 50), 100)   # Limit force
                    if not self.dry_run:
                        success = self.runner.open_gripper(position, speed, force)
                        if not success:
                            self.error_count += 1
                            print(f"[WARN] Step {i} failed: OPEN_GRIPPER")

                elif act == "CLOSE_GRIPPER":
                    g = step.get("gripper", {})
                    position = g.get("position")
                    speed = min(g.get("speed", 100), 200)  # Limit speed
                    force = min(g.get("force", 50), 100)   # Limit force
                    if not self.dry_run:
                        success = self.runner.close_gripper(position, speed, force)
                        if not success:
                            self.error_count += 1
                            print(f"[WARN] Step {i} failed: CLOSE_GRIPPER")

                elif act == "SET_GRIPPER_POSITION":
                    position = step.get("position", 525)
                    speed = min(step.get("speed", 150), 300)  # Limit speed
                    force = min(step.get("force", 50), 100)   # Limit force
                    if not self.dry_run:
                        success = self.runner.set_gripper_position(position, speed, force)
                        if not success:
                            self.error_count += 1
                            print(f"[WARN] Step {i} failed: SET_GRIPPER_POSITION")

                elif act == "GRIPPER_GRASP":
                    target_position = step.get("target_position", 200)
                    speed = min(step.get("speed", 100), 200)  # Limit speed
                    force = min(step.get("force", 50), 100)   # Limit force
                    timeout = min(step.get("timeout", 5.0), 10.0)  # Limit timeout
                    if not self.dry_run:
                        success = self.runner.gripper_grasp(target_position, speed, force, timeout)
                        if not success:
                            self.error_count += 1
                            print(f"[WARN] Step {i} failed: GRIPPER_GRASP")

                elif act == "GRIPPER_RELEASE":
                    target_position = step.get("target_position", 850)
                    speed = min(step.get("speed", 200), 300)  # Limit speed
                    force = min(step.get("force", 50), 100)   # Limit force
                    if not self.dry_run:
                        success = self.runner.gripper_release(target_position, speed, force)
                        if not success:
                            self.error_count += 1
                            print(f"[WARN] Step {i} failed: GRIPPER_RELEASE")

                elif act == "GRIPPER_HALF_OPEN":
                    speed = min(step.get("speed", 150), 300)  # Limit speed
                    force = min(step.get("force", 50), 100)   # Limit force
                    if not self.dry_run:
                        success = self.runner.gripper_half_open(speed, force)
                        if not success:
                            self.error_count += 1
                            print(f"[WARN] Step {i} failed: GRIPPER_HALF_OPEN")

                elif act == "GRIPPER_SOFT_CLOSE":
                    speed = min(step.get("speed", 50), 100)   # Limit speed
                    force = min(step.get("force", 30), 50)    # Limit force
                    if not self.dry_run:
                        success = self.runner.gripper_soft_close(speed, force)
                        if not success:
                            self.error_count += 1
                            print(f"[WARN] Step {i} failed: GRIPPER_SOFT_CLOSE")

                elif act == "GRIPPER_TEST":
                    cycles = min(step.get("cycles", 3), 5)    # Limit cycles
                    delay = min(step.get("delay", 1.0), 2.0)  # Limit delay
                    if not self.dry_run:
                        success = self.runner.gripper_cycle_test(cycles, delay)
                        if not success:
                            self.error_count += 1
                            print(f"[WARN] Step {i} failed: GRIPPER_TEST")

                elif act in ("APPROACH_OBJECT", "MOVE_TO_OBJECT"):
                    # Handle both single label and multiple labels
                    labels = step.get("labels", [])
                    label = step.get("label")
                    if label and not labels:
                        labels = [label]
                    
                    if not labels:
                        raise ValueError(f"Step {i}: No label or labels specified for {act}")
                    
                    # Use first label for now (could be enhanced for multi-object selection)
                    target_label = labels[0]
                    
                    hover = float(step.get("hover_mm", self.hover_mm))
                    offset = step.get("offset_mm", [0.0, 0.0, 0.0])
                    
                    # Limit hover and offset for safety
                    hover = min(hover, 200)  # Max 200mm hover
                    offset = [min(max(x, -100), 100) for x in offset]  # Limit offset to ±100mm
                    
                    if not self.dry_run:
                        obj = self.obj_index.wait_for(target_label, timeout=step.get("timeout_sec", 5.0))
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
                    sleep_time = float(step["seconds"])
                    # Limit sleep time for safety
                    if sleep_time > 30:
                        print(f"[WARN] Long sleep {sleep_time}s, limiting to 30s")
                        sleep_time = 30
                    if not self.dry_run:
                        time.sleep(sleep_time)

                else:
                    raise ValueError(f"Unsupported action: {act}")

                # Reset error count on successful step
                self.error_count = 0

            except Exception as e:
                self.error_count += 1
                print(f"[ERROR] Step {i} failed: {e}")
                
                # Stop execution if too many consecutive errors
                if self.error_count >= self.max_errors:
                    print(f"[ERROR] Stopping execution after {self.max_errors} consecutive errors")
                    break
                
                if not self.sim:
                    print("[ERROR] Stopping execution due to error")
                    break

        # Final safety check
        execution_time = time.time() - self.execution_start_time if self.execution_start_time else 0
        print(f"[Plan] Done. Execution time: {execution_time:.1f}s, Steps completed: {self.step_count}")

    def get_execution_stats(self):
        """Get execution statistics"""
        execution_time = time.time() - self.execution_start_time if self.execution_start_time else 0
        return {
            "execution_time": execution_time,
            "steps_completed": self.step_count,
            "error_count": self.error_count,
            "safety_status": self.runner.get_safety_status()
        }

    def shutdown(self):
        try:
            if ROS2_AVAILABLE:
                rclpy.shutdown()
        except Exception:
            pass
        self.runner.disconnect()
