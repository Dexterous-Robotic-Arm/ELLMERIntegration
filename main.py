#!/usr/bin/env python3
import os, sys, json, time, argparse, threading, subprocess
from pathlib import Path

# --- import path bootstrap (adjust if needed) ---
HERE = Path(__file__).resolve().parent
CANDIDATE_DIRS = [
    HERE,  # repo root
    HERE / "llm_planning_gemini" / "custom_gemini_examples",  # planner_llm.py
    HERE / "ros_xarm",  # if executor.py lives here
    HERE / "src",
]
for d in CANDIDATE_DIRS:
    if d.exists():
        sys.path.insert(0, str(d))


# --- imports (will print SDK_VERSION from planner_llm if you put it there) ---
from planner_llm import plan_with_gemini, plan_fallback
from executor import Executor

# Optional helpers
try:
    from utils_schema import load_schema_from_md, validate_plan
except Exception:
    load_schema_from_md = None
    validate_plan = None
try:
    from utils_plan import normalize_plan, enforce_policies
except Exception:
    normalize_plan = enforce_policies = None

# YAML for pose names
try:
    import yaml
except Exception:
    yaml = None

# ROS wait helper
import rclpy
from rclpy.node import Node
from std_msgs.msg import String as StringMsg

def eprint(msg: str):
    sys.stderr.write(msg.rstrip() + "\n")
    sys.stderr.flush()

class DetectionWaiter(Node):
    def __init__(self, topic="/detected_objects", min_items=0):
        super().__init__('detection_waiter')
        self._min = int(min_items)
        self._ready = False
        self._sub = self.create_subscription(StringMsg, topic, self._on, 10)
    def _on(self, msg: StringMsg):
        try:
            data = json.loads(msg.data)
            items = data.get("items", []) or []
            if len(items) >= self._min:
                self._ready = True
        except Exception:
            pass
    def wait(self, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self._ready:
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return self._ready

def wait_for_detections(timeout_sec: float, min_items: int) -> bool:
    rclpy.init(args=None)
    node = DetectionWaiter(min_items=min_items)
    try:
        ok = node.wait(timeout_sec)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return ok

def load_pose_names(world_yaml: str | None) -> list[str]:
    if world_yaml and yaml:
        p = Path(world_yaml)
        if p.exists():
            try:
                data = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
                return list((data.get("poses") or {}).keys())
            except Exception:
                pass
    try:
        from executor import WORLD_POSES as _WORLD_POSES  # type: ignore
        return list(_WORLD_POSES.keys())
    except Exception:
        return []

def build_executor(ip: str, world_yaml: str | None, sim: bool, dry_run: bool) -> Executor:
    try:
        return Executor(ip, world_yaml=world_yaml, sim=sim, dry_run=dry_run)  # type: ignore[arg-type]
    except TypeError:
        return Executor(ip)  # type: ignore[call-arg]

def spawn_vision(vision_script: str) -> subprocess.Popen:
    cmd = [sys.executable, vision_script]
    eprint(f"[Vision] Spawning: {' '.join(cmd)}")
    return subprocess.Popen(cmd, start_new_session=True)

def stop_process_tree(proc: subprocess.Popen):
    if proc and proc.poll() is None:
        eprint("[Vision] Terminating vision process...")
        try:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                eprint("[Vision] Killing vision process...")
                proc.kill()
        except Exception:
            pass

def get_task_from_user() -> str:
    sys.stderr.write("Task: ")
    sys.stderr.flush()
    try:
        return input().strip()
    except EOFError:
        return ""

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task")
    ap.add_argument("--interactive", "-i", action="store_true")
    ap.add_argument("--loop", "-l", action="store_true")
    ap.add_argument("--use-fallback", action="store_true")
    ap.add_argument("--world", default=os.environ.get("WORLD_YAML", "world_model.yaml"))
    ap.add_argument("--contract", default=os.environ.get("ACTION_CONTRACT_MD", "llm_planning_gemini/custom_gemini_examples/custom_gpt_action_schema.md"))
    ap.add_argument("--no-validate", action="store_true")
    ap.add_argument("--ip", default=os.environ.get("XARM_IP", "192.168.1.241"))
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--spawn-vision", dest="spawn_vision", action="store_true", default=True)
    ap.add_argument("--no-spawn-vision", dest="spawn_vision", action="store_false")
    ap.add_argument("--vision-script", default=str(HERE / "ros_xarm" / "pose_recorder.py"), help="ros_xarm/pose_recorder.py")    
    ap.add_argument("--wait-detections", type=float, default=6.0)
    ap.add_argument("--min-detection-items", type=int, default=0)
    args = ap.parse_args()

    # Determine task mode
    if not args.task and not args.interactive and not args.loop:
        args.interactive = True

    tasks = []
    if args.task:
        tasks = [args.task]
    elif args.interactive and not args.loop:
        t = get_task_from_user()
        if not t:
            eprint("No task entered.")
            return
        tasks = [t]
    elif args.loop:
        eprint("Interactive loop. Empty line to quit.")
        while True:
            t = get_task_from_user()
            if not t:
                eprint("Bye.")
                return
            tasks.append(t)

    # Vision
    proc = None
    if args.spawn_vision:
        vision_path = Path(args.vision_script)
        if not vision_path.exists():
            eprint(f"[Vision] Not found: {vision_path}")
            return
        proc = spawn_vision(str(vision_path))
        time.sleep(0.5)

    try:
        for task in tasks:
            eprint(f"[Task] {task}")
            pose_names = load_pose_names(args.world)

            # Plan in a thread while vision warms
            plan_holder = {}
            def _planner():
                try:
                    if args.use_fallback:
                        plan_holder["plan"] = plan_fallback(task)
                    else:
                        plan_holder["plan"] = plan_with_gemini(task, pose_names)
                except Exception as e:
                    eprint(f"[Planner] LLM failed ({e}); falling back.")
                    plan_holder["plan"] = plan_fallback(task)

            t = threading.Thread(target=_planner, daemon=True)
            t.start()

            # Wait for detections
            if args.wait_detections > 0:
                eprint(f"[Vision] Waiting up to {args.wait_detections:.1f}s for /detected_objects "
                       f"(min items: {args.min_detection_items})…")
                ready = wait_for_detections(args.wait_detections, args.min_detection_items)
                eprint("[Vision] Ready." if ready else "[Vision] Not ready (continuing).")

            t.join()
            plan = plan_holder.get("plan", {"goal": task, "steps": []})

            # Normalize / enforce policies if helpers exist
            if normalize_plan: plan = normalize_plan(plan)
            if enforce_policies: plan = enforce_policies(plan)

            # Validate if possible
            if not args.no_validate and load_schema_from_md and validate_plan:
                try:
                    schema = load_schema_from_md(args.contract)
                    validate_plan(plan, schema)
                except Exception as e:
                    eprint(f"[Validator] Plan failed schema validation: {e}")
                    return

            print(json.dumps(plan, indent=2))  # show plan

            exe = build_executor(args.ip, args.world, args.sim, args.dry_run)
            try:
                eprint("[Exec] Starting execution…")
                exe.execute(plan)
                eprint("[Exec] Done.")
            except KeyboardInterrupt:
                eprint("Interrupted.")
                break
            finally:
                exe.shutdown()
    finally:
        if proc:
            stop_process_tree(proc)

if __name__ == "__main__":
    main()
