#!/usr/bin/env python3
"""
Robot Control System - Main Entry Point
=======================================

This module provides a clean interface for controlling the ufactory850 robot
with LLM planning, computer vision, and autonomous task execution.

Features:
- Natural language task planning with Google Gemini
- Real-time object detection with RealSense D435 + YOLO
- Safe robot control with comprehensive safety limits
- Interactive and batch task execution modes
- Configurable vision and planning systems

Usage:
    python main.py --task "pick up the cup"
    python main.py --interactive
    python main.py --loop
"""

import os
import sys
import json
import time
import argparse
import threading
import subprocess
from pathlib import Path
from typing import Optional, List, Dict, Any
from dataclasses import dataclass

# ROS imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String as StringMsg

# Setup import paths
HERE = Path(__file__).resolve().parent
CANDIDATE_DIRS = [
    HERE,  # repo root
    HERE / "llm_planning_gemini" / "custom_gemini_examples",  # planner_llm.py
    HERE / "ros_xarm",  # executor.py
    HERE / "src",
]

for directory in CANDIDATE_DIRS:
    if directory.exists():
        sys.path.insert(0, str(directory))

# Core imports
from planner_llm import plan_with_gemini, plan_fallback
from executor import Executor

# Optional imports with graceful fallbacks
try:
    from utils_schema import load_schema_from_md, validate_plan
except ImportError:
    load_schema_from_md = None
    validate_plan = None

try:
    from utils_plan import normalize_plan, enforce_policies
except ImportError:
    normalize_plan = None
    enforce_policies = None

try:
    import yaml
except ImportError:
    yaml = None


@dataclass
class SystemConfig:
    """Configuration for the robot control system."""
    robot_ip: str = "192.168.1.241"
    world_yaml: str = "world_model.yaml"
    contract_md: str = "llm_planning_gemini/custom_gemini_examples/custom_gpt_action_schema.md"
    vision_script: str = "ros_xarm/pose_recorder.py"
    wait_detections: float = 6.0
    min_detection_items: int = 0
    sim_mode: bool = False
    dry_run: bool = False
    spawn_vision: bool = True
    use_fallback: bool = False
    no_validate: bool = False


class RobotConnectionManager:
    """Manages robot connection and IP configuration."""
    
    def __init__(self):
        self.default_ip = "192.168.1.241"
    
    def get_robot_ip(self) -> str:
        """Get robot IP address with environment variable override."""
        env_ip = os.environ.get("XARM_IP")
        if env_ip:
            self._log(f"Using IP from environment: {env_ip}")
            return env_ip
        
        self._log(f"Using default robot IP: {self.default_ip}")
        return self.default_ip
    
    def test_connection(self, ip: str) -> bool:
        """Test robot connectivity."""
        try:
            from xarm.wrapper import XArmAPI
            arm = XArmAPI(ip)
            
            if arm.connect():
                self._log(f"Successfully connected to robot at {ip}")
                arm.disconnect()
                return True
            else:
                self._log(f"Failed to connect to robot at {ip}")
                return False
                
        except Exception as e:
            self._log(f"Error connecting to {ip}: {e}")
            return False
    
    def _log(self, message: str):
        """Log connection-related messages."""
        print(f"[Connection] {message}", file=sys.stderr)


class VisionSystem:
    """Manages the computer vision system."""
    
    def __init__(self, script_path: str):
        self.script_path = Path(script_path)
        self.process: Optional[subprocess.Popen] = None
    
    def start(self) -> bool:
        """Start the vision system."""
        if not self.script_path.exists():
            print(f"[Vision] Script not found: {self.script_path}", file=sys.stderr)
            return False
        
        cmd = [sys.executable, str(self.script_path)]
        print(f"[Vision] Starting: {' '.join(cmd)}", file=sys.stderr)
        
        try:
            self.process = subprocess.Popen(cmd, start_new_session=True)
            time.sleep(0.5)  # Allow startup time
            return True
        except Exception as e:
            print(f"[Vision] Failed to start: {e}", file=sys.stderr)
            return False
    
    def stop(self):
        """Stop the vision system gracefully."""
        if self.process and self.process.poll() is None:
            print("[Vision] Stopping vision system...", file=sys.stderr)
            try:
                self.process.terminate()
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                print("[Vision] Force killing vision process...", file=sys.stderr)
                self.process.kill()
            except Exception as e:
                print(f"[Vision] Error stopping process: {e}", file=sys.stderr)


class DetectionWaiter(Node):
    """ROS node for waiting for object detections."""
    
    def __init__(self, topic: str = "/detected_objects", min_items: int = 0):
        super().__init__('detection_waiter')
        self.min_items = int(min_items)
        self.ready = False
        self.subscription = self.create_subscription(
            StringMsg, topic, self._on_message, 10
        )
    
    def _on_message(self, msg: StringMsg):
        """Handle incoming detection messages."""
        try:
            data = json.loads(msg.data)
            items = data.get("items", []) or []
            if len(items) >= self.min_items:
                self.ready = True
        except (json.JSONDecodeError, KeyError):
            pass
    
    def wait_for_detections(self, timeout: float) -> bool:
        """Wait for detections with timeout."""
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.ready:
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.ready


class TaskPlanner:
    """Handles task planning and LLM interaction."""
    
    def __init__(self, config: SystemConfig):
        self.config = config
        self.pose_names: List[str] = []
    
    def load_pose_names(self, world_yaml: str) -> List[str]:
        """Load named poses from world configuration."""
        if world_yaml and yaml:
            yaml_path = Path(world_yaml)
            if yaml_path.exists():
                try:
                    with open(yaml_path, 'r', encoding='utf-8') as f:
                        data = yaml.safe_load(f) or {}
                    poses = data.get("poses", {})
                    return list(poses.keys())
                except Exception as e:
                    print(f"[Planner] Error loading poses from {world_yaml}: {e}", file=sys.stderr)
        
        # Fallback to executor poses
        try:
            from executor import WORLD_POSES
            return list(WORLD_POSES.keys())
        except ImportError:
            return []
    
    def plan_task(self, task: str) -> Dict[str, Any]:
        """Generate a plan for the given task."""
        try:
            if self.config.use_fallback:
                return plan_fallback(task)
            else:
                return plan_with_gemini(task, self.pose_names)
        except Exception as e:
            print(f"[Planner] LLM planning failed ({e}), using fallback", file=sys.stderr)
            return plan_fallback(task)


class TaskExecutor:
    """Executes planned tasks on the robot."""
    
    def __init__(self, config: SystemConfig):
        self.config = config
        self.executor: Optional[Executor] = None
    
    def build_executor(self, robot_ip: str) -> Executor:
        """Create and configure the executor."""
        try:
            return Executor(
                robot_ip, 
                world_yaml=self.config.world_yaml,
                sim=self.config.sim_mode,
                dry_run=self.config.dry_run
            )
        except TypeError:
            # Fallback for older executor versions
            return Executor(robot_ip)
    
    def execute_plan(self, plan: Dict[str, Any]) -> bool:
        """Execute a planned task."""
        try:
            print("[Executor] Starting execution...", file=sys.stderr)
            self.executor.execute(plan)
            print("[Executor] Execution completed successfully", file=sys.stderr)
            return True
        except KeyboardInterrupt:
            print("[Executor] Execution interrupted by user", file=sys.stderr)
            return False
        except Exception as e:
            print(f"[Executor] Execution failed: {e}", file=sys.stderr)
            return False
        finally:
            if self.executor:
                self.executor.shutdown()


class RobotControlSystem:
    """Main robot control system orchestrator."""
    
    def __init__(self, config: SystemConfig):
        self.config = config
        self.connection_manager = RobotConnectionManager()
        self.vision_system = VisionSystem(config.vision_script)
        self.planner = TaskPlanner(config)
        self.executor = TaskExecutor(config)
        self.vision_process: Optional[subprocess.Popen] = None
    
    def setup(self) -> bool:
        """Initialize the system."""
        # Get robot IP
        robot_ip = self.connection_manager.get_robot_ip()
        
        # Test connection (unless in sim/dry-run mode)
        if not self.config.sim_mode and not self.config.dry_run:
            if not self.connection_manager.test_connection(robot_ip):
                self._print_connection_error()
                return False
        
        # Load pose names
        self.planner.pose_names = self.planner.load_pose_names(self.config.world_yaml)
        
        # Start vision system
        if self.config.spawn_vision:
            if not self.vision_system.start():
                return False
        
        return True
    
    def wait_for_vision(self) -> bool:
        """Wait for vision system to be ready."""
        if self.config.wait_detections <= 0:
            return True
        
        print(f"[Vision] Waiting up to {self.config.wait_detections:.1f}s for detections "
              f"(min items: {self.config.min_detection_items})...", file=sys.stderr)
        
        try:
            rclpy.init(args=None)
            waiter = DetectionWaiter(min_items=self.config.min_detection_items)
            ready = waiter.wait_for_detections(self.config.wait_detections)
            waiter.destroy_node()
            rclpy.shutdown()
            
            status = "Ready" if ready else "Not ready (continuing)"
            print(f"[Vision] {status}", file=sys.stderr)
            return ready
        except Exception as e:
            print(f"[Vision] Error waiting for detections: {e}", file=sys.stderr)
            return False
    
    def execute_task(self, task: str) -> bool:
        """Execute a single task."""
        print(f"[Task] {task}", file=sys.stderr)
        
        # Generate plan in background
        plan_holder = {}
        def plan_task():
            try:
                plan_holder["plan"] = self.planner.plan_task(task)
            except Exception as e:
                print(f"[Planner] Planning failed: {e}", file=sys.stderr)
                plan_holder["plan"] = {"goal": task, "steps": []}
        
        planning_thread = threading.Thread(target=plan_task, daemon=True)
        planning_thread.start()
        
        # Wait for vision if needed
        self.wait_for_vision()
        
        # Wait for planning to complete
        planning_thread.join()
        plan = plan_holder.get("plan", {"goal": task, "steps": []})
        
        # Post-process plan
        if normalize_plan:
            plan = normalize_plan(plan)
        if enforce_policies:
            plan = enforce_policies(plan)
        
        # Validate plan
        if not self.config.no_validate and load_schema_from_md and validate_plan:
            try:
                schema = load_schema_from_md(self.config.contract_md)
                validate_plan(plan, schema)
            except Exception as e:
                print(f"[Validator] Plan validation failed: {e}", file=sys.stderr)
                return False
        
        # Display plan
        print(json.dumps(plan, indent=2))
        
        # Execute plan
        robot_ip = self.connection_manager.get_robot_ip()
        self.executor.executor = self.executor.build_executor(robot_ip)
        return self.executor.execute_plan(plan)
    
    def cleanup(self):
        """Clean up system resources."""
        self.vision_system.stop()
    
    def _print_connection_error(self):
        """Print helpful connection error information."""
        print("[Error] Cannot connect to robot. Please check:", file=sys.stderr)
        print("  1. Robot is powered on and connected to network", file=sys.stderr)
        print("  2. Robot IP is correct (set XARM_IP environment variable)", file=sys.stderr)
        print("  3. Network connection is working", file=sys.stderr)
        print("  4. Try --sim or --dry-run mode for testing without robot", file=sys.stderr)


def get_task_from_user() -> str:
    """Get task input from user."""
    print("Task: ", end="", file=sys.stderr)
    sys.stderr.flush()
    try:
        return input().strip()
    except EOFError:
        return ""


def parse_arguments() -> SystemConfig:
    """Parse command line arguments and return configuration."""
    parser = argparse.ArgumentParser(
        description="Robot Control System - Autonomous task execution with LLM planning",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python main.py --task "pick up the cup"
  python main.py --interactive
  python main.py --loop
  python main.py --task "move to home" --sim
  python main.py --task "test gripper" --dry-run
        """
    )
    
    # Task specification
    task_group = parser.add_mutually_exclusive_group()
    task_group.add_argument("--task", help="Task to execute")
    task_group.add_argument("--interactive", "-i", action="store_true", 
                           help="Interactive mode - ask for one task")
    task_group.add_argument("--loop", "-l", action="store_true", 
                           help="Loop mode - continuously ask for tasks")
    
    # Configuration
    parser.add_argument("--world", default=os.environ.get("WORLD_YAML", "world_model.yaml"),
                       help="World configuration YAML file")
    parser.add_argument("--contract", 
                       default=os.environ.get("ACTION_CONTRACT_MD", 
                                             "llm_planning_gemini/custom_gemini_examples/custom_gpt_action_schema.md"),
                       help="Action contract Markdown file")
    parser.add_argument("--ip", help="Robot IP address (defaults to 192.168.1.241)")
    
    # Execution modes
    parser.add_argument("--sim", action="store_true", help="Simulation mode")
    parser.add_argument("--dry-run", action="store_true", help="Dry run - plan only, no execution")
    parser.add_argument("--use-fallback", action="store_true", help="Use fallback planner")
    parser.add_argument("--no-validate", action="store_true", help="Skip plan validation")
    
    # Vision system
    vision_group = parser.add_mutually_exclusive_group()
    vision_group.add_argument("--spawn-vision", dest="spawn_vision", action="store_true", 
                             default=True, help="Start vision system (default)")
    vision_group.add_argument("--no-spawn-vision", dest="spawn_vision", action="store_false",
                             help="Don't start vision system")
    parser.add_argument("--vision-script", 
                       default=str(HERE / "ros_xarm" / "pose_recorder.py"),
                       help="Vision system script path")
    parser.add_argument("--wait-detections", type=float, default=6.0,
                       help="Wait time for detections (seconds)")
    parser.add_argument("--min-detection-items", type=int, default=0,
                       help="Minimum detected items required")
    
    args = parser.parse_args()
    
    # Create configuration
    config = SystemConfig(
        robot_ip=args.ip or "192.168.1.241",
        world_yaml=args.world,
        contract_md=args.contract,
        vision_script=args.vision_script,
        wait_detections=args.wait_detections,
        min_detection_items=args.min_detection_items,
        sim_mode=args.sim,
        dry_run=args.dry_run,
        spawn_vision=args.spawn_vision,
        use_fallback=args.use_fallback,
        no_validate=args.no_validate
    )
    
    return config


def main():
    """Main entry point for the robot control system."""
    # Parse arguments
    config = parse_arguments()
    
    # Create control system
    system = RobotControlSystem(config)
    
    try:
        # Initialize system
        if not system.setup():
            return 1
        
        # Determine task mode
        tasks = []
        if config.task:
            tasks = [config.task]
        elif config.interactive:
            task = get_task_from_user()
            if not task:
                print("No task entered.", file=sys.stderr)
                return 0
            tasks = [task]
        elif config.loop:
            print("Interactive loop. Empty line to quit.", file=sys.stderr)
            while True:
                task = get_task_from_user()
                if not task:
                    print("Bye.", file=sys.stderr)
                    break
                tasks.append(task)
        else:
            # Default to interactive
            task = get_task_from_user()
            if not task:
                print("No task entered.", file=sys.stderr)
                return 0
            tasks = [task]
        
        # Execute tasks
        for task in tasks:
            if not system.execute_task(task):
                return 1
        
        return 0
        
    except KeyboardInterrupt:
        print("\n[System] Interrupted by user", file=sys.stderr)
        return 1
    except Exception as e:
        print(f"[System] Unexpected error: {e}", file=sys.stderr)
        return 1
    finally:
        system.cleanup()


if __name__ == "__main__":
    sys.exit(main())
