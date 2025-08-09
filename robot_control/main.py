#!/usr/bin/env python3
"""
Robot Control System - Main Entry Point

This is the main orchestrator for the ufactory850 robot control system.
It handles argument parsing, robot connection, vision system spawning,
LLM planning, and plan execution.
"""

import sys
import os
import time
import signal
import argparse
import subprocess
import threading
import logging
from dataclasses import dataclass
from typing import Optional, List, Dict, Any

# Add the robot_control package to the path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_control.robot_controller import XArmRunner
from robot_control.robot_controller.executor import TaskExecutor
from robot_control.vision_system import PoseRecorder
# TaskPlanner is defined in this file, no import needed
from robot_control.utils import ConfigManager, setup_logging


@dataclass
class SystemConfig:
    """Configuration for the robot control system."""
    task: str
    robot_ip: str
    world_yaml: Optional[str]
    vision_script: str
    interactive: bool
    loop: bool
    sim: bool
    dry_run: bool
    wait_detections: bool
    min_detection_items: int
    log_level: str
    use_fallback: bool


def parse_arguments() -> SystemConfig:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Robot Control System for ufactory850",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python main.py --task "pick up the cup"
  python main.py --interactive
  python main.py --loop --task "move to home"
  python main.py --sim --dry-run --task "test movement"
        """
    )
    
    parser.add_argument(
        "--task", 
        type=str, 
        default="",
        help="Task description for the robot to execute"
    )
    
    parser.add_argument(
        "--ip", 
        type=str, 
        default="192.168.1.241",
        help="Robot IP address (default: 192.168.1.241)"
    )
    
    parser.add_argument(
        "--world", 
        type=str,
        help="Path to world model YAML file"
    )
    
    parser.add_argument(
        "--vision-script", 
        type=str, 
        default="robot_control/vision_system/pose_recorder.py",
        help="Path to vision system script"
    )
    
    parser.add_argument(
        "--interactive", 
        action="store_true",
        help="Run in interactive mode"
    )
    
    parser.add_argument(
        "--loop", 
        action="store_true",
        help="Run in continuous loop mode"
    )
    
    parser.add_argument(
        "--sim", 
        action="store_true",
        help="Run in simulation mode"
    )
    
    parser.add_argument(
        "--dry-run", 
        action="store_true",
        help="Run without executing robot movements"
    )
    
    parser.add_argument(
        "--wait-detections", 
        action="store_true",
        help="Wait for object detections before planning"
    )
    
    parser.add_argument(
        "--min-detection-items", 
        type=int, 
        default=1,
        help="Minimum number of detected objects required"
    )
    
    parser.add_argument(
        "--log-level", 
        type=str, 
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging level"
    )
    
    parser.add_argument(
        "--use-fallback", 
        action="store_true",
        help="Force use of fallback planner (skip LLM)"
    )
    
    args = parser.parse_args()
    
    return SystemConfig(
        task=args.task,
        robot_ip=args.ip,
        world_yaml=args.world,
        vision_script=args.vision_script,
        interactive=args.interactive,
        loop=args.loop,
        sim=args.sim,
        dry_run=args.dry_run,
        wait_detections=args.wait_detections,
        min_detection_items=args.min_detection_items,
        log_level=args.log_level,
        use_fallback=args.use_fallback
    )


def get_robot_ip() -> str:
    """Get robot IP address - defaults to 192.168.1.241"""
    # Check environment variable first
    env_ip = os.environ.get("XARM_IP")
    if env_ip:
        print(f"[Config] Using IP from environment: {env_ip}", file=sys.stderr)
        return env_ip
    
    # Default IP for ufactory850
    default_ip = "192.168.1.241"
    print(f"[Config] Using default robot IP: {default_ip}", file=sys.stderr)
    return default_ip


class RobotConnectionManager:
    """Manages robot connection and initialization."""
    
    def __init__(self, config: SystemConfig):
        self.config = config
        self.robot_ip = get_robot_ip()
        self.logger = setup_logging(
            level=getattr(logging, config.log_level),
            log_file="logs/robot_control.log"
        )
        
    def connect(self) -> XArmRunner:
        """Connect to the robot and return XArmRunner instance."""
        try:
            self.logger.info(f"Connecting to robot at {self.robot_ip}")
            runner = XArmRunner(self.robot_ip, sim=self.config.sim)
            
            if not self.config.sim:
                self.logger.info("Robot connected successfully")
            else:
                self.logger.info("Running in simulation mode")
                
            return runner
            
        except Exception as e:
            self.logger.error(f"Failed to connect to robot: {e}")
            raise


class VisionSystem:
    """Manages the vision system process."""
    
    def __init__(self, config: SystemConfig):
        self.config = config
        self.process = None
        self.logger = setup_logging(
            level=getattr(logging, config.log_level),
            log_file="logs/vision_system.log"
        )
        
    def start(self):
        """Start the vision system process."""
        try:
            self.logger.info(f"Starting vision system: {self.config.vision_script}")
            self.process = subprocess.Popen(
                [sys.executable, self.config.vision_script],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            self.logger.info("Vision system started")
            
        except Exception as e:
            self.logger.error(f"Failed to start vision system: {e}")
            raise
            
    def stop(self):
        """Stop the vision system process."""
        if self.process:
            self.process.terminate()
            self.process.wait()
            self.logger.info("Vision system stopped")


class TaskPlanner:
    """Manages task planning using RAG system."""
    
    def __init__(self, config: SystemConfig):
        self.config = config
        self.logger = setup_logging(
            level=getattr(logging, config.log_level),
            log_file="logs/rag_planner.log"
        )
        
    def plan_task(self, task: str, world_config: Dict[str, Any]) -> Dict[str, Any]:
        """Generate a plan for the given task using RAG system."""
        try:
            self.logger.info(f"Planning task: {task}")
            
            # Use RAG system for planning
            from robot_control.rag_system.planner.rag_planner import RAGPlanner
            
            # Initialize RAG planner in simulation mode for legacy compatibility
            rag_planner = RAGPlanner(
                robot_controller=None,
                vision_system=None,
                config_path="config/",
                max_retries=3,
                scan_timeout=10.0
            )
            
            # Create RAG context
            context = rag_planner._create_rag_context(task)
            
            # Generate plan
            plan = rag_planner._generate_plan_with_llm(context)
            
            self.logger.info(f"Generated plan with {len(plan.get('steps', []))} steps")
            return plan
            
        except Exception as e:
            self.logger.error(f"Failed to plan task: {e}")
            # Return a simple fallback plan
            return {
                "goal": task,
                "steps": [
                    {"action": "MOVE_TO_NAMED", "name": "home"},
                    {"action": "SLEEP", "duration": 1.0}
                ]
            }


# TaskExecutor is imported from robot_control.robot_controller.executor


class RobotControlSystem:
    """Main robot control system orchestrator."""
    
    def __init__(self, config: SystemConfig):
        self.config = config
        self.logger = setup_logging(
            level=getattr(logging, config.log_level),
            log_file="logs/robot_control.log"
        )
        
        # Initialize components
        self.connection_manager = RobotConnectionManager(config)
        self.vision_system = VisionSystem(config)
        self.task_planner = TaskPlanner(config)
        
        # Load configuration
        self.config_manager = ConfigManager()
        
    def run(self):
        """Run the robot control system."""
        try:
            self.logger.info("Starting Robot Control System")
            
            # Connect to robot
            runner = self.connection_manager.connect()
            
            # Load world configuration
            world_config = {}
            if self.config.world_yaml:
                world_config = self.config_manager.get_world_config()
            
            # Start vision system
            self.vision_system.start()
            
            # Wait for vision system to warm up
            time.sleep(2)
            
            # Main execution loop
            if self.config.interactive:
                self._run_interactive(runner, world_config)
            elif self.config.loop:
                self._run_loop(runner, world_config)
            else:
                self._run_single_task(runner, world_config)
                
        except KeyboardInterrupt:
            self.logger.info("Received interrupt signal")
        except Exception as e:
            self.logger.error(f"System error: {e}")
            raise
        finally:
            self._cleanup()
            
    def _run_interactive(self, runner: XArmRunner, world_config: Dict[str, Any]):
        """Run in interactive mode."""
        self.logger.info("Running in interactive mode")
        print("\nInteractive Robot Control System")
        print("Type 'help' for commands, 'quit' to exit")
        
        while True:
            try:
                command = input("\nrobot> ").strip()
                
                if command.lower() in ['quit', 'exit', 'q']:
                    break
                elif command.lower() == 'help':
                    print("Available commands:")
                    print("  <task description> - Execute a task")
                    print("  home - Move to home position")
                    print("  status - Show robot status")
                    print("  quit - Exit")
                elif command.lower() == 'home':
                    runner.move_gohome(wait=True)
                elif command.lower() == 'status':
                    print(f"Robot connected: {runner.is_connected()}")
                elif command:
                    # Execute task
                    plan = self.task_planner.plan_task(command, world_config)
                    executor = TaskExecutor(self.config.robot_ip, world_yaml=self.config.world_yaml, sim=self.config.sim, dry_run=self.config.dry_run)
                    executor.execute(plan)
                    
            except KeyboardInterrupt:
                break
                
    def _run_loop(self, runner: XArmRunner, world_config: Dict[str, Any]):
        """Run in continuous loop mode."""
        self.logger.info("Running in loop mode")
        
        while True:
            try:
                if self.config.task:
                    plan = self.task_planner.plan_task(self.config.task, world_config)
                    executor = TaskExecutor(self.config.robot_ip, world_yaml=self.config.world_yaml, sim=self.config.sim, dry_run=self.config.dry_run)
                    executor.execute(plan)
                    
                time.sleep(1)
                
            except KeyboardInterrupt:
                break
                
    def _run_single_task(self, runner: XArmRunner, world_config: Dict[str, Any]):
        """Run a single task."""
        if not self.config.task:
            self.logger.error("No task specified")
            return
            
        self.logger.info(f"Executing task: {self.config.task}")
        plan = self.task_planner.plan_task(self.config.task, world_config)
        executor = TaskExecutor(self.config.robot_ip, world_yaml=self.config.world_yaml, sim=self.config.sim, dry_run=self.config.dry_run)
        executor.execute(plan)
        
    def _cleanup(self):
        """Clean up system resources."""
        self.logger.info("Cleaning up system resources")
        self.vision_system.stop()


def main():
    """Main entry point."""
    # Parse arguments
    config = parse_arguments()
    
    # Create and run the robot control system
    system = RobotControlSystem(config)
    system.run()


if __name__ == "__main__":
    main() 