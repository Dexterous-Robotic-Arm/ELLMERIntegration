#!/usr/bin/env python3
"""
RAG-based Robot Control System - Main Entry Point

This is the main orchestrator for the ufactory850 robot control system using
a RAG-based LLM planner as the central decision maker.

The system integrates:
1. RAG-based LLM planner (the "brains")
2. Robot controller for physical actions
3. Vision system for object detection
4. Configuration management
5. Real-time feedback and adaptation
"""

import sys
import os
import time
import signal
import argparse
import logging
import threading
from pathlib import Path
from typing import Optional, Dict, Any

# Add the robot_control package to the path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from robot_control.robot_controller import XArmRunner
from robot_control.robot_controller.executor import TaskExecutor
from robot_control.vision_system import PoseRecorder
from robot_control.rag_system.planner.rag_planner import RAGPlanner
from robot_control.rag_system.integration.rag_integration import RAGIntegration, IntegrationConfig
from robot_control.utils import ConfigManager, setup_logging


class RAGRobotControlSystem:
    """
    RAG-based robot control system that uses LLM as the central decision maker.
    
    This system implements a RAG pipeline where:
    1. LLM receives context from multiple sources (poses, actions, vision, history)
    2. LLM generates plans based on current state and task goals
    3. System executes plans and provides feedback
    4. LLM adapts plans based on feedback and errors
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize the RAG-based robot control system.
        
        Args:
            config: Configuration dictionary
        """
        self.config = config
        self.logger = logging.getLogger(__name__)
        
        # Initialize components
        self.robot_controller = None
        self.vision_system = None
        self.rag_planner = None
        self.executor = None
        
        # State management
        self.running = False
        self.current_task = None
        self.execution_thread = None
        
        # Initialize components
        self._initialize_components()
        
        self.logger.info("RAG Robot Control System initialized successfully")
    
    def _initialize_components(self):
        """Initialize all system components."""
        try:
            # Initialize robot controller
            if not self.config.get('sim', False):
                self.robot_controller = XArmRunner(
                    ip=self.config.get('robot_ip', '192.168.1.241'),
                    sim=self.config.get('sim', False)
                )
                self.logger.info("Robot controller initialized")
            else:
                self.logger.info("Running in simulation mode - no robot controller")
            
            # Initialize vision system
            if self.config.get('enable_vision', True):
                try:
                    self.vision_system = PoseRecorder(standalone=False)
                    self.logger.info("Vision system initialized")
                except Exception as e:
                    self.logger.warning(f"Failed to initialize vision system: {e}")
            
            # Initialize RAG planner
            self.rag_planner = RAGPlanner(
                robot_controller=self.robot_controller,
                vision_system=self.vision_system,
                config_path=self.config.get('config_path', 'config/'),
                max_retries=self.config.get('max_retries', 3),
                scan_timeout=self.config.get('scan_timeout', 10.0)
            )
            self.logger.info("RAG planner initialized")
            
            # Initialize executor (for compatibility)
            if self.robot_controller:
                self.executor = TaskExecutor(
                    arm_ip=self.config.get('robot_ip', '192.168.1.241'),
                    world_yaml=self.config.get('world_yaml'),
                    sim=self.config.get('sim', False),
                    dry_run=self.config.get('dry_run', False)
                )
                self.logger.info("Task executor initialized")
            
        except Exception as e:
            self.logger.error(f"Failed to initialize components: {e}")
            raise
    
    def start(self):
        """Start the RAG-based robot control system."""
        self.logger.info("Starting RAG-based robot control system...")
        self.running = True
        
        # Start vision system if available
        if self.vision_system:
            try:
                # Start vision system in a separate thread
                vision_thread = threading.Thread(target=self._run_vision_system, daemon=True)
                vision_thread.start()
                self.logger.info("Vision system started")
            except Exception as e:
                self.logger.error(f"Failed to start vision system: {e}")
        
        self.logger.info("RAG-based robot control system started successfully")
    
    def stop(self):
        """Stop the RAG-based robot control system."""
        self.logger.info("Stopping RAG-based robot control system...")
        self.running = False
        
        # Stop vision system
        if self.vision_system:
            try:
                self.vision_system.destroy_node()
                self.logger.info("Vision system stopped")
            except Exception as e:
                self.logger.error(f"Error stopping vision system: {e}")
        
        # Disconnect robot controller
        if self.robot_controller:
            try:
                self.robot_controller.disconnect()
                self.logger.info("Robot controller disconnected")
            except Exception as e:
                self.logger.error(f"Error disconnecting robot controller: {e}")
        
        self.logger.info("RAG-based robot control system stopped")
    
    def _run_vision_system(self):
        """Run the vision system in a separate thread."""
        while self.running:
            try:
                if self.vision_system:
                    # This would integrate with the actual vision system
                    # For now, just sleep
                    time.sleep(1.0)
            except Exception as e:
                self.logger.error(f"Error in vision system: {e}")
                time.sleep(1.0)
    
    def execute_task(self, task_description: str) -> Dict[str, Any]:
        """
        Execute a task using the RAG-based planner.
        
        Args:
            task_description: Description of the task to accomplish
            
        Returns:
            Execution results
        """
        self.logger.info(f"Executing task: {task_description}")
        self.current_task = task_description
        
        try:
            # Use the RAG planner to plan and execute
            results = self.rag_planner.plan_and_execute(task_description)
            
            if results.get('success', False):
                self.logger.info("Task completed successfully")
            else:
                self.logger.error(f"Task failed: {results.get('error', 'Unknown error')}")
            
            return results
            
        except Exception as e:
            error_msg = f"Error executing task: {e}"
            self.logger.error(error_msg)
            return {
                "success": False,
                "error": error_msg
            }
        finally:
            self.current_task = None
    
    def get_status(self) -> Dict[str, Any]:
        """
        Get the current status of the system.
        
        Returns:
            Status information
        """
        status = {
            "running": self.running,
            "current_task": self.current_task,
            "rag_planner": self.rag_planner.get_status() if self.rag_planner else None,
            "robot_controller": {
                "connected": self.robot_controller is not None,
                "current_pose": self.robot_controller.get_current_position() if self.robot_controller else None
            } if self.robot_controller else None,
            "vision_system": {
                "enabled": self.vision_system is not None
            }
        }
        
        return status
    
    def interactive_mode(self):
        """Run the system in interactive mode."""
        self.logger.info("Starting interactive mode...")
        print("\n=== RAG-based Robot Control System - Interactive Mode ===")
        print("Type 'quit' or 'exit' to stop the system")
        print("Type 'status' to see current system status")
        print("Type 'help' for available commands")
        print("=" * 60)
        
        while self.running:
            try:
                # Get user input
                user_input = input("\nEnter task description: ").strip()
                
                if not user_input:
                    continue
                
                if user_input.lower() in ['quit', 'exit']:
                    self.logger.info("User requested exit")
                    break
                
                elif user_input.lower() == 'status':
                    status = self.get_status()
                    print(f"\nSystem Status:")
                    print(f"  Running: {status['running']}")
                    print(f"  Current Task: {status['current_task']}")
                    if status['rag_planner']:
                        print(f"  Planner State: {status['rag_planner']['state']}")
                    if status['robot_controller']:
                        print(f"  Robot Connected: {status['robot_controller']['connected']}")
                    continue
                
                elif user_input.lower() == 'help':
                    print("\nAvailable Commands:")
                    print("  <task description> - Execute a task (e.g., 'pick up the cup')")
                    print("  status - Show current system status")
                    print("  help - Show this help message")
                    print("  quit/exit - Stop the system")
                    continue
                
                # Execute the task
                print(f"\nExecuting task: {user_input}")
                results = self.execute_task(user_input)
                
                if results.get('success', False):
                    print(f"✅ Task completed successfully!")
                    if results.get('steps_executed'):
                        print(f"   Steps executed: {results['steps_executed']}")
                    if results.get('execution_time'):
                        print(f"   Execution time: {results['execution_time']:.2f}s")
                else:
                    print(f"❌ Task failed: {results.get('error', 'Unknown error')}")
                
            except KeyboardInterrupt:
                self.logger.info("User interrupted")
                break
            except Exception as e:
                self.logger.error(f"Error in interactive mode: {e}")
                print(f"Error: {e}")
        
        self.logger.info("Interactive mode ended")
    
    def loop_mode(self, task_description: str):
        """Run the system in loop mode, continuously executing the same task."""
        self.logger.info(f"Starting loop mode with task: {task_description}")
        
        iteration = 1
        while self.running:
            try:
                print(f"\n=== Iteration {iteration} ===")
                print(f"Executing task: {task_description}")
                
                results = self.execute_task(task_description)
                
                if results.get('success', False):
                    print(f"✅ Iteration {iteration} completed successfully!")
                else:
                    print(f"❌ Iteration {iteration} failed: {results.get('error', 'Unknown error')}")
                
                iteration += 1
                
                # Wait before next iteration
                if self.running:
                    print("Waiting 5 seconds before next iteration...")
                    time.sleep(5.0)
                
            except KeyboardInterrupt:
                self.logger.info("User interrupted loop mode")
                break
            except Exception as e:
                self.logger.error(f"Error in loop mode: {e}")
                print(f"Error: {e}")
                time.sleep(1.0)
        
        self.logger.info("Loop mode ended")


def parse_arguments() -> Dict[str, Any]:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="RAG-based Robot Control System for ufactory850",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python rag_main.py --task "pick up the cup"
  python rag_main.py --interactive
  python rag_main.py --loop --task "move to home"
  python rag_main.py --sim --dry-run --task "test movement"
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
        help="Run in dry-run mode (no actual robot movement)"
    )
    
    parser.add_argument(
        "--no-vision", 
        action="store_true",
        help="Disable vision system"
    )
    
    parser.add_argument(
        "--max-retries", 
        type=int, 
        default=3,
        help="Maximum retries for failed plans (default: 3)"
    )
    
    parser.add_argument(
        "--scan-timeout", 
        type=float, 
        default=10.0,
        help="Timeout for scanning operations in seconds (default: 10.0)"
    )
    
    parser.add_argument(
        "--log-level", 
        type=str, 
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging level (default: INFO)"
    )
    
    args = parser.parse_args()
    
    # Convert to dictionary
    config = {
        'task': args.task,
        'robot_ip': args.ip,
        'world_yaml': args.world,
        'interactive': args.interactive,
        'loop': args.loop,
        'sim': args.sim,
        'dry_run': args.dry_run,
        'enable_vision': not args.no_vision,
        'max_retries': args.max_retries,
        'scan_timeout': args.scan_timeout,
        'log_level': args.log_level
    }
    
    return config


def main():
    """Main entry point for the RAG-based robot control system."""
    # Parse arguments
    config = parse_arguments()
    
    # Setup logging
    setup_logging(level=config['log_level'])
    logger = logging.getLogger(__name__)
    
    # Create and start the system
    system = None
    try:
        # Create the RAG-based robot control system
        system = RAGRobotControlSystem(config)
        
        # Start the system
        system.start()
        
        # Run based on mode
        if config['interactive']:
            system.interactive_mode()
        elif config['loop']:
            if not config['task']:
                logger.error("Loop mode requires a task description")
                return 1
            system.loop_mode(config['task'])
        elif config['task']:
            # Single task execution
            results = system.execute_task(config['task'])
            
            if results.get('success', False):
                logger.info("Task completed successfully")
                return 0
            else:
                logger.error(f"Task failed: {results.get('error', 'Unknown error')}")
                return 1
        else:
            # Default to interactive mode
            system.interactive_mode()
    
    except KeyboardInterrupt:
        logger.info("User interrupted")
    except Exception as e:
        logger.error(f"Error in main: {e}")
        return 1
    finally:
        # Cleanup
        if system:
            system.stop()
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
