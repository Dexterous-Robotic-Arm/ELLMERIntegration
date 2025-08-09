#!/usr/bin/env python3
"""
RAG Integration Module

This module provides integration between the RAG-based LLM planner and the existing
robot control system components. It handles:
1. Integration with existing robot controller
2. Integration with existing vision system
3. Integration with existing executor
4. Configuration management
5. State synchronization
"""

import os
import json
import time
import logging
import threading
from pathlib import Path
from typing import Dict, Any, Optional, List
from dataclasses import dataclass

from ..planner.rag_planner import RAGPlanner, RAGContext, VisionFeedback, ExecutionContext
from ...robot_controller import XArmRunner
from ...robot_controller.executor import TaskExecutor
from ...vision_system import PoseRecorder
from ...utils import ConfigManager


@dataclass
class IntegrationConfig:
    """Configuration for RAG integration."""
    robot_ip: str = "192.168.1.241"
    sim: bool = False
    dry_run: bool = False
    enable_vision: bool = True
    max_retries: int = 3
    scan_timeout: float = 10.0
    config_path: str = "config/"
    world_yaml: Optional[str] = None


class RAGIntegration:
    """
    Integration layer between RAG planner and existing robot control system.
    
    This class provides a unified interface that:
    1. Manages the RAG planner
    2. Integrates with existing robot controller
    3. Integrates with existing vision system
    4. Handles configuration and state management
    5. Provides a clean API for the main system
    """
    
    def __init__(self, config: IntegrationConfig):
        """
        Initialize the RAG integration.
        
        Args:
            config: Integration configuration
        """
        self.config = config
        self.logger = logging.getLogger(__name__)
        
        # Initialize components
        self.robot_controller = None
        self.vision_system = None
        self.executor = None
        self.rag_planner = None
        
        # State management
        self.running = False
        self.current_task = None
        self.execution_thread = None
        
        # Initialize components
        self._initialize_components()
        
        self.logger.info("RAG Integration initialized successfully")
    
    def _initialize_components(self):
        """Initialize all system components."""
        try:
            # Initialize robot controller
            if not self.config.sim:
                self.robot_controller = XArmRunner(
                    ip=self.config.robot_ip,
                    sim=self.config.sim
                )
                self.logger.info("Robot controller initialized")
            else:
                self.logger.info("Running in simulation mode - no robot controller")
            
            # Initialize vision system
            if self.config.enable_vision:
                try:
                    self.vision_system = PoseRecorder(standalone=False)
                    self.logger.info("Vision system initialized")
                except Exception as e:
                    self.logger.warning(f"Failed to initialize vision system: {e}")
            
            # Initialize executor (for compatibility)
            if self.robot_controller:
                self.executor = TaskExecutor(
                    arm_ip=self.config.robot_ip,
                    world_yaml=self.config.world_yaml,
                    sim=self.config.sim,
                    dry_run=self.config.dry_run
                )
                self.logger.info("Task executor initialized")
            
            # Initialize RAG planner
            self.rag_planner = RAGPlanner(
                robot_controller=self.robot_controller,
                vision_system=self.vision_system,
                config_path=self.config.config_path,
                max_retries=self.config.max_retries,
                scan_timeout=self.config.scan_timeout
            )
            self.logger.info("RAG planner initialized")
            
        except Exception as e:
            self.logger.error(f"Failed to initialize components: {e}")
            raise
    
    def start(self):
        """Start the RAG integration system."""
        self.logger.info("Starting RAG integration system...")
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
        
        self.logger.info("RAG integration system started successfully")
    
    def stop(self):
        """Stop the RAG integration system."""
        self.logger.info("Stopping RAG integration system...")
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
        
        self.logger.info("RAG integration system stopped")
    
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
    
    def get_vision_feedback(self) -> VisionFeedback:
        """
        Get current vision feedback.
        
        Returns:
            Current vision feedback
        """
        if self.vision_system:
            try:
                # This would integrate with the actual vision system
                # For now, return a mock feedback
                return VisionFeedback(
                    objects_detected=[],
                    confidence_scores={},
                    scan_quality=0.8
                )
            except Exception as e:
                self.logger.error(f"Failed to get vision feedback: {e}")
        
        return VisionFeedback()
    
    def scan_area(self, duration: float = 5.0) -> VisionFeedback:
        """
        Scan the current area for objects.
        
        Args:
            duration: Duration of the scan in seconds
            
        Returns:
            Vision feedback with detected objects
        """
        self.logger.info(f"Scanning area for {duration} seconds...")
        
        if self.rag_planner:
            return self.rag_planner._scan_area(duration)
        else:
            self.logger.warning("RAG planner not available for scanning")
            return VisionFeedback()
    
    def get_available_poses(self) -> Dict[str, Dict[str, Any]]:
        """
        Get available poses from the world model.
        
        Returns:
            Dictionary of available poses
        """
        if self.rag_planner:
            return self.rag_planner.available_poses
        else:
            return {}
    
    def get_available_actions(self) -> List[Dict[str, Any]]:
        """
        Get available actions from the action schema.
        
        Returns:
            List of available actions
        """
        if self.rag_planner:
            return self.rag_planner.available_actions
        else:
            return []
    
    def get_execution_context(self) -> ExecutionContext:
        """
        Get current execution context.
        
        Returns:
            Current execution context
        """
        if self.rag_planner:
            return self.rag_planner.execution_context
        else:
            return ExecutionContext()
    
    def update_execution_context(self, context: ExecutionContext):
        """
        Update the execution context.
        
        Args:
            context: New execution context
        """
        if self.rag_planner:
            self.rag_planner.execution_context = context
    
    def get_planning_history(self) -> List[Dict[str, Any]]:
        """
        Get planning history.
        
        Returns:
            List of planning history entries
        """
        if self.rag_planner:
            return self.rag_planner.execution_context.execution_history
        else:
            return []
    
    def clear_history(self):
        """Clear the planning and execution history."""
        if self.rag_planner:
            self.rag_planner.execution_context.execution_history.clear()
            self.rag_planner.execution_context.errors.clear()
    
    def emergency_stop(self):
        """Trigger emergency stop."""
        self.logger.warning("Emergency stop triggered")
        
        if self.robot_controller:
            try:
                self.robot_controller.trigger_emergency_stop()
                self.logger.info("Emergency stop executed")
            except Exception as e:
                self.logger.error(f"Failed to execute emergency stop: {e}")
        
        # Stop the system
        self.stop()
    
    def clear_emergency_stop(self):
        """Clear emergency stop."""
        if self.robot_controller:
            try:
                self.robot_controller.clear_emergency_stop()
                self.logger.info("Emergency stop cleared")
            except Exception as e:
                self.logger.error(f"Failed to clear emergency stop: {e}")
    
    def get_robot_status(self) -> Dict[str, Any]:
        """
        Get robot status.
        
        Returns:
            Robot status information
        """
        if self.robot_controller:
            try:
                return {
                    "connected": True,
                    "current_pose": self.robot_controller.get_current_position(),
                    "gripper_position": self.robot_controller.get_gripper_position(),
                    "gripper_status": self.robot_controller.get_gripper_status(),
                    "safety_status": self.robot_controller.get_safety_status()
                }
            except Exception as e:
                self.logger.error(f"Failed to get robot status: {e}")
                return {"connected": False, "error": str(e)}
        else:
            return {"connected": False, "simulation": True}
    
    def move_to_pose(self, xyz_mm: List[float], rpy_deg: List[float], speed: Optional[float] = None):
        """
        Move robot to a specific pose.
        
        Args:
            xyz_mm: Position in millimeters [x, y, z]
            rpy_deg: Orientation in degrees [roll, pitch, yaw]
            speed: Movement speed (optional)
        """
        if self.robot_controller:
            try:
                self.robot_controller.move_pose(xyz_mm, rpy_deg, speed)
                self.logger.info(f"Moved to pose: {xyz_mm}, {rpy_deg}")
            except Exception as e:
                self.logger.error(f"Failed to move to pose: {e}")
                raise
        else:
            self.logger.warning("Robot controller not available")
    
    def move_to_named_pose(self, pose_name: str):
        """
        Move robot to a named pose.
        
        Args:
            pose_name: Name of the pose
        """
        poses = self.get_available_poses()
        if pose_name in poses:
            pose = poses[pose_name]
            xyz_mm = pose.get('xyz_mm', [0, 0, 0])
            rpy_deg = pose.get('rpy_deg', [0, 0, 0])
            self.move_to_pose(xyz_mm, rpy_deg)
        else:
            raise ValueError(f"Pose '{pose_name}' not found")
    
    def open_gripper(self, position: Optional[float] = None, speed: Optional[float] = None, force: Optional[float] = None):
        """
        Open the gripper.
        
        Args:
            position: Gripper position (optional)
            speed: Gripper speed (optional)
            force: Gripper force (optional)
        """
        if self.robot_controller:
            try:
                self.robot_controller.open_gripper(position, speed, force)
                self.logger.info("Gripper opened")
            except Exception as e:
                self.logger.error(f"Failed to open gripper: {e}")
                raise
        else:
            self.logger.warning("Robot controller not available")
    
    def close_gripper(self, position: Optional[float] = None, speed: Optional[float] = None, force: Optional[float] = None):
        """
        Close the gripper.
        
        Args:
            position: Gripper position (optional)
            speed: Gripper speed (optional)
            force: Gripper force (optional)
        """
        if self.robot_controller:
            try:
                self.robot_controller.close_gripper(position, speed, force)
                self.logger.info("Gripper closed")
            except Exception as e:
                self.logger.error(f"Failed to close gripper: {e}")
                raise
        else:
            self.logger.warning("Robot controller not available")
    
    def gripper_grasp(self, target_position: Optional[float] = None, speed: Optional[float] = None, force: Optional[float] = None, timeout: Optional[float] = None):
        """
        Perform a grasp operation.
        
        Args:
            target_position: Target position (optional)
            speed: Gripper speed (optional)
            force: Gripper force (optional)
            timeout: Timeout in seconds (optional)
        """
        if self.robot_controller:
            try:
                self.robot_controller.gripper_grasp(target_position, speed, force, timeout)
                self.logger.info("Grasp operation completed")
            except Exception as e:
                self.logger.error(f"Failed to perform grasp: {e}")
                raise
        else:
            self.logger.warning("Robot controller not available")
    
    def gripper_release(self, target_position: Optional[float] = None, speed: Optional[float] = None, force: Optional[float] = None):
        """
        Release grasped object.
        
        Args:
            target_position: Target position (optional)
            speed: Gripper speed (optional)
            force: Gripper force (optional)
        """
        if self.robot_controller:
            try:
                self.robot_controller.gripper_release(target_position, speed, force)
                self.logger.info("Object released")
            except Exception as e:
                self.logger.error(f"Failed to release object: {e}")
                raise
        else:
            self.logger.warning("Robot controller not available")
    
    def get_system_info(self) -> Dict[str, Any]:
        """
        Get comprehensive system information.
        
        Returns:
            System information
        """
        return {
            "rag_integration": {
                "running": self.running,
                "current_task": self.current_task,
                "config": {
                    "robot_ip": self.config.robot_ip,
                    "sim": self.config.sim,
                    "dry_run": self.config.dry_run,
                    "enable_vision": self.config.enable_vision,
                    "max_retries": self.config.max_retries,
                    "scan_timeout": self.config.scan_timeout
                }
            },
            "components": {
                "robot_controller": self.robot_controller is not None,
                "vision_system": self.vision_system is not None,
                "executor": self.executor is not None,
                "rag_planner": self.rag_planner is not None
            },
            "status": self.get_status(),
            "robot_status": self.get_robot_status(),
            "available_poses": len(self.get_available_poses()),
            "available_actions": len(self.get_available_actions()),
            "planning_history_length": len(self.get_planning_history())
        }
