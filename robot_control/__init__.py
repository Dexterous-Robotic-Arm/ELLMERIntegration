"""
Robot Control System

A comprehensive robot control system for the ufactory850 robotic arm with
Intel RealSense D435 camera, designed for real-world autonomous tasks using
LLM planning and computer vision.

Key Components:
- rag_system: RAG-based LLM planning system (NEW)
- robot_controller: Robot control and execution
- vision_system: Object detection and pose recording
- utils: Utility functions

Usage:
    # RAG-based system (recommended)
    from robot_control.rag_system import RAGPlanner, MovementLogic
    from robot_control.rag_system.rag_main import main as rag_main
    
    # Legacy system
    from robot_control.robot_controller import XArmRunner
    from robot_control.vision_system import PoseRecorder
"""

from .robot_controller import XArmRunner
from .vision_system import PoseRecorder
from .utils import ConfigManager, setup_logging

__all__ = [
    'XArmRunner',
    'PoseRecorder', 
    'ConfigManager',
    'setup_logging'
] 