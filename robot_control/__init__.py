"""
Robot Control System

A comprehensive robot control system for the ufactory850 robotic arm with
Intel RealSense D435 camera, designed for real-world autonomous tasks using
True RAG-based intelligence and computer vision.

Key Components:
- rag_system: True RAG-based intelligent planning system with semantic knowledge retrieval
- robot_controller: Robot control and execution
- vision_system: Object detection and pose recording
- utils: Utility functions

Usage:
    # True RAG-based system (recommended)
    from robot_control.rag_system import TrueRAGPlanner
    
    # Core components
    from robot_control.robot_controller import XArmRunner
    from robot_control.vision_system import PoseRecorder
"""

from .robot_controller import XArmRunner
from .utils import ConfigManager, setup_logging

__all__ = [
    'XArmRunner',
    'ConfigManager',
    'setup_logging'
] 