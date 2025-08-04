"""
Robot Control System - Main Package

This package provides a complete robotic control system for the ufactory850 arm
with vision, planning, and execution capabilities.
"""

__version__ = "1.0.0"
__author__ = "ELLMER Integration Team"

# Import only non-ROS2 dependent modules
from .utils import ConfigManager

__all__ = [
    'ConfigManager'
] 