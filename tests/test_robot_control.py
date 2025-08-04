#!/usr/bin/env python3
"""
Basic tests for the robot control system.
"""

import sys
import os
import unittest
from pathlib import Path

# Add the robot_control package to the path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_control.utils import ConfigManager, setup_logging


class TestRobotControl(unittest.TestCase):
    """Test cases for the robot control system."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.config_manager = ConfigManager()
        self.logger = setup_logging(level="DEBUG")
        
    def test_config_manager_initialization(self):
        """Test that ConfigManager can be initialized."""
        self.assertIsNotNone(self.config_manager)
        self.assertEqual(self.config_manager.config_dir.name, "config")
        
    def test_logging_setup(self):
        """Test that logging can be set up."""
        logger = setup_logging(level="INFO")
        self.assertIsNotNone(logger)
        self.assertEqual(logger.level, 20)  # INFO level
        
    def test_config_directory_structure(self):
        """Test that configuration directories exist."""
        config_dir = Path("config")
        self.assertTrue(config_dir.exists())
        
        robot_config = config_dir / "robot"
        self.assertTrue(robot_config.exists())
        
        vision_config = config_dir / "vision"
        self.assertTrue(vision_config.exists())
        
        llm_config = config_dir / "llm"
        self.assertTrue(llm_config.exists())
        
    def test_robot_config_files(self):
        """Test that robot configuration files exist."""
        robot_config = Path("config/robot")
        
        safety_config = robot_config / "safety_config.yaml"
        self.assertTrue(safety_config.exists())
        
        world_config = robot_config / "world_model.yaml"
        self.assertTrue(world_config.exists())
        
    def test_vision_config_files(self):
        """Test that vision configuration files exist."""
        vision_config = Path("config/vision")
        
        camera_config = vision_config / "camera_config.yaml"
        self.assertTrue(camera_config.exists())
        
    def test_llm_config_files(self):
        """Test that LLM configuration files exist."""
        llm_config = Path("config/llm")
        
        action_schema = llm_config / "action_schema.md"
        self.assertTrue(action_schema.exists())


if __name__ == "__main__":
    unittest.main() 