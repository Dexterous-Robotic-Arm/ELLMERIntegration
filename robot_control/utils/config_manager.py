"""
Configuration Manager

Handles loading and validation of configuration files for the robot control system.
"""

import yaml
from pathlib import Path
from typing import Dict, Any


class ConfigManager:
    """Manages configuration loading and validation for the robot control system."""
    
    def __init__(self, config_dir: str = "config"):
        """
        Initialize the configuration manager.
        
        Args:
            config_dir: Directory containing configuration files
        """
        self.config_dir = Path(config_dir)
        self._configs = {}
        
    def load_config(self, config_type: str, filename: str) -> Dict[str, Any]:
        """
        Load a configuration file.
        
        Args:
            config_type: Type of config (robot, vision, llm)
            filename: Name of the configuration file
            
        Returns:
            Configuration dictionary
        """
        config_path = self.config_dir / config_type / filename
        
        if not config_path.exists():
            raise FileNotFoundError(f"Configuration file not found: {config_path}")
            
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            
        self._configs[f"{config_type}_{filename}"] = config
        return config
        
    def get_robot_config(self) -> Dict[str, Any]:
        """Load robot configuration."""
        return self.load_config("robot", "safety_config.yaml")
        
    def get_world_config(self) -> Dict[str, Any]:
        """Load world model configuration."""
        return self.load_config("robot", "world_model.yaml")
        
    def get_vision_config(self) -> Dict[str, Any]:
        """Load vision configuration."""
        return self.load_config("vision", "camera_config.yaml")
        
    def get_all_configs(self) -> Dict[str, Any]:
        """Load all configuration files."""
        configs = {}
        configs.update(self.get_robot_config())
        configs.update(self.get_world_config())
        configs.update(self.get_vision_config())
        return configs 