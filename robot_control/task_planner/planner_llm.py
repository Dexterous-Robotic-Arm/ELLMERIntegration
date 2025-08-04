#!/usr/bin/env python3
"""
planner_llm.py - LLM-based task planning

Provides functions for generating robot action plans using Google Gemini LLM
or fallback planning when LLM is not available.
"""

import os
import json
import logging
from typing import List, Dict, Any, Optional

# Optional imports
try:
    import google.generativeai as genai
    GEMINI_AVAILABLE = True
except ImportError:
    GEMINI_AVAILABLE = False
    print("Warning: Google Generative AI not available")

logger = logging.getLogger(__name__)

# Default action schema for the LLM
DEFAULT_ACTION_SCHEMA = """
# Robot Action Schema

The robot can perform these actions:

## Movement Actions
- MOVE_TO_NAMED: Move to a predefined named pose
  - pose_name: string (e.g., "home", "staging")
- APPROACH_NAMED: Move to a position above a named pose
  - pose_name: string
  - hover_mm: number (default: 80)
- MOVE_TO_OBJECT: Move directly to a detected object
  - label: string (object class name)
  - offset_mm: [x, y, z] (default: [0, 0, 0])
- APPROACH_OBJECT: Move to a position above a detected object
  - label: string
  - hover_mm: number (default: 80)
  - offset_mm: [x, y, z] (default: [0, 0, 0])
- MOVE_TO_POSE: Move to specific coordinates
  - xyz_mm: [x, y, z] in millimeters
  - rpy_deg: [roll, pitch, yaw] in degrees
- RETREAT_Z: Move up by a specified distance
  - distance_mm: number

## Gripper Actions
- OPEN_GRIPPER: Open the gripper
  - position: number (default: 850)
  - speed: number (default: 200)
  - force: number (default: 50)
- CLOSE_GRIPPER: Close the gripper
  - position: number (default: 200)
  - speed: number (default: 200)
  - force: number (default: 50)
- SET_GRIPPER_POSITION: Set gripper to specific position
  - position: number (0-850)
  - speed: number (default: 200)
  - force: number (default: 50)
- GRIPPER_GRASP: Grasp with force feedback
  - target_position: number (default: 200)
  - speed: number (default: 200)
  - force: number (default: 50)
  - timeout: number (default: 5.0)
- GRIPPER_RELEASE: Release grasped object
  - target_position: number (default: 850)
  - speed: number (default: 200)
  - force: number (default: 50)

## Utility Actions
- SLEEP: Wait for specified time
  - seconds: number

## Plan Format
Return a JSON array of action objects, each with:
- action: string (action name)
- params: object (action parameters)
"""


def plan_with_gemini(task: str, pose_names: Optional[List[str]] = None) -> List[Dict[str, Any]]:
    """
    Generate a robot action plan using Google Gemini LLM.
    
    Args:
        task: Natural language task description
        pose_names: List of available named poses
        
    Returns:
        List of action dictionaries
    """
    if not GEMINI_AVAILABLE:
        logger.warning("Google Gemini not available, using fallback plan")
        return plan_fallback(task)
    
    try:
        # Get API key
        api_key = os.environ.get('GEMINI_API_KEY')
        if not api_key:
            logger.warning("GEMINI_API_KEY not set, using fallback plan")
            return plan_fallback(task)
        
        # Configure Gemini
        genai.configure(api_key=api_key)
        model = genai.GenerativeModel('gemini-pro')
        
        # Build prompt
        pose_context = ""
        if pose_names:
            pose_context = f"\nAvailable named poses: {', '.join(pose_names)}"
        
        prompt = f"""
{DEFAULT_ACTION_SCHEMA}

{pose_context}

Task: {task}

Generate a JSON plan for the robot to complete this task. 
Return ONLY valid JSON array of action objects, no other text.

Example plan format:
[
  {{"action": "MOVE_TO_NAMED", "params": {{"pose_name": "home"}}}},
  {{"action": "OPEN_GRIPPER", "params": {{}}}},
  {{"action": "MOVE_TO_OBJECT", "params": {{"label": "cup"}}}},
  {{"action": "CLOSE_GRIPPER", "params": {{}}}},
  {{"action": "MOVE_TO_NAMED", "params": {{"pose_name": "home"}}}}
]
"""
        
        # Generate plan
        response = model.generate_content(prompt)
        
        # Parse JSON response
        try:
            plan = json.loads(response.text)
            if isinstance(plan, list):
                logger.info(f"Generated plan with {len(plan)} actions")
                return plan
            else:
                logger.error("LLM response is not a list")
                return plan_fallback(task)
        except json.JSONDecodeError as e:
            logger.error(f"Failed to parse LLM response as JSON: {e}")
            return plan_fallback(task)
            
    except Exception as e:
        logger.error(f"Error generating plan with Gemini: {e}")
        return plan_fallback(task)


def plan_fallback(task: str) -> List[Dict[str, Any]]:
    """
    Generate a fallback plan when LLM is not available.
    
    Args:
        task: Natural language task description
        
    Returns:
        List of action dictionaries
    """
    logger.info("Using fallback plan generator")
    
    # Simple keyword-based planning
    task_lower = task.lower()
    
    if "home" in task_lower or "go home" in task_lower:
        return [
            {"action": "MOVE_TO_NAMED", "params": {"pose_name": "home"}}
        ]
    
    elif "pick" in task_lower and ("cup" in task_lower or "object" in task_lower):
        return [
            {"action": "MOVE_TO_NAMED", "params": {"pose_name": "home"}},
            {"action": "OPEN_GRIPPER", "params": {}},
            {"action": "MOVE_TO_OBJECT", "params": {"label": "cup"}},
            {"action": "CLOSE_GRIPPER", "params": {}},
            {"action": "MOVE_TO_NAMED", "params": {"pose_name": "home"}}
        ]
    
    elif "place" in task_lower or "drop" in task_lower:
        return [
            {"action": "MOVE_TO_NAMED", "params": {"pose_name": "home"}},
            {"action": "OPEN_GRIPPER", "params": {}},
            {"action": "MOVE_TO_NAMED", "params": {"pose_name": "home"}}
        ]
    
    elif "gripper" in task_lower:
        if "open" in task_lower:
            return [{"action": "OPEN_GRIPPER", "params": {}}]
        elif "close" in task_lower:
            return [{"action": "CLOSE_GRIPPER", "params": {}}]
        else:
            return [
                {"action": "OPEN_GRIPPER", "params": {}},
                {"action": "SLEEP", "params": {"seconds": 1}},
                {"action": "CLOSE_GRIPPER", "params": {}}
            ]
    
    else:
        # Default plan
        return [
            {"action": "MOVE_TO_NAMED", "params": {"pose_name": "home"}},
            {"action": "SLEEP", "params": {"seconds": 1}}
        ]


def validate_plan(plan: List[Dict[str, Any]]) -> bool:
    """
    Validate that a plan has the correct structure.
    
    Args:
        plan: List of action dictionaries
        
    Returns:
        True if valid, False otherwise
    """
    if not isinstance(plan, list):
        return False
    
    valid_actions = {
        "MOVE_TO_NAMED", "APPROACH_NAMED", "MOVE_TO_OBJECT", "APPROACH_OBJECT",
        "MOVE_TO_POSE", "RETREAT_Z", "OPEN_GRIPPER", "CLOSE_GRIPPER",
        "SET_GRIPPER_POSITION", "GRIPPER_GRASP", "GRIPPER_RELEASE",
        "SLEEP"
    }
    
    for action in plan:
        if not isinstance(action, dict):
            return False
        if "action" not in action:
            return False
        if action["action"] not in valid_actions:
            return False
        if "params" not in action:
            return False
        if not isinstance(action["params"], dict):
            return False
    
    return True 