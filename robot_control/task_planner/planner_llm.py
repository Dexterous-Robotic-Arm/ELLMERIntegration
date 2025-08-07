#!/usr/bin/env python3
"""
planner_llm.py - LLM-based task planning

Provides functions for generating robot action plans using Google Gemini LLM
or fallback planning when LLM is not available.

This planner reads the action schema from config/llm/action_schema.md and
generates plans in the format expected by the executor:
{
  "goal": "task description",
  "steps": [
    {"action": "MOVE_TO_NAMED", "name": "home"},
    {"action": "OPEN_GRIPPER", "gripper": {"position": 850}}
  ]
}
"""

import os
import json
import logging
import re
from pathlib import Path
from typing import List, Dict, Any, Optional

# Optional imports
try:
    import google.generativeai as genai
    GEMINI_AVAILABLE = True
except ImportError:
    GEMINI_AVAILABLE = False
    print("Warning: Google Generative AI not available")

logger = logging.getLogger(__name__)


def load_action_schema() -> str:
    """
    Load the action schema from config/llm/action_schema.md.
    
    Returns:
        Action schema content as string
    """
    schema_path = Path("config/llm/action_schema.md")
    
    if not schema_path.exists():
        logger.error(f"Action schema file not found: {schema_path}")
        return get_fallback_schema()
    
    try:
        with open(schema_path, 'r', encoding='utf-8') as f:
            content = f.read()
        logger.info(f"Loaded action schema from {schema_path}")
        return content
    except Exception as e:
        logger.error(f"Failed to load action schema: {e}")
        return get_fallback_schema()


def get_fallback_schema() -> str:
    """
    Get a fallback action schema if the file cannot be loaded.
    
    Returns:
        Basic action schema as string
    """
    return """
# xArm Action Plan Contract (LLM-Facing)

## Output Rules
- Output must be a single, valid **JSON object**.
- **Do not** wrap the JSON in markdown fences or add text before/after.
- JSON **must** conform to the schema below.

## Actions (Verbs) — Movement and Gripper Control
- `MOVE_TO_NAMED` — Move to a named pose from the world model.  
  - **fields:** `name` (string)
- `APPROACH_NAMED` — Move to a hover position above a named pose.  
  - **fields:** `name` (string), `hover_mm` (number, optional; default **80**)
- `MOVE_TO_OBJECT` — Move to a position derived from a detected object label.  
  - **fields:** `label` (string) or `labels` (array), `offset_mm` ([x,y,z] mm, optional; default **[0,0,0]**), `timeout_sec` (number, optional; default **5**)
- `APPROACH_OBJECT` — Move to a hover position above a detected object label.  
  - **fields:** `label` (string) or `labels` (array), `hover_mm` (number, optional; default **80**), `timeout_sec` (number, optional; default **5**)
- `RETREAT_Z` — Move upward along Z by `dz_mm` (positive).  
  - **fields:** `dz_mm` (number > 0)
- `MOVE_TO_POSE` — Move to an explicit numeric pose.  
  - **fields:** `pose` (object: `xyz_mm` [x,y,z], `rpy_deg` [r,p,y])
- `SLEEP` — Wait for a number of seconds.  
  - **fields:** `seconds` (number)

### Gripper Actions
- `OPEN_GRIPPER` — Open the gripper to specified position or fully open.  
  - **fields:** `gripper` (object, optional: `position` (number, 0-850), `speed` (number, optional; default **200**), `force` (number, optional; default **50**))
- `CLOSE_GRIPPER` — Close the gripper to specified position or fully closed.  
  - **fields:** `gripper` (object, optional: `position` (number, 0-850), `speed` (number, optional; default **100**), `force` (number, optional; default **50**))
- `SET_GRIPPER_POSITION` — Set gripper to a specific position.  
  - **fields:** `position` (number, 0-850), `speed` (number, optional; default **150**), `force` (number, optional; default **50**)
- `GRIPPER_GRASP` — Perform a grasp operation with force feedback.  
  - **fields:** `target_position` (number, optional; default **200**), `speed` (number, optional; default **100**), `force` (number, optional; default **50**), `timeout` (number, optional; default **5.0**)
- `GRIPPER_RELEASE` — Release grasped object by opening gripper.  
  - **fields:** `target_position` (number, optional; default **850**), `speed` (number, optional; default **200**), `force` (number, optional; default **50**)
- `GRIPPER_HALF_OPEN` — Open gripper to half position.  
  - **fields:** `speed` (number, optional; default **150**), `force` (number, optional; default **50**)
- `GRIPPER_SOFT_CLOSE` — Close gripper gently with low force.  
  - **fields:** `speed` (number, optional; default **50**), `force` (number, optional; default **30**)
- `GRIPPER_TEST` — Test gripper by cycling open/close.  
  - **fields:** `cycles` (number, optional; default **3**), `delay` (number, optional; default **1.0**)

## Required Top-Level Keys
- `goal` (string): the user's task in natural language.
- `steps` (array): ordered list of action objects.

## Valid JSON Example - Pick and Place with Gripper
```json
{
  "goal": "Pick up the cup and place it in the bin",
  "steps": [
    { "action": "MOVE_TO_NAMED", "name": "home" },
    { "action": "OPEN_GRIPPER", "gripper": {"position": 850, "speed": 200} },
    { "action": "APPROACH_OBJECT", "label": "cup", "hover_mm": 80, "timeout_sec": 5 },
    { "action": "MOVE_TO_OBJECT", "label": "cup", "offset_mm": [0, 0, 0], "timeout_sec": 5 },
    { "action": "GRIPPER_GRASP", "target_position": 200, "speed": 100, "force": 50 },
    { "action": "RETREAT_Z", "dz_mm": 80 },
    { "action": "MOVE_TO_NAMED", "name": "bin_drop" },
    { "action": "GRIPPER_RELEASE", "target_position": 850, "speed": 200 },
    { "action": "MOVE_TO_NAMED", "name": "home" }
  ]
}
```
"""


def plan_with_gemini(task: str, pose_names: Optional[List[str]] = None) -> Dict[str, Any]:
    """
    Generate a robot action plan using Google Gemini LLM.
    
    Args:
        task: Natural language task description
        pose_names: List of available named poses
        
    Returns:
        Plan dictionary with 'goal' and 'steps' keys
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
        model = genai.GenerativeModel('gemini-1.5-pro')
        
        # Load action schema
        action_schema = load_action_schema()
        
        # Build optimized prompt (minimal tokens)
        pose_context = ""
        if pose_names:
            pose_context = f"Poses: {','.join(pose_names)}"
        
        # Ultra-compact prompt to minimize token usage
        prompt = f"""Task: {task}
{pose_context}
Actions: MOVE_TO_NAMED,MOVE_TO_POSE,APPROACH_OBJECT,OPEN_GRIPPER,CLOSE_GRIPPER,SLEEP,RETREAT_Z
Return JSON: {{"goal":"description","steps":[{{"action":"ACTION","params"}}]}}"""
        
        # Generate plan
        response = model.generate_content(prompt)
        
        # Parse JSON response
        try:
            # Clean the response text to extract JSON
            response_text = response.text.strip()
            
            # Remove markdown code fences if present
            if response_text.startswith('```json'):
                response_text = response_text[7:]
            if response_text.startswith('```'):
                response_text = response_text[3:]
            if response_text.endswith('```'):
                response_text = response_text[:-3]
            
            response_text = response_text.strip()
            
            plan = json.loads(response_text)
            
            # Validate plan structure
            if isinstance(plan, dict) and "goal" in plan and "steps" in plan:
                logger.info(f"Generated plan with {len(plan['steps'])} steps")
                return plan
            else:
                logger.error("LLM response is not a valid plan structure")
                return plan_fallback(task)
                
        except json.JSONDecodeError as e:
            logger.error(f"Failed to parse LLM response as JSON: {e}")
            logger.error(f"Response text: {response.text}")
            return plan_fallback(task)
            
    except Exception as e:
        logger.error(f"Error generating plan with Gemini: {e}")
        raise e


def plan_fallback(task: str) -> Dict[str, Any]:
    """
    Generate a fallback plan when LLM is not available.
    
    Args:
        task: Natural language task description
        
    Returns:
        Plan dictionary with 'goal' and 'steps' keys
    """
    logger.info("Using fallback plan generator")
    
    # Simple keyword-based planning
    task_lower = task.lower()
    
    if "home" in task_lower or "go home" in task_lower:
        return {
            "goal": task,
            "steps": [
                {"action": "MOVE_TO_NAMED", "name": "home"}
            ]
        }

    # Camera-mounted: scan/search pattern
    elif "scan" in task_lower or "search" in task_lower:
        return {
            "goal": task,
            "steps": [
                {"action": "SCAN_FOR_OBJECTS", "pattern": "horizontal", "sweep_mm": 300, "steps": 5, "pause_sec": 1.0}
            ]
        }

    # Camera-mounted: approach/move/detect patterns
    elif "approach" in task_lower and any(obj in task_lower for obj in ["can", "bottle", "cup", "object"]):
        object_type = "can"
        for obj in ["can", "bottle", "cup", "object"]:
            if obj in task_lower:
                object_type = obj
                break
        return {
            "goal": task,
            "steps": [
                {"action": "APPROACH_OBJECT", "label": object_type, "hover_mm": 80, "timeout_sec": 5}
            ]
        }
    elif "move to" in task_lower and any(obj in task_lower for obj in ["can", "bottle", "cup", "object"]):
        object_type = "can"
        for obj in ["can", "bottle", "cup", "object"]:
            if obj in task_lower:
                object_type = obj
                break
        return {
            "goal": task,
            "steps": [
                {"action": "MOVE_TO_OBJECT", "label": object_type, "offset_mm": [0, 0, 0], "timeout_sec": 5}
            ]
        }
    elif "detect" in task_lower:
        return {
            "goal": task,
            "steps": [
                {"action": "SLEEP", "seconds": 2}
            ]
        }

    elif "pick" in task_lower and ("cup" in task_lower or "object" in task_lower):
        return {
            "goal": task,
            "steps": [
                {"action": "OPEN_GRIPPER", "gripper": {"position": 850}},
                {"action": "APPROACH_OBJECT", "label": "cup", "hover_mm": 80, "timeout_sec": 5},
                {"action": "MOVE_TO_OBJECT", "label": "cup", "offset_mm": [0, 0, 0], "timeout_sec": 5},
                {"action": "GRIPPER_GRASP", "target_position": 200, "speed": 100, "force": 50},
                {"action": "RETREAT_Z", "dz_mm": 80}
            ]
        }
    elif "place" in task_lower or "drop" in task_lower:
        return {
            "goal": task,
            "steps": [
                {"action": "GRIPPER_RELEASE", "target_position": 850, "speed": 200}
            ]
        }
    elif "gripper" in task_lower:
        if "open" in task_lower:
            return {
                "goal": task,
                "steps": [
                    {"action": "OPEN_GRIPPER", "gripper": {"position": 850}}
                ]
            }
        elif "close" in task_lower:
            return {
                "goal": task,
                "steps": [
                    {"action": "CLOSE_GRIPPER", "gripper": {"position": 200}}
                ]
            }
        elif "test" in task_lower:
            return {
                "goal": task,
                "steps": [
                    {"action": "GRIPPER_TEST", "cycles": 3, "delay": 1.0}
                ]
            }
        else:
            return {
                "goal": task,
                "steps": [
                    {"action": "OPEN_GRIPPER", "gripper": {"position": 850}},
                    {"action": "SLEEP", "seconds": 1},
                    {"action": "CLOSE_GRIPPER", "gripper": {"position": 200}}
                ]
            }
    else:
        # Default plan: do nothing (just sleep)
        return {
            "goal": task,
            "steps": [
                {"action": "SLEEP", "seconds": 1}
            ]
        }


def validate_plan(plan: Dict[str, Any]) -> bool:
    """
    Validate that a plan has the correct structure.
    
    Args:
        plan: Plan dictionary with 'goal' and 'steps' keys
        
    Returns:
        True if valid, False otherwise
    """
    if not isinstance(plan, dict):
        return False
    
    if "goal" not in plan or "steps" not in plan:
        return False
    
    if not isinstance(plan["goal"], str):
        return False
    
    if not isinstance(plan["steps"], list):
        return False
    
    valid_actions = {
        "MOVE_TO_NAMED", "APPROACH_NAMED", "MOVE_TO_OBJECT", "APPROACH_OBJECT",
        "MOVE_TO_POSE", "RETREAT_Z", "OPEN_GRIPPER", "CLOSE_GRIPPER",
        "SET_GRIPPER_POSITION", "GRIPPER_GRASP", "GRIPPER_RELEASE",
        "GRIPPER_HALF_OPEN", "GRIPPER_SOFT_CLOSE", "GRIPPER_TEST", "SLEEP",
        "SCAN_FOR_OBJECTS"
    }
    
    for step in plan["steps"]:
        if not isinstance(step, dict):
            return False
        if "action" not in step:
            return False
        if step["action"] not in valid_actions:
            return False
    
    return True


def convert_old_format_to_new(old_plan: List[Dict[str, Any]]) -> Dict[str, Any]:
    """
    Convert old plan format (list of actions with params) to new format (goal + steps).
    
    Args:
        old_plan: Old format plan as list of action dictionaries
        
    Returns:
        New format plan as dictionary with 'goal' and 'steps' keys
    """
    steps = []
    
    for action in old_plan:
        if "action" in action and "params" in action:
            # Convert from old format to new format
            new_step = {"action": action["action"]}
            
            # Flatten params into the step
            for key, value in action["params"].items():
                new_step[key] = value
            
            steps.append(new_step)
    
    return {
        "goal": "Converted from old format",
        "steps": steps
    } 