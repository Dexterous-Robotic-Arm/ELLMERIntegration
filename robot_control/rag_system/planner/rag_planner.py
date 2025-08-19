#!/usr/bin/env python3
"""
RAG-based LLM Planner - The "Brains" of the Robot Operation

This module implements a Retrieval-Augmented Generation (RAG) pipeline for robot control.
The LLM acts as the central decision maker, using:
1. Predefined actions and poses from existing files
2. Real-time vision feedback
3. Task context and goals
4. Execution history and feedback

The system can:
- Scan and rescan when confused
- Make decisions based on current state
- Adapt plans based on feedback
- Handle uncertainty and errors gracefully
"""

import os
import json
import time
import logging
import threading
from pathlib import Path
from typing import List, Dict, Any, Optional, Tuple
from dataclasses import dataclass, field
from enum import Enum

# Optional imports
try:
    import google.generativeai as genai
    GEMINI_AVAILABLE = True
except ImportError:
    GEMINI_AVAILABLE = False
    print("Warning: Google Generative AI not available")

try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False
    print("Warning: YAML not available")

# Import enhanced movement logic
from .movement_logic import MovementLogic, CameraConfig, ObjectDetection

logger = logging.getLogger(__name__)


class PlanningState(Enum):
    """States of the planning system."""
    IDLE = "idle"
    SCANNING = "scanning"
    PLANNING = "planning"
    EXECUTING = "executing"
    WAITING_FOR_FEEDBACK = "waiting_for_feedback"
    ERROR = "error"
    COMPLETED = "completed"


@dataclass
class VisionFeedback:
    """Vision system feedback data."""
    objects_detected: List[Dict[str, Any]] = field(default_factory=list)
    confidence_scores: Dict[str, float] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)
    scan_quality: float = 0.0  # 0.0 to 1.0


@dataclass
class ExecutionContext:
    """Context for plan execution."""
    current_pose: Optional[List[float]] = None
    gripper_state: Optional[str] = None
    last_action: Optional[str] = None
    execution_history: List[Dict[str, Any]] = field(default_factory=list)
    errors: List[str] = field(default_factory=list)
    start_time: float = field(default_factory=time.time)


@dataclass
class RAGContext:
    """RAG context for LLM decision making."""
    task_description: str
    available_actions: List[Dict[str, Any]]
    available_poses: Dict[str, Dict[str, Any]]
    vision_feedback: VisionFeedback
    execution_context: ExecutionContext
    world_model: Dict[str, Any] = field(default_factory=dict)
    planning_history: List[Dict[str, Any]] = field(default_factory=list)
    interaction_requirements: Dict[str, Any] = field(default_factory=dict)


class RAGPlanner:
    """
    RAG-based LLM planner that acts as the central decision maker.
    
    This planner integrates:
    - Predefined actions and poses
    - Real-time vision feedback
    - Execution context and history
    - Task goals and constraints
    - Enhanced movement logic from detect_and_move.py
    """
    
    def __init__(self, 
                 robot_controller=None,
                 vision_system=None,
                 config_path: str = "config/",
                 max_retries: int = 3,
                 scan_timeout: float = 10.0):
        """
        Initialize the RAG planner.
        
        Args:
            robot_controller: Robot controller instance
            vision_system: Vision system instance
            config_path: Path to configuration files
            max_retries: Maximum retries for failed plans
            scan_timeout: Timeout for scanning operations
        """
        self.robot_controller = robot_controller
        self.vision_system = vision_system
        self.config_path = Path(config_path)
        self.max_retries = max_retries
        self.scan_timeout = scan_timeout
        
        # State management
        self.current_state = PlanningState.IDLE
        self.state_lock = threading.Lock()
        
        # Load configurations
        self.action_schema = self._load_action_schema()
        self.world_model = self._load_world_model()
        self.available_poses = self._extract_poses()
        self.available_actions = self._extract_actions()
        
        # Initialize LLM
        self.llm = self._initialize_llm()
        
        # Context storage
        self.execution_context = ExecutionContext()
        self.vision_feedback = VisionFeedback()
        
        # Initialize enhanced movement logic
        self.movement_logic = MovementLogic()
        
        logger.info("RAG Planner initialized successfully with enhanced movement logic")
    
    def _load_action_schema(self) -> str:
        """Load the action schema from config."""
        schema_path = self.config_path / "llm" / "action_schema.md"
        if schema_path.exists():
            with open(schema_path, 'r', encoding='utf-8') as f:
                return f.read()
        else:
            logger.warning(f"Action schema not found at {schema_path}")
            return self._get_fallback_schema()
    
    def _load_world_model(self) -> Dict[str, Any]:
        """Load the world model configuration."""
        world_path = self.config_path / "robot" / "world_model.yaml"
        if world_path.exists() and YAML_AVAILABLE:
            try:
                with open(world_path, 'r', encoding='utf-8') as f:
                    return yaml.safe_load(f) or {}
            except Exception as e:
                logger.error(f"Failed to load world model: {e}")
        return {}
    
    def _extract_poses(self) -> Dict[str, Dict[str, Any]]:
        """Extract available poses from world model."""
        poses = {}
        if self.world_model:
            poses = self.world_model.get("poses", {})
        return poses
    
    def _extract_actions(self) -> List[Dict[str, Any]]:
        """Extract available actions from action schema."""
        # Parse the action schema to extract available actions
        actions = []
        lines = self.action_schema.split('\n')
        current_action = None
        
        for line in lines:
            line = line.strip()
            if line.startswith('- `') and '` —' in line:
                # Extract action name
                action_name = line.split('`')[1]
                description = line.split('—', 1)[1].strip() if '—' in line else ""
                current_action = {
                    "name": action_name,
                    "description": description,
                    "fields": []
                }
                actions.append(current_action)
            elif current_action and line.startswith('  - **fields:**'):
                # Extract fields
                fields_text = line.replace('  - **fields:**', '').strip()
                if fields_text:
                    current_action["fields_text"] = fields_text
        
        return actions
    
    def _get_fallback_schema(self) -> str:
        """Get fallback action schema."""
        return """
# xArm Action Plan Contract (LLM-Facing)

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
- `SCAN_FOR_OBJECTS` — Physically move robot to scan the workspace area for objects.  
  - **fields:** `pattern` (string, optional; default **"horizontal"**), `sweep_mm` (number, optional; default **300**), `steps` (number, optional; default **5**), `pause_sec` (number, optional; default **1.0**)
- `SCAN_AREA` — Alternative name for SCAN_FOR_OBJECTS (same functionality).  
  - **fields:** `scan_duration` (number, optional; default **5**), `scan_area` (string, optional; default **"current"**)

### Gripper Actions
- `OPEN_GRIPPER` — Open the gripper to specified position or fully open.  
  - **fields:** `gripper` (object, optional: `position` (number, 0-850), `speed` (number, optional; default **200**), `force` (number, optional; default **50**))
- `CLOSE_GRIPPER` — Close the gripper to specified position or fully closed.  
  - **fields:** `gripper` (object, optional: `position` (number, 0-850), `speed` (number, optional; default **100**), `force` (number, optional; default **50**))
- `GRIPPER_GRASP` — Perform a grasp operation with force feedback.  
  - **fields:** `target_position` (number, optional; default **200**), `speed` (number, optional; default **100**), `force` (number, optional; default **50**), `timeout` (number, optional; default **5.0**)
- `GRIPPER_RELEASE` — Release grasped object by opening gripper.  
  - **fields:** `target_position` (number, optional; default **850**), `speed` (number, optional; default **200**), `force` (number, optional; default **50**)
        """
    
    def _initialize_llm(self):
        """Initialize the LLM for planning."""
        if GEMINI_AVAILABLE:
            try:
                # Configure Gemini
                genai.configure(api_key=os.getenv('GOOGLE_API_KEY'))
                # Try different Gemini models in order of preference
                model_names = ['gemini-2.0-flash-exp', 'gemini-1.5-flash', 'gemini-pro']
                model = None
                
                for model_name in model_names:
                    try:
                        model = genai.GenerativeModel(model_name)
                        # Test the model with a simple prompt
                        response = model.generate_content("Hello")
                        logger.info(f"Successfully initialized Gemini model: {model_name}")
                        break
                    except Exception as e:
                        logger.warning(f"Failed to initialize {model_name}: {e}")
                        continue
                
                if model is None:
                    raise Exception("No Gemini model could be initialized")
                logger.info("Gemini LLM initialized successfully")
                return model
            except Exception as e:
                logger.error(f"Failed to initialize Gemini: {e}")
        
        logger.warning("Using fallback planning (no LLM available)")
        return None
    
    def _update_state(self, new_state: PlanningState):
        """Update the current planning state."""
        with self.state_lock:
            old_state = self.current_state
            self.current_state = new_state
            logger.info(f"Planning state changed: {old_state.value} -> {new_state.value}")
    
    def _get_vision_feedback(self) -> VisionFeedback:
        """Get current vision feedback."""
        if self.vision_system:
            try:
                # This would integrate with the actual vision system
                # For now, return a mock feedback with enhanced object detection
                return VisionFeedback(
                    objects_detected=self._get_enhanced_object_detections(),
                    confidence_scores=self._get_confidence_scores(),
                    scan_quality=0.8
                )
            except Exception as e:
                logger.error(f"Failed to get vision feedback: {e}")
        
        return VisionFeedback()
    
    def _get_enhanced_object_detections(self) -> List[Dict[str, Any]]:
        """
        Get enhanced object detections including kitchen appliances.
        
        Returns:
            List of detected objects with enhanced information
        """
        # This would integrate with the actual vision system
        # For now, return mock detections with enhanced information
        mock_objects = [
            {
                "class": "cup",
                "pos": [400, -150, 45],
                "confidence": 0.9,
                "size": [50, 50, 80],
                "state": "static",
                "pixel_center": [320, 240],
                "depth": 0.5,
                "bbox": [300, 200, 340, 280]
            },
            {
                "class": "microwave",
                "pos": [350, 100, 0],
                "confidence": 0.95,
                "size": [300, 400, 250],
                "state": "closed",
                "interaction_points": ["door_handle", "control_panel"],
                "pixel_center": [400, 200],
                "depth": 0.8,
                "bbox": [350, 150, 450, 250]
            },
            {
                "class": "oven",
                "pos": [400, 0, 0],
                "confidence": 0.92,
                "size": [400, 500, 300],
                "state": "closed",
                "interaction_points": ["door_handle", "temperature_knob"],
                "pixel_center": [450, 180],
                "depth": 1.2,
                "bbox": [400, 130, 500, 230]
            },
            {
                "class": "dishwasher",
                "pos": [300, -150, 0],
                "confidence": 0.88,
                "size": [350, 450, 200],
                "state": "closed",
                "interaction_points": ["door_handle", "start_button"],
                "pixel_center": [280, 220],
                "depth": 0.9,
                "bbox": [250, 170, 310, 270]
            }
        ]
        
        return mock_objects
    
    def _create_object_detection_from_vision_data(self, vision_data: Dict[str, Any]) -> ObjectDetection:
        """
        Create ObjectDetection from vision system data.
        
        Args:
            vision_data: Raw vision data from the system
            
        Returns:
            ObjectDetection object
        """
        return ObjectDetection(
            class_name=vision_data.get("class", "unknown"),
            confidence=vision_data.get("confidence", 0.0),
            pixel_center=vision_data.get("pixel_center", (320, 240)),
            depth=vision_data.get("depth", 0.5),
            bbox=vision_data.get("bbox", (0, 0, 640, 480))
        )
    
    def _calculate_object_target_position(self, object_data: Dict[str, Any], robot_position: List[float]) -> List[float]:
        """
        Calculate target position for an object using enhanced movement logic.
        
        Args:
            object_data: Object detection data
            robot_position: Current robot position
            
        Returns:
            Target position [x, y, z] in mm
        """
        # Create ObjectDetection from vision data
        detection = self._create_object_detection_from_vision_data(object_data)
        
        # Use movement logic to calculate position
        target_position = self.movement_logic.calculate_object_position(detection, robot_position)
        
        logger.info(
            "Calculated target position for %s: [%.1f, %.1f, %.1f]",
            detection.class_name, *target_position
        )
        
        return target_position
    
    def _get_confidence_scores(self) -> Dict[str, float]:
        """Get confidence scores for detected objects."""
        return {
            "cup": 0.9,
            "microwave": 0.95,
            "oven": 0.92,
            "dishwasher": 0.88
        }
    
    def _analyze_object_interaction_requirements(self, task_description: str, objects_detected: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Analyze what kind of object interactions are required for the task.
        
        Args:
            task_description: Description of the task
            objects_detected: List of detected objects
            
        Returns:
            Dictionary of interaction requirements
        """
        requirements = {
            "needs_vision": False,
            "object_types": [],
            "interaction_types": [],
            "safety_considerations": [],
            "complexity_level": "simple"
        }
        
        task_lower = task_description.lower()
        
        # Check for object-specific tasks
        object_keywords = {
            "cup": ["cup", "mug", "glass"],
            "bottle": ["bottle", "container"],
            "microwave": ["microwave", "microwave oven"],
            "oven": ["oven", "stove"],
            "dishwasher": ["dishwasher", "dish washer"],
            "refrigerator": ["refrigerator", "fridge"],
            "coffee_maker": ["coffee maker", "coffee machine"],
            "toaster": ["toaster"],
            "blender": ["blender"],
            "mixer": ["mixer"],
            "stove": ["stove", "cooktop"],
            "sink": ["sink", "faucet"]
        }
        
        # Identify required objects
        for obj_type, keywords in object_keywords.items():
            if any(keyword in task_lower for keyword in keywords):
                requirements["object_types"].append(obj_type)
                requirements["needs_vision"] = True
        
        # Check for interaction types
        if any(word in task_lower for word in ["open", "close", "door"]):
            requirements["interaction_types"].append("door_operation")
            requirements["complexity_level"] = "moderate"
        
        if any(word in task_lower for word in ["press", "button", "start", "stop"]):
            requirements["interaction_types"].append("button_press")
            requirements["complexity_level"] = "moderate"
        
        if any(word in task_lower for word in ["turn", "knob", "dial"]):
            requirements["interaction_types"].append("knob_turn")
            requirements["complexity_level"] = "moderate"
        
        if any(word in task_lower for word in ["hot", "temperature", "heat"]):
            requirements["safety_considerations"].append("high_temperature")
        
        if any(word in task_lower for word in ["fragile", "careful", "gentle"]):
            requirements["safety_considerations"].append("fragile_object")
        
        return requirements
    
    def _scan_area(self, duration: float = 5.0) -> VisionFeedback:
        """
        Scan the current area for objects.
        
        Args:
            duration: Duration of the scan in seconds
            
        Returns:
            VisionFeedback with detected objects
        """
        self._update_state(PlanningState.SCANNING)
        logger.info(f"Scanning area for {duration} seconds...")
        
        start_time = time.time()
        detected_objects = []
        
        # Simulate scanning process
        while time.time() - start_time < duration:
            if self.vision_system:
                try:
                    # Get current detections
                    current_objects = self._get_vision_feedback()
                    detected_objects.extend(current_objects.objects_detected)
                    time.sleep(0.5)
                except Exception as e:
                    logger.error(f"Error during scanning: {e}")
                    break
        
        # Remove duplicates and calculate confidence
        unique_objects = {}
        for obj in detected_objects:
            label = obj.get('class', 'unknown')
            if label not in unique_objects:
                unique_objects[label] = obj
            else:
                # Merge confidence scores
                existing_conf = unique_objects[label].get('confidence', 0)
                new_conf = obj.get('confidence', 0)
                unique_objects[label]['confidence'] = max(existing_conf, new_conf)
        
        feedback = VisionFeedback(
            objects_detected=list(unique_objects.values()),
            confidence_scores={obj['class']: obj.get('confidence', 0) for obj in unique_objects.values()},
            scan_quality=min(1.0, len(unique_objects) / 10.0)  # Normalize scan quality
        )
        
        logger.info(f"Scan completed. Found {len(feedback.objects_detected)} objects: {[obj.get('class') for obj in feedback.objects_detected]}")
        return feedback
    
    def _create_rag_context(self, task_description: str) -> RAGContext:
        """
        Create RAG context for LLM decision making.
        
        Args:
            task_description: Description of the task to accomplish
            
        Returns:
            RAGContext with all relevant information
        """
        # Update vision feedback
        self.vision_feedback = self._get_vision_feedback()
        
        # Update execution context
        if self.robot_controller:
            try:
                current_pose = self.robot_controller.get_current_position()
                self.execution_context.current_pose = current_pose
            except Exception as e:
                logger.error(f"Failed to get current pose: {e}")
        
        # Analyze object interaction requirements
        interaction_requirements = self._analyze_object_interaction_requirements(
            task_description, 
            self.vision_feedback.objects_detected
        )
        
        # Add interaction requirements to context
        enhanced_context = RAGContext(
            task_description=task_description,
            available_actions=self.available_actions,
            available_poses=self.available_poses,
            vision_feedback=self.vision_feedback,
            execution_context=self.execution_context,
            world_model=self.world_model,
            planning_history=self.execution_context.execution_history
        )
        
        # Add interaction requirements to the context
        enhanced_context.interaction_requirements = interaction_requirements
        
        return enhanced_context
    
    def _generate_plan_with_llm(self, context: RAGContext) -> Dict[str, Any]:
        """
        Generate a plan using the LLM with RAG context.
        
        Args:
            context: RAG context with all relevant information
            
        Returns:
            Generated plan as a dictionary
        """
        if not self.llm:
            return self._generate_fallback_plan(context)
        
        try:
            # Create the prompt for the LLM
            prompt = self._create_llm_prompt(context)
            
            # Generate response
            response = self.llm.generate_content(prompt)
            
            # Parse the response
            plan = self._parse_llm_response(response.text)
            
            logger.info(f"Generated plan with {len(plan.get('steps', []))} steps")
            return plan
            
        except Exception as e:
            logger.error(f"Failed to generate plan with LLM: {e}")
            return self._generate_fallback_plan(context)
    
    def _create_llm_prompt(self, context: RAGContext) -> str:
        """
        Create a comprehensive prompt for the LLM.
        
        Args:
            context: RAG context with all relevant information
            
        Returns:
            Formatted prompt string
        """
        prompt = f"""
You are an intelligent robot control system. Your task is to create a plan to accomplish: "{context.task_description}"

## Current Context

### Available Poses:
{json.dumps(context.available_poses, indent=2)}

### Available Actions:
{json.dumps(context.available_actions, indent=2)}

### Current Vision Feedback:
- Objects detected: {[obj.get('class') for obj in context.vision_feedback.objects_detected]}
- Confidence scores: {context.vision_feedback.confidence_scores}
- Scan quality: {context.vision_feedback.scan_quality}

### Execution Context:
- Current pose: {context.execution_context.current_pose}
- Gripper state: {context.execution_context.gripper_state}
- Last action: {context.execution_context.last_action}
- Errors: {context.execution_context.errors}

### Recent History:
{json.dumps(context.execution_context.execution_history[-5:], indent=2)}

### Object Interaction Requirements:
{json.dumps(context.interaction_requirements, indent=2)}

## Object Interaction Guidelines

### Kitchen Appliances and Complex Objects
When interacting with kitchen appliances or complex objects, consider:

1. **Safety First**: 
   - Always approach hot surfaces (stoves, ovens, microwaves) from safe distances
   - Use appropriate hover distances (80-120mm for most appliances)
   - Consider temperature zones and safety areas

2. **Appliance-Specific Interactions**:
   - **Doors**: Approach from the handle side, use gentle grasping
   - **Buttons/Controls**: Use precise positioning, avoid excessive force
   - **Hot Surfaces**: Maintain safe distances, use longer hover distances
   - **Moving Parts**: Be aware of doors, lids, and moving components

3. **Interaction Patterns**:
   - **Opening**: Approach → Grasp handle → Pull/rotate → Retreat
   - **Closing**: Approach → Grasp handle → Push/rotate → Release → Retreat
   - **Button Pressing**: Approach → Position gripper → Press → Retreat
   - **Knob Turning**: Approach → Grasp knob → Rotate → Release → Retreat

### Object Detection and Recognition
- **Object Classes**: Recognize common object types (cup, bottle, microwave, oven, etc.)
- **Spatial Relationships**: Understand object positions and orientations
- **State Awareness**: Consider if objects are open, closed, running, etc.
- **Safety Considerations**: Identify hot, sharp, or fragile objects

### Adaptive Interaction Strategies
- **Fragile Objects**: Use gentle grasping, lower speeds, careful positioning
- **Heavy Objects**: Use stronger grips, consider weight distribution
- **Hot Objects**: Maintain safe distances, use appropriate tools if needed
- **Complex Objects**: Break down interactions into multiple steps

## Vision Decision Guidelines

**When to use SCAN_AREA:**
1. **Task requires object detection**: If the task mentions specific objects (e.g., "pick up the red cup", "open the microwave")
2. **Low confidence in current vision**: If scan_quality < 0.5 or no relevant objects detected
3. **Dynamic environment**: If objects might have moved or new objects appeared
4. **Complex spatial reasoning**: If precise positioning is required
5. **Error recovery**: If previous actions failed due to missing objects
6. **Appliance interaction**: When interacting with kitchen appliances or complex objects

**When NOT to use SCAN_AREA:**
1. **Simple movements**: If only moving to known poses (e.g., "go to home")
2. **Gripper operations**: If only opening/closing gripper
3. **High confidence objects**: If relevant objects are already detected with high confidence
4. **Time-sensitive tasks**: If scanning would delay critical operations
5. **Known environment**: If the environment is static and well-known

**Vision Quality Assessment:**
- Scan quality 0.0-0.3: Poor - recommend rescanning
- Scan quality 0.3-0.7: Moderate - use existing data if sufficient
- Scan quality 0.7-1.0: Good - no need to rescan unless specific objects missing

## Instructions

1. **Analyze the task requirements**: Determine if vision is needed for this task
2. **Assess current vision quality**: Check if existing vision data is sufficient
3. **Make intelligent vision decisions**: Only scan when necessary
4. **Create efficient plans**: Avoid unnecessary scanning operations
5. **Handle uncertainty gracefully**: Scan when confused or uncertain
6. **Consider task complexity**: Simple tasks may not need vision
7. **Use available context**: Leverage existing object detections when possible
8. **Apply object interaction guidelines**: Use appropriate strategies for different object types
9. **Consider safety**: Always prioritize safety when interacting with objects
10. **Adapt to object characteristics**: Adjust approach based on object type and state

## Action Schema:
{self.action_schema}

## Output Format:
Return ONLY a JSON object with this structure:
{{
  "goal": "task description",
  "reasoning": "brief explanation of your plan including vision decisions and object interaction strategy",
  "steps": [
    {{ "action": "ACTION_NAME", ... }}
  ]
}}

Generate the plan now:
"""
        return prompt
    
    def _parse_llm_response(self, response_text: str) -> Dict[str, Any]:
        """
        Parse the LLM response into a valid plan.
        
        Args:
            response_text: Raw response from the LLM
            
        Returns:
            Parsed plan as a dictionary
        """
        try:
            # Extract JSON from the response
            json_start = response_text.find('{')
            json_end = response_text.rfind('}') + 1
            
            if json_start == -1 or json_end == 0:
                raise ValueError("No JSON found in response")
            
            json_str = response_text[json_start:json_end]
            plan = json.loads(json_str)
            
            # Validate the plan
            if not self._validate_plan(plan):
                raise ValueError("Invalid plan structure")
            
            return plan
            
        except Exception as e:
            logger.error(f"Failed to parse LLM response: {e}")
            logger.error(f"Response text: {response_text}")
            raise
    
    def _validate_plan(self, plan: Dict[str, Any]) -> bool:
        """
        Validate the generated plan.
        
        Args:
            plan: Plan to validate
            
        Returns:
            True if valid, False otherwise
        """
        required_keys = ['goal', 'steps']
        if not all(key in plan for key in required_keys):
            return False
        
        if not isinstance(plan['steps'], list):
            return False
        
        # Validate each step
        for step in plan['steps']:
            if not isinstance(step, dict) or 'action' not in step:
                return False
        
        return True
    
    def _generate_fallback_plan(self, context) -> Dict[str, Any]:
        """
        Generate a fallback plan when LLM is not available.
        
        Args:
            context: RAG context or task description string
            
        Returns:
            Fallback plan
        """
        # Handle both RAGContext and string inputs
        if isinstance(context, str):
            task_description = context
            task_lower = context.lower()
            objects = []
        else:
            task_description = context.task_description
            task_lower = context.task_description.lower()
            objects = context.vision_feedback.objects_detected
        
        # Check for scanning tasks first
        if any(word in task_lower for word in ["scan", "find", "look", "search", "detect"]):
            return {
                "goal": task_description,
                "reasoning": "Fallback plan: Scanning task detected, performing comprehensive scan",
                "steps": [
                    {"action": "SCAN_FOR_OBJECTS", "pattern": "horizontal", "sweep_mm": 300, "steps": 5, "pause_sec": 1.0},
                    {"action": "SLEEP", "seconds": 1},
                    {"action": "MOVE_TO_NAMED", "name": "home"}
                ]
            }
        
        # Check for object-specific tasks
        if any(word in task_lower for word in ["pick up", "grab", "take"]):
            # Find objects in vision feedback
            objects = context.vision_feedback.objects_detected
            if objects:
                target_object = objects[0]  # Use first detected object
                object_name = target_object.get("class", "object")
                
                # Calculate target position using enhanced movement logic
                if self.robot_controller:
                    current_pos = self.robot_controller.get_current_position()
                    if not current_pos:
                        current_pos = [400, 0, 250]  # Default position
                    
                    target_position = self._calculate_object_target_position(target_object, current_pos)
                    
                    # Create movement sequence using enhanced logic
                    movement_sequence = self.movement_logic.create_movement_sequence(
                        target_position, object_name
                    )
                    
                    return {
                        "goal": context.task_description,
                        "reasoning": f"Fallback plan: Pick up {object_name} using enhanced movement logic",
                        "steps": movement_sequence
                    }
        
        # Check for kitchen appliance tasks
        if any(word in task_lower for word in ["microwave", "oven", "dishwasher", "refrigerator"]):
            appliance_type = None
            for word in ["microwave", "oven", "dishwasher", "refrigerator"]:
                if word in task_lower:
                    appliance_type = word
                    break
            
            if appliance_type:
                return {
                    "goal": context.task_description,
                    "reasoning": f"Fallback plan: {appliance_type} operation using enhanced movement logic",
                    "steps": [
                        {"action": "SCAN_AREA", "scan_duration": 5},
                        {"action": "MOVE_TO_NAMED", "name": "home"},
                        {"action": "APPROACH_OBJECT", "label": appliance_type, "hover_mm": 120},
                        {"action": "MOVE_TO_OBJECT", "label": appliance_type},
                        {"action": "SLEEP", "seconds": 2},
                        {"action": "RETREAT_Z", "dz_mm": 120},
                        {"action": "MOVE_TO_NAMED", "name": "home"}
                    ]
                }
        
        # Default fallback plan
        return {
            "goal": task_description,
            "reasoning": "Fallback plan: Simple movement sequence with scanning",
            "steps": [
                {"action": "SCAN_FOR_OBJECTS", "pattern": "horizontal", "sweep_mm": 300, "steps": 5, "pause_sec": 1.0},
                {"action": "MOVE_TO_NAMED", "name": "home"},
                {"action": "SLEEP", "seconds": 1}
            ]
        }
    
    def plan_and_execute(self, task_description: str) -> Dict[str, Any]:
        """
        Main method to plan and execute a task.
        
        Args:
            task_description: Description of the task to accomplish
            
        Returns:
            Execution results
        """
        logger.info(f"Starting planning and execution for task: {task_description}")
        
        retry_count = 0
        while retry_count < self.max_retries:
            try:
                # Create RAG context
                context = self._create_rag_context(task_description)
                
                # Generate plan
                self._update_state(PlanningState.PLANNING)
                plan = self._generate_plan_with_llm(context)
                
                # Execute plan
                self._update_state(PlanningState.EXECUTING)
                results = self._execute_plan(plan)
                
                # Check if task is completed
                if results.get('success', False):
                    self._update_state(PlanningState.COMPLETED)
                    logger.info("Task completed successfully")
                    return results
                else:
                    # Task failed, try again
                    retry_count += 1
                    logger.warning(f"Task failed, retrying ({retry_count}/{self.max_retries})")
                    
                    # If confused, scan again
                    if retry_count < self.max_retries:
                        logger.info("Scanning area again to gather more information...")
                        self._scan_area(duration=3.0)
                
            except Exception as e:
                logger.error(f"Error during planning/execution: {e}")
                retry_count += 1
                
                if retry_count < self.max_retries:
                    logger.info("Retrying after error...")
                    time.sleep(1.0)
        
        # All retries exhausted
        self._update_state(PlanningState.ERROR)
        error_msg = f"Failed to complete task after {self.max_retries} retries"
        logger.error(error_msg)
        
        return {
            "success": False,
            "error": error_msg,
            "retries": retry_count
        }
    
    def _execute_plan(self, plan: Dict[str, Any]) -> Dict[str, Any]:
        """
        Execute the generated plan.
        
        Args:
            plan: Plan to execute
            
        Returns:
            Execution results
        """
        if not self.robot_controller:
            logger.warning("No robot controller available, simulating execution")
            return self._simulate_execution(plan)
        
        results = {
            "success": True,
            "steps_executed": 0,
            "errors": [],
            "execution_time": 0.0
        }
        
        start_time = time.time()
        steps = plan.get('steps', [])
        
        for i, step in enumerate(steps):
            try:
                logger.info(f"Executing step {i+1}/{len(steps)}: {step}")
                
                # Execute the step
                step_result = self._execute_step(step)
                
                if not step_result.get('success', False):
                    results['errors'].append(f"Step {i+1} failed: {step_result.get('error', 'Unknown error')}")
                    results['success'] = False
                    break
                
                results['steps_executed'] += 1
                
                # Update execution context
                self.execution_context.last_action = step.get('action')
                self.execution_context.execution_history.append({
                    'step': i+1,
                    'action': step,
                    'result': step_result,
                    'timestamp': time.time()
                })
                
            except Exception as e:
                error_msg = f"Error executing step {i+1}: {e}"
                logger.error(error_msg)
                results['errors'].append(error_msg)
                results['success'] = False
                break
        
        results['execution_time'] = time.time() - start_time
        return results
    
    def _execute_step(self, step: Dict[str, Any]) -> Dict[str, Any]:
        """
        Execute a single step in the plan.
        
        Args:
            step: Step to execute
            
        Returns:
            Execution result
        """
        action = step.get("action", "").upper()
        result = {"success": False, "action": action, "message": ""}
        
        try:
            if action == "SCAN_AREA" or action == "SCAN_FOR_OBJECTS":
                # Use real scanning movement from executor
                if self.robot_controller and hasattr(self.robot_controller, 'execute'):
                    # Create a scan step for the executor
                    scan_step = {
                        "action": "SCAN_FOR_OBJECTS",
                        "pattern": "horizontal",
                        "sweep_mm": 300,
                        "steps": 5,
                        "pause_sec": 1.0
                    }
                    
                    # Execute the scan using the executor's logic
                    try:
                        # Execute the scan step directly
                        self.robot_controller.execute({"steps": [scan_step]})
                        result["success"] = True
                        result["message"] = f"Executed real scan movement with {scan_step['steps']} positions"
                    except Exception as e:
                        logger.error(f"Real scan failed: {e}")
                        # Fallback to simulated scan
                        duration = step.get("scan_duration", 5.0)
                        self.vision_feedback = self._scan_area(duration)
                        result["success"] = True
                        result["message"] = f"Fallback simulated scan for {duration}s"
                else:
                    # Fallback to simulated scan
                    duration = step.get("scan_duration", 5.0)
                    self.vision_feedback = self._scan_area(duration)
                    result["success"] = True
                    result["message"] = f"Simulated scan for {duration}s (no robot controller)"
                
            elif action == "MOVE_TO_NAMED":
                pose_name = step.get("name", "home")
                if pose_name in self.available_poses:
                    pose_data = self.available_poses[pose_name]
                    if self.robot_controller:
                        self.robot_controller.move_pose(pose_data["position"], pose_data.get("orientation", [0, 90, 0]))
                        result["success"] = True
                        result["message"] = f"Moved to named pose: {pose_name}"
                    else:
                        result["message"] = f"Simulated move to named pose: {pose_name}"
                        result["success"] = True
                else:
                    result["message"] = f"Unknown pose: {pose_name}"
                    
            elif action == "MOVE_TO_OBJECT":
                object_label = step.get("label", "")
                offset_mm = step.get("offset_mm", [0, 0, 0])
                
                # Find object in vision feedback
                target_object = None
                for obj in self.vision_feedback.objects_detected:
                    if obj.get("class", "").lower() == object_label.lower():
                        target_object = obj
                        break
                
                if target_object and self.robot_controller:
                    # Get current robot position
                    current_pos = self.robot_controller.get_current_position()
                    if not current_pos:
                        current_pos = [400, 0, 250]  # Default position
                    
                    # Calculate target position using enhanced movement logic
                    target_position = self._calculate_object_target_position(target_object, current_pos)
                    
                    # Apply offset
                    target_position = [
                        target_position[0] + offset_mm[0],
                        target_position[1] + offset_mm[1],
                        target_position[2] + offset_mm[2]
                    ]
                    
                    # Validate position
                    is_valid, error_msg = self.movement_logic.validate_target_position(
                        target_position, current_pos
                    )
                    
                    if is_valid:
                        self.robot_controller.move_pose(target_position, [0, 90, 0])
                        result["success"] = True
                        result["message"] = f"Moved to {object_label} at {target_position}"
                    else:
                        result["message"] = f"Invalid target position: {error_msg}"
                else:
                    result["message"] = f"Object {object_label} not found or robot not available"
                    
            elif action == "APPROACH_OBJECT":
                object_label = step.get("label", "")
                hover_mm = step.get("hover_mm", 100)
                
                # Find object in vision feedback
                target_object = None
                for obj in self.vision_feedback.objects_detected:
                    if obj.get("class", "").lower() == object_label.lower():
                        target_object = obj
                        break
                
                if target_object and self.robot_controller:
                    # Get current robot position
                    current_pos = self.robot_controller.get_current_position()
                    if not current_pos:
                        current_pos = [400, 0, 250]  # Default position
                    
                    # Calculate target position using enhanced movement logic
                    target_position = self._calculate_object_target_position(target_object, current_pos)
                    
                    # Create approach sequence
                    approach_sequence = self.movement_logic.create_approach_sequence(
                        target_position, object_label, hover_mm
                    )
                    
                    # Execute approach sequence
                    for approach_step in approach_sequence:
                        approach_result = self._execute_step(approach_step)
                        if not approach_result["success"]:
                            result["message"] = f"Approach failed: {approach_result['message']}"
                            return result
                    
                    result["success"] = True
                    result["message"] = f"Approached {object_label} with hover distance {hover_mm}mm"
                else:
                    result["message"] = f"Object {object_label} not found or robot not available"
                    
            elif action == "GRIPPER_GRASP":
                target_position = step.get("target_position", 200)
                if self.robot_controller and hasattr(self.robot_controller, 'gripper'):
                    self.robot_controller.gripper.close_gripper()
                    result["success"] = True
                    result["message"] = f"Gripper grasped at position {target_position}"
                else:
                    result["message"] = "Gripper not available"
                    
            elif action == "GRIPPER_RELEASE":
                if self.robot_controller and hasattr(self.robot_controller, 'gripper'):
                    self.robot_controller.gripper.open_gripper()
                    result["success"] = True
                    result["message"] = "Gripper released"
                else:
                    result["message"] = "Gripper not available"
                    
            elif action == "OPEN_GRIPPER":
                if self.robot_controller and hasattr(self.robot_controller, 'gripper'):
                    self.robot_controller.gripper.open_gripper()
                    result["success"] = True
                    result["message"] = "Gripper opened"
                else:
                    result["message"] = "Gripper not available"
                    
            elif action == "CLOSE_GRIPPER":
                if self.robot_controller and hasattr(self.robot_controller, 'gripper'):
                    self.robot_controller.gripper.close_gripper()
                    result["success"] = True
                    result["message"] = "Gripper closed"
                else:
                    result["message"] = "Gripper not available"
                    
            elif action == "MOVE_DOWN":
                distance_mm = step.get("distance_mm", 300)
                if self.robot_controller:
                    current_pos = self.robot_controller.get_current_position()
                    if current_pos:
                        down_position = current_pos.copy()
                        down_position[2] -= distance_mm
                        self.robot_controller.move_pose(down_position, [0, 90, 0])
                        result["success"] = True
                        result["message"] = f"Moved down {distance_mm}mm"
                    else:
                        result["message"] = "Could not get current position"
                else:
                    result["message"] = "Robot not available"
                    
            elif action == "MOVE_UP":
                distance_mm = step.get("distance_mm", 200)
                if self.robot_controller:
                    current_pos = self.robot_controller.get_current_position()
                    if current_pos:
                        up_position = current_pos.copy()
                        up_position[2] += distance_mm
                        self.robot_controller.move_pose(up_position, [0, 90, 0])
                        result["success"] = True
                        result["message"] = f"Moved up {distance_mm}mm"
                    else:
                        result["message"] = "Could not get current position"
                else:
                    result["message"] = "Robot not available"
                    
            elif action == "SLEEP":
                seconds = step.get("seconds", 1.0)
                time.sleep(seconds)
                result["success"] = True
                result["message"] = f"Slept for {seconds}s"
                
            elif action == "RETREAT_Z":
                dz_mm = step.get("dz_mm", 100)
                if self.robot_controller:
                    current_pos = self.robot_controller.get_current_position()
                    if current_pos:
                        retreat_position = current_pos.copy()
                        retreat_position[2] += dz_mm
                        self.robot_controller.move_pose(retreat_position, [0, 90, 0])
                        result["success"] = True
                        result["message"] = f"Retreated {dz_mm}mm in Z direction"
                    else:
                        result["message"] = "Could not get current position"
                else:
                    result["message"] = "Robot not available"
                    
            else:
                result["message"] = f"Unknown action: {action}"
                
        except Exception as e:
            result["message"] = f"Error executing {action}: {str(e)}"
            logger.error(f"Error executing step {action}: {e}")
            
        return result
    
    def _simulate_execution(self, plan: Dict[str, Any]) -> Dict[str, Any]:
        """
        Simulate plan execution when no robot controller is available.
        
        Args:
            plan: Plan to simulate
            
        Returns:
            Simulation results
        """
        logger.info("Simulating plan execution")
        
        results = {
            "success": True,
            "steps_executed": len(plan.get('steps', [])),
            "errors": [],
            "execution_time": 0.0,
            "simulated": True
        }
        
        for i, step in enumerate(plan.get('steps', [])):
            logger.info(f"Simulating step {i+1}: {step}")
            time.sleep(0.1)  # Simulate execution time
        
        return results
    
    def get_status(self) -> Dict[str, Any]:
        """
        Get the current status of the planner.
        
        Returns:
            Status information
        """
        return {
            "state": self.current_state.value,
            "execution_context": {
                "current_pose": self.execution_context.current_pose,
                "gripper_state": self.execution_context.gripper_state,
                "last_action": self.execution_context.last_action,
                "errors": self.execution_context.errors
            },
            "vision_feedback": {
                "objects_detected": len(self.vision_feedback.objects_detected),
                "scan_quality": self.vision_feedback.scan_quality
            },
            "available_poses": len(self.available_poses),
            "available_actions": len(self.available_actions)
        }
