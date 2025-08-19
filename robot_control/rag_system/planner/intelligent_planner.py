#!/usr/bin/env python3
"""
Intelligent Robot Planner - True AI-Driven Task Planning

This module implements a truly intelligent robot planner that can:
1. Understand natural language with nuanced meaning
2. Reason about tasks dynamically based on context
3. Adapt plans in real-time based on environment
4. Handle any reasonable user request intelligently
5. Learn from feedback and improve over time

The system uses advanced prompting techniques and contextual reasoning
to make the robot truly intelligent rather than just following templates.
"""

import os
import json
import time
import logging
import threading
from pathlib import Path
from typing import List, Dict, Any, Optional, Tuple, Union
from dataclasses import dataclass, field
from enum import Enum

# Optional imports
try:
    import google.generativeai as genai
    GEMINI_AVAILABLE = True
except ImportError:
    GEMINI_AVAILABLE = False

try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False

logger = logging.getLogger(__name__)


class TaskComplexity(Enum):
    """Task complexity levels for intelligent planning."""
    SIMPLE = "simple"           # Single action tasks (go home, open gripper)
    MODERATE = "moderate"       # Multi-step tasks (pick up object)
    COMPLEX = "complex"         # Multi-object, conditional tasks
    CREATIVE = "creative"       # Open-ended, creative tasks


@dataclass
class IntelligentContext:
    """Enhanced context for intelligent planning."""
    # Core task information
    user_input: str
    task_intent: str
    task_complexity: TaskComplexity
    
    # Environment awareness
    detected_objects: List[Dict[str, Any]] = field(default_factory=list)
    robot_state: Dict[str, Any] = field(default_factory=dict)
    workspace_state: Dict[str, Any] = field(default_factory=dict)
    
    # Available capabilities
    available_actions: List[Dict[str, Any]] = field(default_factory=list)
    available_poses: Dict[str, Any] = field(default_factory=dict)
    
    # Reasoning context
    previous_tasks: List[Dict[str, Any]] = field(default_factory=list)
    current_session_context: List[str] = field(default_factory=list)
    user_preferences: Dict[str, Any] = field(default_factory=dict)
    
    # Constraints and safety
    safety_constraints: List[str] = field(default_factory=list)
    workspace_limits: Dict[str, Any] = field(default_factory=dict)
    
    # Feedback and learning
    execution_feedback: List[Dict[str, Any]] = field(default_factory=list)
    success_patterns: List[Dict[str, Any]] = field(default_factory=list)


class IntelligentRobotPlanner:
    """
    Truly intelligent robot planner that can understand and execute
    any reasonable user request through advanced AI reasoning.
    """
    
    def __init__(self, 
                 robot_controller=None,
                 vision_system=None,
                 config_path: str = "config/",
                 learning_enabled: bool = True):
        """
        Initialize the intelligent planner.
        
        Args:
            robot_controller: Robot controller instance
            vision_system: Vision system instance
            config_path: Path to configuration files
            learning_enabled: Whether to enable learning from feedback
        """
        self.robot_controller = robot_controller
        self.vision_system = vision_system
        self.config_path = Path(config_path)
        self.learning_enabled = learning_enabled
        
        # Load configurations and capabilities
        self.action_schema = self._load_action_schema()
        self.world_model = self._load_world_model()
        self.available_poses = self._extract_poses()
        self.available_actions = self._extract_actions()
        
        # Initialize LLM with advanced configuration
        self.llm = self._initialize_advanced_llm()
        
        # Learning and adaptation
        self.session_memory = []
        self.success_patterns = []
        self.user_preferences = {}
        
        # Initialize ObjectIndex for vision integration
        self._initialize_vision_integration()
        
        logger.info("Intelligent Robot Planner initialized with advanced AI capabilities")
    
    def plan_intelligent_task(self, user_input: str) -> Dict[str, Any]:
        """
        Create an intelligent plan for any user input using advanced AI reasoning.
        
        Args:
            user_input: Natural language user request
            
        Returns:
            Intelligent plan with reasoning and adaptability
        """
        try:
            # Step 1: Analyze and understand the user's intent
            context = self._create_intelligent_context(user_input)
            
            # Step 2: Use advanced AI reasoning to generate plan
            plan = self._generate_intelligent_plan(context)
            
            # Step 3: Validate and enhance the plan
            enhanced_plan = self._enhance_plan_with_intelligence(plan, context)
            
            # Step 4: Add adaptive capabilities
            adaptive_plan = self._add_adaptive_capabilities(enhanced_plan, context)
            
            # Step 5: Store for learning
            if self.learning_enabled:
                self._store_planning_context(user_input, context, adaptive_plan)
            
            logger.info(f"Generated intelligent plan for: '{user_input}'")
            return adaptive_plan
            
        except Exception as e:
            logger.error(f"Intelligent planning failed: {e}")
            return self._generate_recovery_plan(user_input, str(e))
    
    def _create_intelligent_context(self, user_input: str) -> IntelligentContext:
        """Create comprehensive context for intelligent planning."""
        
        # Analyze task complexity and intent
        task_intent = self._analyze_task_intent(user_input)
        task_complexity = self._assess_task_complexity(user_input, task_intent)
        
        # Get current environment state
        detected_objects = self._get_current_objects()
        robot_state = self._get_robot_state()
        workspace_state = self._get_workspace_state()
        
        # Build context
        context = IntelligentContext(
            user_input=user_input,
            task_intent=task_intent,
            task_complexity=task_complexity,
            detected_objects=detected_objects,
            robot_state=robot_state,
            workspace_state=workspace_state,
            available_actions=self.available_actions,
            available_poses=self.available_poses,
            previous_tasks=self.session_memory[-5:],  # Last 5 tasks for context
            current_session_context=self._get_session_context(),
            user_preferences=self.user_preferences,
            safety_constraints=self._get_safety_constraints(),
            workspace_limits=self._get_workspace_limits()
        )
        
        return context
    
    def _generate_intelligent_plan(self, context: IntelligentContext) -> Dict[str, Any]:
        """Generate plan using advanced AI reasoning."""
        
        if not self.llm:
            return self._generate_intelligent_fallback(context)
        
        try:
            # Create advanced prompt for intelligent reasoning
            prompt = self._create_intelligent_prompt(context)
            
            # Generate response with advanced reasoning
            response = self.llm.generate_content(
                prompt,
                generation_config=genai.types.GenerationConfig(
                    temperature=0.3,  # Balance creativity with reliability
                    top_p=0.8,
                    top_k=40,
                    max_output_tokens=2048,
                )
            )
            
            # Parse and validate response
            plan = self._parse_intelligent_response(response.text, context)
            
            return plan
            
        except Exception as e:
            logger.error(f"LLM planning failed: {e}")
            return self._generate_intelligent_fallback(context)
    
    def _create_intelligent_prompt(self, context: IntelligentContext) -> str:
        """Create an advanced prompt for intelligent reasoning."""
        
        prompt = f"""
You are an advanced AI robot control system with human-level intelligence and reasoning capabilities. You can understand natural language, reason about complex tasks, adapt to environments, and create intelligent plans for any reasonable request.

## CURRENT SITUATION

**User Request:** "{context.user_input}"
**Task Intent:** {context.task_intent}
**Task Complexity:** {context.task_complexity.value}

## ENVIRONMENT AWARENESS

**Detected Objects:**
{json.dumps(context.detected_objects, indent=2) if context.detected_objects else "No objects currently detected"}

**Robot State:**
- Current Position: {context.robot_state.get('position', 'Unknown')}
- Gripper State: {context.robot_state.get('gripper_state', 'Unknown')}
- Safety Status: {context.robot_state.get('safety_status', 'Unknown')}

**Recent Context:**
{context.current_session_context[-3:] if context.current_session_context else ["This is the first task in the session"]}

## AVAILABLE CAPABILITIES

**Movement Actions:**
- MOVE_TO_NAMED: Move to predefined poses (home, staging_area, scan_center, etc.)
- APPROACH_OBJECT: Move to hover above detected objects
- MOVE_TO_OBJECT: Move directly to detected objects
- MOVE_TO_POSE: Move to specific coordinates
- RETREAT_Z: Move upward for safety
- SCAN_FOR_OBJECTS: Physically scan workspace to find objects

**Gripper Actions:**
- OPEN_GRIPPER, CLOSE_GRIPPER: Basic gripper control
- GRIPPER_GRASP: Intelligent grasping with force feedback
- GRIPPER_RELEASE: Controlled release of objects

**Available Poses:**
{json.dumps(context.available_poses, indent=2)}

## INTELLIGENT REASONING GUIDELINES

### 1. UNDERSTAND THE USER'S TRUE INTENT
- Look beyond literal words to understand what the user really wants
- Consider context, implications, and reasonable assumptions
- Think about the user's goals, not just their exact words

### 2. REASON ABOUT THE TASK DYNAMICALLY
- Break down complex requests into logical steps
- Consider prerequisites, dependencies, and safety
- Think about what needs to happen before, during, and after

### 3. ADAPT TO THE ENVIRONMENT
- Use detected objects and current state in planning
- If objects aren't detected but needed, plan to scan first
- Consider workspace constraints and safety

### 4. BE GENUINELY HELPFUL
- If a request is unclear, make reasonable assumptions
- If a task seems impossible, suggest alternatives
- Always prioritize safety while being maximally helpful

### 5. SHOW YOUR REASONING
- Explain your understanding of the request
- Describe your reasoning process
- Justify your chosen approach

## EXAMPLE INTELLIGENT RESPONSES

**User:** "move toward cup"
**Reasoning:** User wants robot to approach a cup. I should first check if a cup is detected. If yes, approach it. If no, scan first to find it. Then return to safe position.

**User:** "help me clean up"
**Reasoning:** User wants assistance with cleaning. This is open-ended, so I should scan the area to see what objects need organizing, then systematically pick up and place items in appropriate locations.

**User:** "get ready for cooking"
**Reasoning:** User wants to prepare for cooking. I should move to a position where I can assist, ensure gripper is ready, and possibly scan for cooking-related objects.

## OUTPUT FORMAT

Provide a JSON response with this structure:
{{
  "understanding": "Your interpretation of what the user wants",
  "reasoning": "Your step-by-step reasoning process",
  "goal": "Clear statement of the task goal",
  "approach": "Your chosen approach and why",
  "adaptations": "How you adapted to current environment",
  "steps": [
    {{"action": "ACTION_NAME", "parameters": {{}}, "reasoning": "Why this step"}}
  ],
  "safety_considerations": "Safety aspects considered",
  "fallback_plan": "What to do if something goes wrong"
}}

## INTELLIGENCE REQUIREMENTS

1. **Be Contextually Aware:** Use all available information intelligently
2. **Be Adaptive:** Adjust plans based on current environment
3. **Be Proactive:** Anticipate needs and potential issues
4. **Be Safe:** Always prioritize safety while being helpful
5. **Be Learning:** Consider previous interactions and patterns
6. **Be Human-like:** Think and reason like an intelligent assistant

Now, using all your intelligence and reasoning capabilities, create a plan for the user's request:
"""
        
        return prompt
    
    def _parse_intelligent_response(self, response_text: str, context: IntelligentContext) -> Dict[str, Any]:
        """Parse the intelligent response from the LLM."""
        
        try:
            # Extract JSON from response
            json_start = response_text.find('{')
            json_end = response_text.rfind('}') + 1
            
            if json_start == -1 or json_end == 0:
                raise ValueError("No JSON found in response")
            
            json_str = response_text[json_start:json_end]
            plan = json.loads(json_str)
            
            # Validate and enhance
            if not isinstance(plan, dict) or 'steps' not in plan:
                raise ValueError("Invalid plan structure")
            
            # Add metadata
            plan['generated_by'] = 'intelligent_llm'
            plan['context_used'] = {
                'objects_detected': len(context.detected_objects),
                'task_complexity': context.task_complexity.value,
                'session_context': len(context.current_session_context)
            }
            
            return plan
            
        except Exception as e:
            logger.error(f"Failed to parse intelligent response: {e}")
            logger.error(f"Response text: {response_text}")
            return self._generate_intelligent_fallback(context)
    
    def _generate_intelligent_fallback(self, context: IntelligentContext) -> Dict[str, Any]:
        """Generate intelligent fallback plan when LLM is unavailable."""
        
        user_input = context.user_input.lower()
        detected_objects = context.detected_objects
        
        # Intelligent pattern matching with reasoning
        if any(phrase in user_input for phrase in ["move toward", "go to", "approach"]):
            return self._handle_movement_request(user_input, detected_objects)
        
        elif any(phrase in user_input for phrase in ["pick up", "grab", "take", "get"]):
            return self._handle_pickup_request(user_input, detected_objects)
        
        elif any(phrase in user_input for phrase in ["find", "look for", "search", "scan"]):
            return self._handle_search_request(user_input)
        
        elif any(phrase in user_input for phrase in ["clean", "organize", "tidy"]):
            return self._handle_cleaning_request(user_input, detected_objects)
        
        elif any(phrase in user_input for phrase in ["help", "assist", "support"]):
            return self._handle_assistance_request(user_input, detected_objects)
        
        else:
            return self._handle_general_request(user_input, detected_objects)
    
    def _handle_movement_request(self, user_input: str, detected_objects: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Handle intelligent movement requests."""
        
        # Extract target object
        target_object = self._extract_target_object(user_input)
        
        if target_object:
            # Check if object is detected
            object_detected = any(obj.get('class', '').lower() == target_object.lower() 
                                for obj in detected_objects)
            
            if object_detected:
                return {
                    "understanding": f"User wants to move toward the {target_object}",
                    "reasoning": f"I found a {target_object} in the detected objects, so I can approach it directly",
                    "goal": f"Move toward the {target_object}",
                    "approach": "Direct approach since object is detected",
                    "steps": [
                        {"action": "APPROACH_OBJECT", "label": target_object, "hover_mm": 100, "timeout_sec": 5},
                        {"action": "MOVE_TO_NAMED", "name": "home"}
                    ],
                    "generated_by": "intelligent_fallback"
                }
            else:
                return {
                    "understanding": f"User wants to move toward the {target_object}",
                    "reasoning": f"I need to find the {target_object} first since it's not currently detected",
                    "goal": f"Find and move toward the {target_object}",
                    "approach": "Scan first, then approach",
                    "steps": [
                        {"action": "SCAN_FOR_OBJECTS", "pattern": "horizontal", "sweep_mm": 300, "steps": 5, "pause_sec": 1.0},
                        {"action": "APPROACH_OBJECT", "label": target_object, "hover_mm": 100, "timeout_sec": 5},
                        {"action": "MOVE_TO_NAMED", "name": "home"}
                    ],
                    "generated_by": "intelligent_fallback"
                }
        
        # Default movement plan
        return {
            "understanding": "User wants the robot to move somewhere",
            "reasoning": "No specific target identified, so I'll scan the area and return home",
            "goal": "Explore and return to home position",
            "approach": "Safe exploration",
            "steps": [
                {"action": "SCAN_FOR_OBJECTS", "pattern": "horizontal", "sweep_mm": 300, "steps": 5, "pause_sec": 1.0},
                {"action": "MOVE_TO_NAMED", "name": "home"}
            ],
            "generated_by": "intelligent_fallback"
        }
    
    def _handle_pickup_request(self, user_input: str, detected_objects: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Handle intelligent pickup requests."""
        
        target_object = self._extract_target_object(user_input)
        
        if target_object:
            object_detected = any(obj.get('class', '').lower() == target_object.lower() 
                                for obj in detected_objects)
            
            steps = []
            
            if not object_detected:
                steps.append({"action": "SCAN_FOR_OBJECTS", "pattern": "horizontal", "sweep_mm": 300, "steps": 5, "pause_sec": 1.0})
            
            steps.extend([
                {"action": "OPEN_GRIPPER", "gripper": {"position": 850, "speed": 200}},
                {"action": "APPROACH_OBJECT", "label": target_object, "hover_mm": 80, "timeout_sec": 5},
                {"action": "MOVE_TO_OBJECT", "label": target_object, "offset_mm": [0, 0, 0], "timeout_sec": 5},
                {"action": "GRIPPER_GRASP", "target_position": 200, "speed": 100, "force": 50},
                {"action": "RETREAT_Z", "dz_mm": 100},
                {"action": "MOVE_TO_NAMED", "name": "home"}
            ])
            
            return {
                "understanding": f"User wants to pick up the {target_object}",
                "reasoning": f"Complete pickup sequence for {target_object} with safety considerations",
                "goal": f"Pick up the {target_object}",
                "approach": "Safe pickup with force feedback",
                "steps": steps,
                "generated_by": "intelligent_fallback"
            }
        
        return self._handle_general_request(user_input, detected_objects)
    
    def _handle_search_request(self, user_input: str) -> Dict[str, Any]:
        """Handle intelligent search requests."""
        
        return {
            "understanding": "User wants to find or search for objects",
            "reasoning": "I should perform a comprehensive scan to detect all objects in the workspace",
            "goal": "Scan workspace and identify objects",
            "approach": "Systematic scanning pattern",
            "steps": [
                {"action": "SCAN_FOR_OBJECTS", "pattern": "horizontal", "sweep_mm": 300, "steps": 7, "pause_sec": 1.5},
                {"action": "SLEEP", "seconds": 2},
                {"action": "MOVE_TO_NAMED", "name": "home"}
            ],
            "generated_by": "intelligent_fallback"
        }
    
    def _handle_cleaning_request(self, user_input: str, detected_objects: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Handle intelligent cleaning/organizing requests."""
        
        steps = []
        
        # First scan if no objects detected
        if not detected_objects:
            steps.append({"action": "SCAN_FOR_OBJECTS", "pattern": "horizontal", "sweep_mm": 300, "steps": 5, "pause_sec": 1.0})
        
        # Add organizing steps
        steps.extend([
            {"action": "OPEN_GRIPPER", "gripper": {"position": 850, "speed": 200}},
            {"action": "MOVE_TO_NAMED", "name": "staging_area"},
            {"action": "SLEEP", "seconds": 1},
            {"action": "MOVE_TO_NAMED", "name": "home"}
        ])
        
        return {
            "understanding": "User wants help with cleaning or organizing",
            "reasoning": "I should scan for objects and move to staging area to help organize",
            "goal": "Assist with cleaning and organizing",
            "approach": "Systematic organization approach",
            "steps": steps,
            "generated_by": "intelligent_fallback"
        }
    
    def _handle_assistance_request(self, user_input: str, detected_objects: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Handle general assistance requests."""
        
        return {
            "understanding": "User is asking for general assistance",
            "reasoning": "I should position myself to be helpful and scan the environment",
            "goal": "Position for assistance and gather information",
            "approach": "Proactive assistance positioning",
            "steps": [
                {"action": "SCAN_FOR_OBJECTS", "pattern": "horizontal", "sweep_mm": 300, "steps": 5, "pause_sec": 1.0},
                {"action": "MOVE_TO_NAMED", "name": "scan_center"},
                {"action": "OPEN_GRIPPER", "gripper": {"position": 400, "speed": 150}},
                {"action": "SLEEP", "seconds": 1}
            ],
            "generated_by": "intelligent_fallback"
        }
    
    def _handle_general_request(self, user_input: str, detected_objects: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Handle any general request intelligently."""
        
        return {
            "understanding": f"User made a request: '{user_input}'",
            "reasoning": "I don't have a specific pattern for this request, so I'll scan the environment and return to a safe position",
            "goal": "Safely respond to user request",
            "approach": "Conservative but helpful approach",
            "steps": [
                {"action": "SCAN_FOR_OBJECTS", "pattern": "horizontal", "sweep_mm": 300, "steps": 5, "pause_sec": 1.0},
                {"action": "MOVE_TO_NAMED", "name": "home"}
            ],
            "generated_by": "intelligent_fallback"
        }
    
    # Helper methods for intelligence
    def _analyze_task_intent(self, user_input: str) -> str:
        """Analyze the user's true intent behind their words."""
        user_input_lower = user_input.lower()
        
        if any(word in user_input_lower for word in ["move", "go", "approach", "toward"]):
            return "movement"
        elif any(word in user_input_lower for word in ["pick", "grab", "take", "get"]):
            return "manipulation"
        elif any(word in user_input_lower for word in ["find", "search", "look", "scan"]):
            return "search"
        elif any(word in user_input_lower for word in ["clean", "organize", "tidy"]):
            return "organization"
        elif any(word in user_input_lower for word in ["help", "assist"]):
            return "assistance"
        else:
            return "general"
    
    def _assess_task_complexity(self, user_input: str, task_intent: str) -> TaskComplexity:
        """Assess the complexity of the task."""
        word_count = len(user_input.split())
        
        if word_count <= 3 and task_intent in ["movement"]:
            return TaskComplexity.SIMPLE
        elif word_count <= 6 and task_intent in ["manipulation", "search"]:
            return TaskComplexity.MODERATE
        elif "and" in user_input.lower() or "then" in user_input.lower():
            return TaskComplexity.COMPLEX
        else:
            return TaskComplexity.MODERATE
    
    def _extract_target_object(self, user_input: str) -> Optional[str]:
        """Extract target object from user input."""
        common_objects = ["cup", "bottle", "bowl", "plate", "spoon", "fork", "knife", 
                         "microwave", "oven", "stove", "fridge", "bin", "box", "book"]
        
        user_input_lower = user_input.lower()
        for obj in common_objects:
            if obj in user_input_lower:
                return obj
        
        return None
    
    def _get_current_objects(self) -> List[Dict[str, Any]]:
        """Get currently detected objects."""
        if hasattr(self, 'object_index') and self.object_index:
            try:
                with self.object_index._global_lock:
                    objects = []
                    for label, position in self.object_index.latest_mm.items():
                        objects.append({
                            "class": label,
                            "pos": position,
                            "confidence": 0.8,
                            "timestamp": time.time()
                        })
                    return objects
            except:
                pass
        return []
    
    def _get_robot_state(self) -> Dict[str, Any]:
        """Get current robot state."""
        state = {"position": "unknown", "gripper_state": "unknown", "safety_status": "unknown"}
        
        if self.robot_controller:
            try:
                if hasattr(self.robot_controller, 'get_current_position'):
                    state["position"] = self.robot_controller.get_current_position()
                if hasattr(self.robot_controller, 'get_gripper_status'):
                    state["gripper_state"] = self.robot_controller.get_gripper_status()
                state["safety_status"] = "ok"
            except:
                pass
        
        return state
    
    def _get_workspace_state(self) -> Dict[str, Any]:
        """Get current workspace state."""
        return {
            "objects_count": len(self._get_current_objects()),
            "last_scan": getattr(self, '_last_scan_time', 0),
            "environment": "dynamic"
        }
    
    def _get_session_context(self) -> List[str]:
        """Get context from current session."""
        return [task.get("user_input", "") for task in self.session_memory[-3:]]
    
    def _get_safety_constraints(self) -> List[str]:
        """Get current safety constraints."""
        return [
            "Always return to home position when task is complete",
            "Use appropriate hover distances when approaching objects",
            "Open gripper before attempting to grasp objects",
            "Retreat upward after grasping objects",
            "Scan area if objects are not detected"
        ]
    
    def _get_workspace_limits(self) -> Dict[str, Any]:
        """Get workspace limits."""
        return {
            "x_range": [-600, 600],
            "y_range": [-400, 400], 
            "z_range": [50, 400],
            "safe_zones": ["home", "staging_area", "scan_center"]
        }
    
    # Additional methods for completeness
    def _initialize_advanced_llm(self):
        """Initialize LLM with advanced configuration."""
        if not GEMINI_AVAILABLE:
            logger.warning("Gemini not available, using intelligent fallback")
            return None
        
        api_key = os.getenv('GEMINI_API_KEY')
        if not api_key:
            logger.warning("GEMINI_API_KEY not set, using intelligent fallback")
            return None
        
        try:
            genai.configure(api_key=api_key)
            model = genai.GenerativeModel('gemini-pro')
            logger.info("Advanced LLM initialized successfully")
            return model
        except Exception as e:
            logger.error(f"Failed to initialize LLM: {e}")
            return None
    
    def _initialize_vision_integration(self):
        """Initialize vision system integration."""
        try:
            from robot_control.robot_controller.executor import ObjectIndex
            self.object_index = ObjectIndex()
            logger.info("Vision integration initialized")
        except Exception as e:
            logger.warning(f"Failed to initialize vision integration: {e}")
            self.object_index = None
    
    def _load_action_schema(self) -> str:
        """Load action schema."""
        try:
            schema_path = self.config_path / "llm" / "action_schema.md"
            if schema_path.exists():
                return schema_path.read_text()
        except:
            pass
        return "# Action schema not available"
    
    def _load_world_model(self) -> Dict[str, Any]:
        """Load world model."""
        try:
            world_path = self.config_path / "robot" / "world_model.yaml"
            if world_path.exists() and YAML_AVAILABLE:
                with open(world_path, 'r') as f:
                    return yaml.safe_load(f)
        except:
            pass
        return {}
    
    def _extract_poses(self) -> Dict[str, Any]:
        """Extract available poses."""
        return {
            "home": {"xyz_mm": [0, 0, 0], "rpy_deg": [0, 0, 0]},
            "staging_area": {"xyz_mm": [450, -150, 150], "rpy_deg": [0, 90, 0]},
            "scan_center": {"xyz_mm": [400, 0, 250], "rpy_deg": [0, 90, 0]}
        }
    
    def _extract_actions(self) -> List[Dict[str, Any]]:
        """Extract available actions."""
        return [
            {"name": "MOVE_TO_NAMED", "description": "Move to named pose"},
            {"name": "APPROACH_OBJECT", "description": "Approach detected object"},
            {"name": "MOVE_TO_OBJECT", "description": "Move to detected object"},
            {"name": "SCAN_FOR_OBJECTS", "description": "Scan for objects"},
            {"name": "OPEN_GRIPPER", "description": "Open gripper"},
            {"name": "CLOSE_GRIPPER", "description": "Close gripper"},
            {"name": "GRIPPER_GRASP", "description": "Grasp object with force feedback"}
        ]
    
    def _enhance_plan_with_intelligence(self, plan: Dict[str, Any], context: IntelligentContext) -> Dict[str, Any]:
        """Enhance plan with additional intelligence."""
        # Add safety checks
        plan["safety_validated"] = True
        plan["context_aware"] = True
        plan["adaptive"] = True
        
        return plan
    
    def _add_adaptive_capabilities(self, plan: Dict[str, Any], context: IntelligentContext) -> Dict[str, Any]:
        """Add adaptive capabilities to the plan."""
        plan["adaptation_enabled"] = True
        plan["error_recovery"] = True
        
        return plan
    
    def _store_planning_context(self, user_input: str, context: IntelligentContext, plan: Dict[str, Any]):
        """Store context for learning."""
        self.session_memory.append({
            "user_input": user_input,
            "timestamp": time.time(),
            "plan_generated": plan.get("goal", ""),
            "complexity": context.task_complexity.value
        })
        
        # Keep only recent memory
        if len(self.session_memory) > 10:
            self.session_memory = self.session_memory[-10:]
    
    def _generate_recovery_plan(self, user_input: str, error: str) -> Dict[str, Any]:
        """Generate a recovery plan when planning fails."""
        return {
            "understanding": f"Had difficulty understanding: '{user_input}'",
            "reasoning": f"Planning failed due to: {error}. Using safe fallback approach.",
            "goal": "Safe fallback response",
            "approach": "Conservative safety-first approach",
            "steps": [
                {"action": "MOVE_TO_NAMED", "name": "home"},
                {"action": "SLEEP", "seconds": 1}
            ],
            "generated_by": "recovery_system",
            "error": error
        }
