#!/usr/bin/env python3
"""
Comprehensive Robotics Knowledge Base for RAG System

This module contains extensive robotics knowledge, movement patterns,
task examples, and problem-solving strategies that the RAG system
can retrieve and use for intelligent planning.
"""

# Movement Patterns and Strategies
MOVEMENT_PATTERNS = [
    {
        "id": "approach_object_safe",
        "title": "Safe Object Approach Pattern",
        "description": "Standard pattern for safely approaching objects with proper hover distance",
        "context": "object manipulation, safety, precision",
        "pattern": [
            "Scan area to locate target object",
            "Move to position above object with safe hover distance (80-120mm)",
            "Verify object position and adjust if needed",
            "Slowly descend to interaction distance",
            "Perform intended action (grasp, push, etc.)",
            "Retreat to safe height before moving away"
        ],
        "safety_considerations": [
            "Always maintain minimum hover distance",
            "Check for obstacles in path",
            "Verify gripper state before approach",
            "Monitor force feedback during interaction"
        ],
        "applicable_tasks": ["pick up", "move toward", "approach", "grab", "touch"]
    },
    
    {
        "id": "scanning_pattern_systematic",
        "title": "Systematic Area Scanning",
        "description": "Comprehensive scanning pattern to detect all objects in workspace",
        "context": "object detection, exploration, environment mapping",
        "pattern": [
            "Start from home position",
            "Move to scan center with optimal camera angle",
            "Perform horizontal sweep with multiple stops",
            "Pause at each position for object detection",
            "Adjust height if needed for better visibility",
            "Return to starting position"
        ],
        "parameters": {
            "sweep_distance": "300mm",
            "scan_positions": 5,
            "pause_duration": "1.0-2.0 seconds",
            "camera_angle": "90 degrees downward"
        },
        "applicable_tasks": ["find", "search", "scan", "explore", "detect"]
    },
    
    {
        "id": "pick_and_place_sequence",
        "title": "Complete Pick and Place Operation",
        "description": "Full sequence for picking up objects and placing them elsewhere",
        "context": "manipulation, precision, object transfer",
        "pattern": [
            "Scan area to locate source object",
            "Open gripper to appropriate width",
            "Approach object with safe hover distance",
            "Move to object position with small offset",
            "Close gripper with force feedback",
            "Lift object to safe height",
            "Move to destination location",
            "Lower to placement height",
            "Open gripper to release object",
            "Retreat to safe position"
        ],
        "error_recovery": [
            "If object not grasped: retry with adjusted position",
            "If collision detected: retreat and replan path",
            "If object slips: increase grip force within limits"
        ],
        "applicable_tasks": ["pick up", "move", "place", "transfer", "relocate"]
    },
    
    {
        "id": "cleaning_organization_pattern",
        "title": "Workspace Cleaning and Organization",
        "description": "Systematic approach to cleaning and organizing workspace",
        "context": "organization, cleaning, workspace management",
        "pattern": [
            "Scan entire workspace to identify all objects",
            "Categorize objects by type and size",
            "Plan organization strategy based on object types",
            "Pick up items one by one in logical order",
            "Place items in designated areas or containers",
            "Verify workspace is clean and organized"
        ],
        "strategies": [
            "Group similar objects together",
            "Place larger items first, smaller items on top",
            "Keep frequently used items easily accessible",
            "Ensure clear pathways for future operations"
        ],
        "applicable_tasks": ["clean", "organize", "tidy", "arrange", "sort"]
    },
    
    {
        "id": "kitchen_appliance_interaction",
        "title": "Kitchen Appliance Interaction Patterns",
        "description": "Safe patterns for interacting with kitchen appliances",
        "context": "kitchen tasks, appliance operation, safety",
        "pattern": [
            "Identify appliance type and current state",
            "Approach from appropriate angle (handle side for doors)",
            "Use appropriate grip strength for controls",
            "Operate controls with precise movements",
            "Monitor for safety indicators (heat, steam, etc.)",
            "Retreat to safe distance after operation"
        ],
        "appliance_specific": {
            "microwave": {
                "approach": "from door handle side",
                "safety_distance": "120mm",
                "interaction": "gentle pull for door, precise press for buttons"
            },
            "oven": {
                "approach": "from handle side with extra caution",
                "safety_distance": "150mm",
                "safety_notes": "check for heat indicators, use oven mitts if available"
            },
            "refrigerator": {
                "approach": "from handle side",
                "safety_distance": "100mm",
                "interaction": "firm pull for door opening"
            }
        },
        "applicable_tasks": ["open", "close", "operate", "use appliance"]
    }
]

# Task Examples and Solutions
TASK_EXAMPLES = [
    {
        "user_input": "move toward cup",
        "intent": "approach object",
        "solution_strategy": "Use safe object approach pattern",
        "steps": [
            "Scan area if cup not already detected",
            "Approach cup with 100mm hover distance",
            "Verify position and adjust if needed",
            "Return to home position"
        ],
        "reasoning": "User wants to move close to cup, likely for inspection or preparation for manipulation"
    },
    
    {
        "user_input": "pick up the bottle",
        "intent": "object manipulation",
        "solution_strategy": "Use complete pick and place sequence",
        "steps": [
            "Scan area to locate bottle",
            "Open gripper appropriately for bottle size",
            "Approach bottle with safe hover distance",
            "Grasp bottle with appropriate force",
            "Lift to safe height",
            "Move to designated location or await further instruction"
        ],
        "reasoning": "Clear manipulation task requiring careful grasping of potentially fragile object"
    },
    
    {
        "user_input": "help me clean up",
        "intent": "workspace organization",
        "solution_strategy": "Use cleaning organization pattern",
        "steps": [
            "Scan workspace comprehensively",
            "Identify all objects and their current locations",
            "Plan organization strategy",
            "Systematically move objects to appropriate locations",
            "Verify workspace is clean and organized"
        ],
        "reasoning": "Open-ended cleaning task requiring intelligent assessment and systematic approach"
    },
    
    {
        "user_input": "get ready for cooking",
        "intent": "preparation and setup",
        "solution_strategy": "Preparation and positioning pattern",
        "steps": [
            "Move to optimal position for kitchen assistance",
            "Scan area for cooking-related objects",
            "Ensure gripper is ready for manipulation",
            "Position for easy access to common cooking areas",
            "Report readiness to user"
        ],
        "reasoning": "Preparation task requiring proactive positioning and readiness for cooking assistance"
    },
    
    {
        "user_input": "find objects in the workspace",
        "intent": "exploration and detection",
        "solution_strategy": "Use systematic scanning pattern",
        "steps": [
            "Move to optimal scanning position",
            "Perform comprehensive area scan",
            "Identify and catalog all detected objects",
            "Report findings to user",
            "Return to ready position"
        ],
        "reasoning": "Exploration task requiring thorough scanning and object identification"
    }
]

# Problem-Solving Strategies
PROBLEM_SOLVING_STRATEGIES = [
    {
        "problem": "Object not detected",
        "symptoms": ["No objects found in expected location", "Vision system returns empty results"],
        "solutions": [
            "Perform comprehensive area scan with multiple positions",
            "Adjust camera angle and lighting conditions",
            "Move closer to suspected object location",
            "Try scanning from different heights",
            "Check if object might be occluded by other items"
        ],
        "prevention": "Regular scanning and maintaining clear workspace"
    },
    
    {
        "problem": "Grasping failure",
        "symptoms": ["Object slips from gripper", "Unable to pick up object", "Force feedback indicates poor grip"],
        "solutions": [
            "Adjust gripper position and approach angle",
            "Modify grip force based on object properties",
            "Try different grasping points on object",
            "Ensure gripper is properly opened before approach",
            "Check for object surface properties (smooth, wet, etc.)"
        ],
        "prevention": "Proper object assessment before grasping attempts"
    },
    
    {
        "problem": "Path planning conflicts",
        "symptoms": ["Unable to reach target position", "Collision warnings", "Movement blocked"],
        "solutions": [
            "Recompute path with different intermediate waypoints",
            "Adjust approach angle to avoid obstacles",
            "Move obstacles if they are moveable objects",
            "Use alternative movement strategy",
            "Break complex movements into smaller steps"
        ],
        "prevention": "Regular workspace scanning and obstacle awareness"
    },
    
    {
        "problem": "Task ambiguity",
        "symptoms": ["Unclear user instructions", "Multiple possible interpretations", "Insufficient context"],
        "solutions": [
            "Make reasonable assumptions based on context",
            "Choose safest interpretation of ambiguous commands",
            "Scan environment for additional context clues",
            "Default to conservative, safe actions",
            "Provide feedback about chosen interpretation"
        ],
        "prevention": "Maintain context awareness and ask for clarification when possible"
    }
]

# Safety Guidelines and Best Practices
SAFETY_GUIDELINES = [
    {
        "category": "Movement Safety",
        "guidelines": [
            "Always maintain minimum safe distances from objects and surfaces",
            "Use hover distances of 80-120mm for most operations",
            "Retreat to safe height before lateral movements",
            "Monitor force feedback continuously during interactions",
            "Stop immediately if unexpected resistance is encountered"
        ]
    },
    
    {
        "category": "Object Interaction",
        "guidelines": [
            "Assess object properties before interaction (weight, fragility, temperature)",
            "Use appropriate grip force for different object types",
            "Approach objects from optimal angles for stability",
            "Verify successful grasp before lifting objects",
            "Handle fragile objects with extra care and reduced speeds"
        ]
    },
    
    {
        "category": "Workspace Management",
        "guidelines": [
            "Keep workspace clear of unnecessary obstacles",
            "Maintain awareness of all objects in workspace",
            "Plan movements to avoid collisions with static objects",
            "Ensure emergency stop is always accessible",
            "Regular scanning to maintain situational awareness"
        ]
    },
    
    {
        "category": "Error Recovery",
        "guidelines": [
            "Always have a fallback plan for critical operations",
            "Implement graceful degradation when systems fail",
            "Return to known safe positions when errors occur",
            "Log all errors and recovery actions for learning",
            "Never continue operations if safety systems are compromised"
        ]
    }
]

# Learning Patterns and Adaptations
LEARNING_PATTERNS = [
    {
        "pattern": "Task Success Analysis",
        "description": "Analyze successful task completions to identify optimal strategies",
        "metrics": ["completion time", "number of steps", "error rate", "user satisfaction"],
        "adaptation": "Refine movement patterns based on successful executions"
    },
    
    {
        "pattern": "Error Pattern Recognition",
        "description": "Identify common failure modes and develop preventive strategies",
        "metrics": ["error frequency", "error types", "recovery success rate"],
        "adaptation": "Proactively avoid conditions that commonly lead to errors"
    },
    
    {
        "pattern": "Environment Adaptation",
        "description": "Adapt behavior based on workspace characteristics and object types",
        "metrics": ["object detection accuracy", "grasping success rate", "collision frequency"],
        "adaptation": "Customize approach strategies for different environments"
    },
    
    {
        "pattern": "User Preference Learning",
        "description": "Learn user preferences and adapt behavior accordingly",
        "metrics": ["user feedback", "task modification requests", "preferred outcomes"],
        "adaptation": "Customize task execution based on learned user preferences"
    }
]

# Context Understanding Patterns
CONTEXT_PATTERNS = [
    {
        "context_type": "Spatial Context",
        "description": "Understanding spatial relationships between objects and workspace",
        "indicators": ["object positions", "workspace layout", "accessibility paths"],
        "implications": "Affects movement planning and object interaction strategies"
    },
    
    {
        "context_type": "Temporal Context", 
        "description": "Understanding timing and sequence requirements",
        "indicators": ["task urgency", "sequential dependencies", "time constraints"],
        "implications": "Affects task prioritization and execution speed"
    },
    
    {
        "context_type": "Functional Context",
        "description": "Understanding the purpose and goals of tasks",
        "indicators": ["task objectives", "desired outcomes", "success criteria"],
        "implications": "Affects strategy selection and quality assessment"
    },
    
    {
        "context_type": "Safety Context",
        "description": "Understanding safety requirements and risk factors",
        "indicators": ["hazardous materials", "fragile objects", "safety constraints"],
        "implications": "Affects movement parameters and interaction methods"
    }
]

def get_all_knowledge():
    """Return all knowledge base content for RAG indexing."""
    return {
        "movement_patterns": MOVEMENT_PATTERNS,
        "task_examples": TASK_EXAMPLES,
        "problem_solving": PROBLEM_SOLVING_STRATEGIES,
        "safety_guidelines": SAFETY_GUIDELINES,
        "learning_patterns": LEARNING_PATTERNS,
        "context_patterns": CONTEXT_PATTERNS
    }

def get_knowledge_by_category(category: str):
    """Get knowledge for a specific category."""
    knowledge_map = {
        "movement": MOVEMENT_PATTERNS,
        "tasks": TASK_EXAMPLES,
        "problems": PROBLEM_SOLVING_STRATEGIES,
        "safety": SAFETY_GUIDELINES,
        "learning": LEARNING_PATTERNS,
        "context": CONTEXT_PATTERNS
    }
    return knowledge_map.get(category, [])

def search_knowledge_by_keywords(keywords: list):
    """Search knowledge base by keywords."""
    results = []
    all_knowledge = get_all_knowledge()
    
    for category, items in all_knowledge.items():
        for item in items:
            item_text = str(item).lower()
            if any(keyword.lower() in item_text for keyword in keywords):
                results.append({
                    "category": category,
                    "item": item,
                    "relevance_score": sum(1 for keyword in keywords if keyword.lower() in item_text)
                })
    
    # Sort by relevance score
    results.sort(key=lambda x: x["relevance_score"], reverse=True)
    return results
