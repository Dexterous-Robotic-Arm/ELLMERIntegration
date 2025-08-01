# src/ELLMER/ros_kortex/kortex_examples/src/kortex_examples/ELLMER/llm_interface.py

import json

def call_llm(task: str, world: dict):
    """
    Stub LLM planner.  
    Inputs:
      - task: e.g. "Grab the red cup."
      - world: {
          "joints": [j1, j2, …, j6],
          "objects": [ [x,y,z], … ]   # in same order you published them
        }
    Returns:
      - A JSON‐decoded list of steps:
         * action="move_joints" with a 6‐element joint list & duration
         * action="open_gripper" or "close_gripper" with duration
    """
    joints = world.get("joints", [])
    objects = world.get("objects", [])

    # Build a descriptive prompt (for a real LLM you'd include nice system/user roles)
    prompt = f"""
    Task: {task}
    Current joint state: {joints}
    Known objects (x,y,z): {objects}

    Please return ONLY a JSON array of steps.  
    Each step must be an object with:
      • "action": one of "move_joints", "open_gripper", "close_gripper"  
      • "joints": [6 floats]  # only for move_joints  
      • "duration": float seconds  

    Example:
    [
      {{ "action": "move_joints",   "joints":[0.1,0,0,0,0,0], "duration":2.0 }},
      {{ "action": "move_joints",   "joints":[0.1,0,0.1,0,0,0], "duration":1.0 }},
      {{ "action": "close_gripper", "duration":1.0 }},
      {{ "action": "move_joints",   "joints":[0.1,0,0.2,0,0,0], "duration":2.0 }}
    ]
    """

    # --- STUB LOGIC: just grab the first object if any, otherwise do a no-op wave-like plan ---
    plan = []
    if "grab" in task.lower() and objects:
        x,y,z = objects[0]
        # 1) move above
        plan.append({
            "action": "move_joints",
            "joints": [0.0, -0.5, 0.5, 0.0, 0.5, 0.0],
            "duration": 2.0
        })
        # 2) lower (stub joints)
        plan.append({
            "action": "move_joints",
            "joints": [0.0, -0.6, 0.6, 0.0, 0.6, 0.0],
            "duration": 1.0
        })
        # 3) close gripper
        plan.append({
            "action": "close_gripper",
            "duration": 1.0
        })
        # 4) lift up
        plan.append({
            "action": "move_joints",
            "joints": [0.0, -0.5, 0.5, 0.0, 0.5, 0.0],
            "duration": 2.0
        })
    else:
        # default “wave joint 1” plan
        plan = [
            {"action": "move_joints", "joints":[0.7,0,0,0,0,0], "duration":1.0},
            {"action": "move_joints", "joints":[-0.7,0,0,0,0,0], "duration":2.0},
            {"action": "move_joints", "joints":[0.0,0,0,0,0,0], "duration":1.0},
        ]

    return plan
