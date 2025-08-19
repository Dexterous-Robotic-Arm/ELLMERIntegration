# xArm Action Plan Contract (LLM-Facing)
**Version:** 1.1 (movement + gripper control)  
**Purpose:** Define the exact JSON format the planner must output so the executor can move a UFactory xArm based on named poses and detected objects, including gripper control.  
**Important:** Return **ONLY** a single JSON object — **no prose, no comments, no backticks/code fences**.

---

## Output Rules
- Output must be a single, valid **JSON object**.
- **Do not** wrap the JSON in markdown fences or add text before/after.
- JSON **must** conform to the schema in **Machine-Readable JSON Schema** below (field names, enums, and types).

---

## Actions (Verbs) — Movement and Gripper Control
These are the only allowed `action` values. Any other value is invalid.

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

---

## Field Conventions
- **Units:** distances in **millimeters (mm)**; angles in **degrees** (roll-pitch-yaw).
- **Gripper positions:** 0 = fully closed, 850 = fully open.
- **Object labels** must match the detector's class names (e.g., COCO: `"cup"`, `"bottle"`, `"bowl"`).
- If both `hover_mm` and `offset_mm` are omitted, use defaults: `hover_mm = 80`, `offset_mm = [0,0,0]`.
- For `*_OBJECT` actions, if the label is not seen within `timeout_sec`, the executor may fail/abort the plan.

---

## Required Top-Level Keys
- `goal` (string): the user's task in natural language.
- `steps` (array): ordered list of action objects.

---

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

## Valid JSON Example — Using Multiple Labels
```json
{
  "goal": "Pick up the closest cup or bottle and place it in the bin",
  "steps": [
    { "action": "MOVE_TO_NAMED", "name": "home" },
    { "action": "OPEN_GRIPPER" },
    { "action": "APPROACH_OBJECT", "labels": ["cup", "bottle"], "hover_mm": 80, "timeout_sec": 5 },
    { "action": "MOVE_TO_OBJECT", "labels": ["cup", "bottle"], "offset_mm": [0, 0, 0], "timeout_sec": 5 },
    { "action": "GRIPPER_SOFT_CLOSE" },
    { "action": "RETREAT_Z", "dz_mm": 80 },
    { "action": "MOVE_TO_NAMED", "name": "bin_drop" },
    { "action": "GRIPPER_RELEASE" },
    { "action": "MOVE_TO_NAMED", "name": "home" }
  ]
}
```

## Valid JSON Example — Precise Gripper Control
```json
{
  "goal": "Gently grasp the fragile object with precise control",
  "steps": [
    { "action": "MOVE_TO_NAMED", "name": "home" },
    { "action": "GRIPPER_HALF_OPEN" },
    { "action": "APPROACH_OBJECT", "label": "fragile_object", "hover_mm": 100, "timeout_sec": 5 },
    { "action": "MOVE_TO_OBJECT", "label": "fragile_object", "offset_mm": [0, 0, 0], "timeout_sec": 5 },
    { "action": "SET_GRIPPER_POSITION", "position": 300, "speed": 30, "force": 20 },
    { "action": "RETREAT_Z", "dz_mm": 100 },
    { "action": "MOVE_TO_NAMED", "name": "home" }
  ]
}
```

## Machine Readable JSON Schema
```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "title": "RobotPlan",
  "type": "object",
  "required": ["goal", "steps"],
  "additionalProperties": false,
  "properties": {
    "goal": { "type": "string" },
    "steps": {
      "type": "array",
      "minItems": 1,
      "items": {
        "type": "object",
        "additionalProperties": false,
        "required": ["action"],
        "properties": {
          "action": {
            "type": "string",
            "enum": [
              "MOVE_TO_NAMED",
              "APPROACH_NAMED",
              "MOVE_TO_OBJECT",
              "APPROACH_OBJECT",
              "RETREAT_Z",
              "SLEEP",
              "MOVE_TO_POSE",
              "OPEN_GRIPPER",
              "CLOSE_GRIPPER",
              "SET_GRIPPER_POSITION",
              "GRIPPER_GRASP",
              "GRIPPER_RELEASE",
              "GRIPPER_HALF_OPEN",
              "GRIPPER_SOFT_CLOSE",
              "GRIPPER_TEST",
              "SCAN_FOR_OBJECTS",
              "SCAN_AREA"
            ]
          },
          "name": { "type": "string" },
          "label": { "type": "string" },
          "labels": {
            "type": "array",
            "items": { "type": "string" },
            "minItems": 1
          },
          "hover_mm": { "type": "number", "minimum": 0 },
          "dz_mm": { "type": "number", "exclusiveMinimum": 0 },
          "timeout_sec": { "type": "number", "exclusiveMinimum": 0 },
          "min_conf": { "type": "number", "minimum": 0, "maximum": 1 },
          "selector": { "type": "string", "enum": ["nearest", "highest_conf"] },
          "ref": {
            "type": "object",
            "additionalProperties": false,
            "properties": {
              "named": { "type": "string" }
            }
          },
          "index": { "type": "integer", "minimum": 0 },
          "offset_mm": {
            "type": "array",
            "items": { "type": "number" },
            "minItems": 3,
            "maxItems": 3
          },
          "seconds": { "type": "number", "minimum": 0 },
          "pattern": { "type": "string" },
          "sweep_mm": { "type": "number", "minimum": 0 },
          "steps": { "type": "integer", "minimum": 1 },
          "pause_sec": { "type": "number", "minimum": 0 },
          "scan_duration": { "type": "number", "minimum": 0 },
          "scan_area": { "type": "string" },
          "pose": {
            "type": "object",
            "additionalProperties": false,
            "required": ["xyz_mm", "rpy_deg"],
            "properties": {
              "xyz_mm": {
                "type": "array",
                "items": { "type": "number" },
                "minItems": 3,
                "maxItems": 3
              },
              "rpy_deg": {
                "type": "array",
                "items": { "type": "number" },
                "minItems": 3,
                "maxItems": 3
              }
            }
          },
          "gripper": {
            "type": "object",
            "additionalProperties": false,
            "properties": {
              "position": { "type": "number", "minimum": 0, "maximum": 850 },
              "speed": { "type": "number", "minimum": 0 },
              "force": { "type": "number", "minimum": 0 }
            }
          }
        },
        "allOf": [
          {
            "if": { "properties": { "action": { "const": "MOVE_TO_NAMED" } }, "required": ["action"] },
            "then": { "required": ["action", "name"] }
          },
          {
            "if": { "properties": { "action": { "const": "APPROACH_NAMED" } }, "required": ["action"] },
            "then": { "required": ["action", "name"] }
          },
          {
            "if": { "properties": { "action": { "enum": ["MOVE_TO_OBJECT", "APPROACH_OBJECT"] } }, "required": ["action"] },
            "then": {
              "anyOf": [
                { "required": ["labels"] },
                { "required": ["label"] }
              ]
            }
          },
          {
            "if": { "properties": { "action": { "const": "RETREAT_Z" } }, "required": ["action"] },
            "then": { "required": ["action", "dz_mm"] }
          },
          {
            "if": { "properties": { "action": { "const": "SLEEP" } }, "required": ["action"] },
            "then": { "required": ["action", "seconds"] }
          },
          {
            "if": { "properties": { "action": { "const": "MOVE_TO_POSE" } }, "required": ["action"] },
            "then": { "required": ["action", "pose"] }
          },
          {
            "if": { "properties": { "action": { "const": "OPEN_GRIPPER" } }, "required": ["action"] },
            "then": { "required": ["action"] }
          },
          {
            "if": { "properties": { "action": { "const": "CLOSE_GRIPPER" } }, "required": ["action"] },
            "then": { "required": ["action"] }
          },
          {
            "if": { "properties": { "action": { "const": "SET_GRIPPER_POSITION" } }, "required": ["action"] },
            "then": { "required": ["action", "position", "speed", "force"] }
          },
          {
            "if": { "properties": { "action": { "const": "GRIPPER_GRASP" } }, "required": ["action"] },
            "then": { "required": ["action", "target_position", "speed", "force", "timeout"] }
          },
          {
            "if": { "properties": { "action": { "const": "GRIPPER_RELEASE" } }, "required": ["action"] },
            "then": { "required": ["action", "target_position", "speed", "force"] }
          },
          {
            "if": { "properties": { "action": { "const": "GRIPPER_HALF_OPEN" } }, "required": ["action"] },
            "then": { "required": ["action", "speed", "force"] }
          },
          {
            "if": { "properties": { "action": { "const": "GRIPPER_SOFT_CLOSE" } }, "required": ["action"] },
            "then": { "required": ["action", "speed", "force"] }
          },
          {
            "if": { "properties": { "action": { "const": "GRIPPER_TEST" } }, "required": ["action"] },
            "then": { "required": ["action", "cycles", "delay"] }
          },
          {
            "if": { "properties": { "action": { "const": "SCAN_FOR_OBJECTS" } }, "required": ["action"] },
            "then": { "required": ["action"] }
          },
          {
            "if": { "properties": { "action": { "const": "SCAN_AREA" } }, "required": ["action"] },
            "then": { "required": ["action"] }
          }
        ]
      }
    }
  }
}
```