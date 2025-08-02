# xArm Action Plan Contract (LLM-Facing)
**Version:** 1.0 (movement-only)  
**Purpose:** Define the exact JSON format the planner must output so the executor can move a UFactory xArm based on named poses and detected objects.  
**Important:** Return **ONLY** a single JSON object — **no prose, no comments, no backticks/code fences**.

---

## Output Rules
- Output must be a single, valid **JSON object**.
- **Do not** wrap the JSON in markdown fences or add text before/after.
- JSON **must** conform to the schema in **Machine-Readable JSON Schema** below (field names, enums, and types).

---

## Actions (Verbs) — Movement Only
These are the only allowed `action` values. Any other value is invalid.

- `MOVE_TO_NAMED` — Move to a named pose from the world model.  
  - **fields:** `name` (string)
- `APPROACH_NAMED` — Move to a hover position above a named pose.  
  - **fields:** `name` (string), `hover_mm` (number, optional; default **80**)
- `MOVE_TO_OBJECT` — Move to a position derived from a detected object label.  
  - **fields:** `label` (string), `offset_mm` ([x,y,z] mm, optional; default **[0,0,0]**), `timeout_sec` (number, optional; default **5**)
- `APPROACH_OBJECT` — Move to a hover position above a detected object label.  
  - **fields:** `label` (string), `hover_mm` (number, optional; default **80**), `timeout_sec` (number, optional; default **5**)
- `RETREAT_Z` — Move upward along Z by `dz_mm` (positive).  
  - **fields:** `dz_mm` (number > 0)
- `MOVE_TO_POSE` — Move to an explicit numeric pose.  
  - **fields:** `pose` (object: `xyz_mm` [x,y,z], `rpy_deg` [r,p,y])
- `SLEEP` — Wait for a number of seconds.  
  - **fields:** `seconds` (number)

> **No gripper commands** in this contract (those will be handled by a separate hand controller via events).

---

## Field Conventions
- **Units:** distances in **millimeters (mm)**; angles in **degrees** (roll-pitch-yaw).
- **Object labels** must match the detector’s class names (e.g., COCO: `"cup"`, `"bottle"`, `"bowl"`).
- If both `hover_mm` and `offset_mm` are omitted, use defaults: `hover_mm = 80`, `offset_mm = [0,0,0]`.
- For `*_OBJECT` actions, if the label is not seen within `timeout_sec`, the executor may fail/abort the plan.

---

## Required Top-Level Keys
- `goal` (string): the user’s task in natural language.
- `steps` (array): ordered list of action objects.

---

## Valid JSON Example - Object Driven
```json
{
  "goal": "Approach the cup, touch it, lift, and go home",
  "steps": [
    { "action": "APPROACH_OBJECT", "label": "cup", "hover_mm": 80, "timeout_sec": 5 },
    { "action": "MOVE_TO_OBJECT", "label": "cup", "offset_mm": [0, 0, 0], "timeout_sec": 5 },
    { "action": "RETREAT_Z", "dz_mm": 80 },
    { "action": "MOVE_TO_NAMED", "name": "home" }
  ]
}
```

## Valid JSON Example — Using Detected Objects
```json
{
  "goal": "Approach the closest cup, touch it, lift, and go home",
  "steps": [
    { "action": "APPROACH_OBJECT", "labels": ["cup"], "selector": "nearest", "hover_mm": 80, "timeout_sec": 5 },
    { "action": "MOVE_TO_OBJECT", "labels": ["cup"], "selector": "nearest", "offset_mm": [0, 0, 0], "timeout_sec": 5 },
    { "action": "RETREAT_Z", "dz_mm": 80 },
    { "action": "MOVE_TO_NAMED", "name": "home" }
  ]
}
```

## Valid JSON Example — Select second-nearest among multiple
```json
{
  "goal": "Go above the second-nearest bottle relative to bin, then touch it",
  "steps": [
    { "action": "APPROACH_OBJECT",
      "labels": ["bottle"],
      "selector": "nearest",
      "ref": { "named": "bin_drop" },
      "index": 1,
      "hover_mm": 60,
      "timeout_sec": 5
    },
    { "action": "MOVE_TO_OBJECT",
      "labels": ["bottle"],
      "selector": "nearest",
      "ref": { "named": "bin_drop" },
      "index": 1,
      "offset_mm": [0, 0, 0],
      "timeout_sec": 5
    }
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
              "MOVE_TO_POSE"
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
          }
        ]
      }
    }
  }
}
```