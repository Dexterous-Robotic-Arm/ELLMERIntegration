# Custom Gemini Examples

This module demonstrates how to interface Google's Gemini API with robotic planning and gripper control logic using ROS.

## Key Components

- `app.py`: Loads instructions and schemas, then queries Gemini to produce actionable commands.
- `custom_knowledge.md`: Background task and environment knowledge.
- `custom_gpt_instructions.md`: Task-level prompting instructions for Gemini.
- `custom_gpt_action_schema.md`: Format definition for expected output actions.

## Running

```bash
python3 app.py
```

Ensure your `.env` file includes `GEMINI_API_KEY`.
