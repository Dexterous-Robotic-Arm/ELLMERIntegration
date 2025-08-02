"""
Gemini Planner Interface for xArm via ROS
"""

import os
import google.generativeai as genai
from dotenv import load_dotenv

load_dotenv()

GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
genai.configure(api_key=GEMINI_API_KEY)

prompt = """You are controlling a robot gripper arm.
Current task: Pick up the blue cube from the table and place it in the bin.
Generate a structured action plan using this schema:
1. Approach target
2. Grasp
3. Move to destination
4. Release"""

response = genai.generate_text(model="models/gemini-pro", prompt=prompt)

print("[Gemini Plan]:")
print(response.text)
