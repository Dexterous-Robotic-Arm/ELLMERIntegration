# planner_llm.py
import os, re, json
from pathlib import Path
from typing import List
from dotenv import load_dotenv
load_dotenv()

# --- Contract loading helpers ---
CONTRACT_MD = os.environ.get("ACTION_CONTRACT_MD", "custom_gpt_action_schema.md")

def _read_contract_top(md_path: str) -> str:
    minimal_fallback = (
        "You are a planner for a UFactory xArm. "
        "Return ONLY a JSON object with keys: goal (string), steps (array). "
        "Allowed actions: MOVE_TO_NAMED{name}, APPROACH_NAMED{name,hover_mm?}, "
        "MOVE_TO_OBJECT{label|labels,offset_mm?,hover_mm?,timeout_sec?,min_conf?,selector?,ref?,index?}, "
        'APPROACH_OBJECT{...same selection fields..., hover_mm?}, '
        "RETREAT_Z{dz_mm>0}, SLEEP{seconds}, MOVE_TO_POSE{pose{xyz_mm[3], rpy_deg[3]}}. "
        "Distances in mm, angles in degrees. Output ONLY JSON, no prose."
    )
    try:
        text = Path(md_path).read_text(encoding="utf-8")
    except FileNotFoundError:
        return minimal_fallback
    parts = re.split(r"\n##\s*Machine[-\s]*Readable", text, maxsplit=1, flags=re.IGNORECASE)
    top = parts[0].strip() if parts else text.strip()
    return top or minimal_fallback

def build_prompt(task_text: str, pose_names: List[str] | None = None, contract_path: str | None = None) -> str:
    if contract_path is None:
        contract_path = CONTRACT_MD
    contract = _read_contract_top(contract_path)
    pose_names = pose_names or []
    names_list = ", ".join(pose_names) if pose_names else "(none provided)"
    rules = """
PLANNING RULES (MANDATORY):
1) Before touching any object, ALWAYS add: {"action":"APPROACH_OBJECT","hover_mm":80,...} immediately followed by {"action":"MOVE_TO_OBJECT",...}.
2) After touching any object, ALWAYS add {"action":"RETREAT_Z","dz_mm":80}.
3) If the user says "return home" (explicitly or implicitly after each object), ALWAYS insert {"action":"MOVE_TO_NAMED","name":"home"} after that object's retreat.
4) Prefer "labels":[...] over "label". If both present, use "labels".
5) For *_OBJECT steps, always include "timeout_sec":5 and default "selector":"nearest" unless specified.
6) If a requested label isn't common (e.g., "stick"), include aliases in labels: ["stick","baseball bat","umbrella"].
7) Output ONLY a single JSON object. No prose or code fences.
""".strip()
    return (
        f"{contract}\n\n"
        f"Named poses you may use: [{names_list}]\n"
        f"{rules}\n"
        f'User task: "{task_text}"\n'
        f"Return only JSON (no prose, no code fences, no explanations)."
    )

# --- LLM (google.genai) ---
from google import genai
from google.genai import types

def _make_client() -> genai.Client:
    api_key = os.getenv("GEMINI_API_KEY") or os.getenv("GOOGLE_API_KEY")
    if not api_key:
        # If you prefer silent auto-detection, you can return genai.Client() without the guard
        raise RuntimeError("No API key found. Set GEMINI_API_KEY or GOOGLE_API_KEY.")
    return genai.Client(api_key=api_key)

# Optional: schema extractor (if you have utils_schema.py)
try:
    from utils_schema import load_schema_from_md
except Exception:
    load_schema_from_md = None

MODEL = os.getenv("GEMINI_MODEL", "gemini-2.5-flash")

def plan_with_gemini(task_text: str, pose_names: List[str]):
    client = _make_client()
    prompt = build_prompt(task_text, pose_names)
    schema = None
    if load_schema_from_md:
        try:
            schema = load_schema_from_md(CONTRACT_MD)
        except Exception:
            schema = None
    config = types.GenerateContentConfig(
        response_mime_type="application/json",
        response_schema=schema if schema else None
    )
    resp = client.models.generate_content(
        model=MODEL,
        contents=prompt,
        config=config
    )
    text = (resp.text or "").strip()
    if text.startswith("```"):
        text = text.strip("`")
        start, end = text.find("{"), text.rfind("}")
        text = text[start:end+1] if start >= 0 and end >= 0 else text
    return json.loads(text)

def plan_fallback(task_text: str):
    return {
        "goal": task_text,
        "steps": [
            { "action": "APPROACH_OBJECT", "labels": ["cup"], "selector": "nearest", "hover_mm": 80, "timeout_sec": 5 },
            { "action": "MOVE_TO_OBJECT",  "labels": ["cup"], "selector": "nearest", "offset_mm": [0, 0, 0], "timeout_sec": 5 },
            { "action": "RETREAT_Z", "dz_mm": 80 },
            { "action": "MOVE_TO_NAMED", "name": "home" }
        ]
    }
