# utils_schema.py
import re, json
from pathlib import Path
from jsonschema import validate as _validate

def load_schema_from_md(md_path: str) -> dict:
    text = Path(md_path).read_text(encoding="utf-8")
    blocks = re.findall(r"```json\s*(\{[\s\S]*?\})\s*```", text, flags=re.I)
    if not blocks:
        raise RuntimeError("No JSON schema block found in contract MD.")
    return json.loads(blocks[-1])

def validate_plan(plan: dict, schema: dict):
    _validate(instance=plan, schema=schema)