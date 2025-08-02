# utils_plan.py
from copy import deepcopy

# Optional aliases to help when users say non-COCO words:
LABEL_ALIASES = {
    "stick": ["stick","baseball bat","umbrella","remote"],  # adjust to your model
    "cup":   ["cup","mug"],
}

def _ensure_array(val_or_array, key):
    if val_or_array is None:
        return None
    return val_or_array if isinstance(val_or_array, list) else [val_or_array]

def normalize_plan(plan: dict) -> dict:
    """Fill sensible defaults and canonicalize object steps."""
    p = deepcopy(plan)
    steps = p.get("steps", [])
    norm = []
    for i, s in enumerate(steps):
        s = deepcopy(s)
        act = s.get("action", "")

        # Canonicalize label(s)
        if act in ("APPROACH_OBJECT","MOVE_TO_OBJECT"):
            labels = s.get("labels")
            label  = s.get("label")
            if labels:
                labels = list({*labels})  # dedupe
            elif label:
                labels = [label]
                s.pop("label", None)
            else:
                # no label info: keep as-is; schema validation may catch it
                labels = None

            # Expand aliases
            if labels:
                expanded = []
                for L in labels:
                    expanded += LABEL_ALIASES.get(L, [L])
                s["labels"] = list(dict.fromkeys(expanded))  # unique, ordered

            # Defaults
            s.setdefault("timeout_sec", 5)
            if act == "APPROACH_OBJECT":
                s.setdefault("hover_mm", 80)
            s.setdefault("selector", "nearest")
            s.setdefault("offset_mm", [0,0,0])

        if act == "RETREAT_Z":
            # Make sure it's positive and reasonable
            s["dz_mm"] = max(float(s.get("dz_mm", 80)), 20.0)

        norm.append(s)

    p["steps"] = norm
    return p

def enforce_policies(plan: dict) -> dict:
    """
    Enforce safety/task policies:
    - Every MOVE_TO_OBJECT must be preceded by APPROACH_OBJECT (hover >= 60).
    - Every MOVE_TO_OBJECT must be followed by RETREAT_Z (>=60) before any non-vertical move.
    - If the goal mentions 'return home', ensure MOVE_TO_NAMED: home after each object block.
    """
    p = deepcopy(plan)
    steps = p.get("steps", [])
    out = []
    i = 0
    want_home = ("return home" in p.get("goal","").lower())

    while i < len(steps):
        s = steps[i]; act = s.get("action","")
        # Insert approach before a naked MOVE_TO_OBJECT
        if act == "MOVE_TO_OBJECT":
            # Look back: if previous is not APPROACH_OBJECT to same labels, insert one
            prev = out[-1] if out else {}
            if prev.get("action") != "APPROACH_OBJECT":
                out.append({
                    "action":"APPROACH_OBJECT",
                    "labels": s.get("labels") or ( [s.get("label")] if s.get("label") else None ),
                    "hover_mm": max(60, float(s.get("hover_mm", 80))),
                    "timeout_sec": float(s.get("timeout_sec", 5)),
                    "selector": s.get("selector","nearest"),
                })
        out.append(s)

        # Ensure retreat after MOVE_TO_OBJECT
        if act == "MOVE_TO_OBJECT":
            # Peek next
            nxt = steps[i+1] if (i+1) < len(steps) else {}
            if nxt.get("action") != "RETREAT_Z":
                out.append({"action":"RETREAT_Z","dz_mm":80})
            # If user wants home after each object, add it (after retreat)
            if want_home:
                out.append({"action":"MOVE_TO_NAMED","name":"home"})

        i += 1

    p["steps"] = out
    return p
