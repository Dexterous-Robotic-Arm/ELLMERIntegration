#!/usr/bin/env python3
"""
app.py — LLM-only planner with interactive input

- Reads the LLM contract (.md) via planner_llm
- Calls Gemini to generate a strict-JSON plan (or uses fallback)
- Prints ONLY the JSON plan to stdout (no execution, no validation, no ROS)
- Interactive mode: prompts for a task (to stderr), outputs JSON to stdout
"""

import os
import sys
import json
import argparse
from pathlib import Path

from planner_llm import plan_with_gemini, plan_fallback

# Optional: load pose names from world_model.yaml to give LLM better context
try:
    import yaml
except Exception:
    yaml = None


def _pose_names_from_world(world_yaml: str | None) -> list[str]:
    if not world_yaml or not yaml:
        return []
    p = Path(world_yaml)
    if not p.exists():
        return []
    try:
        data = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
        poses = data.get("poses", {}) or {}
        return list(poses.keys())
    except Exception:
        return []


def _emit_json(obj) -> None:
    """Write ONLY JSON to stdout."""
    sys.stdout.write(json.dumps(obj))
    sys.stdout.flush()


def _prompt_once(prompt_text="Task: ") -> str:
    """Prompt on stderr so stdout stays clean for JSON."""
    sys.stderr.write(prompt_text)
    sys.stderr.flush()
    return input().strip()


def generate_plan(task: str, pose_names=None, use_fallback=False):
    pose_names = pose_names or []
    if use_fallback:
        return plan_fallback(task)
    return plan_with_gemini(task, pose_names)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", help='e.g. "approach the closest cup and then go home"')
    ap.add_argument("--interactive", "-i", action="store_true",
                    help="Prompt for a task interactively (to stderr) and output JSON (to stdout)")
    ap.add_argument("--loop", "-l", action="store_true",
                    help="Interactive loop: keep asking for tasks until empty line or Ctrl-C")
    ap.add_argument("--use-fallback", action="store_true",
                    help="Bypass LLM and use a canned plan")
    ap.add_argument("--world", default=os.environ.get("WORLD_YAML", "world_model.yaml"),
                    help="Optional: world_model.yaml to harvest named pose names for the LLM prompt")
    ap.add_argument("--pose-names", default="",
                    help='Optional: comma-separated pose names (overrides --world), e.g. "home,bin_drop,table_pick_blue"')
    args = ap.parse_args()

    # Build pose name list for better LLM context
    pose_names = [s.strip() for s in args.pose_names.split(",") if s.strip()] \
                 or _pose_names_from_world(args.world)

    # Decide mode
    if args.task:
        plan = generate_plan(args.task, pose_names=pose_names, use_fallback=args.use_fallback)
        _emit_json(plan)
        return

    if not args.interactive and not args.loop:
        # Default to single interactive prompt if --task not provided
        args.interactive = True

    if args.interactive and not args.loop:
        task = _prompt_once("Task: ")
        if not task:
            sys.stderr.write("No task entered.\n")
            return
        plan = generate_plan(task, pose_names=pose_names, use_fallback=args.use_fallback)
        _emit_json(plan)
        return

    # Loop mode
    if args.loop:
        sys.stderr.write("Interactive planning loop. Press Enter on empty line to quit.\n")
        while True:
            try:
                task = _prompt_once("Task: ")
                if not task:
                    sys.stderr.write("Bye.\n")
                    break
                plan = generate_plan(task, pose_names=pose_names, use_fallback=args.use_fallback)
                _emit_json(plan)
                sys.stdout.write("\n")  # separate multiple JSON objects with newlines
                sys.stdout.flush()
            except (KeyboardInterrupt, EOFError):
                sys.stderr.write("\nBye.\n")
                break


if __name__ == "__main__":
    main()
