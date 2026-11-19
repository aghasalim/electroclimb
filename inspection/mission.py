#!/usr/bin/env python3
"""LLM mission planner: a sentence in, a climb in the simulator out.

Claude turns an instruction ("inspect the wall every 20 cm up to 60 cm") into a validated
plan (structured output, checked against the robot's limits). The crawl gait then climbs to
each waypoint; at "inspect" waypoints it stops with all four pads stuck, holds, and saves a
camera frame for the crack detector.

    ../.venv/bin/python mission.py "inspect the wall every 20 cm up to 60 cm"
    ../.venv/bin/python mission.py --plan plan.json           # run a saved plan, no API call

Writes missions/<time>/plan.json, log.json and one frame per inspection.
Needs an Anthropic API key for planning (ANTHROPIC_API_KEY or `ant auth login`).
"""
import json
import sys
import time
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, Field

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "sim"))

MODEL = "claude-opus-5-5"
MAX_HEIGHT_CM = 100          # longest climb the simulated test wall allows
MAX_HOLD_S = 10
CM_PER_S = 0.9               # measured crawl-gait speed on a vertical wall (18 cm in 20 s)


class Waypoint(BaseModel):
    height_cm: float = Field(description="height above the start position, in cm")
    action: Literal["inspect", "pass"] = Field(description="inspect = stop, hold and take a photo")
    hold_s: float = Field(description="seconds to hold still at an inspect waypoint, 0 for pass")
    note: str = Field(description="why this waypoint is in the plan")


class MissionPlan(BaseModel):
    summary: str
    waypoints: list[Waypoint]


def check(plan):
    """Reject plans the robot can't fly; returns a list of problems (empty = OK)."""
    problems, last = [], 0.0
    for i, w in enumerate(plan.waypoints):
        if not 0 < w.height_cm <= MAX_HEIGHT_CM:
            problems.append(f"waypoint {i}: height {w.height_cm} cm outside 0..{MAX_HEIGHT_CM} cm")
        if w.height_cm <= last:
            problems.append(f"waypoint {i}: heights must increase (the robot only climbs up)")
        if not 0 <= w.hold_s <= MAX_HOLD_S:
            problems.append(f"waypoint {i}: hold {w.hold_s} s outside 0..{MAX_HOLD_S} s")
        last = w.height_cm
    return problems


def plan_with_claude(instruction, attempts=2):
    import anthropic
    client = anthropic.Anthropic()
    try:
        client.models.retrieve(MODEL)
    except anthropic.AuthenticationError:
        raise SystemExit("The Anthropic API key was rejected. Set a valid ANTHROPIC_API_KEY (console.anthropic.com).")
    system = (f"You plan inspection climbs for ElectroClimb, a wall-climbing robot. It climbs straight up a "
              f"vertical wall at about {CM_PER_S} cm/s, from 0 to at most {MAX_HEIGHT_CM} cm, and can only "
              f"climb up. At 'inspect' waypoints it stops, holds 1 to {MAX_HOLD_S} s and photographs the wall. "
              f"Plan the fewest waypoints that do what the operator asks.")
    messages = [{"role": "user", "content": instruction}]
    for _ in range(attempts):
        response = client.messages.parse(model=MODEL, max_tokens=16000, system=system, messages=messages,
                                         thinking={"type": "adaptive"}, output_format=MissionPlan)
        if response.stop_reason == "refusal" or response.parsed_output is None:
            raise SystemExit("Claude did not return a plan.")
        plan = response.parsed_output
        problems = check(plan)
        if not problems:
            return plan
        messages += [{"role": "assistant", "content": response.content},
                     {"role": "user", "content": "That plan breaks the robot's limits: " + "; ".join(problems)
                      + ". Return a corrected plan."}]
    raise SystemExit("No valid plan: " + "; ".join(problems))


def fly(plan, out):
    """Execute the plan with the crawl gait in MuJoCo. Returns the mission log."""
    import mujoco
    from crawl_gait import CrawlGait
    from env import ClimbEnv
    from render_media import make_camera
    env = ClimbEnv(tilt_deg=90, randomise=False, fail_prob=0.0, episode_s=3600)
    env.reset()
    gait = CrawlGait(env)
    renderer = mujoco.Renderer(env.model, 360, 360)
    cam = make_camera((0.11, 0, 0.05), 0.75, -90, 90)
    dt, t, log = env.model.opt.timestep * env.frame_skip, 0.0, []
    action = gait(env._foot_contacts())
    for i, w in enumerate(plan.waypoints):
        info = {"progress": env.data.xpos[env.torso, 0] - env.start_x, "fell": False}
        # climb until the waypoint, then finish the gait cycle so all four pads are stuck
        while info["progress"] * 100 < w.height_cm or (gait.t % gait.cycle) > dt * 1.5:
            action = gait(env._foot_contacts())
            _, _, _, info = env.step(action)
            t += dt
            if info["fell"] or t > 3000:
                log.append({"waypoint": i, "result": "fell" if info["fell"] else "timeout", "time_s": round(t, 1)})
                return log
        entry = {"waypoint": i, "target_cm": w.height_cm, "reached_cm": round(info["progress"] * 100, 1),
                 "action": w.action, "time_s": round(t, 1), "note": w.note}
        if w.action == "inspect":
            hold = action.copy()
            hold[12:] = 1.0                                         # every pad on while holding
            for _ in range(int(w.hold_s / dt)):
                _, _, _, info = env.step(hold)
                t += dt
            renderer.update_scene(env.data, cam)
            from PIL import Image
            frame = out / f"inspect_{i:02d}_{w.height_cm:.0f}cm.png"
            Image.fromarray(renderer.render()).rotate(90, expand=True).save(frame)
            entry.update(photo=frame.name, slip_during_hold_cm=round((info["progress"] * 100) - entry["reached_cm"], 2))
        log.append(entry)
    return log


def main(argv):
    out = HERE / "missions" / time.strftime("%Y%m%d-%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    if argv[:1] == ["--plan"]:
        plan = MissionPlan.model_validate_json(Path(argv[1]).read_text())
        problems = check(plan)
        if problems:
            raise SystemExit("Invalid plan: " + "; ".join(problems))
    else:
        plan = plan_with_claude(" ".join(argv))
    (out / "plan.json").write_text(plan.model_dump_json(indent=1))
    print(f"plan: {plan.summary}")
    for w in plan.waypoints:
        print(f"  {w.height_cm:5.1f} cm  {w.action:7s} hold {w.hold_s:.0f} s  {w.note}")
    log = fly(plan, out)
    (out / "log.json").write_text(json.dumps(log, indent=1))
    for e in log:
        print("  ->", e)
    print(f"mission files: {out}")


if __name__ == "__main__":
    main(sys.argv[1:])
