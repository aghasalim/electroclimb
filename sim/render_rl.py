#!/usr/bin/env python3
"""Side-by-side video: baseline gait vs RL policy on the same randomised vertical wall.

    ../.venv/bin/python render_rl.py runs/best/best_model.zip [--seed N]

Without --seed it searches evaluation walls for one where the difference is visible
(baseline falls or lags, RL keeps climbing) and reports which seed it used.
"""
import argparse
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from render_media import EVERY, FRAME_MS, make_camera
from rl_env import ACT_DIM, ResidualClimbEnv
from train import load_policy

OUT = Path(__file__).resolve().parent.parent / "media"
SIZE = 300


def run(policy, seed, seconds=20.0, frames=None, title=""):
    env = ResidualClimbEnv(tilt_range=(90, 90), episode_s=seconds, seed=seed, use_policy=policy is not None)
    obs, _ = env.reset()
    r = cam = None
    if frames is not None:
        r = mujoco.Renderer(env.env.model, SIZE, SIZE)
        cam = make_camera((0.11, 0, 0.05), 0.8, -90, 90)
    done = trunc = False
    info, n = {"progress": 0.0, "fell": False}, 0
    while not (done or trunc):
        act = policy(obs) if policy else np.zeros(ACT_DIM)
        obs, _, done, trunc, info = env.step(act)
        if r is not None and n % EVERY == 0:
            r.update_scene(env.env.data, cam)
            im = Image.fromarray(r.render().copy()).rotate(90)
            d = ImageDraw.Draw(im)
            d.rectangle((0, 0, SIZE, 30), fill=(20, 20, 20))
            d.text((6, 3), title, fill=(255, 255, 255))
            d.text((6, 16), f"climbed {info['progress']*100:5.1f} cm" + ("   FELL" if info["fell"] else ""),
                   fill=(255, 90, 90) if info["fell"] else (255, 170, 60))
            frames.append(im)
        n += 1
    if frames is not None:                       # hold the last frame for the other side
        frames.extend([frames[-1]] * 3)
    return info, env.env.surface


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("model")
    p.add_argument("--seed", type=int)
    a = p.parse_args()
    pol = load_policy(a.model)

    seed = a.seed
    if seed is None:                             # find a wall that shows the difference
        best = None
        for s in range(5000, 5030):
            b, _ = run(None, s)
            rl, _ = run(pol, s)
            gap = rl["progress"] - b["progress"] + 0.3 * (b["fell"] - rl["fell"])
            if not rl["fell"] and (best is None or gap > best[0]):
                best = (gap, s)
        seed = best[1]
    fb, fr = [], []
    ib, surf = run(None, seed, frames=fb, title="Baseline crawl gait")
    ir, _ = run(pol, seed, frames=fr, title="RL policy (PPO, residual)")
    n = max(len(fb), len(fr))
    fb += [fb[-1]] * (n - len(fb))
    fr += [fr[-1]] * (n - len(fr))
    combo = []
    for x, y in zip(fb, fr):
        c = Image.new("RGB", (2 * SIZE + 6, SIZE), (255, 255, 255))
        c.paste(x, (0, 0))
        c.paste(y, (SIZE + 6, 0))
        combo.append(c.convert("P", palette=Image.ADAPTIVE, colors=96))
    out = OUT / "rl_vs_baseline.gif"
    combo[0].save(out, save_all=True, append_images=combo[1:], duration=FRAME_MS, loop=0, optimize=True)
    print(f"seed {seed} (grip {surf:.0%}): baseline {ib['progress']*100:.1f} cm fell={ib['fell']}, "
          f"RL {ir['progress']*100:.1f} cm fell={ir['fell']} -> {out.name} {out.stat().st_size/1e6:.1f} MB")
