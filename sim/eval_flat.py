"""The 40 hard test walls again, with pads that only grip when they lie flat (flat.py).

Same walls as eval_gait.py: seeds 5000..5039, grip 55-100 %, humidity 80-100 %, random pad
drop-outs with split pads, vertical wall, 20 s. Success = no fall and at least 10 cm climbed.
The gait is the same crawl gait; only the ankle and the grip rule differ (flat.py).
Waiting for flat contact slows the gait, so the table also runs it with a 12 % faster clock.

    ../.venv/bin/python eval_flat.py                 # table: old gait and flat gait, several pad forces
"""
from multiprocessing import Pool

import numpy as np

from crawl_gait import CrawlGait
from flat import FlatEnv
from rl_env import HARD_WALLS, SUCCESS_CM

FAIL_PROB = 3e-3          # as in rl_env.ResidualClimbEnv


def run(args):
    k, pad_force, speed = args
    env = FlatEnv(tilt_deg=90.0, grip=pad_force, seed=5000 + k, randomise=True, fail_prob=FAIL_PROB,
                  episode_s=20.0, **HARD_WALLS)
    env.reset()
    gait, info = CrawlGait(env), {}
    gait.speed = speed                                # gait clock rate
    for _ in range(env.max_steps):
        _, _, done, info = env.step(gait(env._foot_contacts()))
        if done:
            break
    return info["progress"] * 100, info["fell"], env.surface * env.humidity


def score(pad_force=1.0, speed=1.0, n=40):
    with Pool(8) as p:
        r = p.map(run, [(k, pad_force, speed) for k in range(n)])
    d, f, g = (np.array(x) for x in zip(*r))
    f = f.astype(bool)
    win = ~f & (d >= SUCCESS_CM)
    return win.mean(), f.mean(), d.mean(), d[~f].mean(), [k for k in range(n) if not win[k]], g, f


if __name__ == "__main__":
    for speed in (1.0, 1.12):
        for pf in (1.0, 1.1, 1.2, 1.3, 1.5):
            s, f, d, d_up, bad, grip, fell = score(pf, speed)
            print(f"flat-only pads, clock x{speed:.2f}, pad force x{pf:.1f}: success {s:4.0%}  falls {f:4.0%}  climbed {d:5.1f} cm "
                  f"(walls without a fall: {d_up:4.1f} cm)  failed walls {bad}", flush=True)
            if pf == 1.0 and fell.any():
                print(f"   grip of the walls: no fall {grip[~fell].min():.2f}..{grip[~fell].max():.2f}, "
                      f"fall {grip[fell].min():.2f}..{grip[fell].max():.2f}")
