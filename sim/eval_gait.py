"""Baseline crawl gait on the 40 hard test walls (8 processes).

    ../.venv/bin/python eval_gait.py '{}' '{"DROP_LEVEL": 0.0}'     # split pads vs one-piece pads

Each argument is a JSON dict of settings: crawl_gait module constants (e.g. "STRIDE": 0.05)
DROP_LEVEL (pad voltage left during a drop-out, 0.5 = split pads) and lower (m, body nearer the wall).
"""
import sys
from multiprocessing import Pool
import numpy as np
import crawl_gait
from rl_env import ResidualClimbEnv, HARD_WALLS, ACT_DIM, SUCCESS_CM, LOWER_M, PRESS_M

def run(args):
    k, cfg = args
    for name, v in cfg.items():
        setattr(crawl_gait, name, v)
    env = ResidualClimbEnv(tilt_range=(90, 90), episode_s=20, seed=5000 + k, use_policy=False, **HARD_WALLS)
    env.env.adh_gain0 = env.env.adh_gain0 * cfg.get("PAD_FORCE", 1.0)   # bigger pads (or a lighter robot)
    env.reset()
    env.env.drop_level = cfg.get("DROP_LEVEL", 0.5)
    act = np.zeros(ACT_DIM)
    act[13] = cfg.get("lower", 0.0) / LOWER_M          # body nearer the wall (m)
    act[14] = cfg.get("press", 0.0) / PRESS_M          # stance feet press while one swings (m)
    env.use_policy = True                              # so the fixed action reaches the gait
    done = trunc = False
    while not (done or trunc):
        _, _, done, trunc, info = env.step(act)
    return info["progress"] * 100, info["fell"]

def score(cfg, n=40):
    with Pool(8) as p:
        r = p.map(run, [(k, cfg) for k in range(n)])
    d = np.array([x for x, _ in r]); f = np.array([y for _, y in r])
    win = (~f) & (d >= SUCCESS_CM)
    return win.mean(), f.mean(), d.mean(), [k for k in range(n) if not win[k]]

if __name__ == "__main__":
    import json
    for cfg in map(json.loads, sys.argv[1:]):
        s, f, d, bad = score(cfg)
        print(f"{json.dumps(cfg):55s} success {s:4.0%}  falls {f:4.0%}  climb {d:5.1f} cm  failed {bad}", flush=True)
