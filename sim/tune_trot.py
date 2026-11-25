#!/usr/bin/env python3
"""Learn the two-pad trot's parameters by policy search (cross-entropy method).

The trot has six numbers: stride, sway, two balance-feedback gains, swing height and
clock speed. One trial = walk on the ceiling for 20 s at three grip levels with flat-only
pads; the score is the distance covered, and a walk that ends in a fall counts 10 %.
Each generation samples 32 parameter sets around the current guess, keeps the best 6
and refits the guess to them. The best set found so far is always kept.

    ../.venv/bin/python tune_trot.py            # about 10 minutes on 8 cores
    ../.venv/bin/python tune_trot.py --check    # score the parameters stored in flat.py
"""
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

import flat

NAMES = ["dx", "sway", "kp", "ki", "lift", "speed"]
LO = np.array([0.010, 0.0, -0.3, -3.0, 0.012, 0.6])
HI = np.array([0.045, 2.5, 0.3, 3.0, 0.035, 1.6])
GRIPS = (1.0, 0.9, 0.8)
SECONDS = 20.0


def score(x, tilt=180, grips=GRIPS):
    """Mean distance in m over the grip levels; a walk that ends in a fall counts 10 %."""
    params = dict(zip(NAMES, x))
    total, falls = 0.0, 0
    for grip in grips:
        env = flat.FlatEnv(tilt_deg=tilt, grip=grip)
        env.reset()
        g = flat.TrotGait(env, **params)
        start = flat.pose(env)
        fell, dist = False, 0.0
        for _ in range(int(SECONDS / g.dt)):
            env.step(g(env._foot_contacts()))
            if not 0.02 < env.data.xpos[env.torso, 2] < 0.35:
                fell = True
                break
            dist = (flat.pose(env) - start)[0]
        falls += fell
        total += max(dist, 0.0) * (0.1 if fell else 1.0)
    return total / len(grips), falls


def main(generations=12, pop=32, elite=6, seed=0):
    rng = np.random.default_rng(seed)
    mean = np.array([flat.TROT[n] for n in NAMES])
    std = (HI - LO) / 10
    best_x, best_s = mean, (-1.0, 9)
    with ProcessPoolExecutor(8) as ex:
        for gen in range(generations):
            xs = np.clip(mean + std * rng.standard_normal((pop, len(NAMES))), LO, HI)
            xs[0] = best_x                              # never lose the best set found so far
            res = list(ex.map(score, xs))
            order = np.argsort([-r[0] for r in res])
            if res[order[0]][0] > best_s[0]:
                best_x, best_s = xs[order[0]].copy(), res[order[0]]
            mean = xs[order[:elite]].mean(0)
            std = np.maximum(xs[order[:elite]].std(0), 0.03 * (HI - LO))
            print(f"gen {gen}: best so far {best_s[0]*100:5.1f} cm/20 s mean over grips {GRIPS} ({best_s[1]} falls)  "
                  + " ".join(f"{n}={v:.3f}" for n, v in zip(NAMES, best_x)), flush=True)
    print("TROT = dict(" + ", ".join(f"{n}={v:.4g}" for n, v in zip(NAMES, best_x)) + ")")


if __name__ == "__main__":
    if "--check" in sys.argv:
        x = [flat.TROT[n] for n in NAMES]
        for tilt, name in ((180, "ceiling"), (90, "wall")):
            for grip in (1.0, 0.8, 0.65, 0.5):
                d, falls = score(x, tilt, (grip,))
                print(f"{name:8s} grip {grip:.2f}: " + ("fell" if falls else f"{d*100:5.1f} cm in {SECONDS:.0f} s"))
    else:
        main()
