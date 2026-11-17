#!/usr/bin/env python3
"""Sanity checks for the climbing simulation.

    .venv/bin/python sim/check_physics.py

1. On a vertical wall with all pads energised, the robot must stay put.
2. With pads off, it must slide away.
3. On the floor (tilt 0), it must stand with or without adhesion.
"""
import numpy as np

from env import ClimbEnv

HOLD = np.concatenate([np.zeros(12), np.ones(4)])
OFF = np.concatenate([np.zeros(12), np.zeros(4)])


def run(tilt, action, seconds=3.0):
    env = ClimbEnv(tilt_deg=tilt, randomise=False, fail_prob=0.0, seed=0)
    env.reset()
    info = {"progress": 0.0, "fell": False}
    for _ in range(int(seconds / (env.model.opt.timestep * env.frame_skip))):
        _, _, done, info = env.step(action)
        if done:
            break
    return info


if __name__ == "__main__":
    wall_on = run(90, HOLD)
    wall_off = run(90, OFF)
    floor_off = run(0, OFF)
    print(f"wall, pads on : moved {wall_on['progress']*100:+.1f} cm, fell={wall_on['fell']}")
    print(f"wall, pads off: moved {wall_off['progress']*100:+.1f} cm, fell={wall_off['fell']}")
    print(f"floor, pads off: moved {floor_off['progress']*100:+.1f} cm, fell={floor_off['fell']}")
    assert not wall_on["fell"] and abs(wall_on["progress"]) < 0.02, "should hold on the wall"
    assert wall_off["fell"] or wall_off["progress"] < -0.1, "should slide off without adhesion"
    assert not floor_off["fell"] and abs(floor_off["progress"]) < 0.02, "should stand on the floor"
    print("physics checks pass")
