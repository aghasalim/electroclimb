#!/usr/bin/env python3
"""Steering for the crawl gait: forwards, backwards, sideways and turning, on any tilt.

The crawl gait (crawl_gait.py) only steps straight ahead. SteerGait keeps its timing, its
contact safety rule and its phases, and only changes where each foot lands:

    foot landing point = home point, turned by `yaw` around the body centre, moved by (dx, dy)

After the four steps the body shift brings every foot back to its home point, so the body
moves by (dx, dy) and turns by `yaw` once per cycle. Sideways motion uses the hip abduction
joints, which the straight gait leaves at zero.

    ../.venv/bin/python steer.py        # test table on the wall and the ceiling
"""
import mujoco
import numpy as np

from crawl_gait import STRIDE, CrawlGait
from env import FEET, JOINTS, STAND, ClimbEnv

LEG_JOINTS = ("abd", "hip", "knee")


class SteerGait(CrawlGait):
    def __init__(self, env, dx=STRIDE, dy=0.0, yaw=0.0):
        super().__init__(env)
        self.cmd = (dx, dy, yaw)                      # m, m, rad per gait cycle (body frame)
        self.q3 = {f: STAND[3 * i: 3 * i + 3].copy() for i, f in enumerate(FEET)}
        m = env.model
        self.lim = {f: m.jnt_range[[m.joint(f"{f}_{j}").id for j in LEG_JOINTS]].T for f in FEET}

    def _fk3(self, leg, q):
        for j, v in zip(LEG_JOINTS, q):
            self.ik.d.qpos[self.ik.qadr[f"{leg}_{j}"]] = v
        mujoco.mj_kinematics(self.ik.m, self.ik.d)
        return self.ik.d.xpos[self.ik.foot[leg]].copy()

    def _solve3(self, leg, target):
        """Same damped least squares as LegIK.solve, with the abduction joint added."""
        lo, hi = self.lim[leg]
        q = np.clip(self.q3[leg], lo, hi)
        for _ in range(20):
            p = self._fk3(leg, q)
            err = target - p
            if np.linalg.norm(err) < 2e-4:
                break
            J = np.zeros((3, 3))
            for k in range(3):
                dq = np.zeros(3)
                dq[k] = 1e-4
                J[:, k] = (self._fk3(leg, q + dq) - p) / 1e-4
            step = J.T @ np.linalg.solve(J @ J.T + 1e-5 * np.eye(3), err)
            q = np.clip(q + np.clip(step, -0.15, 0.15), lo, hi)
        self.q3[leg] = q
        return q

    def landing(self, leg):
        """Where this leg's foot lands, as an x/y offset from its home point (body frame)."""
        dx, dy, yaw = self.cmd
        hx, hy = self.ik.home[leg][:2]
        c, s = np.cos(yaw), np.sin(yaw)
        return np.array([c * hx - s * hy + dx - hx, s * hx + c * hy + dy - hy])

    def _targets_to_action(self, volts, swing=None):
        a = np.concatenate([np.zeros(12), volts])
        for i, f in enumerate(FEET):
            # the base gait moves offset[0] from 0 to STRIDE and back: read it as step progress
            frac = self.offset[f][0] / STRIDE
            target = self.ik.home[f].copy()
            target[:2] += frac * self.landing(f)
            target[2] += self.offset[f][1] + self.lower
            if swing is not None and f != swing:
                target[2] -= self.press
            q = self._solve3(f, target)
            a[3 * i: 3 * i + 3] = (q - STAND[3 * i: 3 * i + 3]) / self.half[3 * i: 3 * i + 3]
        return a


def pose(env):
    """Torso position on the surface (x, y) in m and heading in degrees."""
    x, y, _ = env.data.xpos[env.torso]
    R = env.data.xmat[env.torso].reshape(3, 3)
    return np.array([x, y, np.degrees(np.arctan2(R[1, 0], R[0, 0]))])


def walk(tilt, plan, seed=0, on_step=None):
    """Run a list of (dx, dy, yaw, cycles) commands one after another.

    Returns the pose after each command (relative to the start) and whether the robot fell."""
    env = ClimbEnv(tilt_deg=tilt, randomise=False, fail_prob=0.0, seed=seed, episode_s=1e6)
    env.reset()
    gait = SteerGait(env)
    start, out, fell, n = pose(env), [], False, 0
    for dx, dy, yaw, cycles in plan:
        gait.cmd = (dx, dy, yaw)
        t_end = gait.t + cycles * gait.cycle
        while gait.t < t_end - 1e-9 and not fell and n < 60000:
            env.step(gait(env._foot_contacts()))
            # the env's own fall test assumes climbing in +x, so test the wall distance here
            fell = not 0.02 < env.data.xpos[env.torso, 2] < 0.35
            n += 1
            if on_step:
                on_step(env, n)
        out.append(pose(env) - start)
    return out, fell


TESTS = {                                             # (dx, dy, yaw, cycles)
    "forwards":        [(0.04, 0, 0, 5)],
    "backwards":       [(-0.04, 0, 0, 5)],
    "sideways left":   [(0, 0.02, 0, 5)],
    "sideways right":  [(0, -0.02, 0, 5)],
    "turn left":       [(0, 0, 0.12, 5)],
    "turn right":      [(0, 0, -0.12, 5)],
    "curve":           [(0.03, 0, 0.08, 5)],
}

if __name__ == "__main__":
    ok = True
    for tilt, name in ((90, "wall"), (180, "ceiling")):
        print(f"--- {name} ({tilt} deg), 5 gait cycles each")
        for test, plan in TESTS.items():
            ((x, y, h),), fell = walk(tilt, plan)
            print(f"{test:15s} moved {x*100:6.1f} cm forward, {y*100:6.1f} cm left, turned {h:6.1f} deg, fell={fell}")
            ok &= not fell
    # the straight command must reproduce the plain crawl gait
    (x, _, h), = walk(90, TESTS["forwards"])[0]
    assert x > 0.15 and abs(h) < 5, "steered gait must still climb straight"
    # each command must move the robot the way it says
    assert walk(90, TESTS["sideways left"])[0][0][1] > 0.05
    assert walk(90, TESTS["turn left"])[0][0][2] > 20
    assert ok, "a steering test fell"
    print("steering works")
