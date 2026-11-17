#!/usr/bin/env python3
"""Hand-written crawl gait: the baseline the RL policy has to beat.

Feet are planned in the body frame and turned into joint angles with inverse kinematics
(solved on a copy of the MuJoCo model, so it always matches electroclimb.xml).

One leg at a time: release pad -> lift -> move up the wall -> come down until the grip
sensor reports contact (on the robot: the pad capacitance jumps) -> re-stick.
After all four legs have stepped, the body pulls itself up on all four stuck pads.

    ../.venv/bin/python crawl_gait.py          # climb test at 0, 45, 90 deg
    ../.venv/bin/python render_media.py        # videos -> media/
"""
import mujoco
import numpy as np

from env import FEET, JOINTS, STAND, ClimbEnv

ORDER = ["hr", "fr", "hl", "fl"]     # classic crawl sequence
STRIDE = 0.04                        # m, how far each foot moves up per step
LIFT = 0.025                         # m, swing height
MAX_PRESS = 0.05                     # m, how far a foot may reach below stance to find the wall
PHASES = [("release", 0.10), ("lift", 0.15), ("swing", 0.25), ("place", 0.25), ("stick", 0.15)]
SHIFT_S = 0.8                        # s, body pull-up


def smooth(t):
    return 0.5 - 0.5 * np.cos(np.pi * np.clip(t, 0, 1))


class LegIK:
    """Hip-flexion + knee IK in the body frame, by Gauss-Newton on a kinematics-only copy."""

    def __init__(self, model):
        self.m = model
        self.d = mujoco.MjData(model)
        self.qadr = {j: model.jnt_qposadr[model.joint(j).id] for j in JOINTS}
        self.foot = {f: model.body(f"{f}_foot").id for f in FEET}
        self.d.qpos[3] = 1.0                      # torso at origin, identity orientation
        for j, q in zip(JOINTS, STAND):
            self.d.qpos[self.qadr[j]] = q
        mujoco.mj_kinematics(model, self.d)
        self.home = {f: self.d.xpos[self.foot[f]].copy() for f in FEET}

    def _fk(self, leg, hip, knee):
        self.d.qpos[self.qadr[f"{leg}_hip"]] = hip
        self.d.qpos[self.qadr[f"{leg}_knee"]] = knee
        mujoco.mj_kinematics(self.m, self.d)
        return self.d.xpos[self.foot[leg]][[0, 2]].copy()

    def solve(self, leg, target_xz, q0):
        """Damped least squares, step-limited and clipped to joint ranges (never diverges)."""
        lo = np.array([self.m.jnt_range[self.m.joint(f"{leg}_{j}").id][0] for j in ("hip", "knee")])
        hi = np.array([self.m.jnt_range[self.m.joint(f"{leg}_{j}").id][1] for j in ("hip", "knee")])
        q = np.clip(np.array(q0, dtype=float), lo, hi)
        for _ in range(20):
            p = self._fk(leg, *q)
            err = target_xz - p
            if np.linalg.norm(err) < 2e-4:
                break
            J = np.zeros((2, 2))
            for k in range(2):
                dq = np.zeros(2)
                dq[k] = 1e-4
                J[:, k] = (self._fk(leg, *(q + dq)) - p) / 1e-4
            step = J.T @ np.linalg.solve(J @ J.T + 1e-5 * np.eye(2), err)
            q = np.clip(q + np.clip(step, -0.15, 0.15), lo, hi)
        return q


class CrawlGait:
    def __init__(self, env):
        self.env = env
        self.ik = LegIK(env.model)
        self.dt = env.model.opt.timestep * env.frame_skip
        self.step_s = sum(p for _, p in PHASES)
        self.cycle = 4 * self.step_s + SHIFT_S
        self.t = 0.0
        self.speed = 1.0                              # gait clock rate (RL can change it)
        self.offset = {f: np.zeros(2) for f in FEET}  # foot x/z offset from home (body frame)
        self.q = {f: STAND[3 * i + 1: 3 * i + 3].copy() for i, f in enumerate(FEET)}
        self.half = (env.jnt_hi - env.jnt_lo) / 2

    def _targets_to_action(self, volts):
        a = np.concatenate([np.zeros(12), volts])
        for i, f in enumerate(FEET):
            target = self.ik.home[f][[0, 2]] + self.offset[f]
            self.q[f] = self.ik.solve(f, target, self.q[f])
            for k, j in enumerate((1, 2)):           # hip, knee
                a[3 * i + j] = (self.q[f][k] - STAND[3 * i + j]) / self.half[3 * i + j]
        return a

    def __call__(self, touching):
        t = self.t % self.cycle
        volts = np.ones(4)

        # Safety rule: never release a foot (or pull the body up) unless every other foot
        # reports contact. Until then, freeze the clock and press the missing feet down.
        k = int(t // self.step_s)
        releasing = t < 4 * self.step_s and (t - k * self.step_s) < PHASES[0][1]
        shifting_start = t >= 4 * self.step_s and t - 4 * self.step_s < self.dt * self.speed
        if releasing or shifting_start:
            others = [i for i in range(4) if not (releasing and FEET[i] == ORDER[k])]
            missing = [i for i in others if not touching[i]]
            if missing:
                for i in missing:
                    f = FEET[i]
                    self.offset[f][1] = max(self.offset[f][1] - 0.002, -MAX_PRESS)
                return self._targets_to_action(volts)
        self.t += self.dt * self.speed

        if t >= 4 * self.step_s:                      # pull the body up: feet slide back to home
            s = smooth((t - 4 * self.step_s) / SHIFT_S)
            for f in FEET:
                if not hasattr(self, "shift_from"):
                    self.shift_from = {g: self.offset[g].copy() for g in FEET}
                self.offset[f] = self.shift_from[f] * (1 - s)
            if s >= 1 and hasattr(self, "shift_from"):
                del self.shift_from
            return self._targets_to_action(volts)

        k = int(t // self.step_s)
        tl = t - k * self.step_s
        leg = ORDER[k]
        i = FEET.index(leg)
        start = 0.0
        for name, dur in PHASES:
            s = smooth((tl - start) / dur) if tl >= start else None
            if s is not None:
                if name == "release":
                    volts[i] = 1 - s
                elif name == "lift":
                    volts[i] = 0
                    self.offset[leg][1] = max(self.offset[leg][1], LIFT * s)
                elif name == "swing":
                    volts[i] = 0
                    self.offset[leg][0] = STRIDE * s
                elif name == "place":
                    volts[i] = 0
                    if not touching[i]:                # keep reaching until the pad touches
                        self.offset[leg][1] = max(self.offset[leg][1] - 0.002, -MAX_PRESS)
                elif name == "stick":
                    volts[i] = s
            start += dur
        return self._targets_to_action(volts)


def climb(tilt, seconds=20.0, on_step=None, seed=0):
    """Run the gait. on_step(env, n) is called every control step (e.g. to grab video frames)."""
    env = ClimbEnv(tilt_deg=tilt, randomise=False, fail_prob=0.0, seed=seed, episode_s=seconds)
    env.reset()
    gait = CrawlGait(env)
    info = {"progress": 0.0, "fell": False}
    for n in range(env.max_steps):
        _, _, done, info = env.step(gait(env._foot_contacts()))
        if on_step:
            on_step(env, n)
        if done:
            break
    return info


if __name__ == "__main__":
    results = {}
    for tilt in (0, 45, 90):
        info = results[tilt] = climb(tilt)
        print(f"tilt {tilt:2d} deg: climbed {info['progress']*100:6.1f} cm in 20 s "
              f"({info['progress'] / 20 * 100:.2f} cm/s), fell={info['fell']}")
    assert not results[90]["fell"] and results[90]["progress"] > 0.10, "gait must climb a vertical wall"
    print("crawl gait climbs")
