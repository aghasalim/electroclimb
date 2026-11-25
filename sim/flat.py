#!/usr/bin/env python3
"""Honest pad physics: a pad only grips while it lies FLAT on the surface.

MuJoCo's adhesion actuator pulls with full force as soon as any part of the pad touches.
With the original foot (ankle spring 6 N*m/rad) the crawl gait left the pads standing on
an edge, tilted about 30 degrees, for most of every step, and they still "gripped". A real
electroadhesive pad needs area contact, so that result was too kind.

FlatEnv removes the gift and changes the foot so that the pads can lie flat:

  * grip is scaled by flatness: full below 3 degrees of tilt, zero above 8 degrees;
  * the gait only counts a pad as placed when it touches AND lies flat;
  * the ankle is a soft pivot: 0.1 N*m/rad, +-45 degrees, with the inertia of a pad instead of
    the model's default servo-rotor inertia (which made the free pad swing for seconds);
  * an optional ceiling plane, for the wall-to-ceiling corner (corner.py).

With that ankle the unchanged crawl gait climbs: the pad touches with an edge and lies down
under a push of well under 1 N. It works up to 0.15 N*m/rad (wall and ceiling), on the wall
only up to 0.25, and fails from 0.3 on.

TrotGait steps two diagonal legs at the same time: only two pads hold the robot.

    ../.venv/bin/python flat.py         # test table: crawl and trot, wall and ceiling
"""
import mujoco
import numpy as np

from crawl_gait import MAX_PRESS, PHASES, smooth
from env import FEET, STAND, XML, ClimbEnv
from steer import SteerGait, pose

ANKLE_K, ANKLE_RANGE, ANKLE_DAMP = 0.1, 45.0, 0.005   # N*m/rad, deg, N*m*s/rad
ANKLE_INERTIA = 1e-4     # kg*m^2. The model's default (0.005, a servo rotor) made the free pad swing for seconds
FLAT_FULL, FLAT_ZERO, FLAT_OK = 3.0, 8.0, 5.0          # deg: full grip, no grip, "placed"
# trot parameters, found by tune_trot.py (cross-entropy search over 20 s ceiling walks)
TROT = dict(dx=0.03697, sway=1.304, kp=0.04284, ki=0.05118, lift=0.02418, speed=1.019)


def build_model(ceiling_x=None):
    spec = mujoco.MjSpec.from_file(str(XML))
    if ceiling_x is not None:                           # a plane facing -x: up the wall is +x
        spec.worldbody.add_geom(name="ceiling", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[3, 3, 0.1],
                                pos=[ceiling_x, 0, 0], quat=[np.cos(-np.pi / 4), 0, np.sin(-np.pi / 4), 0],
                                material="concrete")
    m = spec.compile()
    for f in FEET:
        for ax in "xy":
            j = m.joint(f"{f}_ank_{ax}").id
            m.jnt_stiffness[j] = ANKLE_K
            m.jnt_range[j] = np.radians([-ANKLE_RANGE, ANKLE_RANGE])
            m.dof_damping[m.jnt_dofadr[j]] = ANKLE_DAMP
            m.dof_armature[m.jnt_dofadr[j]] = ANKLE_INERTIA
    return m


class FlatEnv(ClimbEnv):
    def __init__(self, tilt_deg=90.0, ceiling_x=None, grip=1.0, **kw):
        kw.setdefault("randomise", False)
        kw.setdefault("fail_prob", 0.0)
        kw.setdefault("episode_s", 1e6)
        super().__init__(tilt_deg=tilt_deg, model=build_model(ceiling_x), **kw)
        self.adh_gain0 = self.adh_gain0 * grip          # grip < 1: weaker pads, to find the margin
        self.surfaces = {self.wall} | ({self.model.geom("ceiling").id} if ceiling_x is not None else set())
        self.foot_body = [self.model.body(f"{f}_foot").id for f in FEET]
        self.pad_tilt = np.zeros(4)
        self.raw_touch = np.zeros(4)

    def _sense(self):
        """Per pad: does it touch a surface, and how far is it tilted from that surface (deg)."""
        d = self.data
        self.raw_touch[:] = 0
        self.pad_tilt[:] = 90.0
        for c in d.contact[: d.ncon]:
            for g, other in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                if other in self.surfaces and g in self.pad_geoms:
                    i = self.pad_geoms[g]
                    z = d.xmat[self.foot_body[i]].reshape(3, 3)[:, 2]
                    self.raw_touch[i] = 1.0
                    self.pad_tilt[i] = np.degrees(np.arccos(min(1.0, abs(float(np.dot(c.frame[:3], z))))))

    def _foot_contacts(self):
        self._sense()
        return self.raw_touch * (self.pad_tilt < FLAT_OK)

    def flatness(self):
        return np.clip((FLAT_ZERO - self.pad_tilt) / (FLAT_ZERO - FLAT_FULL), 0, 1) * self.raw_touch

    def step(self, action):
        a = np.array(action, dtype=float)
        self._sense()
        a[12:] *= self.flatness()
        self.gripping = a[12:] > 0.5
        return super().step(a)


FlatGait = SteerGait      # the steering crawl needs no change: with the soft ankle its pads lie down by themselves


class TrotGait(SteerGait):
    """Two diagonal legs step together: only two pads hold the robot.

    Two pads on soft ankles are a hinge: the body can swing freely about the line between
    them. On a ceiling it hangs like a pendulum, level only while its centre of mass is
    under that line. So while one pair is in the air, the standing pair pulls the body
    forward AND moves it sideways (`sway`), which keeps the centre on the line.
    Each pair lands at its home point, where the pad is flat.
    """
    PAIRS = (("fl", "hr"), ("fr", "hl"))

    def __init__(self, env, dx=None, dy=0.0, yaw=0.0, **params):
        self.p = {**TROT, **params}
        super().__init__(env, self.p["dx"] if dx is None else dx, dy, yaw)
        self.cycle = 2 * self.step_s
        self.speed = self.p["speed"]
        self.sway = self.p["sway"]
        self.lean_i = 0.0
        self.o = {f: np.zeros(3) for f in FEET}        # foot offset from home (body frame)
        self.lift_from = {f: np.zeros(3) for f in FEET}
        hx, hy = np.abs(self.ik.home["fl"][:2])
        self.sway_gain = hy / hx                        # sideways shift per metre of sweep that keeps
                                                        # the body centre on the standing diagonal

    def _targets_to_action(self, volts, swing=None):
        a = np.concatenate([np.zeros(12), volts])
        for i, f in enumerate(FEET):
            q = self._solve3(f, self.ik.home[f] + self.o[f])
            a[3 * i: 3 * i + 3] = (q - STAND[3 * i: 3 * i + 3]) / self.half[3 * i: 3 * i + 3]
        return a

    def __call__(self, touching):
        t = self.t % self.cycle
        k = int(t // self.step_s)
        tl = t - k * self.step_s
        swing, stance = self.PAIRS[k], self.PAIRS[1 - k]
        volts = np.ones(4)
        # never release a pair unless both pads of the other pair lie flat
        if tl < PHASES[0][1]:
            missing = [f for f in stance if not touching[FEET.index(f)]]
            if missing:
                for f in missing:
                    self.o[f][2] = max(self.o[f][2] - 0.002, -MAX_PRESS)
                return self._targets_to_action(volts)
            for f in swing:
                self.lift_from[f] = self.o[f].copy()
        self.t += self.dt * self.speed

        t0, air = PHASES[0][1], PHASES[1][1] + PHASES[2][1]
        s_air = smooth((tl - t0) / air)                 # 0 -> 1 while the swing pair is in the air
        side = 1.0 if "fr" in stance else -1.0
        # balance feedback: how far the body leans about the standing diagonal (from the IMU)
        n = self.env.data.xmat[self.env.torso].reshape(3, 3)[2, :2]
        perp = np.array([self.sway_gain, side]) / np.hypot(self.sway_gain, 1.0)
        lean = float(n @ perp)
        self.lean_i = np.clip(self.lean_i + lean * self.dt, -0.05, 0.05)
        fix = (self.p["kp"] * lean + self.p["ki"] * self.lean_i) * perp
        for f in stance:                                # pull the body forward, sway to stay balanced
            step = self.landing(f)
            self.o[f][:2] = -s_air * step + fix
            self.o[f][1] += side * self.sway * self.sway_gain * s_air * step[0]
        start = 0.0
        for name, dur in PHASES:
            if tl >= start:
                s = smooth((tl - start) / dur)
                for f in swing:
                    i = FEET.index(f)
                    volts[i] = {"release": 1 - s, "stick": s}.get(name, 0.0)
                    if name == "lift":
                        self.o[f][2] = max(self.o[f][2], self.p["lift"] * s)
                    elif name == "swing":
                        self.o[f][:2] = self.lift_from[f][:2] * (1 - s)
                    elif name == "place" and not touching[i]:
                        self.o[f][2] = max(self.o[f][2] - 0.002, -MAX_PRESS)
            start += dur
        return self._targets_to_action(volts)


def walk(tilt, plan, gait=FlatGait, grip=1.0, on_step=None, max_s=120.0):
    """Run (dx, dy, yaw, cycles) commands. Returns poses after each, fell, mean pads gripping, peak torque."""
    env = FlatEnv(tilt_deg=tilt, grip=grip)
    env.reset()
    g = gait(env)
    start, out, fell, n, grips, torque = pose(env), [], False, 0, [], 0.0
    for dx, dy, yaw, cycles in plan:
        g.cmd = (g.cmd[0] if dx is None else dx, dy, yaw)
        t_end = g.t + cycles * g.cycle
        while g.t < t_end - 1e-9 and not fell and n * g.dt < max_s:
            env.step(g(env._foot_contacts()))
            fell = not 0.02 < env.data.xpos[env.torso, 2] < 0.35
            grips.append(env.gripping.sum())
            torque = max(torque, np.abs(env.data.actuator_force[:12]).max())
            n += 1
            if on_step:
                on_step(env, n)
        out.append(pose(env) - start)
    done = g.t >= t_end - 1e-9
    return out, fell or not done, float(np.mean(grips)), torque, n * g.dt


def min_grip(tilt, plan, gait):
    """Weakest pads (fraction of 15 N) that still finish the plan, by bisection to 5 %."""
    lo, hi = 0.0, 1.0
    if walk(tilt, plan, gait, grip=hi)[1]:
        return None
    while hi - lo > 0.05:
        mid = (lo + hi) / 2
        lo, hi = (lo, mid) if not walk(tilt, plan, gait, grip=mid)[1] else (mid, hi)
    return hi


TESTS = {
    "forwards":       [(None, 0, 0, 5)],               # None = the gait's own stride
    "backwards":      [(-0.04, 0, 0, 5)],
    "sideways left":  [(0, 0.02, 0, 5)],
    "turn left":      [(0, 0, 0.12, 5)],
}

if __name__ == "__main__":
    results = {}
    for gait in (FlatGait, TrotGait):
        for tilt, name in ((90, "wall"), (180, "ceiling")):
            print(f"--- {gait.__name__} on the {name}, 5 gait cycles each, pads grip only when flat")
            for test, plan in TESTS.items():
                ((x, y, h),), failed, grips, torque, secs = results[gait, tilt, test] = walk(tilt, plan, gait)
                print(f"{test:14s} {x*100:6.1f} cm forward {y*100:6.1f} cm left {h:6.1f} deg in {secs:4.0f} s, "
                      f"failed={failed}, pads gripping {grips:.1f}, peak torque {torque:.2f} N*m")
    assert not results[FlatGait, 90, "forwards"][1] and results[FlatGait, 90, "forwards"][0][0][0] > 0.10
    assert not results[FlatGait, 180, "forwards"][1], "the crawl must walk on the ceiling with flat pads"
    assert not results[TrotGait, 180, "forwards"][1], "the trot must walk on the ceiling on two pads"
    print("flat-pad gaits checked")
