#!/usr/bin/env python3
"""From the wall onto the ceiling through an inside corner, with flat-only pads (flat.py).

Side view: wall at z = 0, ceiling at x = 0, robot in x < 0, z > 0, gravity along -x.
Points are complex numbers x + i z; a joint rotation by a (about +y) multiplies by exp(-i a).

Body path ("ladder against the corner"): the hips keep a fixed distance ZC from the surface
they are next to. The body climbs until the front hips are ZC under the ceiling, then turns
90 degrees with the hind hips sliding up the wall and the front hips sliding out along the
ceiling, then walks away under the ceiling.

Feet: a planted foot stays where it is while the body moves; the soft ankle takes up the
angle, up to ANKLE_MAX. When a leg pair is about to run out of reach or ankle travel, it
steps (left leg, then right leg) to a place where the pad lands FLAT. From a given hip
there are at most two such places per surface, so the plan is a short greedy search.

    ../.venv/bin/python corner.py           # plan, run in MuJoCo, then walk on under the ceiling
"""
import cmath
import math

import numpy as np

from env import FEET, STAND
from flat import FlatEnv, FlatGait

T = complex(0.04, -0.07)             # thigh: hip -> knee at hip angle 0 (electroclimb.xml)
S = complex(-0.03, -0.06)            # shin: knee -> foot at knee angle 0
A0 = math.radians(34.4)              # hip + knee that lays the pad flat when the body is parallel
L1, L2 = abs(T), abs(S)
BODY = 0.2                           # front hip to hind hip
PAD_OFF = 0.007                      # foot origin above the surface when the pad lies flat
WALL, CEIL = 0.0, -math.pi / 2       # world angle of a flat pad's leg on each surface
SHIN = {WALL: cmath.exp(-1j * A0) * S, CEIL: 1j * cmath.exp(-1j * A0) * S}
NORMAL = {WALL: 1j, CEIL: -1.0}      # out of the surface, into the room
ZC = abs((cmath.exp(-1j * STAND[1]) * (T + cmath.exp(-1j * STAND[2]) * S)).imag) + PAD_OFF  # hip height when standing

LIMITS_SIM = dict(hip=(-90, 67), knee=(0, 132))       # what env.py's action range reaches (joints: +-90, 0..150)
LIMITS_CAD = dict(hip=(-90, 60), knee=(0, 80))        # hardware/robot/cad: checked free of collisions there
ANKLE_MAX = math.radians(35)         # planned ankle travel (the joint allows 45)
EDGE = 0.04                          # pad centre to the corner line (pads are 60 mm long)
PAD_GAP = 0.07                       # centre distance of two pads on one surface
APPROACH, AWAY = 0.10, 0.12          # m of straight path before and after the turn


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


REACH_UP, REACH_FADE = 0.024, 0.45  # m, turn fraction: see body()


def body(u):
    """Hind hip, front hip and body pitch at path position u (m, m, turn 0..1, m).

    At the start of the turn the front hips hang REACH_UP lower than ZC under the ceiling.
    From there the front legs reach up and their pads land flat on the ceiling while the
    body is still parallel to the wall; from ZC the hip joint would need more than its 90 deg.
    """
    if u < 0:
        ph = complex(-ZC - REACH_UP - BODY + u, ZC)
        return ph, ph + BODY, 0.0
    if u <= 1:
        a = u * math.pi / 2
        d = ZC + REACH_UP * max(0.0, 1 - u / REACH_FADE)
        return complex(-d - BODY * math.cos(a), ZC), complex(-d, ZC + BODY * math.sin(a)), -a
    ph = complex(-ZC, ZC + (u - 1))
    return ph, ph + BODY * 1j, -math.pi / 2


U0, U1 = -APPROACH, 1 + AWAY


def ik(hip, theta, foot, limits, near=None):
    """Hip and knee angles that put the foot origin at `foot`. None if out of reach or limits."""
    p = cmath.exp(1j * theta) * (foot - hip)
    c = (abs(p) ** 2 - L1 ** 2 - L2 ** 2) / (2 * L1 * L2)
    if abs(c) > 1:
        return None
    sols = []
    for sgn in (1, -1):
        k = wrap(-cmath.phase(T * S.conjugate()) + sgn * math.acos(c))
        h = wrap(-cmath.phase(p / (T + cmath.exp(-1j * k) * S)))
        if (math.radians(limits["hip"][0]) <= h <= math.radians(limits["hip"][1])
                and math.radians(limits["knee"][0]) <= k <= math.radians(limits["knee"][1])):
            sols.append((h, k))
    if not sols:
        return None
    return min(sols, key=lambda q: 0 if near is None else abs(q[0] - near[0]) + abs(q[1] - near[1]))


def ankle(q, theta, surface):
    """How far the ankle is turned when the pad lies flat on `surface`."""
    return wrap(surface + A0 - theta - q[0] - q[1])


def leg_ok(hip, theta, surface, foot, limits):
    q = ik(hip, theta, foot, limits)
    if q is None:
        return None
    # the other IK branch may be the one inside the ankle travel
    best = min((s for s in (ik(hip, theta, foot, limits, near=(-9, 9)), ik(hip, theta, foot, limits, near=(9, -9)))
                if s), key=lambda s: abs(ankle(s, theta, surface)))
    return best if abs(ankle(best, theta, surface)) <= ANKLE_MAX else None


def along(surface, foot):
    return -foot.real if surface == WALL else foot.imag


def flat_places(hip, theta, limits):
    """Places where a foot from this hip lands with the pad flat: (surface, foot point, angles)."""
    out = []
    for surface in (WALL, CEIL):
        centre = hip + SHIN[surface]                    # the knee circle, moved to the foot
        off = (PAD_OFF - centre.imag) if surface == WALL else (-PAD_OFF - centre.real)
        if abs(off) >= L1:
            continue
        r = math.sqrt(L1 ** 2 - off ** 2)
        for sgn in (-1, 1):
            foot = complex(centre.real + sgn * r, PAD_OFF) if surface == WALL else complex(-PAD_OFF, centre.imag + sgn * r)
            q = leg_ok(hip, theta, surface, foot, limits)
            if q and abs(ankle(q, theta, surface)) < math.radians(1) and along(surface, foot) >= EDGE:
                out.append((surface, foot, q))
    return out


def reach(u, pair, surface, foot, limits, du=0.005):
    """How far along the path this foot can stay planted."""
    while u < U1:
        ph, pf, theta = body(min(u + du, U1))
        if leg_ok(pf if pair == "f" else ph, theta, surface, foot, limits) is None:
            break
        u += du
    return u


def plan(limits=LIMITS_SIM, du=0.005):
    """Returns the list of steps [(u, pair, surface, foot point)], or None with the reason."""
    ph, pf, theta = body(U0)
    feet = {}
    for pair, hip in (("f", pf), ("h", ph)):
        feet[pair] = min((p for p in flat_places(hip, theta, limits) if p[0] == WALL), key=lambda p: abs(p[2][0] - STAND[1]))[:2]
    steps, u = [("start", "f", *feet["f"]), ("start", "h", *feet["h"])], U0
    while u < U1 - 1e-9:
        nxt = min(u + du, U1)
        ph, pf, theta = body(nxt)
        stuck = [p for p, hip in (("f", pf), ("h", ph)) if leg_ok(hip, theta, *feet[p], limits) is None]
        if not stuck:
            u = nxt
            continue
        ph, pf, theta = body(u)
        moved = False
        for pair in stuck:
            other = feet["h" if pair == "f" else "f"]
            options = []
            for surface, foot, _ in flat_places(pf if pair == "f" else ph, theta, limits):
                if surface == other[0]:                 # front pad stays ahead of the hind pad
                    gap = (along(surface, foot) - along(*other)) * (1 if surface == CEIL else -1)
                    if gap * (1 if pair == "f" else -1) < PAD_GAP:
                        continue
                if pair == "h" and surface == CEIL and other[0] == WALL:
                    continue
                options.append((reach(u, pair, surface, foot, limits), surface, foot))
            options = [o for o in options if o[0] > u + du / 2]
            if options:
                far, surface, foot = max(options, key=lambda o: o[0])
                feet[pair] = (surface, foot)
                steps.append((u, pair, surface, foot))
                moved = True
        if not moved:
            return None, f"stuck at u = {u:.3f} (body turned {math.degrees(-theta):.0f} deg): {stuck} cannot step"
    ph, pf, theta = body(U1)                            # finish in the standing pose under the ceiling
    for pair, hip in (("f", pf), ("h", ph)):
        surface, foot, _ = min((p for p in flat_places(hip, theta, limits) if p[0] == CEIL), key=lambda p: abs(p[2][0] - STAND[1]))
        steps.append((U1, pair, surface, foot))
    return steps, "ok"


def poses(steps, limits, du=0.01):
    """Every (pair, hip, knee) in degrees that the planted legs pass through, sampled along the path."""
    out, feet, i = [], {}, 0
    us = [U0 + k * du for k in range(int((U1 - U0) / du) + 1)] + [U1]
    for u in us:
        while i < len(steps) and (steps[i][0] == "start" or steps[i][0] <= u):
            feet[steps[i][1]] = steps[i][2:]
            i += 1
        ph, pf, theta = body(u)
        for pair, hip in (("f", pf), ("h", ph)):
            q = leg_ok(hip, theta, *feet[pair], limits)
            if q:
                out.append((pair, round(math.degrees(q[0])), round(math.degrees(q[1]))))
    return sorted(set(out))


def describe(steps):
    name = {WALL: "wall", CEIL: "ceiling"}
    return [f"{'front' if pair == 'f' else 'hind'} feet -> {name[surface]}, {along(surface, foot) * 100:4.1f} cm from the corner"
            f"  (body turned {math.degrees(-body(u)[2]):3.0f} deg)" if u != "start" else
            f"{'front' if pair == 'f' else 'hind'} feet start on the wall, {along(surface, foot) * 100:4.1f} cm below the corner"
            for u, pair, surface, foot in steps]


# ---------------------------------------------------------------- run it in MuJoCo

MOVE_V = 0.02                        # m/s along the straight parts
TURN_S = 10.0                        # s for the 90 degree turn
LIFT = 0.02                          # m, swing clearance
T_RELEASE, T_LIFT, T_MOVE, T_SETTLE, T_STICK, T_PULL = 0.15, 0.25, 0.6, 0.4, 0.25, 0.3
PRESS = 0.03                         # m, how far a foot may reach past the plan to find the surface
# With the front pads on the ceiling and the hind pads on the wall, the weight hangs further
# from the wall than the front pads are, so they would have to pull with more than the
# weight. The front legs therefore pull outwards, away from the wall, by this much (m of
# commanded foot position, about 2.5 N per mm): the hind pads are pressed onto the wall and
# the pull on the ceiling pads drops by about four times that force.
SQUEEZE = 0.002
MAX_STEP = math.radians(5)           # per 20 ms control step = 250 deg/s (STS3215: about 270 deg/s)
MAX_SLIDE = 0.02                     # m, how far a landing point may slide to get the pad parallel
OFF_PATH = 0.04                      # m, body further than this from its planned place = fallen


def run(limits=LIMITS_SIM, grip=1.0, after_cycles=3, on_step=None, verbose=False):
    """Returns (ok, reason, info). ok = all four pads grip the ceiling at the end."""
    steps, why = plan(limits)
    if steps is None:
        return False, "no plan: " + why, {}
    probe = FlatEnv(tilt_deg=90.0)
    probe.reset()
    hip_x = probe.data.xpos[probe.model.body("hl_thigh").id][0]
    ceiling_x = hip_x - body(U0)[0].real                # the plan's x = 0
    env = FlatEnv(tilt_deg=90.0, ceiling_x=ceiling_x, grip=grip)
    env.reset()
    m, d = env.model, env.data
    half = (env.jnt_hi - env.jnt_lo) / 2
    legs = {"f": ("fl", "fr"), "h": ("hl", "hr")}
    planted = {f: None for f in FEET}                   # (surface, foot point)
    q = {f: (STAND[1], STAND[2]) for f in FEET}
    for u, pair, surface, foot in steps[:2]:
        for f in legs[pair]:
            planted[f] = (surface, foot)
    info = dict(torque=0.0, min_grip=4, time=0.0, steps=len(steps) - 2, off_path=0.0)
    n = [0]

    def tick(u, volts, target):
        """One control step: `target[f]` overrides the planted foot point of leg f."""
        ph, pf, theta = body(u)
        a = np.concatenate([np.zeros(12), volts])
        for i, f in enumerate(FEET):
            hip = pf if f[0] == "f" else ph
            foot = target.get(f, planted[f][1])
            if f not in target and planted[f][0] == CEIL and f[0] == "f" and WALL in (planted["hl"][0], planted["hr"][0]):
                foot += SQUEEZE * 1j                    # see SQUEEZE
            sol = ik(hip, theta, foot, limits, near=q[f])
            if sol is None:                             # out of reach: aim at the nearest reachable point
                v = foot - hip
                sol = ik(hip, theta, hip + v / abs(v) * min(max(abs(v), abs(L1 - L2) + 1e-3), L1 + L2 - 1e-3), limits, near=q[f])
            if sol is not None:                         # no faster than the servo turns
                q[f] = tuple(old + max(-MAX_STEP, min(MAX_STEP, new - old)) for old, new in zip(q[f], sol))
            a[3 * i + 1] = (q[f][0] - STAND[1]) / half[3 * i + 1]
            a[3 * i + 2] = (q[f][1] - STAND[2]) / half[3 * i + 2]
        touching = env._foot_contacts()
        env.step(a)
        n[0] += 1
        info["torque"] = max(info["torque"], float(np.abs(d.actuator_force[:12]).max()))
        info["min_grip"] = min(info["min_grip"], int(env.gripping.sum()))
        x, _, z = d.xpos[env.torso]
        info["off_path"] = abs(complex(x - ceiling_x, z) - (ph + pf) / 2)
        if on_step:
            on_step(env, n[0])
        return touching

    dt = m.opt.timestep * env.frame_skip
    ones = np.ones(4)

    def fail(reason):
        info["time"] = n[0] * dt
        return False, reason, info

    def move(u0, u1):
        seconds = TURN_S * (min(u1, 1) - max(u0, 0)) if u1 > 0 and u0 < 1 else 0.0
        seconds += (max(0, min(u1, 0) - u0) + max(0, u1 - max(u0, 1))) / MOVE_V
        k = max(1, int(seconds / dt))
        for j in range(1, k + 1):
            tick(u0 + (u1 - u0) * j / k, ones, {})
            if info["off_path"] > OFF_PATH:
                return False
        return True

    def step_leg(u, f, surface, foot):
        i = FEET.index(f)
        old_surface, old = planted[f]
        for _ in range(100):                            # wait until the other three lie flat
            touching = tick(u, ones, {})
            if all(touching[j] for j in range(4) if j != i):
                break
        else:
            return "the other three pads never all lay flat"
        volts = ones.copy()
        for j in range(int(T_RELEASE / dt)):
            volts[i] = 1 - (j + 1) / int(T_RELEASE / dt)
            tick(u, volts, {})
        volts[i] = 0
        up, pre = old + LIFT * NORMAL[old_surface], foot + LIFT * NORMAL[surface]
        for j in range(int(T_LIFT / dt)):
            s = (j + 1) / int(T_LIFT / dt)
            tick(u, volts, {f: old + s * (up - old)})
        for j in range(int(T_MOVE / dt)):
            s = 0.5 - 0.5 * math.cos(math.pi * (j + 1) / int(T_MOVE / dt))
            tick(u, volts, {f: up + s * (pre - up)})
            if info["off_path"] > OFF_PATH:
                return "the body left its path during the swing"
        for _ in range(int(T_SETTLE / dt)):             # let the leg arrive (the joints are speed-limited)
            tick(u, volts, {f: pre})
        reach_in, seated, slid = -LIFT, 0, 0.0
        tangent = 1.0 if surface == WALL else 1j
        hip_of = lambda: (body(u)[1] if f[0] == "f" else body(u)[0])
        while reach_in < PRESS:                         # come down until the pad lies flat on the surface
            # The body is never exactly where the plan says, so the pad arrives a few degrees
            # off. Slide the landing point along the surface until the pad is parallel to it
            # (pad angle from the IMU and the joint angles; here read from the simulator).
            z = d.xmat[env.foot_body[i]].reshape(3, 3)[:, 2]
            err = wrap(math.atan2(z[0], z[2]) - surface)
            if not env.raw_touch[i]:
                ahead, behind = (ik(hip_of(), body(u)[2], foot + sgn * 0.001 * tangent - reach_in * NORMAL[surface], limits, near=q[f])
                                 for sgn in (1, -1))
                if ahead and behind and abs(sum(ahead) - sum(behind)) > 1e-4:
                    ds = -err / ((sum(ahead) - sum(behind)) / 0.002)
                    ds = max(-0.0015, min(0.0015, ds))
                    if abs(slid + ds) <= MAX_SLIDE:
                        slid += ds
                        foot += ds * tangent
            reach_in += 0.002 if reach_in < -0.004 else 0.0007
            seated = seated + 1 if tick(u, volts, {f: foot - reach_in * NORMAL[surface]})[i] else 0
            if seated >= 4:                             # flat for four control steps in a row
                break
        else:
            return f"pad tilted {env.pad_tilt[i]:.0f} deg at landing, never flat"
        for j in range(int(T_STICK / dt)):
            volts[i] = (j + 1) / int(T_STICK / dt)
            tick(u, volts, {f: foot - reach_in * NORMAL[surface]})
        # The body hangs a little away from its planned place, so the foot reached further
        # than planned. Now that it grips, pull back to the planned point: this lifts the
        # body back onto its path instead of letting the sag add up step after step.
        for j in range(int(T_PULL / dt)):
            tick(u, volts, {f: foot - reach_in * (1 - (j + 1) / int(T_PULL / dt)) * NORMAL[surface]})
        planted[f] = (surface, foot)
        return None

    u = U0
    for u_step, pair, surface, foot in steps[2:]:
        if not move(u, u_step):
            return fail(f"the body left its path while moving (turned {math.degrees(-body(u_step)[2]):.0f} deg)")
        u = u_step
        for f in legs[pair]:
            err = step_leg(u, f, surface, foot)
            if verbose:
                print(f"  t {n[0]*dt:5.1f} s  {f} -> {'ceiling' if surface == CEIL else 'wall'} "
                      f"{along(surface, foot)*100:4.1f} cm  body {math.degrees(-body(u)[2]):3.0f} deg  {err or 'ok'}")
            if err:
                return fail(f"{f}: {err} (body turned {math.degrees(-body(u)[2]):.0f} deg)")
    for _ in range(25):
        tick(u, ones, {})
    info["corner_s"] = n[0] * dt
    if not all(planted[f][0] == CEIL for f in FEET) or env.gripping.sum() < 4:
        return fail("did not end with four pads on the ceiling")

    gait = FlatGait(env)                                # walk on under the ceiling
    z0 = d.xpos[env.torso, 2]
    while gait.t < after_cycles * gait.cycle:
        env.step(gait(env._foot_contacts()))
        n[0] += 1
        if on_step:
            on_step(env, n[0])
        if ceiling_x - d.xpos[env.torso, 0] > 0.3:
            return fail("fell off the ceiling after the corner")
    info["time"] = n[0] * dt
    info["ceiling_walk"] = d.xpos[env.torso, 2] - z0
    return True, "ok", info


if __name__ == "__main__":
    for label, limits in (("simulation joint limits", LIMITS_SIM), ("CAD joint limits", LIMITS_CAD)):
        steps, why = plan(limits)
        print(f"--- plan with {label}: hip {limits['hip']}, knee {limits['knee']} deg")
        print("  no plan: " + why if steps is None else "\n".join("  " + line for line in describe(steps)))
    for label, limits in (("simulation joint limits", LIMITS_SIM), ("CAD joint limits", LIMITS_CAD)):
        ok, why, info = run(limits, verbose=label.startswith("CAD"))
        print(f"MuJoCo run, {label}: {'reached the ceiling' if ok else 'FAILED: ' + why}; corner {info.get('corner_s', 0):.0f} s, "
              f"peak torque {info['torque']:.2f} N*m, then walked {info.get('ceiling_walk', 0) * 100:.0f} cm under the ceiling")
        assert ok, "the simulated robot must get from the wall onto the ceiling"
