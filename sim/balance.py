#!/usr/bin/env python3
"""Gravity and balance: how the robot stands on four pads, and on three while one leg steps.

    ../.venv/bin/python balance.py            # tables + self-check
    ../.venv/bin/python balance.py figure     # also writes media/balance.png

Two independent answers to the same question, "how much grip does each stance need?":

1. A rigid-body force model. Each pad is a point that can pull the robot towards the wall
   (at most its adhesion force) or push it away, and carry shear up to mu x its contact
   pressure. Force and moment balance (6 equations) plus those limits is a linear program;
   minimising the grip gives the smallest adhesion that keeps the stance in equilibrium.
2. The MuJoCo simulation: stand in the same stance, lower the grip until the robot lets go.

Frame: the wall is the plane z = 0, +x is up the wall, +z points away from it.
The model ignores the small moments a flat pad and its spring ankle can carry.
"""
import sys
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import linprog

from crawl_gait import LIFT, CrawlGait, smooth
from env import FEET, ClimbEnv, gravity_for_tilt

MU = 0.6            # pad friction (electroclimb.xml)
PAD_FORCE = 15.0    # N per pad at 100 % grip: a placeholder until the hold test (hardware/foot/BUILD.md)
CASES = [None] + FEET
NAMES = {None: "all four down", "fl": "front-left lifted", "fr": "front-right lifted",
         "hl": "hind-left lifted", "hr": "hind-right lifted"}
OUT = Path(__file__).resolve().parent.parent / "media"


# -- 1. force model ----------------------------------------------------------
def min_grip(pads, com, weight, tilt_deg=90.0, mu=MU, pad_force=PAD_FORCE):
    """Smallest grip (1.0 = 15 N per pad) with a static equilibrium, and the pad loads there.

    pads: (n, 2) pad centres on the wall, com: (3,), weight: N.
    Returns grip, pull (n,): what each pad pulls the robot to the wall with (negative = it
    pushes), shear (n, 2). grip = 0 means the stance balances without adhesion.
    """
    n = len(pads)
    G = weight / 9.81 * gravity_for_tilt(tilt_deg)
    # unknowns: pull T_i (n), shear Sx_i, Sy_i (2n), grip g.  Pad force on the robot: (Sx, Sy, -T)
    nv = 3 * n + 1
    A_eq, b_eq = np.zeros((6, nv)), np.zeros(6)
    for i, (x, y) in enumerate(pads):
        A_eq[0, n + 2 * i] = A_eq[1, n + 2 * i + 1] = 1          # sum Fx, Fy
        A_eq[2, i] = -1                                          # sum Fz
        A_eq[3, i], A_eq[4, i] = -y, x                           # moments about x, y (r x F, z_pad = 0)
        A_eq[5, n + 2 * i], A_eq[5, n + 2 * i + 1] = -y, x       # moment about z
    b_eq[:3] = -G
    b_eq[3:] = -np.cross(com, G)
    rows = []
    for i in range(n):
        r = np.zeros(nv)
        r[i], r[-1] = 1, -pad_force                              # contact pressure >= 0: T <= g * A
        rows.append(r)
        for k in range(16):                                      # friction circle as a 16-gon (inside it)
            a = np.pi / 8 * k
            r = np.zeros(nv)
            r[n + 2 * i], r[n + 2 * i + 1] = np.cos(a), np.sin(a)
            r[i], r[-1] = mu * np.cos(np.pi / 16), -mu * np.cos(np.pi / 16) * pad_force
            rows.append(r)
    c = np.zeros(nv)
    c[-1] = 1
    res = linprog(c, A_ub=np.array(rows), b_ub=np.zeros(len(rows)), A_eq=A_eq, b_eq=b_eq,
                  bounds=[(None, None)] * (3 * n) + [(0, None)])
    if not res.success:
        return np.inf, None, None
    return res.x[-1], res.x[:n], res.x[n:3 * n].reshape(n, 2)


# -- 2. the same stances in MuJoCo -------------------------------------------
def stand(lifted=None, grip=1.0, tilt=90.0, seconds=3.0):
    """Stand on the wall, lift one leg the way the gait does, wait. Returns the env."""
    env = ClimbEnv(tilt_deg=tilt, randomise=False, fail_prob=0.0, seed=0, episode_s=60)
    env.reset()
    env.model.actuator_gainprm[env.adh_ids, 0] = env.adh_gain0 * grip
    gait = CrawlGait(env)
    volts = np.ones(4)
    dt = env.model.opt.timestep * env.frame_skip
    env.held = True
    for n in range(int(seconds / dt)):
        if lifted:
            s = smooth(n * dt / 0.5)                             # release and lift over 0.5 s
            volts[FEET.index(lifted)] = 1 - s
            gait.offset[lifted][1] = LIFT * s
        _, _, done, info = env.step(gait._targets_to_action(volts, swing=lifted))
        if done or abs(info["progress"]) > 0.01:
            env.held = False
            break
    return env


def pad_loads(env):
    """Per pad: contact pressure force (N, 0 = about to peel) and shear force (N)."""
    normal, shear = np.zeros(4), np.zeros(4)
    f = np.zeros(6)
    for k in range(env.data.ncon):
        c = env.data.contact[k]
        for g, other in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
            if other == env.wall and g in env.pad_geoms:
                mujoco.mj_contactForce(env.model, env.data, k, f)
                normal[env.pad_geoms[g]] += f[0]
                shear[env.pad_geoms[g]] += np.hypot(f[1], f[2])
    return normal, shear


def geometry(env, lifted=None):
    """Pad centres on the wall (stance pads only), centre of mass, weight."""
    keep = [f for f in FEET if f != lifted]
    pads = np.array([env.data.geom_xpos[env.model.geom(f"{f}_pad").id][:2] for f in keep])
    return keep, pads, env.data.subtree_com[env.torso].copy(), mujoco.mj_getTotalmass(env.model) * 9.81


def sim_min_grip(lifted, tilt=90.0):
    lo, hi = 0.0, 1.0                                            # holds at hi, lets go at lo
    if stand(lifted, 0.0, tilt).held:
        return 0.0
    for _ in range(6):
        mid = (lo + hi) / 2
        lo, hi = (lo, mid) if stand(lifted, mid, tilt).held else (mid, hi)
    return hi


# -- report ------------------------------------------------------------------
def report():
    out = {}
    print(f"{'stance':20s}{'needs grip (model)':>20s}{'(MuJoCo)':>10s}   pad pressure N, model -> MuJoCo, at 100 % grip")
    for case in CASES:
        env = stand(case)
        keep, pads, com, weight = geometry(env, case)
        g, _, _ = min_grip(pads, com, weight)
        # pressure at 100 % grip. Three pads: fully determined by balance. Four: one unknown too many.
        if case:                                                 # determinate: solve the 3 normal equations
            A = np.array([np.ones(3), pads[:, 1], pads[:, 0]])
            b = np.array([0.0, 0.0, weight * com[2]])            # sum T = 0, roll = 0, pitch: sum T x = W h
            pull = np.linalg.solve(A, b)
        normal, shear = pad_loads(env)
        gs = sim_min_grip(case)
        model_p = dict(zip(keep, PAD_FORCE - pull)) if case else {}
        line = "  ".join(f"{f} {model_p[f]:4.1f} -> {normal[FEET.index(f)]:4.1f}" if case
                         else f"{f} {normal[FEET.index(f)]:4.1f}" for f in keep)
        print(f"{NAMES[case]:20s}{g * 100:17.0f} % {gs * 100:7.0f} %   {line}")
        out[case] = dict(keep=keep, pads=pads, com=com, weight=weight, grip=g, grip_sim=gs,
                         normal=normal, shear=shear, model=model_p)
    return out


def tilt_sweep(out, tilts=np.arange(0, 181, 5)):
    return {c: [min_grip(o["pads"], o["com"], o["weight"], t)[0] for t in tilts] for c, o in out.items()}, tilts


def gait_trace(seconds=4.6):
    """Pad contact pressure through one gait cycle on a vertical wall."""
    from crawl_gait import climb
    rows = []
    climb(90, seconds=seconds, on_step=lambda env, n: rows.append(pad_loads(env)[0]))
    return np.array(rows)


def check(out):
    for case in FEET:                                            # three pads: model and simulation must agree
        o = out[case]
        for f in o["keep"]:
            assert abs(o["model"][f] - o["normal"][FEET.index(f)]) < 4.0, (case, f)
        assert abs(o["grip"] - o["grip_sim"]) < 0.15, case
    flat = min_grip(out[None]["pads"], out[None]["com"], out[None]["weight"], tilt_deg=0)[0]
    assert flat < 1e-6, "on the floor, four feet need no adhesion"
    assert np.isinf(min_grip(out[None]["pads"][:2], out[None]["com"], out[None]["weight"])[0]), "two point pads cannot balance"
    print("balance checks pass: force model and MuJoCo agree")


def figure(out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from matplotlib.patches import Polygon, Rectangle
    try:                                                         # Apple's system font (San Francisco)
        font_manager.fontManager.addfont("/System/Library/Fonts/SFNS.ttf")
        plt.rcParams["font.family"] = "System Font"
    except OSError:
        pass
    ink, mute, hot, cool, ok = "#1d2327", "#6b767d", "#d2552b", "#2b6cb0", "#2f8f5b"
    fig = plt.figure(figsize=(13, 6.4), dpi=150)
    gs = fig.add_gridspec(2, 5, height_ratios=[1.0, 1.0], hspace=0.38, wspace=0.3)

    # row 1: the five stances, seen looking at the wall (up the wall = up the page)
    for k, case in enumerate(CASES):
        ax, o = fig.add_subplot(gs[0, k]), out[case]
        ax.set_aspect("equal"), ax.axis("off")
        order = [f for f in ("fl", "fr", "hr", "hl") if f in o["keep"]]
        poly = np.array([o["pads"][o["keep"].index(f)] for f in order])
        ax.add_patch(Polygon(np.c_[-poly[:, 1], poly[:, 0]] * 100, closed=True, fc="#e8eef2", ec=mute, lw=1))
        for f in FEET:
            i = FEET.index(f)
            if f in o["keep"]:
                x, y = o["pads"][o["keep"].index(f)]
                p = o["normal"][i]
                col = hot if p < 0.8 * PAD_FORCE else ok                # carries less than its share
                ax.add_patch(Rectangle((-y * 100 - 3, x * 100 - 4), 6, 8, fc=col, ec="none", alpha=0.9))
                ax.text(-y * 100, x * 100, f"{p:.0f} N", ha="center", va="center", color="white", fontsize=9)
            else:
                x, y = out[None]["pads"][i]
                ax.add_patch(Rectangle((-y * 100 - 3, x * 100 - 4), 6, 8, fc="none", ec=mute, ls="--"))
                ax.text(-y * 100, x * 100, "lifted", ha="center", va="center", color=mute, fontsize=8)
        ax.plot(-o["com"][1] * 100, o["com"][0] * 100, "o", color=ink, ms=7)
        ax.annotate("", xy=(-o["com"][1] * 100, o["com"][0] * 100 - 6), xytext=(-o["com"][1] * 100, o["com"][0] * 100),
                    arrowprops=dict(arrowstyle="-|>", color=ink, lw=1.6))
        ax.set_xlim(-16, 16), ax.set_ylim(-17, 17)
        ax.set_title(f"{NAMES[case].capitalize()}\nneeds {o['grip_sim'] * 100:.0f} % grip", fontsize=10, color=ink)
    fig.text(0.125, 0.94, "Looking at a vertical wall, 100 % grip (15 N per pad). Numbers: how hard each pad presses on the wall (MuJoCo). "
             "Orange pads carry the least, so they slip or peel first.", fontsize=10, color=ink)

    # row 2 left: required grip against wall angle
    ax = fig.add_subplot(gs[1, :2])
    sweep, tilts = tilt_sweep(out)
    for case, col in zip(CASES, (ink, hot, "#e39b3b", cool, "#6aa7d8")):
        ax.plot(tilts, np.array(sweep[case]) * 100, color=col, lw=2 if case is None else 1.5, label=NAMES[case])
    ax.set_xticks([0, 45, 90, 135, 180], ["floor", "45°", "wall", "135°", "ceiling"])
    ax.set_ylabel("grip needed (%)"), ax.set_ylim(0, 100), ax.legend(frameon=False, fontsize=8, loc="upper left")
    ax.set_title("From floor to ceiling (force model)", loc="left", fontsize=10)

    # row 2 right: pad pressure through one gait cycle
    ax = fig.add_subplot(gs[1, 2:])
    trace = gait_trace()
    t = np.arange(len(trace)) * 0.02
    for i, (f, col) in enumerate(zip(FEET, (hot, "#e39b3b", cool, "#6aa7d8"))):
        ax.plot(t, trace[:, i], color=col, lw=1.4, label={"fl": "front-left", "fr": "front-right", "hl": "hind-left", "hr": "hind-right"}[f])
    ax.set_xlabel("time (s)"), ax.set_ylabel("pad pressure (N)"), ax.legend(frameon=False, fontsize=8, ncol=4, loc="upper center")
    ax.set_ylim(0, trace.max() * 1.25)
    ax.set_title("One gait cycle on the wall (MuJoCo): a pad at 0 N is stepping", loc="left", fontsize=10)
    for a in fig.axes[5:]:
        a.spines[["top", "right"]].set_visible(False)
    fig.savefig(OUT / "balance.png", bbox_inches="tight", facecolor="white")
    print("saved media/balance.png")


if __name__ == "__main__":
    results = report()
    check(results)
    if sys.argv[1:2] == ["figure"]:
        figure(results)
