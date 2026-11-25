#!/usr/bin/env python3
"""Render the wall-to-ceiling corner move and the two-pad ceiling trot (flat-only pads).

    ../.venv/bin/python render_corner.py    # media/corner.gif, corner.png, trot_ceiling.gif, steer_wall.gif
"""
import mujoco
import numpy as np
from PIL import Image

import corner
import flat
from render_media import OUT, label, make_camera, save_gif, show_voltage

W, H = 560, 480
SPEEDUP = 6                            # the corner takes about 90 s; play it 6x faster


def corner_gif():
    frames, st = [], {}

    def grab(env, n):
        if "r" not in st:
            env.model.vis.global_.offwidth, env.model.vis.global_.offheight = max(W, H), max(W, H)
            st["r"] = mujoco.Renderer(env.model, W, H)   # rotated by 90 deg below: up the wall = up
            cx = env.model.geom_pos[env.model.geom("ceiling").id][0]
            st["cx"] = cx
            st["cam"] = make_camera((cx - 0.17, 0, 0.17), 0.95, -4, 90)
        if n % (3 * SPEEDUP):
            return
        show_voltage(env)
        st["r"].update_scene(env.data, st["cam"])
        im = Image.fromarray(st["r"].render().copy()).rotate(90, expand=True)
        R = env.data.xmat[env.torso].reshape(3, 3)
        turned = -np.degrees(np.arctan2(-R[2, 0], R[0, 0]))
        t = n * env.model.opt.timestep * env.frame_skip
        frames.append(label(im, "ElectroClimb - from the wall onto the ceiling (sim, pads grip only when flat)",
                            f"t = {t:4.0f} s   body turned {min(max(turned, 0), 90):2.0f} deg   {SPEEDUP}x speed"))

    ok, why, info = corner.run(corner.LIMITS_CAD, on_step=grab)     # the joint ranges the printed parts allow
    assert ok, why
    save_gif(frames, OUT / "corner.gif")
    strip = [frames[int(i)] for i in np.linspace(0, len(frames) - 1, 5)]
    sheet = Image.new("RGB", (sum(f.width for f in strip), strip[0].height))
    x = 0
    for f in strip:
        sheet.paste(f, (x, 0))
        x += f.width
    sheet.save(OUT / "corner.png")
    print("saved corner.png")


def trot_gif(cycles=3):
    frames, st = [], {}

    def grab(env, n):
        if "r" not in st:
            st["r"] = mujoco.Renderer(env.model, 360, 360)
            st["start"] = env.data.xpos[env.torso, 0]
        if n % 3:
            return
        show_voltage(env)
        x = env.data.xpos[env.torso, 0]
        # the ceiling is the plane the robot stands on with gravity turned over: look up at it
        cam = make_camera((x + 0.02, 0, 0.05), 0.75, -18, 60)
        st["r"].update_scene(env.data, cam)
        im = Image.fromarray(st["r"].render().copy()).rotate(180)
        t = n * env.model.opt.timestep * env.frame_skip
        frames.append(label(im, "ElectroClimb - ceiling trot: only two pads hold (sim)",
                            f"t = {t:4.1f} s   walked {(x - st['start']) * 100:4.1f} cm   real time"))

    out, failed, grips, torque, secs = flat.walk(180, [(None, 0, 0, cycles)], flat.TrotGait, on_step=grab)
    assert not failed
    save_gif(frames, OUT / "trot_ceiling.gif")


def steer_gif():
    """One walk on a vertical wall: up, turn left on the spot, across, sideways. Flat-only pads."""
    route = [("climbing up", (0.04, 0, 0), 3), ("turning left on the spot", (0, 0, 0.2), 10),
             ("walking across the wall", (0.04, 0, 0), 3), ("stepping sideways", (0, -0.02, 0), 3)]
    env = flat.FlatEnv(tilt_deg=90.0)
    env.reset()
    gait = flat.FlatGait(env)
    r = mujoco.Renderer(env.model, 400, 400)
    start = flat.pose(env)
    cam = make_camera((start[0] + 0.07, 0.07, 0.05), 0.95, -90, 90)       # looking straight at the wall
    frames, n = [], 0
    for name, cmd, cycles in route:
        gait.cmd = cmd
        t_end = gait.t + cycles * gait.cycle
        while gait.t < t_end - 1e-9:
            env.step(gait(env._foot_contacts()))
            assert 0.02 < env.data.xpos[env.torso, 2] < 0.35, "fell during the steering render"
            n += 1
            if n % 18 == 0:                             # 60 ms frames: 6x real time
                show_voltage(env)
                r.update_scene(env.data, cam)
                x, y, h = flat.pose(env) - start
                frames.append(label(Image.fromarray(r.render().copy()).rotate(90, expand=True),
                                    f"ElectroClimb steering on a wall (sim): {name}",
                                    f"t = {n * gait.dt:3.0f} s   up {x * 100:5.1f} cm   left {y * 100:5.1f} cm   "
                                    f"heading {h:4.0f} deg   6x speed"))
    save_gif(frames, OUT / "steer_wall.gif")
    frames[len(frames) // 2].save(OUT / "steer_wall.png")


def pad_compare():
    """One hind foot just after its first step: original foot (pad on its edge) next to the soft ankle (pad flat)."""
    from crawl_gait import CrawlGait
    from env import ClimbEnv
    shots = []
    for title, env in (("Original foot: ankle 6 N*m/rad", ClimbEnv(tilt_deg=90.0, randomise=False, fail_prob=0.0)),
                       ("Soft ankle: 0.1 N*m/rad", flat.FlatEnv(tilt_deg=90.0))):
        env.reset()
        gait = CrawlGait(env)
        foot = env.model.body("hr_foot").id
        while gait.t < 1.0:                             # the hind-right leg has stepped and gripped again
            env.step(gait(env._foot_contacts()))
        z = env.data.xmat[foot].reshape(3, 3)[:, 2]
        tilt = np.degrees(np.arccos(min(1.0, abs(z[2]))))
        show_voltage(env)
        r = mujoco.Renderer(env.model, 560, 560)
        x, y, _ = env.data.xpos[foot]
        r.update_scene(env.data, make_camera((x + 0.005, y, 0.035), 0.42, -4, 90))
        im = Image.fromarray(r.render().copy()).rotate(90, expand=True)
        shots.append(label(im, title, f"pad tilted {tilt:4.1f} deg to the wall   (simulation, wall on the right)"))
    sheet = Image.new("RGB", (shots[0].width * 2 + 8, shots[0].height), (20, 22, 26))
    sheet.paste(shots[0], (0, 0))
    sheet.paste(shots[1], (shots[0].width + 8, 0))
    sheet.save(OUT / "pad_tilt.png")
    print("saved pad_tilt.png")


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    corner_gif()
    trot_gif()
    steer_gif()
    pad_compare()
