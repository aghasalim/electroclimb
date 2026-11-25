#!/usr/bin/env python3
"""Render the wall-to-ceiling corner move and the two-pad ceiling trot (flat-only pads).

    ../.venv/bin/python render_corner.py    # media/corner.gif, media/corner.png, media/trot_ceiling.gif
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


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    corner_gif()
    trot_gif()
