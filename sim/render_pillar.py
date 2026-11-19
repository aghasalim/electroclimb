#!/usr/bin/env python3
"""Render the baseline gait climbing a square concrete pillar.

    ../.venv/bin/python render_pillar.py

The physics is unchanged: the robot climbs the flat face of the wall plane (gravity
tilted 90 deg). The pillar and the ground are drawn on top of it, so the image shows
a 40 cm column standing on the floor. Writes media/climb_pillar.gif and media/pillar.png.
"""
import mujoco
import numpy as np
from PIL import Image

from crawl_gait import climb
from render_media import OUT, label, make_camera, save_gif, show_voltage

W = H = 480
EVERY = 6
PILLAR = 0.20          # half width: a 40 cm square column
FLOOR_X = -0.35        # floor 35 cm below the robot's start (up the wall = +x)
TOP_X = 1.60

VIEWS = {
    "pillar": dict(lookat=(0.2, 0, -0.12), distance=1.25, elevation=-55, azimuth=90),
}


def add_box(scn, pos, half, rgba, matid=-1):
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_BOX, np.array(half), np.array(pos),
                        np.eye(3).flatten(), np.array(rgba, dtype=np.float32))
    g.matid = matid
    scn.ngeom += 1


def dress_scene(scn, model):
    mat = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MATERIAL, "concrete")
    mid = (FLOOR_X + TOP_X) / 2
    # the pillar: its top face (z = 0) is the face the robot climbs
    add_box(scn, (mid, 0, -PILLAR), ((TOP_X - FLOOR_X) / 2, PILLAR, PILLAR), (0.8, 0.8, 0.78, 1), mat)
    # the floor the pillar stands on
    add_box(scn, (FLOOR_X - 0.01, 0, -PILLAR), (0.01, 20, 20), (0.62, 0.6, 0.55, 1))
    # a room wall behind the pillar, so it doesn't float in black space
    add_box(scn, (mid, 0, -2.5), (20, 20, 0.01), (0.85, 0.86, 0.9, 1))


def main(seconds=20.0):
    v = VIEWS["pillar"]
    frames, st = [], {}

    def grab(env, n):
        if "r" not in st:
            wall = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, "wall")
            env.model.geom_rgba[wall, 3] = 0.0     # hide the infinite plane (render only)
            env.model.vis.headlight.ambient[:] = 0.35
            env.model.vis.headlight.diffuse[:] = 0.6
            env.model.vis.map.zfar = 50.0
            st["r"] = mujoco.Renderer(env.model, H, W)
            st["cam"] = make_camera(v["lookat"], v["distance"], v["elevation"], v["azimuth"])
        if n % EVERY:
            return
        r = st["r"]
        show_voltage(env)
        r.update_scene(env.data, st["cam"])
        dress_scene(r.scene, env.model)
        im = Image.fromarray(r.render().copy()).rotate(90, expand=True)
        t = n * env.model.opt.timestep * env.frame_skip
        x = (env.data.xpos[env.torso, 0] - env.start_x) * 100
        frames.append(label(im, "ElectroClimb - climbing a 40 cm concrete pillar (sim)",
                            f"t = {t:4.1f} s   climbed {x:4.1f} cm"))

    info = climb(90, seconds=seconds, on_step=grab)
    return frames, info


if __name__ == "__main__":
    import sys
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    frames, info = main(secs)
    frames[len(frames) // 2].save(OUT / "pillar.png")
    if secs >= 10:
        save_gif(frames, OUT / "climb_pillar.gif")
    print(f"climbed {info['progress'] * 100:.1f} cm, fell={info['fell']}")
