#!/usr/bin/env python3
"""Render README media from the simulation (real physics, not AI art).

    ../.venv/bin/python render_media.py

Writes media/climb_wall_front.gif, media/climb_wall_side.gif, media/leg_closeup.gif and media/hero.png.
Gravity is tilted 90 deg, so the plane is a vertical wall. Images are rotated so
"up the wall" (+x) is up on screen.
"""
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from crawl_gait import climb

from PIL import ImageFont
FONT = ImageFont.load_default(13)
OUT = Path(__file__).resolve().parent.parent / "media"
W = H = 480
EVERY = 6            # grab every 6th control step (0.12 s) ...
FRAME_MS = 60        # ... and play at 60 ms -> 2x real time


def make_camera(lookat, distance, elevation, azimuth):
    cam = mujoco.MjvCamera()
    cam.lookat[:] = lookat
    cam.distance, cam.elevation, cam.azimuth = distance, elevation, azimuth
    return cam


VIEWS = {
    # looking straight at the wall
    "front": dict(lookat=(0.11, 0, 0.05), distance=0.72, elevation=-90, azimuth=90, rotate=90),
    # from the side, wall on the right
    "side": dict(lookat=(0.11, 0, 0.06), distance=0.75, elevation=-5, azimuth=90, rotate=90),
    # close-up of the hind-right leg, looking down the wall at an angle
    "leg": dict(lookat=(-0.05, -0.07, 0.04), distance=0.42, elevation=-35, azimuth=70, rotate=90),
    # 3/4 view for the hero image
    "hero": dict(lookat=(0.07, 0, 0.05), distance=0.58, elevation=-34, azimuth=58, rotate=90),
}


PAD_ON, PAD_OFF = np.array([1.0, 0.66, 0.12, 1]), np.array([0.45, 0.33, 0.22, 1])
LED_ON, LED_OFF = np.array([0.25, 1.0, 0.45, 1]), np.array([0.12, 0.14, 0.13, 1])


def show_voltage(env):
    """Colour each pad and its LED by the pad voltage: bright = energised, dull = released."""
    for i, f in enumerate(("fl", "fr", "hl", "hr")):
        v = float(env.data.ctrl[env.adh_ids[i]])
        env.model.geom_rgba[env.model.geom(f"{f}_pad").id] = PAD_OFF + v * (PAD_ON - PAD_OFF)
        env.model.geom_rgba[env.model.geom(f"{f}_led").id] = LED_OFF + v * (LED_ON - LED_OFF)


def label(im, text, sub):
    d = ImageDraw.Draw(im)
    d.rectangle((0, 0, im.width, 40), fill=(20, 22, 26))
    d.text((10, 6), text, fill=(255, 255, 255), font=FONT)
    d.text((10, 22), sub, fill=(255, 170, 60), font=FONT)
    return im


def render_view(name, seconds=20.0):
    v = VIEWS[name]
    frames, state = [], {}

    def grab(env, n):
        if "r" not in state:
            state["r"] = mujoco.Renderer(env.model, H, W)
            state["cam"] = make_camera(v["lookat"], v["distance"], v["elevation"], v["azimuth"])
        if n % EVERY:
            return
        r = state["r"]
        show_voltage(env)
        r.update_scene(env.data, state["cam"])
        im = Image.fromarray(r.render().copy()).rotate(v["rotate"], expand=True)
        t = n * env.model.opt.timestep * env.frame_skip
        x = (env.data.xpos[env.torso, 0] - env.start_x) * 100
        frames.append(label(im, "ElectroClimb - learned-adhesion climbing robot (sim)",
                            f"vertical wall   t = {t:4.1f} s   climbed {x:4.1f} cm"))

    info = climb(90, seconds=seconds, on_step=grab)
    return frames, info


def save_gif(frames, path):
    pal = [f.convert("P", palette=Image.ADAPTIVE, colors=96) for f in frames]
    pal[0].save(path, save_all=True, append_images=pal[1:], duration=FRAME_MS, loop=0, optimize=True)
    print(f"saved {path.name}: {len(frames)} frames, {path.stat().st_size / 1e6:.1f} MB")


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    for name in ("front", "side"):
        frames, info = render_view(name)
        save_gif(frames, OUT / f"climb_wall_{name}.gif")
    frames, _ = render_view("leg", seconds=9.0)
    save_gif(frames, OUT / "leg_closeup.gif")
    frames, _ = render_view("hero", seconds=6.0)
    frames[-1].save(OUT / "hero.png")
    print("saved hero.png")
