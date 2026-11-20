#!/usr/bin/env python3
"""Render README media from the simulation (real physics, not AI art).

    ../.venv/bin/python render_media.py

Writes media/climb_wall_front.gif, climb_wall_side.gif, leg_closeup.gif, hero.png, model_views.png
and model_turntable.gif.
Gravity is tilted 90 deg, so the plane is a vertical wall. Images are rotated so
"up the wall" (+x) is up on screen.
"""
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from crawl_gait import climb

from PIL import ImageFont
APPLE_FONT = "/System/Library/Fonts/SFNS.ttf"      # San Francisco, Apple's system font


def apple_font(size, weight="Semibold"):
    """Apple's system font; falls back to PIL's default on machines without it."""
    try:
        f = ImageFont.truetype(APPLE_FONT, size)
        f.set_variation_by_name(weight)
        return f
    except OSError:
        return ImageFont.load_default(size)


FONT = apple_font(13)
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
FLEX_ON, FLEX_OFF = np.array([1.0, 1.0, 1.0, 1]), np.array([0.5, 0.42, 0.38, 1])
LED_ON, LED_OFF = np.array([0.25, 1.0, 0.45, 1]), np.array([0.12, 0.14, 0.13, 1])


def show_voltage(env):
    """Colour each pad and its LED by the pad voltage: bright = energised, dull = released."""
    for i, f in enumerate(("fl", "fr", "hl", "hr")):
        v = float(env.data.ctrl[env.adh_ids[i]])
        env.model.geom_rgba[env.model.geom(f"{f}_pad").id] = PAD_OFF + v * (PAD_ON - PAD_OFF)
        env.model.geom_rgba[env.model.geom(f"{f}_led").id] = LED_OFF + v * (LED_ON - LED_OFF)
        env.model.geom_rgba[env.model.geom(f"{f}_flex").id] = FLEX_OFF + v * (FLEX_ON - FLEX_OFF)


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


def render_showcase():
    """Beauty shots of the model standing on the wall: hero.png, model_views.png, model_turntable.gif."""
    from crawl_gait import CrawlGait
    from env import ClimbEnv
    env = ClimbEnv(tilt_deg=90, randomise=False, fail_prob=0.0)
    env.reset()
    gait = CrawlGait(env)
    for _ in range(60):                                   # mid-step: one foot lifted, its pad switched off
        env.step(gait(env._foot_contacts()))
    show_voltage(env)
    x = env.data.xpos[env.torso, 0]
    r = mujoco.Renderer(env.model, 1000, 1000)

    def shot(lookat, distance, elevation, azimuth, rotate=90):
        r.update_scene(env.data, make_camera(lookat, distance, elevation, azimuth))
        return Image.fromarray(r.render().copy()).rotate(rotate)

    shot((x, 0, 0.06), 0.50, -38, 55).save(OUT / "hero.png")
    views = [("Whole robot", shot((x, 0, 0.06), 0.52, -38, 55)),
             ("Top: electronics bay, brain PCB", shot((x + 0.01, 0, 0.14), 0.30, -89.9, 90)),
             ("Leg: hip servo, thigh plate, knee servo, foot", shot((x - 0.07, -0.08, 0.04), 0.30, -30, 65)),
             ("Foot: electrode pad, HV driver, ankle", shot((x - 0.105, -0.085, 0.01), 0.15, -50, 40))]
    sheet = Image.new("RGB", (1000, 1000))
    for k, (name, im) in enumerate(views):
        im = im.resize((500, 500), Image.LANCZOS)
        d = ImageDraw.Draw(im)
        d.rectangle((0, 0, 500, 30), fill=(20, 22, 26))
        d.text((10, 7), name, fill=(255, 255, 255), font=FONT)
        sheet.paste(im, ((k % 2) * 500, (k // 2) * 500))
    sheet.save(OUT / "model_views.png")
    small = mujoco.Renderer(env.model, 420, 420)
    frames = []
    for az in range(0, 360, 8):                           # turntable: orbit around the wall normal
        small.update_scene(env.data, make_camera((x, 0, 0.05), 0.55, -32, az))
        frames.append(Image.fromarray(small.render().copy()))
    pal = [f.convert("P", palette=Image.ADAPTIVE, colors=96) for f in frames]
    pal[0].save(OUT / "model_turntable.gif", save_all=True, append_images=pal[1:], duration=100, loop=0, optimize=True)
    print("saved hero.png, model_views.png, model_turntable.gif")


def save_gif(frames, path):
    pal = [f.convert("P", palette=Image.ADAPTIVE, colors=128) for f in frames]
    pal[0].save(path, save_all=True, append_images=pal[1:], duration=FRAME_MS, loop=0, optimize=True)
    print(f"saved {path.name}: {len(frames)} frames, {path.stat().st_size / 1e6:.1f} MB")


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    for name in ("front", "side"):
        frames, info = render_view(name)
        save_gif(frames, OUT / f"climb_wall_{name}.gif")
    frames, _ = render_view("leg", seconds=9.0)
    save_gif(frames, OUT / "leg_closeup.gif")
    render_showcase()
