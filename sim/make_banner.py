#!/usr/bin/env python3
"""Render the repository banner, the GitHub social preview and the post image ("Fringe Field" design).

    ../.venv/bin/python make_banner.py     # -> ../media/banner.png, social_preview.png, post_square.png

The robot is rendered in MuJoCo on a graphite wall; the electrode row and its fringe-field arcs
are drawn with PIL at 2x and scaled down. Design notes: ../docs/DESIGN.md.
"""
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

from crawl_gait import CrawlGait
from env import ClimbEnv
from render_media import APPLE_FONT, OUT, make_camera, show_voltage

S = 2                                   # supersampling
AMBER, BLUE = (242, 154, 31), (86, 146, 208)
WHITE, GREY, FAINT = (240, 242, 244), (150, 158, 166), (58, 64, 70)


def sf(size, weight="Regular"):
    try:
        f = ImageFont.truetype(APPLE_FONT, size * S)
        f.set_variation_by_name(weight)
        return f
    except OSError:
        return ImageFont.load_default(size * S)


def robot_render(px=1400):
    """The robot mid-step on a plain graphite wall. Returns (image, wall colour)."""
    env = ClimbEnv(tilt_deg=90, randomise=False, fail_prob=0.0)
    env.reset()
    gait = CrawlGait(env)
    for _ in range(60):
        env.step(gait(env._foot_contacts()))
    show_voltage(env)
    m = env.model
    wall = m.geom("wall").id
    m.geom_matid[wall] = -1
    m.geom_rgba[wall] = (0.085, 0.095, 0.105, 1)
    r = mujoco.Renderer(m, px, px)
    x = env.data.xpos[env.torso, 0]
    r.update_scene(env.data, make_camera((x + 0.005, 0, 0.05), 0.50, -40, 52))
    im = Image.fromarray(r.render().copy()).rotate(90)
    edge = np.asarray(im)[:, :12].reshape(-1, 3)
    return im, tuple(int(v) for v in np.median(edge, 0))


def spaced(d, xy, text, font, fill, gap):
    """Letter-spaced text. Coordinates and gap in 1x pixels; returns the end x (1x)."""
    x, y = xy[0] * S, xy[1] * S
    for ch in text:
        d.text((x, y), ch, font=font, fill=fill)
        x += d.textlength(ch, font=font) + gap * S
    return x / S


def spaced_width(d, text, font, gap):
    return sum(d.textlength(ch, font=font) for ch in text) / S + gap * len(text)


def compose(W, H, robot, bg, title_px, name, square=False):
    im = Image.new("RGB", (W * S, H * S), bg)
    # robot on the right, fading into the field
    rh = int(H * (0.70 if square else 0.98))
    rob = robot.resize((rh * S, rh * S), Image.LANCZOS)
    mask = Image.new("L", rob.size, 0)
    ImageDraw.Draw(mask).ellipse((rob.width * 0.04, rob.height * 0.04, rob.width * 0.96, rob.height * 0.96), fill=255)
    mask = mask.filter(ImageFilter.GaussianBlur(rob.width * 0.06))
    im.paste(rob, (int((W - rh) / 2 * S) if square else int((W - rh * 1.04) * S), int(H * 0.235 * S) if square else int((H - rh) / 2 * S)), mask)
    d = ImageDraw.Draw(im, "RGBA")
    L = int(W * 0.055)                                  # left margin
    # climb scale along the left edge
    for k in range(0, 61):
        y = H - 56 - k * (H - 112) / 60
        d.line(((L - 34) * S, y * S, (L - 34 + (14 if k % 10 == 0 else 6)) * S, y * S), fill=FAINT + (255,), width=S)
    # title block
    ty = int(H * (0.065 if square else 0.17))
    spaced(d, (L, ty - 6), "OPEN-SOURCE CLIMBING ROBOT", sf(13, "Medium"), AMBER, 3.2)
    d.text((L * S - 4 * S, (ty + 22) * S), "ElectroClimb", font=sf(title_px, "Semibold"), fill=WHITE)
    sub_y = ty + 22 + int(title_px * 1.28)
    d.text((L * S, sub_y * S), "Walks up concrete, glass and wood on electric feet.", font=sf(int(title_px * 0.285), "Light"), fill=GREY)
    if not square:
        # electrode row and its fringe field (cross-section of one pad)
        ey = int(H * 0.665)
        ew, eg, n = 30, 16, 12
        for k in range(n):
            x0 = L + k * (ew + eg)
            d.rounded_rectangle((x0 * S, ey * S, (x0 + ew) * S, (ey + 7) * S), 2 * S, fill=(AMBER if k % 2 == 0 else BLUE) + (255,))
            if k < n - 1:
                cx = x0 + ew + eg / 2
                for j, rad in enumerate((eg / 2 + 5, eg / 2 + 13, eg / 2 + 21)):
                    a = (150, 95, 52)[j]
                    d.arc(((cx - rad) * S, (ey + 7 - rad * 0.9) * S, (cx + rad) * S, (ey + 7 + rad * 0.9) * S), 0, 180,
                          fill=(190, 198, 206, a), width=S)
        end = L + n * (ew + eg) - eg
        d.line((L * S, (ey + 7.5) * S, end * S, (ey + 7.5) * S), fill=FAINT + (255,), width=S)
        spaced(d, (L, ey - 26), "PAD CROSS-SECTION", sf(10, "Medium"), GREY, 2.4)
        label = "FRINGE FIELD INTO THE WALL"
        spaced(d, (end - spaced_width(d, label, sf(10, "Medium"), 2.4) + 2.4, ey - 26), label, sf(10, "Medium"), GREY, 2.4)
    # facts
    fy, x = H - 62, L
    for value, unit in (("3 kV", "FEET"), ("1.46 kg", "ROBOT"), ("12", "SERVOS"), ("88 %", "HARD WALLS IN SIM")):
        d.text((x * S, fy * S), value, font=sf(19, "Semibold"), fill=WHITE)
        x += d.textlength(value, font=sf(19, "Semibold")) / S + 8
        x = spaced(d, (x, fy + 9), unit, sf(10, "Medium"), GREY, 2.0) + 26
    im = im.resize((W, H), Image.LANCZOS)
    im.save(OUT / name)
    print(f"saved {name}")


if __name__ == "__main__":
    robot, bg = robot_render()
    compose(1600, 640, robot, bg, 104, "banner.png")
    compose(1280, 640, robot, bg, 88, "social_preview.png")
    compose(1200, 1200, robot, bg, 96, "post_square.png", square=True)
