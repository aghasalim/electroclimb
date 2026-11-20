#!/usr/bin/env python3
"""Generate the robot's detailed visual model: meshes, textures and electroclimb.xml.

    ../.venv/bin/python make_assets.py        # writes assets/*.obj, assets/*.png, electroclimb.xml

Everything drawn here is visual only (class "vis": no mass, no contacts). The physics lives in
the few hidden "col" shapes and the four pads, exactly as before, so changing the look can
never change how the robot climbs. Dimensions in metres; the servo is an STS3215
(45.2 x 24.7 x 35 mm).

Layout (torso frame: x = climbing direction, z = away from the wall):
  torso  - graphite chassis tub, 4 abduction servos at the corners, stereo camera nose,
           tail block (tether ring, antenna, e-stop), battery under the belly,
           electronics bay on top (brain PCB, power board) under a clear lid
  hip    - hip-pitch servo, bolted to the abduction servo's horn by a strap
  thigh  - one outboard plate between the hip horn and the knee horn
  shin   - knee servo built into the shin, carbon tube to the ankle, HV cable
  foot   - ball ankle, TPU spider, flex-PCB electrode pad, HV driver board
"""
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

import meshkit as mk

HERE = Path(__file__).resolve().parent
ASSETS = HERE / "assets"
FONT_FILE = "/System/Library/Fonts/SFNS.ttf"      # San Francisco, Apple's system font
THIGH = np.array([0.04, 0.0, -0.07])          # hip axis -> knee axis (thigh frame)
SHIN = np.array([-0.03, 0.0, -0.06])          # knee axis -> ankle (shin frame)
LEGS = (("fl", 1, 1), ("fr", 1, -1), ("hl", -1, 1), ("hr", -1, -1))


def font(size):
    try:
        f = ImageFont.truetype(FONT_FILE, size)
        f.set_variation_by_name("Bold")
        return f
    except OSError:
        return ImageFont.load_default(size)


class Parts(dict):
    """material name -> Mesh, for one body."""

    def put(self, material, mesh, R=None, p=(0, 0, 0)):
        self.setdefault(material, mk.Mesh()).merge(mesh.moved(R, p))
        return self

    def take(self, other, R=None, p=(0, 0, 0)):
        for material, mesh in other.items():
            self.put(material, mesh, R, p)
        return self


def at(mesh, x=0, y=0, z=0):
    return mesh.moved(None, (x, y, z))


# -- parts -------------------------------------------------------------------
def servo():
    """STS3215-style servo. Local frame: output shaft along +z at the origin, top face at z = 0,
    body from x = -35.2 mm to +10 mm."""
    s = Parts()
    cx, hx, hy = -0.0126, 0.0226, 0.01235
    s.put("graphite", at(mk.rounded_box(hx, hy, 0.004, 0.002, 3), cx, 0, -0.031))
    s.put("graphite", at(mk.rounded_box(hx - 0.0005, hy - 0.0005, 0.0095, 0.0015, 3), cx, 0, -0.017))
    s.put("graphite", at(mk.rounded_box(hx, hy, 0.0035, 0.002, 3), cx, 0, -0.0035))
    for x in (-0.0372, 0.0120):                                   # mounting ears
        s.put("graphite", at(mk.rounded_box(0.0022, 0.009, 0.0012, 0.0006, 2), x, 0, -0.008))
    s.put("alu", mk.cyl(0.006, 0.0, 0.001, 16, 0.0003))                               # output boss
    s.put("alu", mk.cyl(0.0045, -0.0366, -0.035, 14, 0.0003))                         # idler
    for x in (-0.0325, 0.0072):
        for y in (-0.0095, 0.0095):
            s.put("alu", at(mk.cyl(0.0013, 0.0, 0.0005, 8), x, y))                    # case screws
    for y in (-hy, hy):                                           # label
        s.put("shell", at(mk.box(0.009, 0.00015, 0.0035), cx - 0.004, y, -0.017))
        s.put("accent", at(mk.box(0.002, 0.0002, 0.0035), cx + 0.009, y, -0.017))
    s.put("rubber", at(mk.rounded_box(0.0018, 0.004, 0.002, 0.0008, 2), -0.0362, 0, -0.022))   # cable exit
    return s


def horn():
    """Round servo horn, on the shaft from z = 1 to 3 mm, with screws."""
    h = Parts()
    h.put("alu", mk.cyl(0.0105, 0.001, 0.003, 22, 0.0006))
    h.put("carbon", mk.cyl(0.0026, 0.003, 0.0037, 10, 0.0003))
    for a in np.arange(4) * np.pi / 2 + np.pi / 4:
        h.put("carbon", at(mk.cyl(0.0011, 0.003, 0.0033, 6), 0.0072 * np.cos(a), 0.0072 * np.sin(a)))
    return h


def torso():
    t = Parts()
    # chassis tub
    core = mk.rounded_rect(0.049, 0.057, 0.008)
    t.put("graphite", mk.loft([(mk.inset(core, 0.002), -0.019), (core, -0.017), (core, 0.011), (mk.inset(core, 0.002), 0.013)]))
    for sy in (-1, 1):                                            # cooling fins + grab rails
        for x in np.linspace(-0.036, 0.036, 9):
            t.put("alu", at(mk.box(0.0012, 0.0022, 0.011), x, sy * 0.0588, -0.003))
        rail = mk.smooth([(-0.040, sy * 0.058, 0.006), (-0.034, sy * 0.068, 0.006), (0.034, sy * 0.068, 0.006), (0.040, sy * 0.058, 0.006)])
        t.put("alu", mk.tube(rail, 0.0022, 8))
    # abduction servos: shaft along +-x through the hip origin, body pointing inboard
    for _, sx, sy in LEGS:
        R = mk.frame(x=(0, sy, 0), z=(sx, 0, 0))
        t.take(servo(), R, (0.085 * sx, 0.06 * sy, 0))
        cable = mk.smooth([(0.062 * sx, 0.024 * sy, 0.0), (0.058 * sx, 0.016 * sy, 0.004), (0.050 * sx, 0.012 * sy, 0.006)])
        t.put("rubber", mk.tube(cable, 0.0016, 6))
    # nose: stereo camera head
    t.put("graphite", at(mk.rounded_box(0.0275, 0.0235, 0.014, 0.004, 4), 0.0765, 0, -0.002))
    X = mk.frame(x=(0, 0, -1), z=(1, 0, 0))
    for y in (-0.0125, 0.0125):
        t.put("alu", mk.lathe([(0.0, 0), (0.0082, 0), (0.0082, 0.003), (0.0068, 0.004), (0.0058, 0.004)], 20).moved(X, (0.104, y, 0.001)))
        t.put("lens", mk.lathe([(0.0058, 0.0032), (0.0035, 0.0038), (0, 0.004)], 20).moved(X, (0.104, y, 0.001)))
        t.put("glass", mk.cyl(0.0026, 0.0038, 0.0041, 12).moved(X, (0.104, y, 0.001)))
    t.put("glass", mk.cyl(0.0024, 0, 0.0012, 12).moved(X, (0.104, 0, 0.001)))                 # IR projector
    t.put("lamp", at(mk.rounded_box(0.0008, 0.019, 0.0022, 0.0006, 2), 0.1045, 0, -0.011))      # inspection light bar
    t.put("accent", at(mk.rounded_box(0.0006, 0.021, 0.0009, 0.0004, 2), 0.1043, 0, 0.0095))
    # tail: IO block, tether ring, antenna, e-stop, charge port
    t.put("graphite", at(mk.rounded_box(0.0275, 0.0235, 0.014, 0.004, 4), -0.0765, 0, -0.002))
    t.put("alu", mk.torus(0.0085, 0.0021, 24, 8).moved(mk.frame(x=(1, 0, 0), z=(0, 1, 0)), (-0.111, 0, -0.002)))
    t.put("alu", at(mk.cyl(0.004, -0.004, 0.0, 12), 0, 0, 0).moved(mk.frame(x=(0, 0, -1), z=(-1, 0, 0)), (-0.100, 0, -0.002)))
    t.put("accent", at(mk.rounded_box(0.0008, 0.005, 0.003, 0.0005, 2), -0.1042, -0.013, -0.006))   # charge port
    t.put("glass", at(mk.rounded_box(0.0008, 0.004, 0.002, 0.0005, 2), -0.1042, 0.013, -0.006))     # USB-C
    # battery under the belly, held by two straps
    t.put("battery", at(mk.rounded_box(0.040, 0.021, 0.0075, 0.003, 3), -0.004, 0, -0.0265))
    t.put("accent", at(mk.box(0.012, 0.0212, 0.0077), -0.022, 0, -0.0265))
    for x in (-0.030, 0.022):
        t.put("rubber", at(mk.rounded_box(0.004, 0.0222, 0.0085, 0.002, 2), x, 0, -0.0262))
    t.put("accent", at(mk.rounded_box(0.004, 0.0035, 0.003, 0.001, 2), 0.039, 0.008, -0.0265))     # XT30
    # electronics bay: white tray + rim on top of the chassis and servos
    bay = mk.rounded_rect(0.096, 0.050, 0.018, 8)
    t.put("shell", mk.loft([(mk.inset(bay, 0.0015), 0.013), (bay, 0.0145), (bay, 0.022), (mk.inset(bay, 0.0015), 0.0235),
                            (mk.inset(bay, 0.0045), 0.0235), (mk.inset(bay, 0.0045), 0.0165), (mk.inset(bay, 0.03), 0.0165)]))
    t.put("graphite", mk.plate(mk.inset(bay, 0.0046), 0.0165, 0.0168, 0.0001))                 # dark bay floor
    t.put("smoke", mk.plate(mk.inset(bay, 0.0025), 0.0236, 0.0246, 0.0004))                    # clear lid
    for x, y in ((0.070, 0.0462), (0.0, 0.0472), (-0.070, 0.0462)):
        for sy in (-1, 1):
            t.put("alu", at(mk.cyl(0.0019, 0.0246, 0.0252, 8, 0.0002), x, sy * y))             # lid screws
    # brain PCB on standoffs, with its tall parts
    bx, bz = 0.014, 0.0185
    t.put("brain", at(mk.slab_uv(0.045, 0.030, 0.0175, bz), bx))
    for x in (-0.041, 0.041):
        for y in (-0.026, 0.026):
            t.put("alu", at(mk.cyl(0.0016, 0.0165, 0.0190, 8), bx + x, y))
    for k in range(8):                                            # CPU heat sink
        t.put("alu", at(mk.box(0.0095, 0.0005, 0.002), bx + 0.018, 0.004 - 0.0084 + k * 0.0024, bz + 0.0025))
    t.put("alu", at(mk.box(0.0095, 0.0095, 0.0006), bx + 0.018, 0.004, bz + 0.0006))
    t.put("carbon", at(mk.lathe([(0.0095, 0), (0.0095, 0.0035), (0.0082, 0.0035), (0.0082, 0.0008), (0.003, 0.0008), (0.003, 0.0032), (0, 0.0032)], 22), bx - 0.006, 0.006, bz))  # fan
    for a in np.arange(7) * 2 * np.pi / 7:
        blade = mk.box(0.0026, 0.0009, 0.0003).moved(mk.frame(x=(np.cos(a), np.sin(a), 0), z=(0.35 * np.sin(a), -0.35 * np.cos(a), 1)))
        t.put("graphite", at(blade, bx - 0.006 + 0.0056 * np.cos(a), 0.006 + 0.0056 * np.sin(a), bz + 0.002))
    for y in (-0.012, 0.004):                                     # USB / Ethernet at the front edge
        t.put("alu", at(mk.rounded_box(0.0055, 0.0062, 0.0022, 0.0005, 2), bx + 0.0395, y, bz + 0.0022))
    t.put("carbon", at(mk.rounded_box(0.0255, 0.0025, 0.0020, 0.0004, 2), bx - 0.004, 0.0262, bz + 0.0020))   # GPIO header
    for x in np.linspace(-0.028, 0.020, 20):
        t.put("gold", at(mk.box(0.0003, 0.0014, 0.0003), bx + x, 0.0262, bz + 0.0041))
    for x, y in ((-0.030, -0.020), (-0.022, -0.020), (-0.038, -0.006)):
        t.put("alu", at(mk.cyl(0.0026, bz, bz + 0.0042, 12, 0.0004), bx + x, y))               # capacitors
    t.put("battery", at(mk.rounded_box(0.005, 0.004, 0.0022, 0.0005, 2), bx - 0.034, 0.014, bz + 0.0022))   # HV interlock relay
    t.put("accent", at(mk.rounded_box(0.0035, 0.0035, 0.0006, 0.0003, 2), bx + 0.030, -0.021, bz + 0.0006))  # IMU
    for k, y in enumerate((-0.024, -0.016, 0.014, 0.022)):       # 4 leg connectors (CAN + power)
        t.put("shell", at(mk.rounded_box(0.0032, 0.0028, 0.0018, 0.0004, 2), bx - 0.0405, y, bz + 0.0018))
    # power board at the rear of the bay, and the e-stop + antenna through the rim
    px = -0.062
    t.put("pcb", at(mk.box(0.014, 0.030, 0.0005), px, 0, 0.0180))
    t.put("carbon", at(mk.cyl(0.0048, 0.0185, 0.0228, 14, 0.0005), px + 0.003, 0.016))         # inductor
    t.put("alu", at(mk.cyl(0.004, 0.0185, 0.0228, 14, 0.0004), px + 0.003, -0.002))            # bulk capacitor
    t.put("accent", at(mk.rounded_box(0.004, 0.0045, 0.0022, 0.0006, 2), px - 0.004, -0.019, 0.0207))        # battery plug
    t.put("carbon", at(mk.rounded_box(0.0045, 0.0035, 0.0008, 0.0003, 2), px - 0.005, 0.004, 0.0193))
    for y in (-0.008, 0.010, 0.022):
        t.put("carbon", at(mk.box(0.002, 0.0015, 0.0006), px - 0.008, y, 0.0191))
    t.put("red", at(mk.lathe([(0.0, 0.0246), (0.0058, 0.0246), (0.0058, 0.0270), (0.0045, 0.0285), (0, 0.0288)], 18), -0.078, -0.030))   # e-stop
    t.put("alu", at(mk.cyl(0.0072, 0.0246, 0.0254, 18, 0.0003), -0.078, -0.030))
    t.put("alu", at(mk.cyl(0.0032, 0.0246, 0.0275, 12, 0.0004), -0.080, 0.032))
    t.put("rubber", at(mk.lathe([(0, 0.0275), (0.0022, 0.0275), (0.0019, 0.060), (0.0026, 0.062), (0.0019, 0.064), (0, 0.065)], 10), -0.080, 0.032))  # antenna
    t.put("logo", at(mk.slab_uv(0.026, 0.0075, 0.0246, 0.0249), -0.020, -0.0335))
    return t


def hip(sx, sy):
    h = Parts()
    R = mk.frame(x=(0, 0, -1), z=(0, sy, 0))                      # shaft along +-y, body pointing up
    h.take(servo(), R, (0, 0.032 * sy, 0))
    band = mk.rounded_rect(0.0142, 0.0245, 0.0025, 4)             # strap around the servo
    strap = mk.loft([(band, -0.0035), (band, 0.0035)]).moved(mk.frame(x=(1, 0, 0), z=(0, sy, 0)), (0, 0.016 * sy, 0.0126))
    h.put("alu", strap)
    h.take(horn(), mk.frame(x=(0, 0, 1), z=(-sx, 0, 0)), (-0.011 * sx, 0, 0))                  # abduction horn
    h.put("alu", at(mk.box(0.0008, 0.011, 0.012), -0.0131 * sx, 0.010 * sy, 0.004))
    return h


def thigh(sy):
    t = Parts()
    Y = mk.frame(x=(1, 0, 0), z=(0, sy, 0))                       # local (x, y) = plate plane, z = outboard
    knee = (THIGH[0], -THIGH[2])                                  # plate plane: x forward, y = -z (mirrored below)
    P = lambda mesh: mesh.moved(mk.frame(x=(1, 0, 0), y=(0, 0, -1), z=(0, sy, 0)))
    outline = mk.link_outline((0, 0), 0.0150, knee, 0.0135)
    t.put("shell", P(mk.plate(outline, 0.015, 0.0182, 0.0007)))
    d = np.array(knee) / np.linalg.norm(knee)
    slot = mk.link_outline(np.array(knee) * 0.30, 0.0046, np.array(knee) * 0.70, 0.0042, 16)
    t.put("carbon", P(mk.plate(slot, 0.0182, 0.0186, 0.0002)))
    for c in ((0, 0), knee):
        t.take(horn(), mk.frame(x=(1, 0, 0), y=(0, 0, -1), z=(0, sy, 0)), (c[0], 0.011 * sy, -c[1]))
        t.put("accent", P(at(mk.cyl(0.0074, 0.0182, 0.0196, 20, 0.0005), c[0], c[1])))
        t.put("alu", P(at(mk.cyl(0.0024, 0.0196, 0.0202, 10, 0.0002), c[0], c[1])))
        for a in np.arange(6) * np.pi / 3:
            t.put("alu", P(at(mk.cyl(0.0011, 0.0182, 0.0187, 6), c[0] + 0.0108 * np.cos(a), c[1] + 0.0108 * np.sin(a))))
    return t


def shin(sy):
    s = Parts()
    d = SHIN / np.linalg.norm(SHIN)
    R = mk.frame(x=-d, z=(0, sy, 0))                              # knee servo: body runs down the shin
    s.take(servo(), R, (0, 0.012 * sy, 0))
    L = np.linalg.norm(SHIN)
    Z = mk.z_to(d)
    s.put("alu", mk.cyl(0.0068, 0.0345, 0.0405, 16, 0.0006).moved(Z))                        # tube clamp
    s.put("carbon", mk.cyl(0.0046, 0.040, L - 0.004, 14).moved(Z))                           # carbon tube
    s.put("alu", mk.lathe([(0.0046, L - 0.012), (0.0058, L - 0.011), (0.0058, L - 0.006), (0.0040, L - 0.003)], 14).moved(Z))   # ankle socket
    side = np.cross(d, [0, 1.0, 0])
    cable = mk.smooth([0.030 * d + 0.012 * side, 0.040 * d + 0.008 * side, 0.052 * d + 0.0062 * side, (L - 0.006) * d + 0.0062 * side])
    s.put("hv", mk.tube(cable, 0.0013, 6))
    for k in (0.045, 0.058):
        s.put("rubber", mk.torus(0.0062, 0.0008, 12, 5).moved(Z, k * d + 0.0008 * side))     # cable ties
    return s


def foot():
    f = Parts()
    f.put("alu", mk.sphere(0.0052, 14))                                                       # ball ankle
    f.put("alu", at(mk.torus(0.0074, 0.0014, 18, 6), 0, 0, 0.0046))
    f.put("rubber", at(mk.rounded_box(0.0125, 0.0105, 0.0024, 0.0015, 3), 0, 0, 0.0022))      # TPU hub
    for sx in (-1, 1):
        for sy in (-1, 1):
            arm = [(sx * 0.009, sy * 0.007, 0.0022), (sx * 0.018, sy * 0.015, 0.0016), (sx * 0.0255, sy * 0.0205, 0.0008)]
            f.put("rubber", mk.tube(arm, 0.0021, 7))
            f.put("rubber", at(mk.cyl(0.0042, -0.0006, 0.0022, 12, 0.0006), sx * 0.0255, sy * 0.0205))
            f.put("alu", at(mk.cyl(0.0014, 0.0022, 0.0026, 6), sx * 0.0255, sy * 0.0205))
    # HV driver board on the pad, behind the ankle
    px = -0.0195
    f.put("pcb", at(mk.box(0.0075, 0.0115, 0.0005), px, 0, 0.0008))
    f.put("carbon", at(mk.rounded_box(0.0048, 0.0062, 0.0030, 0.0006, 2), px + 0.0005, 0.0035, 0.0043))    # HV module
    f.put("shell", at(mk.rounded_box(0.0045, 0.0016, 0.0016, 0.0004, 2), px + 0.0005, -0.0060, 0.0029))    # reed relay
    for y in (-0.0095, -0.0025):
        f.put("battery", at(mk.box(0.0024, 0.0009, 0.0007), px - 0.003, y, 0.0020))                        # HV resistors
    f.put("hv", mk.tube(mk.smooth([(px - 0.006, 0.006, 0.0015), (px - 0.0085, 0.014, 0.0015), (-0.0285, 0.0215, -0.0004)]), 0.0009, 5))
    f.put("rubber", mk.tube(mk.smooth([(px - 0.006, -0.006, 0.0015), (px - 0.0085, -0.014, 0.0015), (-0.0285, -0.0215, -0.0004)]), 0.0009, 5))
    f.put("hv", mk.tube(mk.smooth([(px + 0.005, 0.0, 0.0030), (-0.010, 0.0, 0.0085), (-0.004, 0.0, 0.0105), (0.0005, 0.0, 0.0110)]), 0.0013, 6))
    return f


# -- textures ----------------------------------------------------------------
def tex_pad():
    """Flex-PCB electrode pad seen from above: polyimide with two interdigitated copper combs."""
    W, H = 720, 600                                # 60 x 50 mm, 12 px / mm
    im = Image.new("RGB", (W, H), (214, 142, 36))
    d = ImageDraw.Draw(im)
    cu = (176, 96, 34)
    d.rounded_rectangle((14, 14, W - 14, 46), 8, fill=cu)          # bus bars
    d.rounded_rectangle((14, H - 46, W - 14, H - 14), 8, fill=cu)
    pitch, wdt = 36, 15
    for k, x in enumerate(range(34, W - 40, pitch)):
        if k % 2 == 0:
            d.rounded_rectangle((x, 30, x + wdt, H - 70), 7, fill=cu)
        else:
            d.rounded_rectangle((x, 70, x + wdt, H - 30), 7, fill=cu)
    for xy in ((0, 0), (0, H - 60)):                               # solder tabs
        d.rounded_rectangle((xy[0] + 6, xy[1] + 6, xy[0] + 60, xy[1] + 54), 6, fill=(226, 190, 96))
    d.text((W - 190, H - 42), "EA-PAD v1  3 kV", fill=(250, 226, 170), font=font(17))
    for x, y in ((W - 26, 26), (W - 26, H - 26)):
        d.ellipse((x - 9, y - 9, x + 9, y + 9), outline=(250, 226, 170), width=2)
    im.putpixel((0, 0), (190, 120, 30))
    im.save(ASSETS / "pad.png")


def tex_brain():
    """Brain PCB: compute module, CAN, IMU, silk-screen labels."""
    W, H = 900, 600                                # 90 x 60 mm, 10 px / mm
    im = Image.new("RGB", (W, H), (14, 86, 54))
    d = ImageDraw.Draw(im)
    rng = np.random.default_rng(3)
    for _ in range(140):                           # traces
        x, y = int(rng.integers(30, W - 30)), int(rng.integers(30, H - 30))
        pts = [(x, y)]
        for _ in range(3):
            if rng.random() < 0.5:
                x = int(np.clip(x + rng.integers(-150, 150), 20, W - 20))
            else:
                y = int(np.clip(y + rng.integers(-110, 110), 20, H - 20))
            pts.append((x, y))
        d.line(pts, fill=(24, 118, 74), width=3)
        d.ellipse((x - 4, y - 4, x + 4, y + 4), fill=(206, 172, 88))
    silk, chip, gold = (232, 236, 228), (18, 20, 22), (206, 172, 88)
    d.rounded_rectangle((8, 8, W - 8, H - 8), 26, outline=silk, width=3)
    for x, y in ((40, 40), (W - 40, 40), (40, H - 40), (W - 40, H - 40)):
        d.ellipse((x - 20, y - 20, x + 20, y + 20), fill=gold)
        d.ellipse((x - 10, y - 10, x + 10, y + 10), fill=(10, 30, 20))

    def part(box, name, pins=True, f=15):
        x0, y0, x1, y1 = box
        if pins:
            for x in range(x0 + 6, x1 - 4, 12):
                d.rectangle((x, y0 - 7, x + 5, y1 + 7), fill=gold)
        d.rounded_rectangle(box, 5, fill=chip)
        d.text((x0 + 6, y0 + 5), name, fill=(150, 156, 160), font=font(f))

    part((520, 250, 720, 450), "CPU", f=26)
    part((330, 150, 450, 240), "RAM")
    part((330, 420, 470, 500), "FLASH")
    part((190, 110, 290, 160), "CAN FL")
    part((190, 190, 290, 240), "CAN FR")
    part((190, 380, 290, 430), "CAN HL")
    part((190, 460, 290, 510), "CAN HR")
    part((700, 60, 800, 120), "IMU", pins=False)
    part((560, 80, 660, 150), "PMIC")
    d.text((330, 40), "ELECTROCLIMB  BRAIN v1", fill=silk, font=font(30))
    d.text((330, 78), "RL policy  /  vision  /  CAN x4", fill=silk, font=font(18))
    d.text((60, 280), "LEGS", fill=silk, font=font(18))
    d.text((760, 520), "HV LOCK", fill=silk, font=font(16))
    d.text((80, 540), "5V", fill=silk, font=font(16))
    for x in range(120, 880, 24):                  # header pads along the top edge
        d.rectangle((x, H - 52, x + 12, H - 34), fill=gold)
    im.putpixel((0, 0), (14, 86, 54))
    im.save(ASSETS / "brain.png")


def tex_logo():
    W, H = 832, 240                                # 52 x 15 mm, 16 px / mm
    im = Image.new("RGB", (W, H), (18, 20, 24))
    d = ImageDraw.Draw(im)
    d.rounded_rectangle((6, 6, W - 6, H - 6), 24, outline=(250, 140, 26), width=6)
    d.polygon([(52, 178), (112, 62), (172, 178)], fill=(250, 190, 40))
    d.polygon([(118, 92), (98, 132), (114, 132), (104, 166), (130, 120), (114, 120)], fill=(18, 20, 24))
    d.text((200, 54), "ELECTROCLIMB", fill=(245, 246, 248), font=font(74))
    d.text((204, 146), "HIGH VOLTAGE 3 kV   EC-01", fill=(250, 140, 26), font=font(34))
    im.putpixel((0, 0), (18, 20, 24))
    im.save(ASSETS / "logo.png")


def tex_concrete():
    """Cast concrete: mottled grey, formwork joints every 20 cm, tie holes (one tile = 40 cm)."""
    S = 1024
    rng = np.random.default_rng(7)
    img = np.zeros((S, S))
    for scale, amp in ((4, 10), (16, 7), (64, 5), (256, 4), (1024, 5)):
        n = rng.normal(0, 1, (scale, scale))
        n = np.asarray(Image.fromarray(np.tile(n, (2, 2))).resize((2 * S, 2 * S), Image.BICUBIC))[:S, :S]   # tiles seamlessly
        img += amp * n
    img = 126 + img
    im = Image.fromarray(np.clip(img, 0, 255).astype(np.uint8)).convert("RGB")
    d = ImageDraw.Draw(im)
    for p in (0, S // 2):
        d.line((p, 0, p, S), fill=(92, 93, 95), width=3)
        d.line((0, p, S, p), fill=(92, 93, 95), width=3)
        d.line((p + 3, 0, p + 3, S), fill=(146, 147, 148), width=1)
        d.line((0, p + 3, S, p + 3), fill=(146, 147, 148), width=1)
    for x in (S // 4, 3 * S // 4):
        for y in (S // 4, 3 * S // 4):
            d.ellipse((x - 9, y - 9, x + 9, y + 9), fill=(84, 85, 87))
            d.ellipse((x - 5, y - 6, x + 5, y + 4), fill=(66, 67, 69))
    for _ in range(260):                           # air pores
        x, y, r = rng.integers(0, S), rng.integers(0, S), rng.integers(1, 4)
        d.ellipse((x - r, y - r, x + r, y + r), fill=(96, 97, 99))
    im.filter(ImageFilter.GaussianBlur(0.6)).save(ASSETS / "concrete.png")


# -- model file --------------------------------------------------------------
MATERIALS = '''
    <material name="shell" rgba="0.93 0.94 0.95 1" specular="0.5" shininess="0.6"/>
    <material name="graphite" rgba="0.15 0.16 0.18 1" specular="0.45" shininess="0.5"/>
    <material name="carbon" rgba="0.07 0.07 0.08 1" specular="0.7" shininess="0.8"/>
    <material name="alu" rgba="0.74 0.76 0.80 1" specular="0.85" shininess="0.8"/>
    <material name="gold" rgba="0.86 0.70 0.30 1" specular="0.8" shininess="0.8"/>
    <material name="accent" rgba="0.98 0.55 0.10 1" specular="0.4" shininess="0.5"/>
    <material name="red" rgba="0.86 0.10 0.10 1" specular="0.5" shininess="0.6"/>
    <material name="hv" rgba="0.85 0.12 0.10 1" specular="0.3" shininess="0.4"/>
    <material name="rubber" rgba="0.09 0.09 0.10 1" specular="0.05" shininess="0.1"/>
    <material name="pcb" rgba="0.06 0.34 0.21 1" specular="0.3" shininess="0.4"/>
    <material name="battery" rgba="0.10 0.24 0.52 1" specular="0.3" shininess="0.4"/>
    <material name="glass" rgba="0.05 0.07 0.10 1" specular="0.9" shininess="0.95"/>
    <material name="lens" rgba="0.12 0.32 0.62 1" specular="0.9" shininess="0.95"/>
    <material name="smoke" rgba="0.30 0.40 0.52 0.13" specular="1" shininess="0.98"/>
    <material name="lamp" rgba="1 0.98 0.9 1" emission="1"/>
    <material name="led" rgba="0.25 1.0 0.45 1" emission="1"/>
    <material name="brain" texture="brain" specular="0.3" shininess="0.4"/>
    <material name="logo" texture="logo" specular="0.2" shininess="0.3"/>
    <material name="flex" texture="pad" specular="0.5" shininess="0.6"/>'''


def geoms(body, parts, indent):
    return "".join(f'{indent}<geom class="vis" type="mesh" mesh="{body}_{m}" material="{m}"/>\n' for m in parts)


def write_model(bodies):
    meshes = "".join(f'    <mesh name="{b}_{m}" file="{b}_{m}.obj" inertia="shell"/>\n' for b, parts in bodies.items() for m in parts)
    legs = ""
    for name, sx, sy in LEGS:
        h, th, sh = f"hip_{'f' if sx > 0 else 'h'}{'l' if sy > 0 else 'r'}", f"thigh_{'l' if sy > 0 else 'r'}", f"shin_{'l' if sy > 0 else 'r'}"
        y = 0.02 * sy
        legs += f'''
      <body name="{name}_hip" pos="{0.1 * sx:g} {0.06 * sy:g} 0">
        <joint name="{name}_abd" axis="1 0 0" range="-30 30"/>
        <geom class="col" type="capsule" fromto="0 0 0 0 {y:g} 0" size="0.012" mass="0.04"/>
{geoms(h, bodies[h], "        ")}        <body name="{name}_thigh" pos="0 {y:g} 0">
          <joint name="{name}_hip" axis="0 1 0" range="-90 90"/>
          <geom class="col" type="capsule" fromto="0 0 0 0.04 0 -0.07" size="0.01" mass="0.04"/>
{geoms(th, bodies[th], "          ")}          <body name="{name}_shin" pos="0.04 0 -0.07">
            <joint name="{name}_knee" axis="0 1 0" range="0 150"/>
            <geom class="col" type="capsule" fromto="0 0 0 -0.03 0 -0.06" size="0.008" mass="0.03"/>
{geoms(sh, bodies[sh], "            ")}            <body name="{name}_foot" pos="-0.03 0 -0.06" euler="0 -34.4 0">
              <joint name="{name}_ank_x" class="ankle" axis="1 0 0"/>
              <joint name="{name}_ank_y" class="ankle" axis="0 1 0"/>
              <geom name="{name}_pad" class="pad" pos="0 0 -0.004"/>
              <geom name="{name}_flex" class="vis" type="mesh" mesh="flex" material="flex"/>
{geoms("foot", bodies["foot"], "              ")}              <geom name="{name}_led" class="vis" type="sphere" pos="-0.024 -0.0085 0.0022" size="0.0016" material="led"/>
            </body>
          </body>
        </body>
      </body>
'''
    xml = f'''<!--
ElectroClimb v0 simulation model. GENERATED by make_assets.py: edit that file, then re-run it.

The robot always stands on the plane "wall". Climbing is simulated by rotating gravity
(env.py): tilt 0 deg = floor, 90 deg = vertical wall (up = +x), 180 deg = ceiling.
12 actuated joints (hip abduction, hip flexion, knee per leg), 2 passive ankle joints per
foot so the pad lies flat, and one MuJoCo adhesion actuator per foot pad. Mass 1.46 kg.

Two kinds of geoms:
  class "col" (group 3, hidden): the simple shapes that carry all mass and collisions.
  class "vis" (group 1): the detailed meshes you see. No mass, no contacts, so the look
                         never changes the physics.
The pad geoms are both: they collide, stick and are drawn.
-->
<mujoco model="electroclimb_v0">
  <compiler angle="degree" autolimits="true" meshdir="assets" texturedir="assets"/>
  <option timestep="0.002" gravity="0 0 -9.81" cone="elliptic" impratio="10"/>

  <!-- small scene extent: keeps the shadow map sharp around the robot (rendering only) -->
  <statistic extent="0.7" center="0.1 0 0.05"/>

  <visual>
    <global offwidth="1600" offheight="1600"/>
    <quality shadowsize="8192" offsamples="8"/>
    <headlight ambient="0.36 0.36 0.38" diffuse="0.30 0.30 0.30" specular="0.05 0.05 0.05"/>
    <map znear="0.01"/>
  </visual>

  <default>
    <joint damping="0.05" armature="0.005"/>
    <geom friction="0.6 0.02 0.002" condim="4" solref="0.005 1"/>
    <default class="servo">
      <position kp="25" kv="0.5" forcerange="-3 3"/>
    </default>
    <default class="ankle">
      <joint range="-20 20" damping="0.05" stiffness="6"/>
    </default>
    <default class="pad">
      <geom type="box" size="0.03 0.025 0.003" mass="0.03" rgba="0.98 0.62 0.12 1" margin="0.004" gap="0.004"/>
    </default>
    <default class="col">
      <geom group="3"/>
    </default>
    <default class="vis">
      <geom contype="0" conaffinity="0" mass="0" group="1"/>
    </default>
  </default>

  <asset>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.78 0.84 0.92" rgb2="0.95 0.96 0.98" width="512" height="512"/>
    <texture name="concrete" type="2d" file="concrete.png"/>
    <texture name="brain" type="2d" file="brain.png"/>
    <texture name="logo" type="2d" file="logo.png"/>
    <texture name="pad" type="2d" file="pad.png"/>
    <!-- one texture tile = 40 cm of wall: formwork joints every 20 cm show the climbing progress -->
    <material name="concrete" texture="concrete" texrepeat="15 15" reflectance="0" specular="0.05" shininess="0.1"/>{MATERIALS}
    <mesh name="flex" file="flex.obj" inertia="shell"/>
{meshes}  </asset>

  <worldbody>
    <light directional="true" pos="0.6 -0.8 1.5" dir="-0.28 0.36 -1" castshadow="true" diffuse="0.62 0.62 0.60" specular="0.3 0.3 0.3"/>
    <geom name="wall" type="plane" size="3 3 0.1" material="concrete"/>

    <body name="torso" pos="0 0 0.14">
      <freejoint/>
      <geom class="col" type="box" size="0.1 0.06 0.02" mass="0.9"/>
      <site name="imu"/>
{geoms("torso", bodies["torso"], "      ")}      <geom name="status_led" class="vis" type="sphere" pos="0.046 -0.024 0.0192" size="0.0014" material="led"/>
{legs}    </body>
  </worldbody>

  <actuator>
    <position class="servo" name="fl_abd" joint="fl_abd"/> <position class="servo" name="fl_hip" joint="fl_hip"/> <position class="servo" name="fl_knee" joint="fl_knee"/>
    <position class="servo" name="fr_abd" joint="fr_abd"/> <position class="servo" name="fr_hip" joint="fr_hip"/> <position class="servo" name="fr_knee" joint="fr_knee"/>
    <position class="servo" name="hl_abd" joint="hl_abd"/> <position class="servo" name="hl_hip" joint="hl_hip"/> <position class="servo" name="hl_knee" joint="hl_knee"/>
    <position class="servo" name="hr_abd" joint="hr_abd"/> <position class="servo" name="hr_hip" joint="hr_hip"/> <position class="servo" name="hr_knee" joint="hr_knee"/>
    <!-- electroadhesion: ctrl 0..1 = pad voltage 0..100 %, gain = max normal pull in N (scaled per episode by env.py) -->
    <adhesion name="fl_adh" body="fl_foot" ctrlrange="0 1" gain="15"/>
    <adhesion name="fr_adh" body="fr_foot" ctrlrange="0 1" gain="15"/>
    <adhesion name="hl_adh" body="hl_foot" ctrlrange="0 1" gain="15"/>
    <adhesion name="hr_adh" body="hr_foot" ctrlrange="0 1" gain="15"/>
  </actuator>

  <sensor>
    <framequat name="imu_quat" objtype="site" objname="imu"/>
    <gyro name="imu_gyro" site="imu"/>
    <accelerometer name="imu_acc" site="imu"/>
  </sensor>
</mujoco>
'''
    (HERE / "electroclimb.xml").write_text(xml)


def main():
    ASSETS.mkdir(exist_ok=True)
    for old in ASSETS.glob("*.obj"):
        old.unlink()
    bodies = {"torso": torso(), "foot": foot()}
    for sx, a in ((1, "f"), (-1, "h")):
        for sy, b in ((1, "l"), (-1, "r")):
            bodies[f"hip_{a}{b}"] = hip(sx, sy)
    for sy, b in ((1, "l"), (-1, "r")):
        bodies[f"thigh_{b}"], bodies[f"shin_{b}"] = thigh(sy), shin(sy)
    tris = 0
    for body, parts in bodies.items():
        for material, mesh in parts.items():
            tris += mesh.write(ASSETS / f"{body}_{material}.obj")
    tris += mk.slab_uv(0.03, 0.025, -0.001, -0.0005).write(ASSETS / "flex.obj")
    tex_pad(), tex_brain(), tex_logo(), tex_concrete()
    write_model(bodies)
    size = sum(f.stat().st_size for f in ASSETS.iterdir()) / 1e6
    print(f"{sum(len(p) for p in bodies.values()) + 1} meshes, {tris:,} triangles, 4 textures, {size:.1f} MB -> {ASSETS}")


if __name__ == "__main__":
    main()
