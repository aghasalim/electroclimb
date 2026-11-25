#!/usr/bin/env python3
"""Parametric CAD for the ElectroClimb body and legs (CadQuery / OpenCascade).

    uv pip install --python ../../../.venv cadquery
    ../../../.venv/bin/python robot_cad.py     # checks, then writes STEP + STL for each part and the assembly

Dimensions in mm. Robot frame as in sim/electroclimb.xml: x forward (up the wall), y left,
z away from the wall, origin in the middle between the four hip abduction joints.

  chassis       base plate with the 4 abduction servos, Pi 5, battery, buck module, camera, tether eye
  abd_bracket   on the abduction servo's horn and idler, holds the hip servo body (left / right)
  thigh         two plates and a crossbar, on the hip servo's horn and idler and on the knee servo's
  shin_link     holds the knee servo body and clamps the 10 mm carbon shin tube (left / right)
  top_cover     thin cover over the electronics
  servo_dummy   simplified STS3215, only for the assembly and the collision checks

All four legs are the same (right = mirror image). Each leg: abduction servo in the chassis,
hip servo body in the bracket, knee servo body in the shin link, the thigh is a plain link.
Change the numbers in PARAMS and re-run: `check()` runs again.
"""
import math
import sys
from pathlib import Path

import cadquery as cq
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder

HERE = Path(__file__).resolve().parent
FOOT = HERE.parents[1] / "foot" / "cad"
sys.path.insert(0, str(FOOT))
import foot_cad  # noqa: E402  the existing foot: its PARAMS define where the shin tube sits

V = cq.Vector
STANCE = (math.degrees(-0.4), math.degrees(1.0))       # hip, knee in the simulation's stance
THIGH, SHIN = (40.0, 0.0, -70.0), (-30.0, 0.0, -60.0)   # link vectors in the zero pose (sim)

# Servo dimension sources:
#   [D] Feetech "STS3215 product specification" A/0, 2020-04-10, section 9 outline drawing
#   [S] measured in STEP/SO100/STS3215_03a.step of github.com/TheRobotStudio/SO-ARM100 (same servo)
PARAMS = dict(
    # ---- Feetech STS3215, servo frame: origin on the output axis at mid height, +z = horn side, body towards -x
    sv_l=45.23, sv_w=24.73,        # [D] body length and width ([S]: 45.4 x 24.8)
    sv_h=32.0,                     # [D] body height between the horn-side and rear faces ([S]: 31.8)
    sv_axis=12.5,                  # [D] and [S]: output axis from the middle of the body length
    sv_horn_z=18.7,                # [S] outer face of the output horn above mid height  # NOT VERIFIED on a real servo
    sv_idler_z=17.7,               # [S] outer face of the rear idler horn below mid height  # NOT VERIFIED on a real servo
    sv_horn_d=20.0,                # [S] horn and idler horn diameter (horn 2.5 thick, idler 2.1)
    sv_horn_pcd=14.0,              # [S] 4 x M3 thread in the horn at (+-4.95, +-4.95); same pattern in the idler horn
                                   #     # NOT VERIFIED that the idler horn holes are threaded M3
    sv_boss_d=6.0,                 # [D] rear idler axle, held by a PA3.0x5 self-tapping screw
    sv_m2_top=(-8.3, -29.0),       # [S] x of the 4 case screw holes on the horn face, y = +-sv_m2_y
    sv_m2_rear=(-8.3, -32.8),      # [S] same on the rear face
    sv_m2_y=10.25,                 # [S]  # NOT VERIFIED: [D] says "8-PA2.0" self-tapping screws but gives no positions,
                                   #      and it is not confirmed that all 8 holes are free for mounting
    sv_plug=(-14.05, 6.8, 19.0),   # [S] two 3-pin 5264 sockets on the rear face: x centre, window x and y size
                                   #     # NOT VERIFIED: direction the plugs leave in, and their height over the rear face
    sv_mass=55.0,                  # [D] 55 +- 1 g (7.4 V version)  # NOT VERIFIED for the 12 V version
    # ---- joints (sim/electroclimb.xml)
    hip_x=100.0, abd_y=60.0,       # abduction axes along x through (+-100, +-60, 0)
    hip_y=20.0,                    # leg plane (hip servo mid height) outboard of the abduction joint
    # ---- common
    arm_t=3.0,                     # plates bolted to horns
    ear_t=2.4,                     # servo holder plates; they fit between the servo face and the horn's outer face
    ear_over=1.5,                  # holder plate wider than the servo: wall around the M2 holes
    horn_clear=10.8,               # radius of the horn clearance in a holder plate
    boss_h=3.0,                    # idler-side spacer: leaves 2.6 mm for the servo cable over the rear face
    arm_r=11.0,                    # disc around a horn pattern: 2.4 mm wall outside the M3 holes
    m3=3.4, m2=2.3,                # clearance holes
    # ---- chassis
    abd_back=20.0,                 # abduction horn face this far behind the hip axis (thigh plate must pass it)
    plate_t=2.4, spine_hw=36.0, pad_hw=70.0,
    pi_x=-42.0, pi_holes=(58.0, 49.0),     # Raspberry Pi 5: 58 x 49 mm, M2.5
    batt_x=2.0, batt=(75.0, 35.0, 26.0),   # 3S LiPo strapped under the plate
    buck_x=20.0,
    cam_holes=(21.0, 12.5),        # Raspberry Pi Camera Module 3, M2
    tether_d=6.0, tether_wall=7.0, tether_t=8.0,
    cover_x=(-95.0, 42.0), cover_hw=33.2, cover_h=25.2, cover_wall=1.2, cover_boss=5.0,
    # ---- leg
    web_y=16.8,                    # bracket strip over the abduction servo starts here: clears the servo when it turns
    bar_u=35.5, bar_v=(10.0, 19.0), bar_w=6.0,   # thigh crossbar: off the centre line, where the shin never comes
    tube_d=10.0, tube_wall=3.0,    # carbon tube and the wall around it
    tube_stop=1.6,                 # end wall of the tube tunnel
    density=1.27,                  # PETG g/cm3 (PA-CF is about 1.2)
)


# ---------------------------------------------------------------- helpers

def box(x0, x1, y0, y1, z0, z1):
    (x0, x1), (y0, y1), (z0, z1) = sorted((x0, x1)), sorted((y0, y1)), sorted((z0, z1))
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def cyl(origin, direction, r, h):
    return cq.Workplane(cq.Plane(origin=tuple(origin), normal=tuple(direction))).circle(r).extrude(h)


def put(part, origin, x, z):
    """Move a part built in a local frame to where that frame's origin, x and z axes are given."""
    loc = cq.Plane(origin=tuple(origin), xDir=tuple(x), normal=tuple(z)).location
    return part.newObject([s.moved(loc) for s in part.vals()])


def yz(x):   # sketch in (y, z), extrude towards +x
    return cq.Workplane(cq.Plane(origin=(x, 0, 0), xDir=(0, 1, 0), normal=(1, 0, 0)))


def xz(y):   # sketch in (x, z), extrude towards -y
    return cq.Workplane(cq.Plane(origin=(0, y, 0), xDir=(1, 0, 0), normal=(0, -1, 0)))


def horn_holes(p, origin, direction, length, centre_d):
    """4 x M3 clearance on the horn circle plus a centre hole, as one cutting tool."""
    plane = cq.Plane(origin=tuple(origin), normal=tuple(direction))
    k = p["sv_horn_pcd"] / 2 / math.sqrt(2)
    return (cq.Workplane(plane).pushPoints([(k, k), (k, -k), (-k, k), (-k, -k)]).circle(p["m3"] / 2).extrude(length)
            .union(cq.Workplane(plane).circle(centre_d / 2).extrude(length)))


def axes(part, d):
    """(point, direction) of every cylindrical face of diameter d: where the holes really are."""
    out = []
    for f in part.faces().vals():
        s = BRepAdaptor_Surface(f.wrapped)
        if s.GetType() == GeomAbs_Cylinder and abs(2 * s.Cylinder().Radius() - d) < 1e-3:
            ax = s.Cylinder().Axis()
            bb = f.BoundingBox()
            out.append((V(ax.Location()), V(ax.Direction()), bb))
    return out


# ---------------------------------------------------------------- servo

def servo_dummy(p=PARAMS):
    tail, nose = p["sv_l"] / 2 + p["sv_axis"], p["sv_l"] / 2 - p["sv_axis"]
    h, r = p["sv_h"] / 2, p["sv_horn_d"] / 2
    body = box(-tail, nose, -p["sv_w"] / 2, p["sv_w"] / 2, -h, h)
    horn = cyl((0, 0, h), (0, 0, 1), r, p["sv_horn_z"] - h)
    idler = cyl((0, 0, -p["sv_idler_z"]), (0, 0, 1), r, p["sv_idler_z"] - h)
    return body.union(horn).union(idler)      # ponytail: no spline, screws, plugs or cables


def ear(p, rear, x0, y0, y1, x1=-5.0):
    """Servo holder plate in the servo frame: lies on the horn face (or the rear face), is fastened with
    4 x M2 self-tapping screws into the case, and leaves the horn (or idler) free. Outline x0..x1, y0..y1."""
    h, t = p["sv_h"] / 2, p["ear_t"]
    z = -h - t if rear else h
    e = box(x0, x1, y0, y1, z, z + t).cut(cyl((0, 0, z), (0, 0, 1), p["horn_clear"], t))
    pts = [(x, s * p["sv_m2_y"]) for x in (p["sv_m2_rear"] if rear else p["sv_m2_top"]) for s in (1, -1)]
    e = e.cut(cq.Workplane("XY").workplane(offset=z).pushPoints(pts).circle(p["m2"] / 2).extrude(t))
    cx, wx, wy = p["sv_plug"]                  # window over the two cable sockets; same cut-out on the horn side
    return e.cut(box(cx - wx / 2, cx + wx / 2, -wy / 2, wy / 2, z, z + t))


# ---------------------------------------------------------------- parts

def chassis(p=PARAMS):
    zt = -p["sv_w"] / 2 - 0.05                 # plate top: the abduction servos lie on it
    zb = zt - p["plate_t"]
    half, over = p["sv_h"] / 2 + p["ear_t"], p["sv_w"] / 2 + p["ear_over"]
    tail = p["sv_l"] / 2 + p["sv_axis"]
    xs = [h - p["abd_back"] - p["sv_horn_z"] for h in (p["hip_x"], -p["hip_x"])]   # servo origins, front and hind
    xr, xf = xs[1] - half, xs[0] + half
    sw, y_in = p["spine_hw"], p["abd_y"] - tail - 3.0
    plate = box(xr, xf, -sw, sw, zb, zt)
    side = None                                # everything on the left side, mirrored below
    for x in xs:
        pad = box(x - half, x + half, y_in, p["pad_hw"], zb, zt).cut(box(x - 11, x + 11, y_in + 6, y_in + 30, zb, zt))
        walls = put(ear(p, False, -tail - 3.0, zt, over).union(ear(p, True, -tail - 3.0, zt, over)),
                    (x, p["abd_y"], 0), (0, 1, 0), (1, 0, 0))
        tie = box(x - half, x + half, y_in, y_in + p["ear_t"], zt, 0)       # closes the pocket behind the servo
        side = pad.union(walls).union(tie) if side is None else side.union(pad).union(walls).union(tie)
    side = side.union(box(xs[1] + half, xs[0] - half, sw - 2.4, sw, zt, zt + 6))   # edge rib
    body = plate.union(side).union(side.mirror("XZ"))
    # tether eye, rear centre: 6 mm hole with 7 mm of material all round, 8 mm thick
    te = p["tether_d"] / 2 + p["tether_wall"]
    body = body.union(box(xr, xr + 2 * te, -te, te, zb, zb + p["tether_t"]).edges("|Z").fillet(3.0))
    body = body.cut(cyl((xr + te, 0, zb), (0, 0, 1), p["tether_d"] / 2, p["tether_t"]))
    # camera plate at the front, between the front servos
    cw, ch = p["cam_holes"]
    zc = zt + 16.0
    body = body.union(box(xf - 2.4, xf, -cw / 2 - 3.5, cw / 2 + 3.5, zt, zc + ch / 2 + 3.5))
    body = body.cut(cq.Workplane(cq.Plane(origin=(xf - 2.4, 0, zc), xDir=(0, 1, 0), normal=(1, 0, 0)))
                    .rect(cw, ch, forConstruction=True).vertices().circle(1.1).extrude(2.4))
    body = body.cut(box(xf - 2.4, xf, -8.5, 8.5, zc - ch / 2 - 5.5, zc - ch / 2 - 2.5))     # ribbon cable slot
    # Pi 5 standoffs (M2.5 heat-set inserts) and cover bosses (M3 heat-set inserts)
    px, py = p["pi_holes"]
    pi = cq.Workplane("XY").workplane(offset=zt).center(p["pi_x"], 0).rect(px, py, forConstruction=True).vertices()
    body = body.union(pi.circle(3.5).extrude(5.0)).cut(pi.circle(1.6).extrude(5.0))
    cv = cq.Workplane("XY").workplane(offset=zt).pushPoints(cover_points(p))
    body = body.union(cv.circle(4.0).extrude(p["cover_boss"])).cut(cv.circle(2.0).extrude(p["cover_boss"]))
    # strap slots: battery (under the plate) and buck module (on it); lightening windows
    bx, bl, bw = p["batt_x"], p["batt"][0], p["batt"][1]
    cuts = [box(bx + sx * 20 - 8, bx + sx * 20 + 8, sy * (bw / 2 + 1), sy * (bw / 2 + 4), zb, zt)
            for sx in (1, -1) for sy in (1, -1)]
    cuts += [box(p["buck_x"] + sx * 10 - 5, p["buck_x"] + sx * 10 + 5, sy * 13, sy * 16, zb, zt)
             for sx in (1, -1) for sy in (1, -1)]
    cuts += [box(-117, -99, -26, 26, zb, zt), box(-91, -58, -18, 18, zb, zt), box(-52, -24, -18, 18, zb, zt),
             box(bx - 20, bx + 32, -11, 11, zb, zt)]
    for c in cuts:
        body = body.cut(c)
    assert bx - bl / 2 > xs[1] + half and bx + bl / 2 < xf, "battery must fit under the plate"
    return body


def cover_points(p=PARAMS):
    (x0, x1), y = p["cover_x"], p["cover_hw"] - p["cover_wall"] - 4.2
    return [(x0 + 5.6, y), (x0 + 5.6, -y), (x1 - 5.6, y), (x1 - 5.6, -y)]


def top_cover(p=PARAMS):
    zt, w, hw, H = -p["sv_w"] / 2 - 0.05, p["cover_wall"], p["cover_hw"], p["cover_h"]
    (x0, x1), zb = p["cover_x"], -p["sv_w"] / 2 - 0.05 + p["cover_boss"]
    c = box(x0, x1, -hw, hw, zt, zt + H).cut(box(x0 + w, x1 - w, -hw + w, hw - w, zt, zt + H - w))
    pts = lambda z: cq.Workplane("XY").workplane(offset=z).pushPoints(cover_points(p))
    c = c.union(pts(zb).circle(4.4).extrude(zt + H - zb)).cut(pts(zb).circle(p["m3"] / 2).extrude(H))
    c = c.cut(pts(zb + 3.0).circle(3.25).extrude(H))                         # screw head goes down the post
    for x in range(-78, 0, 8):                                              # ventilation over the Pi
        c = c.cut(box(x - 1.25, x + 1.25, -20, 20, zt + H - w, zt + H))
    for x in (x0, x1 - w):                                                  # cable notches, front and rear
        c = c.cut(box(x, x + w, -9, 9, zt, zt + 8))
    return c.cut(box(-40, -20, -hw, hw, zt, zt + 6))                        # and one on each side


def abd_bracket(p=PARAMS):
    """Left bracket in the bracket frame: origin on the abduction axis at the hip's x, robot axes."""
    t, et, hy = p["arm_t"], p["ear_t"], p["hip_y"]
    xa = -p["abd_back"]                                         # horn face of the abduction servo
    xi = xa - p["sv_horn_z"] - p["sv_idler_z"] - p["boss_h"]    # inner face of the idler arm
    half, over = p["sv_h"] / 2 + et, p["sv_w"] / 2 + p["ear_over"]
    y0, y1 = hy - half, hy + half
    top0 = p["sv_l"] / 2 + p["sv_axis"] + 0.5                   # hip servo points up; roof just above its tail
    top1 = top0 + 2.4
    r = p["arm_r"]
    # horn arm: disc on the horn, wall behind the hip servo
    arm = yz(xa).circle(r).extrude(t).union(box(xa, xa + t, -2.0, y1, -r, top1))
    arm = arm.cut(box(xa, xa + t, 9.0, y1 - 7.0, -3.0, top0 - 8.0))
    # idler arm: disc with a spacer boss on the idler horn, band up to the roof
    idl = yz(xi - t).circle(r).extrude(t).union(
        yz(xi - t).polyline([(-9, 6), (9, -6), (y1, top0 - 10), (y1, top1), (p["web_y"], top1)]).close().extrude(t))
    idl = idl.union(yz(xi).circle(p["sv_horn_d"] / 2).extrude(p["boss_h"]))
    # roof over the hip servo, strip and rib back to the idler arm
    roof = box(xa, over, y0, y1, top0, top1).union(box(xi - t, xa, p["web_y"], y1, top0, top1))
    roof = roof.union(box(xi - t, xa, y1 - et, y1, top0 - 12.0, top1))
    # hip servo holder: axis along y, body pointing up (+z)
    ears = ear(p, True, -top1, -over, -xa).union(ear(p, False, -top1, -over, -xa))
    ears = put(ears, (0, hy, 0), (0, 0, -1), (0, 1, 0))
    low = box(xa, -p["sv_w"] / 2 + 0.85, y0, y0 + et, -r, 5.0)
    b = arm.union(idl).union(roof).union(ears).union(low)
    corner = lambda y: cq.selectors.BoxSelector((xa + t - 0.1, y - 0.1, -50), (xa + t + 0.1, y + 0.1, 50))
    b = b.edges(corner(y0 + et) + corner(y1 - et)).fillet(2.5)       # where the holder plates meet the arm
    b = b.cut(horn_holes(p, (xa, 0, 0), (1, 0, 0), t, 8.0))
    b = b.cut(horn_holes(p, (xi - t, 0, 0), (1, 0, 0), t + p["boss_h"], p["sv_boss_d"] + 1.0))
    return b


def thigh(p=PARAMS):
    """Thigh frame: x from the hip axis to the knee axis, y lateral (+ = horn side), z across."""
    L, r, t = math.hypot(THIGH[0], THIGH[2]), p["arm_r"], p["arm_t"]
    yo, yi = p["sv_horn_z"], -p["sv_idler_z"] - p["boss_h"]      # inner faces of the outer and inner plate
    u, (v0, v1), bw = p["bar_u"], p["bar_v"], p["bar_w"]
    hump = [(u - bw / 2 - v1, 0), (u - bw / 2 - 2, v1), (u + bw / 2 + 2, v1), (u + bw / 2 + v1, 0)]

    def plate(y_outer):
        return (xz(y_outer).center(L / 2, 0).slot2D(L + 2 * r, 2 * r).extrude(t)
                .union(xz(y_outer).polyline(hump).close().extrude(t)))
    inner = plate(yi).cut(xz(yi).center(L / 2 + 6, 0).slot2D(34, 9).extrude(t))
    body = plate(yo + t).union(inner).union(box(u - bw / 2, u + bw / 2, yi, yo, v0, v1))
    for x in (0, L):                                             # spacer bosses on the idler side
        body = body.union(cyl((x, yi, 0), (0, 1, 0), p["sv_horn_d"] / 2, p["boss_h"]))
    sel = cq.selectors.BoxSelector((u - bw / 2 - 0.1, yi - 0.1, v0 - 0.1), (u + bw / 2 + 0.1, yo + 0.1, v1 - 0.5))
    body = body.edges(sel).edges("|Z").fillet(2.0)
    for x in (0, L):
        body = body.cut(horn_holes(p, (x, yi - t, 0), (0, 1, 0), yo + t - yi + t, 8.0))
    # channel for the red high-voltage pair on the outside: two ribs with a 45 deg underside (print edge down)
    ya = yo + t
    for v in (-4.2, 3.6):
        rib = (cq.Workplane(cq.Plane(origin=(56, 0, 0), xDir=(0, 0, 1), normal=(-1, 0, 0)))
               .polyline([(v - 3, ya), (v, ya + 3), (v + 1.2, ya + 3), (v + 1.2, ya)]).close().extrude(32))
        body = body.union(rib)
    return body


def foot_frame(p=PARAMS):
    """Where the existing foot (hardware/foot/cad/foot_cad.py) sits in the shin frame (x knee -> ankle,
    z = -tube side): the shin adapter's tube axis is fixed to the pad, and the ankle is the flexure neck."""
    f = foot_cad.PARAMS
    a = math.radians(f["tube_tilt_deg"])
    z_top = f["plate_t"] + 2.0 + f["neck_len"] + (f["flange_d"] - f["neck_d"]) / 2 + f["flange_t"]   # as in carrier()
    z_ank = f["plate_t"] + 2.0 + f["neck_len"] / 2                # middle of the flexure neck
    xf, zf = V(-math.sin(a), 0, math.cos(a)), V(-math.cos(a), 0, -math.sin(a))   # pad x and pad normal
    origin = V(math.hypot(SHIN[0], SHIN[2]), 0, 0) - zf * z_ank
    p0 = origin + xf * 9.0 + zf * (z_top + f["base_t"] + 1.0)     # socket base point, hard-coded in shin_adapter()
    x_bot = p0.x - (f["socket_len"] - f["tube_depth"])            # lower end of the tube
    x_stop = -(p["sv_l"] / 2 + p["sv_axis"]) - 0.3 + p["tube_stop"]
    return dict(origin=origin, xf=xf, zf=zf, z_top=z_top, off=-p0.z, x_mouth=p0.x - f["socket_len"],
                x_bot=x_bot, x_stop=x_stop, tube_len=x_bot - x_stop)


def shin_link(p=PARAMS):
    """Left shin link in the shin frame: x from the knee axis to the ankle, y lateral (+ = horn side).
    The knee servo lies in it, tail pointing away from the foot; the tube runs along its side."""
    ff = foot_frame(p)
    et, over, hw = p["ear_t"], p["sv_w"] / 2 + p["ear_over"], p["sv_h"] / 2 + p["ear_t"]
    tail, nose = p["sv_l"] / 2 + p["sv_axis"] + 0.3, p["sv_l"] / 2 - p["sv_axis"]
    zf0 = -p["sv_w"] / 2 - 0.4                                   # floor on the servo's side face
    off, ro, rb = ff["off"], p["tube_d"] / 2 + p["tube_wall"], p["tube_d"] / 2 + 0.1
    xm = ff["x_mouth"] - 0.5                                     # stop short of the foot adapter's socket
    ears = put(ear(p, False, -tail, -over, -zf0 + et).union(ear(p, True, -tail, -over, -zf0 + et)),
               (0, 0, 0), (1, 0, 0), (0, 1, 0))
    floor = box(-tail, nose, -hw, hw, zf0 - et, zf0)
    tunnel = cyl((-tail, 0, -off), (1, 0, 0), ro, xm + tail).cut(box(-tail, nose + 0.4, -ro, ro, zf0, 0))
    lug = (cq.Workplane(cq.Plane(origin=(0, 7, 0), xDir=(1, 0, 0), normal=(0, -1, 0)))
           .polyline([(xm - 17, -off - ro + 0.5), (xm - 8, -off - 17), (xm, -off - 17), (xm, -off - ro + 0.5)])
           .close().extrude(14))
    b = ears.union(floor).union(tunnel).union(lug)
    b = b.cut(cyl((ff["x_stop"], 0, -off), (1, 0, 0), rb, xm - ff["x_stop"]))            # tube bore
    b = b.cut(box(-2.0, xm, -0.75, 0.75, -off - 18, -off))                              # pinch slit
    pin = (xm - 4.0, -off - 12.5)
    b = b.cut(cyl((pin[0], -8, pin[1]), (0, 1, 0), p["m3"] / 2, 16))                    # M3 pinch screw
    b = b.cut(cq.Workplane(cq.Plane(origin=(pin[0], 4, pin[1]), xDir=(1, 0, 0), normal=(0, 1, 0)))
              .polygon(6, 6.4).extrude(4))                                              # M3 nut trap
    for z in (2.5, 9.4):                                         # channel for the high-voltage pair, outside
        b = b.union(box(-27, -14, hw, hw + 3.5, z, z + 1.6))
    return b


# ---------------------------------------------------------------- assembly

def build(p=PARAMS):
    ff = foot_frame(p)
    tube = cyl((ff["x_stop"], 0, -ff["off"]), (1, 0, 0), p["tube_d"] / 2, ff["tube_len"]).cut(
        cyl((ff["x_stop"], 0, -ff["off"]), (1, 0, 0), p["tube_d"] / 2 - 1.0, ff["tube_len"]))
    foot = lambda name: cq.importers.importStep(str(FOOT / f"{name}.step"))
    P = dict(chassis=chassis(p), top_cover=top_cover(p), abd_bracket=abd_bracket(p), thigh=thigh(p),
             shin_link=shin_link(p), servo=servo_dummy(p), tube=tube, ff=ff,
             carrier=put(foot("foot_carrier_tpu"), ff["origin"], ff["xf"], ff["zf"]),
             adapter=put(foot("shin_adapter_petg"), ff["origin"] + ff["zf"] * ff["z_top"], ff["xf"], ff["zf"]))
    return relief(P, p)


HIP_MAX = 60        # deg. The wall-to-ceiling corner needs the hind hips at 60 (sim/corner.py LIMITS_CAD)
KNEES = (0, 20, 40, 57)
CORNER_POSES = dict(      # (hip, knee) in degrees that the corner plan passes through, sampled
    front=[(h, k) for h in (-83, -60, -30, 0, 30) for k in KNEES] + [(40, 10), (45, 20), (53, 0)],
    hind=[(h, k) for h in (-23, 0, 30, 50, 56, HIP_MAX) for k in KNEES])


def relief(P, p=PARAMS):
    """Trim the thigh and the hip bracket where the leg touches between 55 and HIP_MAX degrees of hip.

    Untrimmed, the leg is free up to 55 deg. Above that a corner of the thigh plate runs into the
    bracket and the chassis, and the folded foot adapter into the bracket: a few mm3 each. The cuts
    are swept 2 deg past HIP_MAX for clearance."""
    O, Y, hy = (0, 0, 0), (0, 1, 0), p["hip_y"]
    bt = math.degrees(math.atan2(-THIGH[2], THIGH[0]))
    thigh0, bracket0 = P["thigh"], P["abd_bracket"]
    thigh_v, bracket_v = thigh0.val().Volume(), bracket0.val().Volume()
    thigh_new, bracket_new = thigh0, bracket0
    for front in (True, False):
        j0 = V(p["hip_x"] if front else -p["hip_x"], p["abd_y"], 0)
        for hip in range(56, HIP_MAX + 3, 2):
            for fixed in (P["chassis"].translate((-j0).toTuple()), bracket0):      # into the thigh's own frame
                thigh_new = thigh_new.cut(fixed.translate((0, -hy, 0)).rotate(O, Y, -(bt + hip)))
            for knee in (46, 50, 54, 58):
                for name, solid, _ in leg(P, "x", front, True, 0.0, hip, knee, p):
                    if name in ("x_foot_adapter", "x_foot_carrier"):
                        bracket_new = bracket_new.cut(solid.translate((-j0).toTuple()))
    # the folded foot adapter also grazes the chassis edge from 50 deg on (knee near 57)
    chassis_new, chassis_v = P["chassis"], P["chassis"].val().Volume()
    for front in (True, False):
        for hip in range(48, HIP_MAX + 3, 2):
            for knee in (54, 58):
                for name, solid, _ in leg(P, "x", front, True, 0.0, hip, knee, p):
                    if name == "x_foot_adapter":
                        chassis_new = chassis_new.cut(solid).cut(solid.mirror("XZ"))
    P = dict(P, thigh=thigh_new, abd_bracket=bracket_new, chassis=chassis_new)
    P["relief"] = (thigh_v - thigh_new.val().Volume(), bracket_v - bracket_new.val().Volume(),
                   chassis_v - chassis_new.val().Volume())
    return P


WHITE, DARK, SERVO, CARBON, TPU = (0.92, 0.93, 0.95), (0.2, 0.21, 0.23), (0.1, 0.1, 0.11), (0.05, 0.05, 0.06), (0.13, 0.13, 0.15)
GREY, ACCENT = (0.62, 0.65, 0.70), (0.98, 0.55, 0.10)


def leg(P, name, front, left, abd=0.0, hip=STANCE[0], knee=STANCE[1], p=PARAMS):
    """[(name, solid, colour)] of one leg in the robot frame. Angles in degrees with the simulation's
    signs for a left leg; a right leg is the mirror image."""
    O, Y = (0, 0, 0), (0, 1, 0)
    j0 = V(p["hip_x"] if front else -p["hip_x"], p["abd_y"], 0)
    h = math.radians(hip)
    knee_pt = (THIGH[0] * math.cos(h) + THIGH[2] * math.sin(h), p["hip_y"], -THIGH[0] * math.sin(h) + THIGH[2] * math.cos(h))
    bt = math.degrees(math.atan2(-THIGH[2], THIGH[0]))          # thigh frame x -> zero-pose thigh direction
    bs = math.degrees(math.atan2(-SHIN[2], SHIN[0]))

    def B(part):                                                # bracket frame -> robot
        return part.rotate(O, (1, 0, 0), abd).translate(j0)

    def S(part):                                                # shin frame -> robot
        return B(part.rotate(O, Y, bs + hip + knee).translate(knee_pt))
    items = [
        ("abd_servo", put(P["servo"], j0 - V(p["abd_back"] + p["sv_horn_z"], 0, 0), (0, 1, 0), (1, 0, 0)), SERVO),
        ("abd_bracket", B(P["abd_bracket"]), WHITE),
        ("hip_servo", B(put(P["servo"], (0, p["hip_y"], 0), (0, 0, -1), (0, 1, 0))), SERVO),
        ("thigh", B(P["thigh"].rotate(O, Y, bt + hip).translate((0, p["hip_y"], 0))), GREY),
        ("shin_link", S(P["shin_link"]), ACCENT),
        ("knee_servo", S(put(P["servo"], O, (1, 0, 0), (0, 1, 0))), SERVO),
        ("tube", S(P["tube"]), CARBON),
        ("foot_adapter", S(P["adapter"]), WHITE),
        ("foot_carrier", S(P["carrier"]), TPU),
    ]
    return [(f"{name}_{n}", s if left else s.mirror("XZ"), c) for n, s, c in items]


LEGS = (("fl", True, True), ("fr", True, False), ("hl", False, True), ("hr", False, False))


def assembly(P, hip=STANCE[0], knee=STANCE[1], abd=0.0):
    items = [("chassis", P["chassis"], DARK), ("top_cover", P["top_cover"], WHITE)]
    for name, front, left in LEGS:
        items += leg(P, name, front, left, abd, hip, knee)
    return items


def overlaps(moving, fixed=()):
    """Pairs that intersect by more than 1 mm3: every moving part against every other part."""
    out = []
    for i, (na, a, _) in enumerate(moving):
        A, ba = a.val(), a.val().BoundingBox()
        for nb, b, _ in list(moving[i + 1:]) + list(fixed):
            bb = b.val().BoundingBox()
            if (ba.xmin > bb.xmax or bb.xmin > ba.xmax or ba.ymin > bb.ymax or bb.ymin > ba.ymax
                    or ba.zmin > bb.zmax or bb.zmin > ba.zmax):
                continue
            vol = A.intersect(b.val()).Volume()
            if vol > 1.0:
                out.append((na, nb, round(vol, 1)))
    return out


def free_range(P, joint, values):
    """Largest run of `values` around the stance pose in which one front and one hind leg (left) hit
    neither themselves nor the chassis, cover and their neighbour's parts. Other joints stay in stance."""
    fixed = [("chassis", P["chassis"], 0), ("top_cover", P["top_cover"], 0)]
    stance = dict(abd=0.0, hip=STANCE[0], knee=STANCE[1])
    ok = {}
    for v in values:
        pose = dict(stance, **{joint: v})
        legs = leg(P, "fl", True, True, **pose) + leg(P, "hl", False, True, **pose)
        ok[v] = overlaps(legs, fixed)
    lo = hi = min(values, key=lambda v: abs(v - stance[joint]))
    assert not ok[lo], f"collision next to the stance pose: {ok[lo]}"
    while lo > values[0] and not ok[values[values.index(lo) - 1]]:
        lo = values[values.index(lo) - 1]
    while hi < values[-1] and not ok[values[values.index(hi) + 1]]:
        hi = values[values.index(hi) + 1]
    first = {v: hits[0] for v, hits in ok.items() if hits}
    return lo, hi, first


PRINT = dict(                   # how each part lies on the bed: (rotation axis, degrees), and what to tell the slicer
    chassis=((1, 0, 0), 0, "plate on the bed, walls up; no support"),
    top_cover=((1, 0, 0), 180, "roof on the bed; no support"),
    abd_bracket=((1, 0, 0), 180, "roof on the bed, all plates standing; no support"),
    thigh=((1, 0, 0), 0, "on the straight edges of the two plates, crossbar is a 36 mm bridge; no support"),
    shin_link=((0, 1, 0), -90, "standing on the servo-tail end, tube tunnel vertical; no support"),
)


def printable(P):
    """{file name: solid in print orientation}; left and right where they differ."""
    out = {}
    for name, (axis, deg, _) in PRINT.items():
        sides = (("", P[name]),) if name in ("chassis", "top_cover") else (
            ("_left", P[name]), ("_right", P[name].mirror("XZ")))
        for suffix, part in sides:
            out[name + suffix] = part.rotate((0, 0, 0), axis, deg)
    return out


def check(P, p=PARAMS):
    report = []
    ff = P["ff"]
    parts = printable(P)
    # 1. every part is one valid solid and fits the bed
    for name, part in dict(parts, servo_dummy=P["servo"]).items():
        assert part.val().isValid(), f"{name} is not a valid solid"
        assert len(part.solids().vals()) == 1, f"{name} fell apart into {len(part.solids().vals())} pieces"
        bb = part.val().BoundingBox()
        assert max(bb.xlen, bb.ylen, bb.zlen) <= 250.0, f"{name} does not fit a 250 mm bed"
    # 2. joint positions, measured on the holes and arcs of the parts
    Lt, Ls = math.hypot(THIGH[0], THIGH[2]), math.hypot(SHIN[0], SHIN[2])
    arcs = axes(P["chassis"], 2 * p["horn_clear"])
    assert {(round(a.y), round(a.z)) for a, _, _ in arcs} == {(60, 0), (-60, 0)}, "abduction axes not at y = +-60, z = 0"
    for sign in (1, -1):                                         # hip axis x: servo origin + horn + bracket offset
        xs = [b.center.x for a, _, b in arcs if a.y > 0 and b.center.x * sign > 0]
        hip = (min(xs) + max(xs)) / 2 + p["sv_horn_z"] + p["abd_back"]
        assert abs(hip - sign * p["hip_x"]) < 0.5, f"hip axis at x = {hip:.2f}"
    br = P["abd_bracket"]
    a0 = [a for a, d, _ in axes(br, 8.0) if abs(d.x) > 0.99][0]                # abduction axis (horn centre hole)
    hips = [(a, b) for a, d, b in axes(br, 2 * p["horn_clear"]) if abs(d.y) > 0.99]
    hy = sum(b.center.y for _, b in hips) / len(hips)
    assert abs(a0.y) < 0.01 and abs(a0.z) < 0.01 and all(abs(a.x) < 0.01 and abs(a.z) < 0.01 for a, _ in hips)
    assert abs(hy - p["hip_y"]) < 0.5, f"hip servo mid plane {hy:.2f} mm outboard, simulation has {p['hip_y']}"
    ends = sorted({round(a.x, 3) for a, d, _ in axes(P["thigh"], 8.0)})
    assert len(ends) == 2 and abs(ends[1] - ends[0] - Lt) < 0.5, f"thigh hole distance {ends}"
    sh = P["shin_link"]
    knee = [a for a, d, _ in axes(sh, 2 * p["horn_clear"])][0]
    bore = [a for a, d, _ in axes(sh, p["tube_d"] + 0.2) if abs(d.x) > 0.99][0]
    sock = [(a, d) for a, d, _ in axes(P["adapter"], foot_cad.PARAMS["tube_d"] + 0.2)][0]   # the real foot adapter
    assert abs(abs(sock[1].x) - 1) < 1e-6, "foot adapter socket is not parallel to the shin tube"
    assert abs(sock[0].z - bore.z) < 0.05 and abs(sock[0].y - bore.y) < 0.05, "tube does not line up with the foot adapter"
    # ankle from the tube: lower tube end -> socket base point -> pad -> flexure neck
    f = foot_cad.PARAMS
    p0 = V(ff["x_stop"] + ff["tube_len"] + f["socket_len"] - f["tube_depth"], bore.y, bore.z)
    ankle = p0 - ff["xf"] * 9.0 - ff["zf"] * (ff["z_top"] + f["base_t"] + 1.0 - (f["plate_t"] + 2.0 + f["neck_len"] / 2))
    d_shin = (ankle - V(knee.x, 0, knee.z)).Length
    assert abs(d_shin - Ls) < 0.5, f"knee to ankle {d_shin:.2f} mm"
    report.append(f"joint distances: thigh {ends[1] - ends[0]:.2f} mm (sim {Lt:.2f}), knee-ankle {d_shin:.2f} mm (sim {Ls:.2f}), "
                  f"hip plane {hy:.2f} mm outboard (sim {p['hip_y']:.0f})")
    # 3. no collisions in stance and in the zero pose; nothing below the pads in stance
    for label, hip, knee_a in (("stance", *STANCE), ("zero pose", 0.0, 0.0)):
        items = assembly(P, hip, knee_a)
        hits = overlaps(items)
        assert not hits, f"{label}: parts intersect: {hits}"
        report.append(f"{label}: {len(items)} solids, no pair intersects by more than 1 mm3")
        if label == "stance":
            pad_z = min(s.val().BoundingBox().zmin for n, s, _ in items if n.endswith("foot_carrier"))
            low = min((s.val().BoundingBox().zmin, n) for n, s, _ in items if not n.endswith("foot_carrier"))
            assert low[0] > pad_z, f"{low[1]} goes below the pad plane"
            belly = P["chassis"].val().BoundingBox().zmin - p["batt"][2] - pad_z
            report.append(f"stance: pad plane at z = {pad_z:.1f} mm, lowest other part {low[1]} {low[0] - pad_z:.1f} mm above it, "
                          f"{belly:.0f} mm under the battery")
    # 4. how far the joints can move (others in stance), one front and one hind leg
    lo, hi, why = free_range(P, "abd", [-30, -20, -10, 0, 10, 20, 30])
    assert (lo, hi) == (-30, 30), f"abduction is only free from {lo} to {hi} deg: {why}"
    report.append("abduction -30..+30 deg: free (sim range -30..30)")
    for joint, values, sim in (("hip", list(range(-90, 91, 10)), "-90..90"), ("knee", list(range(0, 151, 10)), "0..150")):
        values = sorted(set(values) | {round(STANCE[joint == "knee"])})
        lo, hi, why = free_range(P, joint, values)
        ends = "; ".join(f"at {v}: {why[v][0]} x {why[v][1]}" for v in why if v in
                         (values[max(values.index(lo) - 1, 0)], values[min(values.index(hi) + 1, len(values) - 1)]))
        report.append(f"{joint} {lo}..{hi} deg free (sim range {sim}; 10 deg steps){' - stops ' + ends if ends else ''}")
    fixed = [("chassis", P["chassis"], 0), ("top_cover", P["top_cover"], 0)]
    for hip, knee_a in ((-60, 20), (-60, 140), (0, 80), (60, 20), (60, 140)):
        hits = overlaps(leg(P, "fl", True, True, 0.0, hip, knee_a) + leg(P, "hl", False, True, 0.0, hip, knee_a), fixed)
        report.append(f"sample pose hip {hip:+d}, knee {knee_a}: " + ("free" if not hits else f"COLLISION {hits[0][0]} x {hits[0][1]}"
                      + (f" and {len(hits) - 1} more" if len(hits) > 1 else "")))
    # 4b. every pose the wall-to-ceiling corner passes through (sim/corner.py)
    for name, front in (("front", True), ("hind", False)):
        for hip, knee_a in CORNER_POSES[name]:
            hits = overlaps(leg(P, "l", front, True, 0.0, hip, knee_a), fixed)
            assert not hits, f"corner pose {name} hip {hip}, knee {knee_a}: {hits}"
    report.append(f"corner poses free: front hip -83..53, hind hip -23..{HIP_MAX}, knee 0..57 deg, {sum(map(len, CORNER_POSES.values()))} poses "
                  f"(relief cuts: {P['relief'][0]:.0f} mm3 off each thigh, {P['relief'][1]:.0f} mm3 off each hip bracket, {P['relief'][2]:.0f} mm3 off the chassis)")
    # 5. wall rule for the structural parts (the cover and the cable ribs are not structural)
    assert min(p["arm_t"], p["ear_t"], p["plate_t"], p["tube_wall"]) >= 2.4, "structural wall under 2.4 mm"
    assert p["tether_wall"] >= 5.0 and p["tether_d"] == 6.0, "tether eye too weak"
    report.append("walls: structural plates >= 2.4 mm, tether eye 6 mm hole with "
                  f"{p['tether_wall']:.0f} mm around it; all parts within 250 mm")
    return parts, report


# (count, what, where, grams each): fasteners used by these parts; masses are typical catalogue values
FASTENERS = (
    (96, "M2 x 6 self-tapping pan head (PA2.0)", "servo cases to holder plates, 8 per servo", 0.15),
    (48, "M3 x 6 button head + M3 washer", "horn-side plates into the 12 output horns, 4 each", 0.55),
    (24, "M3 x 8 button head", "idler-side plates into the 12 idler horns, 2 each (4 holes)", 0.55),
    (4, "M3 x 16 socket head + M3 nut", "shin tube clamps", 1.5),
    (4, "M3 x 8 + M3 heat-set insert", "top cover", 1.2),
    (4, "M2.5 x 6 + M2.5 heat-set insert", "Raspberry Pi 5", 0.6),
    (4, "M2 x 8 + M2 nut", "camera", 0.3),
)


def main():
    p = PARAMS
    P = build(p)
    parts, report = check(P, p)
    for name, part in parts.items():
        cq.exporters.export(part, str(HERE / f"{name}.step"))
        cq.exporters.export(part, str(HERE / f"{name}.stl"), tolerance=0.02, angularTolerance=0.1)
    asm = cq.Assembly()
    for name, solid, colour in assembly(P):
        asm.add(solid, name=name, color=cq.Color(*colour))
    asm.save(str(HERE / "robot_assembly.step"))

    print("part              print size (mm)        g each  qty  g total   print orientation")
    total = 0.0
    for name, (_, _, how) in PRINT.items():
        key = name if name in parts else name + "_left"
        bb, qty = parts[key].val().BoundingBox(), 1 if name in parts else 4
        g = parts[key].val().Volume() / 1000 * p["density"]
        total += g * qty
        print(f"{name:<17} {bb.xlen:5.0f} x {bb.ylen:3.0f} x {bb.zlen:3.0f}        {g:6.1f}  {qty:3d}  {g * qty:7.1f}   {how}")
    screws = sum(n * g for n, _, _, g in FASTENERS)
    servos = 12 * p["sv_mass"]
    print(f"{'printed parts':<54}{total:7.1f}   solid PETG {p['density']} g/cm3 (4 walls fill these thin plates)")
    print(f"{'fasteners (estimate)':<54}{screws:7.1f}")
    print(f"{'12 x STS3215':<54}{servos:7.1f}   55 g each, Feetech specification")
    print(f"{'frame + servos':<54}{total + screws + servos:7.1f}")
    rest = 46 + 4 + 170 + 15 + 4 * 3.5 + 4 * 35 + 40      # Pi 5, camera, 3S LiPo, buck, tubes, feet, wiring: estimates
    print(f"{'+ Pi 5, camera, battery, buck, tubes, feet, wiring':<54}{rest:7.1f}   estimates, no high-voltage supply")
    print(f"{'robot, about':<54}{total + screws + servos + rest:7.1f}   target 1500 g")
    print(f"shin tube: cut 4 pieces of 10 mm carbon tube to {P['ff']['tube_len']:.1f} mm "
          f"({P['ff']['tube_len'] - foot_cad.PARAMS['tube_depth']:.1f} mm in the shin link, {foot_cad.PARAMS['tube_depth']:.0f} mm in the foot)")
    print("fasteners:")
    for n, what, where, _ in FASTENERS:
        print(f"  {n:3d} x {what}: {where}")
    print("checks:")
    for line in report:
        print("  " + line)


if __name__ == "__main__":
    main()
