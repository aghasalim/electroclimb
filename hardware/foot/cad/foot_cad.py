#!/usr/bin/env python3
"""Parametric CAD for one ElectroClimb foot (CadQuery / OpenCascade).

    uv pip install --python ../../../.venv cadquery
    ../../../.venv/bin/python foot_cad.py      # writes STEP + STL for each part and the assembly

Two printed parts, dimensions in mm, foot frame: z = away from the wall, pad face at z = 0.

  foot_carrier_tpu   TPU 95A, one piece: the plate the flex-PCB pad is glued under, a round
                     thin flexure neck (the soft 2-axis ankle) and a flange with a captive M4 nut.
  shin_adapter_petg  PETG or PA-CF: bolts onto the flange with one M4 screw, clamps the
                     10 mm carbon shin tube at the angle it has when the robot stands.

Change the numbers in PARAMS and re-run. `check()` verifies the parts are valid solids, fit
together and can be printed without thin walls.
"""
import math
from pathlib import Path

import cadquery as cq

HERE = Path(__file__).resolve().parent

PARAMS = dict(
    pad_w=80.0, pad_h=60.0,        # flex-PCB pad (hardware/foot/electroclimb.pretty)
    plate_margin=2.0,              # carrier plate is this much larger than the pad on each side
    plate_t=2.0,                   # thin, so the pad can follow a rough wall
    plate_r=6.0,
    neck_d=4.5, neck_len=5.0,      # flexure: bending stiffness ~ E * pi d^4 / (64 L). Soft on purpose:
                                   # the pad must lie flat by itself (sim/flat.py: 0.1 N*m/rad, +-45 deg)
    ankle_travel_deg=45.0,
    pad_pull_n=15.0, press_n=20.0, # largest pull on the pad and largest push through the neck
    flange_d=30.0, flange_t=4.15,  # thinner than before: the longer cone below it holds part of the nut pocket
    nut_af=7.0, nut_t=3.2,         # M4 hex nut (ISO 4032)
    bolt_d=4.4,                    # M4 clearance
    nub_d=4.0, nub_h=2.0, nub_y=10.0,   # two anti-rotation nubs
    tube_d=10.0,                   # carbon tube outer diameter
    socket_wall=3.0, socket_len=28.0, tube_depth=22.0,
    tube_tilt_deg=61.0,            # tube axis from the pad normal, in stance (from the sim: hip -0.4, knee 1.0 rad)
    base_t=5.0,
    tpu_E_mpa=26.0,                # TPU 95A, typical; real prints vary by about +-50 %
)


def carrier(p=PARAMS):
    w, h = p["pad_w"] + 2 * p["plate_margin"], p["pad_h"] + 2 * p["plate_margin"]
    t = p["plate_t"]
    plate = cq.Workplane("XY").box(w, h, t, centered=(True, True, False)).edges("|Z").fillet(p["plate_r"])
    rn, rf = p["neck_d"] / 2, p["flange_d"] / 2
    z1 = t + 2.0                        # end of the base fillet cone
    z2 = z1 + p["neck_len"]             # end of the neck
    z3 = z2 + (rf - rn)                 # 45 deg cone: prints without support
    z4 = z3 + p["flange_t"]
    post = (cq.Workplane("XZ")
            .polyline([(0, t), (rn + 4, t), (rn, z1), (rn, z2), (rf, z3), (rf, z4), (0, z4)]).close()
            .revolve(360, (0, 0, 0), (0, 1, 0)))
    body = plate.union(post)
    # captive M4 nut: slides in from the -x side, 3 mm of TPU above it
    nut_z = z4 - 3.0 - (p["nut_t"] + 0.2)
    slot = (cq.Workplane("XY").workplane(offset=nut_z)
            .center(-(rf + 1 - p["nut_af"] / 2 - 0.1) / 2, 0)
            .rect(rf + 1 + p["nut_af"] / 2 + 0.1, p["nut_af"] + 0.2).extrude(p["nut_t"] + 0.2))
    bolt = cq.Workplane("XY").workplane(offset=nut_z - 4.0).circle(p["bolt_d"] / 2).extrude(z4 - nut_z + 4.0)
    body = body.cut(slot).cut(bolt)
    nubs = (cq.Workplane("XY").workplane(offset=z4)
            .pushPoints([(0, p["nub_y"]), (0, -p["nub_y"])]).circle(p["nub_d"] / 2).extrude(p["nub_h"]))
    body = body.union(nubs)
    # wire notches at the pad's two solder tabs (top edge), and two posts for the driver board
    tab_x = p["pad_w"] / 2 - 3.0
    notch = (cq.Workplane("XY").pushPoints([(tab_x, h / 2), (-tab_x, h / 2)]).rect(5.0, 5.0).extrude(t))
    body = body.cut(notch)
    posts = (cq.Workplane("XY").workplane(offset=t).pushPoints([(-30, 9), (-30, -9)]).circle(2.6).extrude(4.0))
    pilot = (cq.Workplane("XY").workplane(offset=t + 0.8).pushPoints([(-30, 9), (-30, -9)]).circle(0.8).extrude(3.2))
    return body.union(posts).cut(pilot), z4


def shin_adapter(p=PARAMS):
    a = math.radians(p["tube_tilt_deg"])
    u = cq.Vector(math.sin(a), 0, math.cos(a))            # tube axis, pointing up the shin
    up = cq.Vector(-math.cos(a), 0, math.sin(a))          # across the socket, on its upper side
    r_out = p["tube_d"] / 2 + p["socket_wall"]
    p0 = cq.Vector(9.0, 0, p["base_t"] + 1.0)
    plane = lambda origin: cq.Plane(origin=origin.toTuple(), xDir=(0, 1, 0), normal=u.toTuple())
    base = cq.Workplane("XY").circle(p["flange_d"] / 2).extrude(p["base_t"])
    socket = cq.Workplane(plane(p0)).circle(r_out).extrude(p["socket_len"])
    lug0 = p0 + u * (p["socket_len"] - 12.0)
    lug = cq.Workplane(plane(lug0)).center(0, r_out + 2.0).rect(14.0, 10.0).extrude(8.0)
    body = base.union(socket).union(lug)
    body = body.cut(cq.Workplane("XY").workplane(offset=-40).rect(200, 200).extrude(40))   # nothing below the flange face
    bore0 = p0 + u * (p["socket_len"] - p["tube_depth"])
    body = body.cut(cq.Workplane(plane(bore0)).circle(p["tube_d"] / 2 + 0.1).extrude(p["tube_depth"] + 1))
    slit0 = p0 + u * (p["socket_len"] - 16.0)             # pinch slit through the upper wall and the lug
    body = body.cut(cq.Workplane(plane(slit0)).center(0, r_out + 1.0).rect(1.5, 2 * r_out).extrude(17.0))
    pin = lug0 + u * 4.0 + up * (r_out + 3.0)             # M3 pinch screw across the lug
    body = body.cut(cq.Workplane(cq.Plane(origin=(pin.x, -12, pin.z), xDir=(1, 0, 0), normal=(0, 1, 0)))
                    .circle(1.7).extrude(24))
    nut = cq.Workplane(cq.Plane(origin=(pin.x, 5.0, pin.z), xDir=(1, 0, 0), normal=(0, 1, 0))).polygon(6, 6.4).extrude(4)
    body = body.cut(nut)
    # centre M4 screw with room for its head and a hex key, and the two nub sockets
    body = body.cut(cq.Workplane("XY").circle(p["bolt_d"] / 2).extrude(p["base_t"]))
    body = body.cut(cq.Workplane("XY").workplane(offset=p["base_t"]).circle(3.9).extrude(40))
    body = body.cut(cq.Workplane("XY").pushPoints([(0, p["nub_y"]), (0, -p["nub_y"])])
                    .circle(p["nub_d"] / 2 + 0.2).extrude(p["nub_h"] + 0.5))
    return body


def flexure_stiffness(p=PARAMS):
    """Bending stiffness of the neck in N*m/rad (beam theory, small angles)."""
    inertia = math.pi * p["neck_d"] ** 4 / 64
    return p["tpu_E_mpa"] * inertia / p["neck_len"] / 1000


def neck_checks(p=PARAMS):
    """Strain at full travel, buckling load and pull stress of the neck (beam theory)."""
    d, L, E = p["neck_d"], p["neck_len"], p["tpu_E_mpa"]
    strain = d / 2 * math.radians(p["ankle_travel_deg"]) / L
    buckle = math.pi ** 2 * E * (math.pi * d ** 4 / 64) / (2 * L) ** 2      # N, worst case: one end free
    pull = p["pad_pull_n"] / (math.pi * d ** 2 / 4)                          # MPa
    return strain, buckle, pull


def check(car, adp, z_top, p=PARAMS):
    for name, part in (("carrier", car), ("adapter", adp)):
        solid = part.val()
        assert solid.isValid(), f"{name} is not a valid solid"
        assert len(part.solids().vals()) == 1, f"{name} fell apart into several pieces"
    bb = car.val().BoundingBox()
    assert abs(bb.zmin) < 1e-6 and bb.zmax > z_top, "carrier must sit on z = 0"
    assert abs(adp.val().BoundingBox().zmin) < 1e-6, "adapter flange must be flat"
    placed = adp.translate((0, 0, z_top))
    overlap = car.intersect(placed)
    vol = overlap.val().Volume() if overlap.vals() else 0.0
    nub_vol = 2 * math.pi * (p["nub_d"] / 2) ** 2 * p["nub_h"]
    assert vol <= nub_vol * 0.02, f"parts collide ({vol:.1f} mm3)"     # nubs sit in oversized sockets
    assert p["plate_t"] >= 1.6 and p["socket_wall"] >= 2.4, "walls too thin to print"
    assert 0.03 <= flexure_stiffness(p) <= 0.15, "ankle stiffness outside the range that climbs in sim/flat.py"
    strain, buckle, pull = neck_checks(p)
    assert strain <= 0.5, f"neck strain {strain:.0%} at full travel: too much for TPU in fatigue"
    assert buckle >= 2 * p["press_n"], f"neck buckles at {buckle:.0f} N"
    assert pull <= 3.0, f"neck pull stress {pull:.1f} MPa"
    # the 45 deg cone under the flange lies down on the plate at 45 deg of tilt: that is the end stop
    return placed


def main():
    car, z_top = carrier()
    adp = shin_adapter()
    placed = check(car, adp, z_top)
    for name, part in (("foot_carrier_tpu", car), ("shin_adapter_petg", adp)):
        cq.exporters.export(part, str(HERE / f"{name}.step"))
        cq.exporters.export(part, str(HERE / f"{name}.stl"), tolerance=0.02, angularTolerance=0.1)
    asm = cq.Assembly()
    asm.add(car, name="foot_carrier_tpu", color=cq.Color(0.1, 0.1, 0.11))
    asm.add(placed, name="shin_adapter_petg", color=cq.Color(0.92, 0.93, 0.95))
    asm.save(str(HERE / "foot_assembly.step"))
    tpu, petg = car.val().Volume() / 1000 * 1.21, adp.val().Volume() / 1000 * 1.27
    bb = car.val().BoundingBox()
    print(f"carrier  {bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.1f} mm, {tpu:.1f} g solid TPU")
    bb = adp.val().BoundingBox()
    print(f"adapter  {bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.1f} mm, {petg:.1f} g solid PETG")
    strain, buckle, pull = neck_checks()
    print(f"ankle stiffness {flexure_stiffness():.2f} N*m/rad (sim/flat.py uses 0.1), neck strain {strain:.0%} at "
          f"{PARAMS['ankle_travel_deg']:.0f} deg, buckles at {buckle:.0f} N, pull stress {pull:.1f} MPa")
    print("checks passed: valid solids, one piece each, no collision, printable walls")


if __name__ == "__main__":
    main()
