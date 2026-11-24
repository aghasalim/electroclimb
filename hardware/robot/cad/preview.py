#!/usr/bin/env python3
"""Render the robot CAD in the stance pose to preview.png with MuJoCo (3/4 view and side view)."""
import tempfile
from pathlib import Path

import cadquery as cq
import mujoco
from PIL import Image, ImageDraw, ImageFont

import robot_cad as rc

HERE = Path(__file__).resolve().parent
FONT = "/System/Library/Fonts/SFNS.ttf"
W, H = 1000, 760
XML = """
<mujoco>
  <compiler meshdir="{dir}"/>
  <visual><global offwidth="{w}" offheight="{h}"/><quality shadowsize="4096" offsamples="8"/>
    <headlight ambient="0.30 0.30 0.32" diffuse="0.40 0.40 0.40"/></visual>
  <statistic extent="0.5" center="0 0 0"/>
  <asset>
    {meshes}
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.93 0.94 0.96" rgb2="0.80 0.83 0.88" width="256" height="256"/>
  </asset>
  <worldbody>
    <light directional="true" dir="-0.3 0.4 -1" diffuse="0.6 0.6 0.6" castshadow="true"/>
    <geom type="plane" size="1 1 0.01" pos="0 0 {floor}" rgba="0.50 0.52 0.55 1"/>
    {geoms}
  </worldbody>
</mujoco>"""


def main():
    p = rc.PARAMS
    P = rc.build()
    items = rc.assembly(P)
    pad_z = min(s.val().BoundingBox().zmin for n, s, _ in items if n.endswith("foot_carrier")) / 1000
    meshes, geoms = [], []
    with tempfile.TemporaryDirectory() as tmp:
        for name, solid, colour in items:
            cq.exporters.export(solid, f"{tmp}/{name}.stl", tolerance=0.05, angularTolerance=0.2)
            meshes.append(f'<mesh name="{name}" file="{name}.stl" scale="0.001 0.001 0.001"/>')
            rgba = " ".join(f"{c:.2f}" for c in colour) + (" 0.6" if name == "top_cover" else " 1")
            geoms.append(f'<geom type="mesh" mesh="{name}" rgba="{rgba}"/>')
        for name, solid, _ in items:                      # the flex-PCB pads under the feet
            if name.endswith("foot_carrier"):
                c = solid.val().BoundingBox().center
                geoms.append(f'<geom type="box" size="0.04 0.03 0.0004" pos="{c.x / 1000:.4f} {c.y / 1000:.4f} {pad_z - 0.0005:.4f}" rgba="0.95 0.62 0.14 1"/>')
        bl, bw, bh = (v / 2000 for v in p["batt"])        # battery under the plate, Pi 5 board on its standoffs
        zb = P["chassis"].val().BoundingBox().zmin / 1000
        geoms.append(f'<geom type="box" size="{bl} {bw} {bh}" pos="{p["batt_x"] / 1000} 0 {zb - bh}" rgba="0.10 0.24 0.52 1"/>')
        geoms.append(f'<geom type="box" size="0.0425 0.028 0.0008" pos="{p["pi_x"] / 1000} 0 {zb + 0.0024 + 0.0058}" rgba="0.06 0.34 0.21 1"/>')
        m = mujoco.MjModel.from_xml_string(XML.format(dir=tmp, w=W, h=H, meshes="\n".join(meshes),
                                                      geoms="\n".join(geoms), floor=pad_z - 0.001))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, H, W)
    sheet = Image.new("RGB", (2 * W, H))
    draw = ImageDraw.Draw(sheet)
    font = ImageFont.truetype(FONT, 26)
    views = (("3/4 view, stance pose (front is right)", 0.62, -24, 128), ("side view (front is right)", 0.60, -4, 90))
    for i, (label, dist, elev, azim) in enumerate(views):
        cam = mujoco.MjvCamera()
        cam.lookat[:] = (0.0, 0, -0.03)
        cam.distance, cam.elevation, cam.azimuth = dist, elev, azim
        r.update_scene(d, cam)
        sheet.paste(Image.fromarray(r.render().copy()), (i * W, 0))
        draw.text((i * W + 24, 20), label, font=font, fill=(40, 42, 46))
    sheet.save(HERE / "preview.png")
    print("saved preview.png")


if __name__ == "__main__":
    main()
