#!/usr/bin/env python3
"""Render the foot CAD (the exported STL files) to preview.png with MuJoCo."""
from pathlib import Path

import mujoco
from PIL import Image

HERE = Path(__file__).resolve().parent
XML = f"""
<mujoco>
  <compiler meshdir="{HERE}"/>
  <visual><global offwidth="1400" offheight="1000"/><quality shadowsize="4096" offsamples="8"/>
    <headlight ambient="0.45 0.45 0.47" diffuse="0.45 0.45 0.45"/></visual>
  <statistic extent="0.15" center="0 0 0.02"/>
  <asset>
    <mesh name="carrier" file="foot_carrier_tpu.stl" scale="0.001 0.001 0.001"/>
    <mesh name="adapter" file="shin_adapter_petg.stl" scale="0.001 0.001 0.001"/>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.93 0.94 0.96" rgb2="0.80 0.83 0.88" width="256" height="256"/>
  </asset>
  <worldbody>
    <light directional="true" dir="-0.3 0.4 -1" diffuse="0.6 0.6 0.6" castshadow="true"/>
    <geom type="plane" size="0.5 0.5 0.01" pos="0 0 -0.0035" rgba="0.82 0.83 0.85 1"/>
    <geom type="box" size="0.04 0.03 0.0015" pos="0 0 -0.0016" rgba="0.95 0.62 0.14 1"/>
    <geom type="mesh" mesh="carrier" rgba="0.13 0.13 0.15 1"/>
    <geom type="mesh" mesh="adapter" pos="{{x}} 0 {{z}}" rgba="0.93 0.94 0.95 1"/>
    <geom type="cylinder" fromto="{{t0}} {{t1}}" size="0.005" rgba="0.07 0.07 0.08 1"/>
  </worldbody>
</mujoco>"""


def main():
    import math
    from foot_cad import PARAMS, carrier
    _, z_top = carrier()
    a = math.radians(PARAMS["tube_tilt_deg"])
    u = (math.sin(a), 0, math.cos(a))
    start = [0.009 + 0.006 * u[0], 0, (z_top + PARAMS["base_t"] + 1) / 1000 + 0.006 * u[2]]
    end = [start[0] + 0.07 * u[0], 0, start[2] + 0.07 * u[2]]
    views = []
    for explode in (0.0, 0.022):
        xml = XML.format(x=0, z=z_top / 1000 + explode,
                         t0=" ".join(f"{v + (explode if i == 2 else 0):.4f}" for i, v in enumerate(start)),
                         t1=" ".join(f"{v + (explode if i == 2 else 0):.4f}" for i, v in enumerate(end)))
        m = mujoco.MjModel.from_xml_string(xml)
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        r = mujoco.Renderer(m, 700, 700)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = (0.012, 0, 0.022 + explode / 2)
        cam.distance, cam.elevation, cam.azimuth = 0.24, -24, 118
        r.update_scene(d, cam)
        views.append(Image.fromarray(r.render().copy()))
    sheet = Image.new("RGB", (1400, 700))
    sheet.paste(views[0], (0, 0)), sheet.paste(views[1], (700, 0))
    sheet.save(HERE / "preview.png")
    print("saved preview.png")


if __name__ == "__main__":
    main()
