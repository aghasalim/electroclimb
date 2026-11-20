#!/usr/bin/env python3
"""Generate an interdigitated electroadhesion pad as a KiCad footprint.

Two nets: pad "1" (HV+, bus on the left) and pad "2" (HV-, bus on the right).
Fingers run horizontally and alternate between the two buses.

    python3 tools/gen_electrode.py --width 80 --height 60 --finger 1.0 --gap 1.0
    python3 tools/gen_electrode.py --strips 8 --gap 2.0       # WallScan test pad: 8 separate strips
    python3 tools/gen_electrode.py --selftest
"""
import argparse
import itertools
from pathlib import Path

BUS = 2.0  # bus bar width, mm


def layout(width, height, finger, gap, margin):
    """Return (rects, info). rects = [(net, x0, y0, x1, y1)] in mm, origin top-left."""
    x0, x1 = margin, width - margin
    y0, y1 = margin, height - margin
    if x1 - x0 < 2 * BUS + 4 * gap or y1 - y0 < finger:
        raise ValueError("pad too small for these parameters")

    rects = [
        ("1", x0, y0, x0 + BUS, y1),  # bus A
        ("2", x1 - BUS, y0, x1, y1),  # bus B
        # connection tabs: same width as their bus, run to the board edge
        ("1", x0, 0.0, x0 + BUS, y0),  # HV+ tab (top-left)
        ("2", x1 - BUS, 0.0, x1, y0),  # HV- tab (top-right)
    ]
    pitch = finger + gap
    n = int((y1 - y0 - finger) // pitch) + 1
    for k in range(n):
        fy0 = y0 + k * pitch
        if k % 2 == 0:  # net 1 grows from bus A, stops a gap before bus B
            rects.append(("1", x0 + BUS, fy0, x1 - BUS - gap, fy0 + finger))
        else:  # net 2 grows from bus B, stops a gap after bus A
            rects.append(("2", x0 + BUS + gap, fy0, x1 - BUS, fy0 + finger))

    area = sum((r[3] - r[1]) * (r[4] - r[2]) for r in rects)
    info = {"fingers": n, "copper_mm2": round(area, 1),
            "fill": round(area / ((x1 - x0) * (y1 - y0)), 2)}
    return rects, info


def strips_layout(width, height, n, gap, margin):
    """WallScan test pad: n parallel strips, each its own net ("1".."n") with a tab to the top edge.
    Returns (rects, info) like layout()."""
    x0, x1 = margin, width - margin
    y0, y1 = margin, height - margin
    w = (x1 - x0 - (n - 1) * gap) / n
    if w < 1.0:
        raise ValueError("pad too small for this many strips")
    rects = []
    for k in range(n):
        sx = x0 + k * (w + gap)
        rects.append((str(k + 1), sx, y0, sx + w, y1))                                # strip
        rects.append((str(k + 1), sx + w / 2 - 1.0, 0.0, sx + w / 2 + 1.0, y0))       # 2 mm tab to the edge
    area = sum((r[3] - r[1]) * (r[4] - r[2]) for r in rects)
    return rects, {"strips": n, "strip_mm": round(w, 2), "copper_mm2": round(area, 1)}


def to_svg(rects, width, height):
    colours = ["#e08a14", "#2f6fb3"]
    body = "".join(f'<rect x="{x0:.2f}" y="{y0:.2f}" width="{x1 - x0:.2f}" height="{y1 - y0:.2f}" '
                   f'fill="{colours[(int(net) - 1) % 2]}"/>' for net, x0, y0, x1, y1 in rects)
    return (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="-2 -2 {width + 4} {height + 4}" width="{width * 6}">'
            f'<rect width="{width}" height="{height}" fill="#f6dfb8" stroke="#333" stroke-width="0.3"/>{body}</svg>\n')


def min_spacing(rects):
    """Smallest edge-to-edge distance between rectangles of different nets."""
    best = float("inf")
    for a, b in itertools.combinations(rects, 2):
        if a[0] == b[0]:
            continue
        dx = max(b[1] - a[3], a[1] - b[3], 0)
        dy = max(b[2] - a[4], a[2] - b[4], 0)
        best = min(best, (dx * dx + dy * dy) ** 0.5)
    return best


def to_kicad(name, rects, width, height):
    # KiCad footprint coordinates: origin at pad centre, y down
    cx, cy = width / 2, height / 2
    out = [
        f'(footprint "{name}" (version 20240108) (generator "electroclimb") (layer "F.Cu")',
        '  (attr smd)',
        f'  (fp_text reference "REF**" (at 0 {-cy - 2:.3f}) (layer "F.SilkS")'
        ' (effects (font (size 1 1) (thickness 0.15))))',
        f'  (fp_text value "{name}" (at 0 {cy + 2:.3f}) (layer "F.Fab")'
        ' (effects (font (size 1 1) (thickness 0.15))))',
        f'  (fp_rect (start {-cx:.3f} {-cy:.3f}) (end {cx:.3f} {cy:.3f})'
        ' (stroke (width 0.1) (type solid)) (fill none) (layer "Edge.Cuts"))',
    ]
    for net, x0, y0, x1, y1 in rects:
        w, h = x1 - x0, y1 - y0
        px, py = (x0 + x1) / 2 - cx, (y0 + y1) / 2 - cy
        out.append(f'  (pad "{net}" smd rect (at {px:.3f} {py:.3f}) (size {w:.3f} {h:.3f})'
                   ' (layers "F.Cu"))')
    out.append(')')
    return "\n".join(out) + "\n"


def selftest():
    for finger, gap in [(0.5, 0.5), (1.0, 1.0), (2.0, 1.0), (1.0, 2.0)]:
        rects, info = layout(80, 60, finger, gap, 2.0)
        s = min_spacing(rects)
        assert abs(s - gap) < 1e-6, f"spacing {s} != gap {gap}"  # no shorts, no wasted space
        assert all(0 <= r[1] < r[3] <= 80 and 0 <= r[2] < r[4] <= 60 for r in rects), "outside outline"
        assert {r[0] for r in rects} == {"1", "2"}
        assert info["fingers"] >= 2
    rects, info = strips_layout(80, 60, 8, 2.0, 2.0)
    assert abs(min_spacing(rects) - 2.0) < 1e-6, "strip spacing"
    assert {r[0] for r in rects} == {str(k) for k in range(1, 9)}
    assert all(0 <= r[1] < r[3] <= 80 and 0 <= r[2] < r[4] <= 60 for r in rects), "outside outline"
    try:
        layout(10, 10, 1, 1, 2)
        raise AssertionError("tiny pad should fail")
    except ValueError:
        pass
    print("selftest ok")


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--width", type=float, default=80.0)
    p.add_argument("--height", type=float, default=60.0)
    p.add_argument("--finger", type=float, default=1.0)
    p.add_argument("--gap", type=float, default=1.0)
    p.add_argument("--margin", type=float, default=2.0)
    p.add_argument("--out", default="hardware/foot/electroclimb.pretty")
    p.add_argument("--strips", type=int, default=0, help="WallScan test pad with this many separate strips")
    p.add_argument("--selftest", action="store_true")
    a = p.parse_args()
    if a.selftest:
        return selftest()
    if a.strips:
        rects, info = strips_layout(a.width, a.height, a.strips, a.gap, a.margin)
        assert abs(min_spacing(rects) - a.gap) < 1e-6
        name = f"WS_Pad_{a.width:g}x{a.height:g}_S{a.strips}_G{a.gap:g}"
        out = Path(a.out)
        out.mkdir(parents=True, exist_ok=True)
        (out / f"{name}.kicad_mod").write_text(to_kicad(name, rects, a.width, a.height))
        (out.parent / "preview").mkdir(exist_ok=True)
        (out.parent / "preview" / f"{name}.svg").write_text(to_svg(rects, a.width, a.height))
        print(f"{name}: {info}")
        return

    rects, info = layout(a.width, a.height, a.finger, a.gap, a.margin)
    assert abs(min_spacing(rects) - a.gap) < 1e-6
    name = f"EA_Pad_{a.width:g}x{a.height:g}_F{a.finger:g}_G{a.gap:g}"
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / f"{name}.kicad_mod").write_text(to_kicad(name, rects, a.width, a.height))
    print(f"{name}: {info}")


if __name__ == "__main__":
    main()
