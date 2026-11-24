#!/usr/bin/env python3
"""ElectroClimb foot HV driver board v1: the single source for netlist, BOM and PCB.

Run with KiCad's own Python (it has the `pcbnew` module), then let KiCad check the result:

    KPY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/3.9/bin/python3
    $KPY driver_board.py                 # writes the .kicad_pcb, .kicad_dru, bom.csv, netlist.md
    kicad-cli pcb drc --exit-code-violations -o drc.rpt electroclimb_driver.kicad_pcb

PARTS below is the schematic: every part, where it sits, and which net each pin is on.
The three high-voltage nets are routed by hand (HV_ROUTES); a small grid router does the rest.
KiCad's DRC is the check: it must report 0 violations and 0 unconnected pads.

Status: generated and rule-checked, never manufactured. Read README.md before ordering.
"""
import csv
import heapq
import math
from pathlib import Path

import numpy as np
import pcbnew

HERE = Path(__file__).resolve().parent
NAME = "electroclimb_driver"
LIB = "/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints/"
W, H = 82.0, 50.0                 # board size, mm
OX, OY = 100.0, 100.0             # where the board sits on the KiCad page
HV_NETS = ("HV_OUT", "PAD_A", "DUMP")
HV_CLEAR = 5.5                    # mm from any HV copper to any other net. Set by the relay: its coil pins are
                                  # 7.62 mm from its contact pins. Coat the finished board (see README).
POWER = ("GND", "+5V", "VHV_IN", "+5V_ARM", "5V_XIAO", "3V3", "COIL_N")
MODULE_BODY = (21.33, 5.91, 49.98, 17.34)    # U1 outline: XP Power asks for no top-side traces under it
HOLES = [(3.0, 3.0), (3.5, 47.0), (53.0, 46.5)]   # M3, use nylon screws and spacers

R, C = "Resistor_SMD:R_0805_2012Metric", "Capacitor_SMD:C_0805_2012Metric"
SOT23, SOD = "Package_TO_SOT_SMD:SOT-23", "Diode_SMD:D_SOD-123"
# custom footprints: {pad name: (x, y)} relative to the part position, then (drill, pad diameter), then outline
XIAO_TOP = ["5V", "GND", "3V3", "D10", "D9", "D8", "D7"]      # the XIAO's right side, USB-C towards the board edge
XIAO_BOT = ["D0", "D1", "D2", "D3", "D4", "D5", "D6"]         # the XIAO's left side
CUSTOM = {
    # XP Power A30P-5, datasheet SF_A_Series p.6 "recommended layout" (top view). Origin = pin 3.
    "A30P-5": ({"3": (0, 0), "4": (-16.23, 0), "1": (-18.11, -1.57), "2": (-16.99, 4.40), "5": (-18.11, 6.81)},
               (0.9, 1.5), (-26.67, -3.09, 1.98, 8.34)),
    # Standex-Meder SHV series, datasheet p.3: 4 pins in line, 7.62 / 5.08 / 7.62 mm. Origin = pin 1.
    "SHV": ({"1": (0, 0), "2": (-7.62, 0), "3": (-12.70, 0), "4": (-20.32, 0)}, (0.8, 1.6), (-22.2, -3.1, 1.9, 3.1)),
    "VR68": ({"1": (0, 0), "2": (25.4, 0)}, (1.0, 2.0), (3.2, -3.4, 22.2, 3.4)),          # body 19 x 6.8 mm
    "MOX750": ({"1": (0, 0), "2": (-27.94, 0)}, (0.8, 1.8), (-25.2, -2.1, -2.8, 2.1)),     # body 22.35 x 4.19 mm
    "VR37": ({"1": (0, 0), "2": (-15.24, 0)}, (0.9, 1.8), (-13.6, -2.0, -1.6, 2.0)),       # body 12 x 4 mm
    "WIREPAD": ({"1": (0, 0)}, (1.3, 2.6), (-1.6, -1.6, 1.6, 1.6)),
    # Seeed XIAO ESP32-S3 on two 1x7 pin sockets, 2.54 mm pitch, rows 15.24 mm apart. Origin = pad "5V".
    "XIAO": ({**{n: (2.54 * k, 0) for k, n in enumerate(XIAO_TOP)}, **{n: (2.54 * k, 15.24) for k, n in enumerate(XIAO_BOT)}},
             (1.0, 1.7), (-2.88, -1.28, 18.12, 16.52)),
}

# ref, value, footprint, x, y, rotation, {pin: net}, part to order
PARTS = [
    # --- connectors and power ---
    ("J1", "leg cable", "Connector_JST:JST_XH_B5B-XH-A_1x05_P2.50mm_Vertical", 8, 9.5, 0,
     {"1": "+5V", "2": "+5V_ARM", "3": "GND", "4": "CANH", "5": "CANL"}, "JST B5B-XH-A"),
    ("J2", "contact switch", "Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical", 27, 45.5, 0,
     {"1": "+5V_ARM", "2": "VHV_IN"}, "JST B2B-XH-A (jumper it for bench tests)"),
    ("D3", "schottky", SOD, 3.2, 23.8, 90, {"1": "5V_XIAO", "2": "+5V"}, "MBR0520 or B5819W"),
    ("C4", "10u", C, 7.5, 24, 90, {"1": "+5V", "2": "GND"}, "10 uF 16 V X5R 0805"),
    ("C2", "10u", C, 11.6, 14.5, 0, {"1": "VHV_IN", "2": "GND"}, "10 uF 16 V X5R 0805"),
    ("C3", "100n", C, 16.2, 14.5, 0, {"1": "VHV_IN", "2": "GND"}, "100 nF 50 V X7R 0805"),
    ("U2", "XIAO ESP32-S3", "XIAO", 4.4, 27.4, 0,
     {"5V": "5V_XIAO", "GND": "GND", "3V3": "3V3", "D10": "LED", "D9": "ARM_S", "D8": "RELAY", "D0": "ISENSE",
      "D1": "VMON_ADC", "D3": "PWM", "D4": "CAN_TX", "D5": "CAN_RX"}, "Seeed XIAO ESP32-S3 + two 1x7 sockets"),
    # --- HV module and its 0..5 V control (the XIAO only gives 3.3 V, so U3 multiplies by 1.51) ---
    ("U1", "A30P-5", "A30P-5", 48, 9, 0, {"1": "GND", "2": "VHV_IN", "3": "HV_OUT", "4": "GND", "5": "CTRL"}, "XP Power A30P-5"),
    ("R1", "10k", R, 24.3, 20, 0, {"1": "PWM", "2": "SET_F"}, "10 k 1 % 0805"),
    ("C1", "1u", C, 24.3, 23, 0, {"1": "GND", "2": "SET_F"}, "1 uF 16 V X7R 0805"),
    ("R11", "100k", R, 24.3, 26, 0, {"1": "GND", "2": "SET_F"}, "100 k 1 % 0805"),
    ("U3", "MCP6001", "Package_TO_SOT_SMD:SOT-23-5", 29.3, 23, 0,
     {"1": "CTRL_O", "2": "GND", "3": "SET_F", "4": "FB", "5": "VHV_IN"}, "Microchip MCP6001T-I/OT"),
    ("C9", "100n", C, 29.3, 19.4, 0, {"1": "VHV_IN", "2": "GND"}, "100 nF 50 V X7R 0805"),
    ("R8", "5k1", R, 34.8, 20, 0, {"1": "CTRL_O", "2": "FB"}, "5.1 k 1 % 0805"),
    ("R9", "10k", R, 34.8, 23, 0, {"1": "FB", "2": "GND"}, "10 k 1 % 0805"),
    ("R10", "1k", R, 34.8, 26, 0, {"1": "CTRL_O", "2": "CTRL"}, "1 k 1 % 0805"),
    ("C5", "100n", C, 29.3, 26.8, 0, {"1": "CTRL", "2": "GND"}, "100 nF 50 V X7R 0805"),
    # --- high voltage: limit, bleeder (also the voltage monitor), release relay ---
    ("R2", "47M", "VR68", 52, 9, 0, {"1": "HV_OUT", "2": "PAD_A"}, "Vishay VR68000004705JAC00 (10 kV, 1 W)"),
    ("R3", "1G", "MOX750", 77.4, 40, 0, {"1": "PAD_A", "2": "VMON"}, "Ohmite MOX-750231007FE (5 kV, 1 %)"),
    ("K1", "SHV05-1A85-78x4K", "SHV", 77.4, 24, 0, {"1": "PAD_A", "2": "+5V", "3": "COIL_N", "4": "DUMP"},
     "Standex-Meder SHV05-1A85-78D4K (4 kV breakdown; see README on its switching rating)"),
    ("R4", "1M", "VR37", 53.5, 24, 0, {"1": "DUMP", "2": "PAD_B"}, "Vishay VR37000001004JA100 (3.5 kV, pulse rated)"),
    ("J3", "PAD A", "WIREPAD", 77.4, 46, 0, {"1": "PAD_A"}, "solder pad for silicone HV wire, >= 5 kV"),
    ("J4", "PAD B", "WIREPAD", 62, 46, 0, {"1": "PAD_B"}, "solder pad for silicone HV wire, >= 5 kV"),
    # --- grip sensing: pad return current through R5 ---
    ("R5", "10k", R, 39, 28.5, 0, {"1": "PAD_B", "2": "GND"}, "10 k 0.1 % 0805"),
    ("R6", "1k", R, 39, 31.5, 0, {"1": "PAD_B", "2": "ISENSE"}, "1 k 1 % 0805"),
    ("D1", "BAV199", SOT23, 39, 35, 0, {"1": "GND", "2": "3V3", "3": "ISENSE"}, "BAV199 (check pin numbers on the datasheet)"),
    # --- pad voltage monitor: 1 G / 750 k gives 2.15 V at full pad voltage ---
    ("R7", "750k", R, 44, 38.5, 0, {"1": "VMON", "2": "GND"}, "750 k 1 % 0805"),
    ("C6", "1n", C, 44, 41.5, 0, {"1": "VMON", "2": "GND"}, "1 nF 50 V C0G 0805"),
    ("R12", "1k", R, 44, 44.5, 0, {"1": "VMON", "2": "VMON_ADC"}, "1 k 1 % 0805"),
    ("D2", "BAV199", SOT23, 39.5, 42, 0, {"1": "GND", "2": "3V3", "3": "VMON_ADC"}, "BAV199"),
    # --- relay driver ---
    ("R13", "100R", R, 44, 29.5, 0, {"1": "RELAY", "2": "GATE"}, "100 R 1 % 0805"),
    ("R14", "100k", R, 44, 32.5, 0, {"1": "GATE", "2": "GND"}, "100 k 1 % 0805"),
    ("Q1", "2N7002", SOT23, 48.5, 31, 0, {"1": "GATE", "2": "GND", "3": "COIL_N"}, "2N7002"),
    ("D4", "1N4148W", SOD, 48, 35.5, 0, {"1": "+5V", "2": "COIL_N"}, "1N4148W"),
    # --- status and arm sense ---
    ("D5", "red", "LED_SMD:LED_0805_2012Metric", 24.3, 30, 0, {"1": "GND", "2": "LED_A"}, "red LED 0805"),
    ("R15", "1k", R, 24.3, 33, 0, {"1": "LED", "2": "LED_A"}, "1 k 1 % 0805"),
    ("R16", "10k", R, 24.3, 36, 0, {"1": "VHV_IN", "2": "ARM_S"}, "10 k 1 % 0805"),
    ("R17", "15k", R, 24.3, 39, 0, {"1": "ARM_S", "2": "GND"}, "15 k 1 % 0805"),
    # --- CAN ---
    ("U4", "SN65HVD230", "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm", 16.5, 21.5, 0,
     {"1": "CAN_TX", "2": "GND", "3": "3V3", "4": "CAN_RX", "6": "CANL", "7": "CANH", "8": "RS"}, "TI SN65HVD230DR"),
    ("C8", "100n", C, 11.2, 23, 90, {"1": "3V3", "2": "GND"}, "100 nF 50 V X7R 0805"),
    ("R18", "10k", R, 18.6, 17.2, 0, {"1": "RS", "2": "GND"}, "10 k 1 % 0805"),
    ("R19", "120R", R, 7, 14.5, 0, {"1": "CANH", "2": "TERM"}, "120 R 1 % 0805"),
    ("JP1", "CAN end", "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical", 2.5, 13, 0,
     {"1": "TERM", "2": "CANL"}, "2-pin header + jumper (fit on the last board of the bus)"),
]
# hand-routed high-voltage tracks on the bottom layer: net, [(x, y), ...]
HV_ROUTES = [("HV_OUT", [(48, 9), (52, 9)]), ("PAD_A", [(77.4, 9), (77.4, 46)]), ("DUMP", [(57.08, 24), (53.5, 24)])]

mm = pcbnew.FromMM
P = lambda x, y: pcbnew.VECTOR2I(mm(OX + x), mm(OY + y))


def build():
    b = pcbnew.BOARD()
    nets = {}
    for part in PARTS:
        for n in part[6].values():
            if n not in nets:
                nets[n] = pcbnew.NETINFO_ITEM(b, n)
                b.Add(nets[n])

    def silk_rect(parent, x0, y0, x1, y1, layer=pcbnew.F_SilkS, w=0.15):
        s = pcbnew.PCB_SHAPE(parent)
        s.SetShape(pcbnew.SHAPE_T_RECT)
        s.SetStart(P(x0, y0)); s.SetEnd(P(x1, y1)); s.SetLayer(layer); s.SetWidth(mm(w))
        parent.Add(s)

    def text(s, x, y, size=1.0, layer=pcbnew.F_SilkS):
        t = pcbnew.PCB_TEXT(b)
        t.SetText(s); t.SetPosition(P(x, y)); t.SetLayer(layer)
        t.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size))); t.SetTextThickness(mm(size * 0.15))
        b.Add(t)

    for ref, value, fp, x, y, rot, pins, _ in PARTS:
        if fp in CUSTOM:
            pads, (drill, dia), (bx0, by0, bx1, by1) = CUSTOM[fp]
            f = pcbnew.FOOTPRINT(b)
            f.SetPosition(P(x, y))
            for name, (px, py) in pads.items():
                pad = pcbnew.PAD(f)
                pad.SetNumber(name); pad.SetAttribute(pcbnew.PAD_ATTRIB_PTH)
                pad.SetShape(pcbnew.F_Cu, pcbnew.PAD_SHAPE_CIRCLE)
                pad.SetSize(pcbnew.F_Cu, pcbnew.VECTOR2I(mm(dia), mm(dia)))
                pad.SetDrillSize(pcbnew.VECTOR2I(mm(drill), mm(drill)))
                pad.SetLayerSet(pcbnew.PAD.PTHMask()); pad.SetPosition(P(x + px, y + py))
                f.Add(pad)
            silk_rect(f, x + bx0, y + by0, x + bx1, y + by1)
            f.Reference().SetPosition(P(x + (bx0 + bx1) / 2, y + by0 - 1.0))
        else:
            lib, name = fp.split(":")
            f = pcbnew.FootprintLoad(LIB + lib + ".pretty", name)
            f.SetPosition(P(x, y)); f.SetOrientationDegrees(rot)
        f.SetReference(ref); f.SetValue(value)
        f.Value().SetVisible(False)
        b.Add(f)
        seen = set()
        for pad in f.Pads():
            if pad.GetNumber() in pins:
                pad.SetNet(nets[pins[pad.GetNumber()]]); seen.add(pad.GetNumber())
        assert seen == set(pins), f"{ref}: footprint has no pad {set(pins) - seen}"

    silk_rect(b, 0, 0, W, H, pcbnew.Edge_Cuts, 0.1)
    for x, y in HOLES:
        f = pcbnew.FootprintLoad(LIB + "MountingHole.pretty", "MountingHole_3.2mm_M3")
        f.SetPosition(P(x, y)); f.Reference().SetVisible(False); b.Add(f)
    text("ElectroClimb driver v1", 14.5, 48.5, 0.9)
    text("github.com/aghasalim/electroclimb  CERN-OHL-S", 40, 2.2, 0.9)
    text("HIGH VOLTAGE 3 kV", 66, 31, 1.4)
    text("coat this area", 66, 33.5, 1.0)
    silk_rect(b, 50.5, 3.5, 81.5, 49.5)                    # the HV zone
    silk_rect(b, 35.5, 44.2, 41.5, 49.2)                       # sponsor logo goes here (AISLER asks for it on the silkscreen)
    text("LOGO", 38.5, 46.7, 0.8)
    text("A", 77.4, 48.8, 1.0); text("B", 62, 48.8, 1.0)
    text("USB", 3.6, 35, 0.8)                              # which way round the XIAO goes
    return b, nets


# ---------------------------------------------------------------- grid router (low-voltage nets only)
# ponytail: no rip-up. If a net cannot be connected, the nets are tried in another order (main()).
# Good for a board this size; a denser board should go through KiCad's interactive router or Freerouting.
G = 0.1                                   # grid, mm
NX, NY = int(W / G) + 1, int(H / G) + 1
BLOCK = 60000                             # cell owner: 0 = free, n = only net n may use it, BLOCK = nobody
TRACK = {True: 0.4, False: 0.25}          # track width for power / signal nets
KEEP = 0.4                                # clearance 0.2 + half a signal track 0.125 + margin 0.075
VIA_D, VIA_DRILL, VIA_COST = 0.8, 0.4, 25
XS, YS = np.meshgrid(np.arange(NX) * G, np.arange(NY) * G, indexing="ij")


def claim(grid, layers, x0, y0, x1, y1, grow, net):
    """Give every cell within `grow` mm of the rectangle to `net`; cells two nets want go to nobody."""
    i0, i1 = max(0, int((x0 - grow) / G)), min(NX - 1, int((x1 + grow) / G) + 1)
    j0, j1 = max(0, int((y0 - grow) / G)), min(NY - 1, int((y1 + grow) / G) + 1)
    X, Y = XS[i0:i1 + 1, j0:j1 + 1], YS[i0:i1 + 1, j0:j1 + 1]
    dx, dy = np.maximum(np.maximum(x0 - X, X - x1), 0), np.maximum(np.maximum(y0 - Y, Y - y1), 0)
    near = dx * dx + dy * dy <= grow * grow
    for layer in layers:
        a = grid[layer, i0:i1 + 1, j0:j1 + 1]
        a[near & (a != 0) & (a != net)] = BLOCK
        a[near & (a == 0)] = net


def find_path(free, via_ok, sources, target):
    """A* from any source cell to the target cell. Cells are (layer, i, j). Returns the path or None."""
    tl, ti, tj = target
    dist, prev, heap = {}, {}, []
    for s in sources:
        dist[s] = 0.0
        heapq.heappush(heap, (max(abs(s[1] - ti), abs(s[2] - tj)), 0.0, s))
    while heap:
        _, d, cur = heapq.heappop(heap)
        if d > dist[cur]:
            continue
        l, i, j = cur
        if (i, j) == (ti, tj) and (tl is None or l == tl):
            path = [cur]
            while cur in prev:
                cur = prev[cur]; path.append(cur)
            return path[::-1]
        steps = [(l, i + di, j + dj, 1.0 if di == 0 or dj == 0 else 1.4142)
                 for di in (-1, 0, 1) for dj in (-1, 0, 1) if (di or dj)]
        if via_ok[i, j]:
            steps.append((1 - l, i, j, VIA_COST))
        for nl, ni, nj, cost in steps:
            if not free[nl, ni, nj]:
                continue
            nd, nxt = d + cost, (nl, ni, nj)
            if nd < dist.get(nxt, 1e18):
                dist[nxt], prev[nxt] = nd, cur
                heapq.heappush(heap, (nd + max(abs(ni - ti), abs(nj - tj)), nd, nxt))
    return None


def shrink(a, n):
    """True only where every cell within n cells is True (so a wider track or a via fits)."""
    out = a.copy()
    for di in range(-n, n + 1):
        for dj in range(-n, n + 1):
            out &= np.roll(np.roll(a, di, axis=-2), dj, axis=-1)
    return out


def route(b, nets, order_seed=0):
    """Adds all tracks to the board. Returns the nets it could not finish."""
    grid = np.zeros((2, NX, NY), dtype=np.uint16)          # layer 0 = F.Cu, 1 = B.Cu
    ids = {n: k + 1 for k, n in enumerate(nets)}
    grid[:, :8, :] = grid[:, -8:, :] = grid[:, :, :8] = grid[:, :, -8:] = BLOCK      # 0.7 mm from the board edge
    claim(grid, (0,), *MODULE_BODY, 0.3, BLOCK)
    for ref, _, fp, x, y, *_ in PARTS:                     # no top-side tracks under the HV resistors either:
        if fp in ("VR68", "MOX750", "VR37"):               # their bodies sit at up to 3 kV
            bx0, by0, bx1, by1 = CUSTOM[fp][2]
            claim(grid, (0,), x + bx0, y + by0, x + bx1, y + by1, 1.0, BLOCK)
    for x, y in HOLES:
        claim(grid, (0, 1), x, y, x, y, 3.2, BLOCK)                                   # room for the screw head
    pins, spare = {}, 50000
    drilled = np.zeros((1, NX, NY), dtype=np.uint16)       # no via next to a pad's hole: change layer in the pad itself
    for f in b.GetFootprints():
        for pad in f.Pads():
            bb, c = pad.GetBoundingBox(), pad.GetPosition()
            box = [pcbnew.ToMM(v) - o for v, o in ((bb.GetLeft(), OX), (bb.GetTop(), OY), (bb.GetRight(), OX), (bb.GetBottom(), OY))]
            n = pad.GetNetname()
            layers = (0, 1) if pad.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH) else (0,)
            spare += 1
            claim(grid, layers, *box, HV_CLEAR + 0.3 if n in HV_NETS else KEEP, ids.get(n, spare))
            if len(layers) == 2:
                claim(drilled, (0,), *box, 0.5, 1)
            if n and n not in HV_NETS:
                xy = (pcbnew.ToMM(c.x) - OX, pcbnew.ToMM(c.y) - OY)
                pins.setdefault(n, []).append((layers, (round(xy[0] / G), round(xy[1] / G)), xy))

    def track(net, a, c, layer, width):
        t = pcbnew.PCB_TRACK(b)
        t.SetStart(P(*a)); t.SetEnd(P(*c)); t.SetWidth(mm(width)); t.SetNet(nets[net])
        t.SetLayer(pcbnew.B_Cu if layer else pcbnew.F_Cu); b.Add(t)

    for net, pts in HV_ROUTES:
        for a, c in zip(pts, pts[1:]):
            track(net, a, c, 1, 0.6)
            claim(grid, (0, 1), min(a[0], c[0]), min(a[1], c[1]), max(a[0], c[0]), max(a[1], c[1]), HV_CLEAR + 0.6, ids[net])

    span = lambda n: max(abs(p[1][0] - q[1][0]) + abs(p[1][1] - q[1][1]) for p in pins[n] for q in pins[n])
    order = sorted((n for n in pins if n != "GND"), key=span)
    if order_seed:
        np.random.RandomState(order_seed).shuffle(order)
    failed = []
    for net in order + ["GND"]:
        k, width = ids[net], TRACK[net in POWER]
        todo = list(pins[net])
        first = todo.pop(0)
        tree = {(l, *first[1]) for l in first[0]}
        while todo:
            todo.sort(key=lambda p: min(abs(p[1][0] - t[1]) + abs(p[1][1] - t[2]) for t in tree))
            layers, cell, xy = todo.pop(0)
            mine = (grid == 0) | (grid == k)
            free = shrink(mine, 1) if net in POWER else mine.copy()
            for l in (0, 1):                               # a pad's own centre is always a legal end point
                for _, pc, _ in pins[net]:
                    free[l, pc[0], pc[1]] = mine[l, pc[0], pc[1]]
            via_ok = shrink(mine[0] & mine[1], 5) & (drilled[0] == 0)
            holes = {pc for ls, pc, _ in pins[net] if len(ls) == 2}
            for pc in holes:
                via_ok[pc] = True
            path = find_path(free, via_ok, tree, (None if len(layers) == 2 else layers[0], *cell))
            if path is None:
                failed.append(f"{net} at ({xy[0]:.1f}, {xy[1]:.1f})"); break
            tree.update(path)
            if len(layers) == 2:
                tree.update({(0, *cell), (1, *cell)})
            pts = [(l, i * G, j * G) for l, i, j in path]
            runs, start = [], 0                            # split at vias, then merge straight stretches
            for n in range(1, len(pts) + 1):
                if n == len(pts) or pts[n][0] != pts[start][0]:
                    runs.append(pts[start:n]); start = n
            for r, run in enumerate(runs):
                keep = [run[0]] + [q for p, q, s in zip(run, run[1:], run[2:])
                                   if (round(q[1] - p[1], 3), round(q[2] - p[2], 3)) != (round(s[1] - q[1], 3), round(s[2] - q[2], 3))] + [run[-1]]
                for p, q in zip(keep, keep[1:]):
                    if p[1:] != q[1:]:
                        track(net, p[1:], q[1:], p[0], width)
                for _, x, y in run[::2] + [run[-1]]:
                    claim(grid, (run[0][0],), x, y, x, y, width / 2 + KEEP, k)
                if r and (round(run[0][1] / G), round(run[0][2] / G)) not in holes:
                    v = pcbnew.PCB_VIA(b)
                    v.SetPosition(P(*run[0][1:])); v.SetWidth(pcbnew.F_Cu, mm(VIA_D)); v.SetDrill(mm(VIA_DRILL))
                    v.SetNet(nets[net]); b.Add(v)
                    claim(grid, (0, 1), run[0][1], run[0][2], run[0][1], run[0][2], VIA_D / 2 + KEEP, k)
    return failed


def write_tables():
    with open(HERE / "bom.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["ref", "value", "footprint", "part to order"])
        for ref, value, fp, *_, order in sorted(PARTS, key=lambda p: p[0]):
            w.writerow([ref, value, fp.split(":")[-1], order])
    by_net = {}
    for ref, *_, pins, _ in PARTS:
        for pin, net in pins.items():
            by_net.setdefault(net, []).append(f"{ref}.{pin}")
    lines = ["# Driver board netlist", "", "Generated by `driver_board.py`. Every net and the pins on it.", "",
             "| Net | Pins |", "|---|---|"]
    lines += [f"| `{n}`{' (high voltage)' if n in HV_NETS else ''} | {', '.join(p)} |" for n, p in sorted(by_net.items())]
    (HERE / "netlist.md").write_text("\n".join(lines) + "\n")
    hv = " || ".join(f"A.NetName == '{n}'" for n in HV_NETS)
    (HERE / f"{NAME}.kicad_dru").write_text(
        f'(version 1)\n(rule "HV clearance"\n  (condition "{hv}")\n  (constraint clearance (min {HV_CLEAR}mm)))\n')


def main():
    refs = [p[0] for p in PARTS]
    assert len(refs) == len(set(refs)), "duplicate reference"
    for seed in range(12):                 # seed 0 = shortest nets first; then shuffled orders
        b, nets = build()
        failed = route(b, nets, seed)
        print(f"routing order {seed}: " + ("all nets connected" if not failed else f"could not connect {failed}"))
        if not failed:
            break
    assert not failed, "router gave up: move parts apart"
    pcbnew.SaveBoard(str(HERE / f"{NAME}.kicad_pcb"), b)
    write_tables()
    print(f"{len(PARTS)} parts, {len(nets)} nets, board {W:.0f} x {H:.0f} mm, {len(b.GetTracks())} track segments and vias")


if __name__ == "__main__":
    main()
