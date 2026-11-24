#!/usr/bin/env python3
"""Draws the whole-robot diagrams as SVG: system_scheme.svg and wiring_diagram.svg.

    python3 hardware/system/draw.py

Plain SVG text, no dependencies. Edit the coordinates below and re-run.
"""
from pathlib import Path

HERE = Path(__file__).resolve().parent
FONT = "-apple-system, 'SF Pro Text', 'Helvetica Neue', Helvetica, Arial, sans-serif"
# one colour per kind of connection, used by both diagrams
C = dict(v12="#d62728", v5="#e8860c", arm="#8c2d04", data="#1f77b4", hv="#c2185b", mech="#7f7f7f", ink="#1a1a1a")


class Svg:
    def __init__(self, w, h, title, sub):
        self.w, self.h, self.out = w, h, []
        self.text(30, 38, title, 22, weight="600")
        self.text(30, 60, sub, 13, color="#555")

    def text(self, x, y, s, size=12, color=C["ink"], anchor="start", weight="400"):
        s = s.replace("&", "&amp;").replace("<", "&lt;")
        self.out.append(f'<text x="{x}" y="{y}" font-size="{size}" fill="{color}" text-anchor="{anchor}" font-weight="{weight}">{s}</text>')

    def frame(self, x, y, w, h, caption):
        self.out.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="10" fill="#f6f7f9" stroke="#c9ced6"/>')
        self.text(x + 12, y + 20, caption, 13, "#555", weight="600")

    def box(self, x, y, w, h, title, lines=(), fill="#ffffff", stroke=C["ink"]):
        self.out.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="6" fill="{fill}" stroke="{stroke}" stroke-width="1.4"/>')
        self.text(x + 10, y + 20, title, 13, weight="600")
        for i, s in enumerate(lines):
            self.text(x + 10, y + 38 + 15 * i, s, 11.5, "#444")

    def wire(self, pts, kind, label=None, at=None, dash=False, width=2.4):
        d = " ".join(f"{'M' if i == 0 else 'L'}{x} {y}" for i, (x, y) in enumerate(pts))
        style = ' stroke-dasharray="6 5"' if dash else ""
        self.out.append(f'<path d="{d}" fill="none" stroke="{C[kind]}" stroke-width="{width}" stroke-linejoin="round"{style}/>')
        if label:
            x, y = at or ((pts[0][0] + pts[1][0]) / 2, pts[0][1] - 6)
            self.text(x, y, label, 11, C[kind], "middle", "600")

    def dot(self, x, y, kind):
        self.out.append(f'<circle cx="{x}" cy="{y}" r="4" fill="{C[kind]}"/>')

    def legend(self, x, y, items):
        for i, (kind, s, dash) in enumerate(items):
            self.wire([(x, y + 18 * i), (x + 34, y + 18 * i)], kind, dash=dash)
            self.text(x + 44, y + 4 + 18 * i, s, 11.5, "#444")

    def save(self, name):
        body = "\n".join(self.out)
        (HERE / name).write_text(
            f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {self.w} {self.h}" width="{self.w}" height="{self.h}" '
            f'font-family="{FONT}">\n<rect width="{self.w}" height="{self.h}" fill="#ffffff"/>\n{body}\n</svg>\n')
        print("wrote", name)


def system_scheme():
    s = Svg(1400, 850, "ElectroClimb: system scheme",
            "What is on the robot and how the parts are connected. Design status: not built yet; the simulation uses the same layout.")
    s.frame(30, 80, 250, 720, "On the ground")
    s.frame(310, 80, 580, 720, "Body")
    s.frame(920, 80, 450, 720, "One leg (the robot has 4)")

    # ground
    s.box(50, 330, 210, 75, "Operator laptop", ["mission, voice, live view", "can stop the robot"])
    s.box(50, 470, 210, 75, "Safety tether", ["rope from an anchor above", "to the tether eye on the body"])
    s.wire([(260, 365), (350, 365)], "data", "Wi-Fi", (296, 358), dash=True)
    s.wire([(260, 507), (310, 507)], "mech")

    # body: power on top and bottom, data in the middle, so the lines do not cross
    s.box(350, 110, 150, 65, "Battery", ["3S LiPo, 11.1 V", "XT60 plug"])
    s.box(530, 110, 170, 65, "Fuse + main switch", ["15 A blade fuse", "everything off here"])
    s.wire([(500, 142), (530, 142)], "v12")
    s.box(350, 215, 100, 50, "Camera", ["CSI"])
    s.box(470, 215, 100, 50, "IMU", ["I2C"])
    s.wire([(400, 265), (400, 290)], "data")
    s.wire([(520, 265), (520, 290)], "data")
    s.box(350, 290, 220, 180, "Raspberry Pi 5", ["walking gait and RL policy", "crack detection on camera", "mission plan and voice",
                                                  "sets each pad's voltage", "reads grip, IMU, servo angles", "", "Wi-Fi to the operator"])
    s.box(640, 300, 190, 65, "Servo bus adapter", ["USB to one-wire serial", "1 Mbit/s, servo IDs 1 to 12"])
    s.box(640, 400, 190, 65, "USB-CAN adapter", ["500 kbit/s", "120 Ω at both bus ends"])
    s.wire([(570, 332), (640, 332)], "data", "USB", (605, 326))
    s.wire([(570, 432), (640, 432)], "data", "USB", (605, 426))
    s.box(350, 610, 170, 65, "5 V converter", ["12 V in, 5 V / 5 A out", "feeds the Pi and the feet"])
    s.box(600, 610, 230, 65, "Arm key switch + E-stop", ["both must be closed", "or no pad gets high voltage"])
    s.wire([(545, 175), (545, 197), (332, 197), (332, 642), (350, 642)], "v12", "12 V", (440, 191))
    s.wire([(435, 610), (435, 470)], "v5", "5 V", (452, 545))
    s.wire([(520, 642), (600, 642)], "v5")
    s.dot(560, 642, "v5")

    # leg
    s.box(950, 250, 400, 110, "3 servos: abduction, hip, knee", ["Feetech STS3215, 12 V version, daisy-chained",
                                                                  "each reports angle, load, voltage and temperature",
                                                                  "0.9 A rated, 2.7 A stalled, each"])
    s.box(950, 400, 250, 250, "Foot driver board", ["XIAO ESP32-S3 microcontroller", "", "0 to 3 kV module, set by the XIAO",
                                                    "current limit: 64 µA in a short", "bleeder: pad safe in seconds",
                                                    "release relay: about 1 ms", "measures pad voltage and charge",
                                                    "(charge / voltage = grip signal)", "", "one board per pad;",
                                                    "a split pad needs two"])
    s.box(1235, 415, 115, 150, "Pad", ["flexible PCB", "80 × 60 mm", "", "electrode A", "electrode B", "", "TPU carrier,", "spring ankle"])
    s.box(1235, 590, 115, 55, "Contact switch", ["foot on the wall"])
    s.wire([(1200, 470), (1235, 470)], "hv")
    s.wire([(1200, 500), (1235, 500)], "hv")
    s.wire([(1200, 617), (1235, 617)], "data")
    s.wire([(700, 142), (1150, 142), (1150, 250)], "v12", "12 V servo power", (925, 136))
    s.wire([(830, 332), (950, 332)], "data", "servo bus", (890, 326))
    s.wire([(830, 432), (950, 432)], "data", "CAN bus", (890, 426))
    s.wire([(830, 642), (880, 642), (880, 700), (1010, 700), (1010, 650)], "arm", "armed 5 V for the HV modules", (990, 716))
    s.wire([(560, 642), (560, 745), (1100, 745), (1100, 650)], "v5", "5 V logic", (700, 739))

    s.legend(50, 600, [("v12", "12 V battery power", False), ("v5", "5 V logic power", False),
                       ("arm", "5 V after arm switch and E-stop", False), ("data", "data", False),
                       ("data", "wireless", True), ("hv", "high voltage, 0 to 3 kV, microamps", False),
                       ("mech", "mechanical", False)])
    s.text(50, 745, "Ground (0 V) returns are not drawn.", 11.5, "#444")
    s.text(50, 762, "They run next to every power line.", 11.5, "#444")
    s.save("system_scheme.svg")


LEGS = [("FL", "front left", 1), ("FR", "front right", 4), ("HL", "hind left", 7), ("HR", "hind right", 10)]


def wiring_diagram():
    s = Svg(1500, 1000, "ElectroClimb: wiring diagram",
            "Every cable on the robot. Ground wires are not drawn: each power wire has a ground wire of the same size next to it.")
    # 12 V chain along the top
    s.box(40, 110, 150, 80, "BT1 battery", ["3S LiPo 11.1 V", "XT60 plug"])
    s.box(230, 110, 130, 80, "F1 fuse", ["15 A blade fuse", "inline holder"])
    s.box(400, 110, 130, 80, "SW1 main switch", ["rated 15 A DC", "or more"])
    s.box(570, 110, 150, 80, "X1 12 V terminals", ["two lever", "connectors: + and −"])
    for x0, x1 in ((190, 230), (360, 400), (530, 570)):
        s.wire([(x0, 150), (x1, 150)], "v12", width=3.4)
    s.text(380, 215, "battery to X1: 16 AWG silicone wire, as short as possible", 11, C["v12"], "middle", "600")

    # servos
    s.box(800, 110, 200, 245, "A1 adapter + X3 splitter", ["Feetech FE-URT-2: USB to the", "Pi, one data line out", "", "X3 splitter: four 3-pin",
                                                      "headers (2.54 mm). 12 V and", "ground from X1 on every port,",
                                                      "the data line common to all.", "", "Set servo IDs one by one", "before wiring the chains."])
    s.wire([(720, 150), (800, 150)], "v12", "12 V, 16 AWG", (760, 143), width=3.4)
    for i, (code, name, first) in enumerate(LEGS):
        y = 110 + 65 * i
        s.box(1080, y, 380, 50, f"{code} servos ({name})", [f"ID {first} abduction → ID {first + 1} hip → ID {first + 2} knee"])
        s.wire([(1000, y + 22), (1080, y + 22)], "v12")
        s.wire([(1000, y + 28), (1080, y + 28)], "data")
    s.text(1040, 100, "3-wire servo cables", 11, "#444", "middle")
    s.text(1270, 385, "Servo cable: GND, 12 V, data. Servos are daisy-chained with the cables Feetech supplies.", 11, "#444", "middle")

    # 5 V
    s.wire([(645, 190), (645, 430)], "v12", "12 V, 18 AWG", (690, 300))
    s.box(570, 430, 150, 80, "U4 5 V converter", ["12 V in", "5 V / 5 A out"])
    s.box(800, 430, 150, 80, "X2 5 V terminals", ["two lever", "connectors: + and −"])
    s.wire([(720, 470), (800, 470)], "v5", "18 AWG", (760, 463), width=3.4)
    s.box(1030, 430, 130, 80, "SW2 arm key", ["key switch,", "off = no HV"])
    s.box(1200, 430, 130, 80, "SW3 E-stop", ["normally closed,", "push = no HV"])
    s.wire([(950, 470), (1030, 470)], "v5")
    s.wire([(1160, 470), (1200, 470)], "arm")

    # Pi and what hangs on it
    s.box(40, 430, 130, 70, "Camera", ["Camera Module 3"])
    s.box(200, 430, 150, 70, "IMU", ["on I2C, 3.3 V"])
    s.wire([(105, 500), (105, 560)], "data", "CSI ribbon", (140, 535))
    s.wire([(275, 500), (275, 560)], "data", "pins 1, 3, 5, 9", (325, 535))
    s.box(40, 560, 380, 240, "Raspberry Pi 5", ["power: 5 V to header pins 2 and 4, ground to pin 6",
                                               "  (set usb_max_current_enable=1, or the Pi limits USB current)",
                                               "USB: A1 servo bus adapter", "USB: A2 CAN adapter",
                                               "CSI: camera", "I2C (pins 3 and 5): IMU", "Wi-Fi: operator laptop", "",
                                               "The Pi never touches 12 V or high voltage.",
                                               "A stuck Pi cannot keep a pad live: the arm key and",
                                               "E-stop cut the 5 V feed of the HV modules directly."])
    s.wire([(840, 510), (840, 600), (420, 600)], "v5", "5 V to the Pi, 18 AWG", (630, 593), width=3.4)
    s.wire([(385, 560), (385, 390), (900, 390), (900, 355)], "data", "USB", (500, 383))
    s.box(500, 650, 180, 80, "A2 USB-CAN adapter", ["any SocketCAN (gs_usb) type", "500 kbit/s, 120 Ω on"])
    s.wire([(420, 690), (500, 690)], "data", "USB", (460, 683))

    # buses to the four foot boards
    xs = [800, 970, 1140, 1310]
    s.wire([(880, 510), (880, 620)], "v5")
    s.wire([(xs[0] + 40, 620), (xs[-1] + 40, 620)], "v5", "5 V logic", (990, 613))
    s.wire([(1265, 510), (1265, 650), (xs[-1] + 75, 650)], "arm")
    s.wire([(xs[0] + 75, 650), (1265, 650)], "arm", "armed 5 V", (1130, 643))
    s.wire([(680, 690), (xs[-1] + 110, 690)], "data", "CAN H and CAN L, twisted", (760, 683))
    for x, (code, name, _) in zip(xs, LEGS):
        s.wire([(x + 40, 620), (x + 40, 760)], "v5")
        s.wire([(x + 75, 650), (x + 75, 760)], "arm")
        s.wire([(x + 110, 690), (x + 110, 760)], "data")
        for dx, y, kind in ((40, 620, "v5"), (75, 650, "arm"), (110, 690, "data")):
            s.dot(x + dx, y, kind)
        s.box(x, 760, 150, 120, f"Foot driver {code}", ["J1: leg cable", "J2: contact switch", "pads A and B:", "two HV wires to the pad"])
        s.wire([(x + 60, 880), (x + 60, 925)], "hv", width=3)
        s.wire([(x + 90, 880), (x + 90, 925)], "hv", width=3)
        s.box(x, 925, 150, 40, f"Pad {code}")
    s.box(500, 760, 260, 205, "Leg cable (4 of them)", ["5 wires, 26 AWG silicone, about 35 cm", "5-pin plug on the foot board (J1)", "",
                                                         "1  5 V logic", "2  armed 5 V (HV module feed)", "3  ground", "4  CAN H", "5  CAN L", "",
                                                         "The last board on the bus gets jumper JP1."])
    s.legend(40, 850, [("v12", "12 V battery power", False), ("v5", "5 V logic power", False), ("arm", "5 V after arm key and E-stop", False),
                       ("data", "data", False), ("hv", "high voltage, 0 to 3 kV, microamps", False)])
    s.save("wiring_diagram.svg")


# ------------------------------------------------------------------ foot driver schematic
def driver_schematic():
    s = Svg(1500, 740, "ElectroClimb foot driver v1: schematic",
            "One board per pad. Red = high voltage (up to 3 kV, microamps). Nets with the same name are connected. "
            "Source of truth: hardware/foot/driver/driver_board.py")
    ink, hv = "ink", "hv"

    def w(pts, kind=ink, width=1.6):
        s.wire(pts, kind, width=width)

    def res(x, y, name, val, kind=ink, vertical=False):
        a, b = (7, 22) if vertical else (22, 7)
        s.out.append(f'<rect x="{x - a}" y="{y - b}" width="{2 * a}" height="{2 * b}" fill="#fff" stroke="{C[kind]}" stroke-width="1.6"/>')
        if vertical:
            s.text(x + 12, y - 2, name, 11, C[kind], weight="600"); s.text(x + 12, y + 11, val, 11, C[kind])
        else:
            s.text(x, y - 12, f"{name}  {val}", 11, C[kind], "middle", "600")

    def cap(x, y, label, kind=ink, left=False):   # vertical capacitor, wires meet it at y-4 and y+4
        w([(x - 11, y - 4), (x + 11, y - 4)], kind, 2.2); w([(x - 11, y + 4), (x + 11, y + 4)], kind, 2.2)
        s.text(x - 16 if left else x + 16, y + 4, label, 11, C[kind], "end" if left else "start")

    def gnd(x, y):
        w([(x, y), (x, y + 8)]); w([(x - 9, y + 8), (x + 9, y + 8)]); w([(x - 5, y + 12), (x + 5, y + 12)]); w([(x - 2, y + 16), (x + 2, y + 16)])

    def flag(x, y, name, where="right", kind=ink):
        s.out.append(f'<circle cx="{x}" cy="{y}" r="3" fill="{C[kind]}"/>')
        tx, ty, anchor = {"right": (x + 7, y + 4, "start"), "left": (x - 7, y + 4, "end"), "above": (x, y - 8, "middle")}[where]
        s.text(tx, ty, name, 11.5, C[kind], anchor, "600")

    def chip(x, y, wd, ht, title, left=(), right=()):
        s.box(x, y, wd, ht, title)
        for i, p in enumerate(left):
            s.text(x + 8, y + 45 + 25 * i, p, 11, "#444")
        for i, p in enumerate(right):
            s.text(x + wd - 8, y + 45 + 25 * i, p, 11, "#444", "end")
        return [y + 41 + 25 * i for i in range(max(len(left), len(right)))]

    def notes(x, y, lines, color="#444", size=11):
        for i, line in enumerate(lines):
            s.text(x, y + 16 * i, line, size, color)

    # --- leg cable and power
    s.frame(20, 80, 330, 300, "Leg cable and power")
    ys = chip(40, 110, 100, 160, "J1", right=["1  +5V", "2  +5V_ARM", "3  GND", "4  CANH", "5  CANL"])
    for y, name in zip(ys, ["+5V", "+5V_ARM", None, "CANH", "CANL"]):
        if name:
            w([(140, y), (190, y)]); flag(190, y, name)
        else:
            w([(140, y), (170, y)]); gnd(170, y)
    notes(40, 295, ["+5V → D3 schottky → 5V_XIAO.  C4 10 µF to GND.", "+5V_ARM → J2 → VHV_IN.  C2 10 µF, C3 100 nF to GND.",
                    "J2 is the foot contact switch (a jumper for bench tests).",
                    "+5V_ARM is cut by the arm key and the E-stop on the body."])

    # --- microcontroller
    s.frame(370, 80, 230, 300, "Microcontroller")
    chip(390, 110, 190, 230, "U2 XIAO ESP32-S3", left=["5V   5V_XIAO", "3V3  3V3", "GND"],
         right=["PWM  D3", "ISENSE  D0", "VMON_ADC  D1", "RELAY  D8", "ARM_S  D9", "LED  D10", "CAN_TX  D4", "CAN_RX  D5"])
    s.text(398, 362, "D2, D6 and D7 are not used", 10.5, "#888")

    # --- CAN
    s.frame(20, 400, 330, 190, "CAN bus")
    chip(40, 430, 150, 145, "U4 SN65HVD230", left=["1 D   CAN_TX", "4 R   CAN_RX", "3 VCC  3V3", "2 GND"], right=["CANH 7", "CANL 6", "RS 8"])
    notes(205, 462, ["R19 120 Ω + jumper JP1", "between CANH and CANL,", "only on the last board.", "", "R18 10 k: RS to GND", "C8 100 nF: 3V3 to GND"])

    # --- status
    s.frame(370, 400, 230, 190, "Status and arm sense")
    notes(385, 440, ["LED → R15 1 k → D5 red → GND"])
    notes(385, 456, ["lit by firmware while the pad is live"], "#888", 10.5)
    notes(385, 495, ["VHV_IN → R16 10 k → ARM_S", "ARM_S → R17 15 k → GND"])
    notes(385, 530, ["3 V when armed and the foot switch", "is closed, else 0 V"], "#888", 10.5)

    # --- voltage setpoint: PWM -> filter -> x1.51 -> control pin
    s.frame(620, 80, 370, 300, "Voltage setpoint: 3.3 V PWM to 0–5 V")
    flag(645, 160, "PWM", "above"); w([(645, 160), (668, 160)]); res(690, 160, "R1", "10 k"); w([(712, 160), (815, 160)])
    w([(730, 160), (730, 196)]); cap(730, 200, "C1 1 µ", left=True); w([(730, 204), (730, 232)]); gnd(730, 232)
    w([(762, 160), (762, 178)]); res(762, 200, "R11", "100 k", vertical=True); w([(762, 222), (762, 232)]); gnd(762, 232)
    s.out.append('<path d="M815 140 L815 220 L885 180 Z" fill="#fff" stroke="#1a1a1a" stroke-width="1.6"/>')
    s.text(821, 164, "+", 12); s.text(821, 206, "−", 12); s.text(836, 184, "U3", 11, weight="600")
    w([(885, 180), (908, 180)]); res(930, 180, "R10", "1 k"); w([(952, 180), (975, 180)]); flag(975, 180, "CTRL", "above")
    w([(963, 180), (963, 196)]); cap(963, 200, "C5 100 n", left=True); w([(963, 204), (963, 232)]); gnd(963, 232)
    w([(896, 180), (896, 270), (872, 270)]); res(850, 270, "R8", "5.1 k"); w([(828, 270), (795, 270), (795, 200), (815, 200)])
    w([(795, 270), (795, 283)]); res(795, 305, "R9", "10 k", vertical=True); w([(795, 327), (795, 335)]); gnd(795, 335)
    notes(632, 290, ["U3 MCP6001,", "supplied from VHV_IN.", "Gain 1 + 5.1/10 = 1.51."])
    notes(632, 368, ["PWM off or XIAO in reset: R11 holds the setpoint at 0 V."])

    # --- relay driver
    s.frame(620, 400, 370, 190, "Release relay driver")
    flag(645, 490, "RELAY", "above"); w([(645, 490), (690, 490)]); res(712, 490, "R13", "100 Ω"); w([(734, 490), (800, 490)])
    w([(765, 490), (765, 508)]); res(765, 530, "R14", "100 k", vertical=True); w([(765, 552), (765, 558)]); gnd(765, 558)
    s.box(800, 465, 70, 50, "Q1", ["2N7002"])
    w([(835, 515), (835, 558)]); gnd(835, 558)
    w([(835, 465), (835, 445), (900, 445)]); s.box(900, 428, 70, 34, "K1 coil")
    w([(935, 428), (935, 415)]); s.text(945, 422, "+5V", 11.5, weight="600")
    notes(895, 490, ["D4 1N4148W", "across the coil"])
    notes(895, 535, ["RELAY high =", "pad discharged"], "#888", 10.5)

    # --- high voltage
    s.frame(1010, 80, 470, 510, "High voltage")
    ys = chip(1030, 110, 150, 150, "U1 XP Power A30P-5", left=["2 +IN  VHV_IN", "5 CTRL  CTRL", "1 −IN  GND", "4 HV RTN  GND"], right=["HV OUT 3"])
    top, bot = ys[0], 440
    w([(1180, top), (1203, top)], hv, 2.4); res(1225, top, "R2", "47 M", hv); w([(1247, top), (1450, top)], hv, 2.4)
    flag(1450, top, "PAD_A", "above", hv)
    s.text(1225, top + 24, "limits a short to 64 µA", 10.5, C["hv"], "middle")
    xa, xp, xk = 1290, 1360, 1430
    w([(xa, top), (xa, 228)], hv, 2.4); res(xa, 250, "R3", "1 G", hv, vertical=True); w([(xa, 272), (xa, 310)], hv, 2.4)
    w([(xa, 310), (xa, 326)]); res(xa, 348, "R7", "750 k", vertical=True); w([(xa, 370), (xa, 378)]); gnd(xa, 378)
    w([(xa, 310), (1235, 310)]); res(1213, 310, "R12", "1 k"); w([(1191, 310), (1170, 310)]); flag(1170, 310, "VMON_ADC", "left")
    notes(1030, 335, ["C6 1 nF and clamp D2 (BAV199).", "R3 with R7: 2.15 V at full voltage."], size=10.5)
    w([(xp, top), (xp, 316)], hv, 2.4); cap(xp, 320, "", hv); w([(xp, 324), (xp, bot)], hv, 2.4)
    s.text(xp + 16, 318, "pad", 11, C["hv"], weight="600"); s.text(xp + 16, 332, "0.5–1 nF", 10.5, C["hv"])
    w([(xk, top), (xk, 200)], hv, 2.4)
    s.out.append(f'<path d="M{xk} 200 L{xk + 12} 228" fill="none" stroke="{C["hv"]}" stroke-width="2.4"/>')
    s.text(xk - 10, 212, "K1", 11, C["hv"], "end", "600"); s.text(xk - 10, 226, "contact", 10.5, C["hv"], "end")
    w([(xk, 232), (xk, 368)], hv, 2.4); res(xk, 390, "R4", "1 M", hv, vertical=True); w([(xk, 412), (xk, bot), (xp, bot)], hv, 2.4)
    w([(xp, bot), (1230, bot)]); s.text(1310, bot - 8, "PAD_B", 10.5, "#888", "middle"); flag(xp, bot, "", "right")
    w([(1255, bot), (1255, 458)]); res(1255, 480, "R5", "10 k", vertical=True); w([(1255, 502), (1255, 512)]); gnd(1255, 512)
    res(1208, bot, "R6", "1 k"); w([(1186, bot), (1170, bot)]); flag(1170, bot, "ISENSE", "left")
    notes(1030, 470, ["Clamp D1 (BAV199).", "The pad's charging current", "returns through R5: 0.64 V peak.",
                      "Its integral is the pad's charge,", "and charge / voltage is the grip signal."], size=10.5)

    notes(30, 630, [
        "How it works: the XIAO sets the pad voltage with PWM. U3 scales it to the 0–5 V the module's control pin needs. R3 and R7 report the real pad voltage, so the firmware can",
        "close the loop: the module is not regulated and overshoots at light load. K1 with R4 empties the pad in about 1 ms for a fast release.",
        "Safety: R3 always bleeds the pad (time constant 0.5 s). The arm key, the E-stop and the foot switch cut VHV_IN in hardware, without the microcontroller.",
        "Not yet verified on a bench: every value. K1 closes onto up to 3 kV, above its 1 kV switching rating (through 1 MΩ: 3 mA, 4.5 mJ). See the driver README.",
    ], "#333", 12)
    s.save("../foot/driver_schematic.svg")


if __name__ == "__main__":
    system_scheme()
    wiring_diagram()
    driver_schematic()
