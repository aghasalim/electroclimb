# Build one foot

A step-by-step guide to build and test **one ElectroClimb foot**. It is split into three levels so you can get real measurements early and cheaply, and only handle high voltage at the end.

<p align="center"><img src="cad/preview.png" width="760" alt="Foot CAD: assembled and exploded"></p>

> ⚠️ **High voltage in levels 2 and 3.** 1–3 kV at microamps can still give a painful shock and destroy electronics. Work with a supervisor, use the current-limiting resistor and bleeder described below, and discharge the pad before touching it. Level 1 uses no high voltage.

> **Status:** nobody has built this foot yet. The CAD is checked by script (valid solids, parts fit, printable walls) but not printed. Treat every number as a starting point and write down what you change.

| Level | What you learn | High voltage | Rough cost |
|---|---|---|---|
| **1. WallScan bench test** | Can pad electrodes sense a void, water or steel in concrete? | none | €30–60 |
| **2. Hold test** | How many newtons does a pad hold on glass, wood and concrete? | lab supply | €40–70 + a borrowed HV supply |
| **3. Foot with its own driver** | Switching, release time, grip sensing on the foot | on-board | + €230–270 for the HV module |

## Files

| File | What it is |
|---|---|
| [`cad/foot_carrier_tpu.step`](cad/foot_carrier_tpu.step) / `.stl` | Pad carrier with the flexure ankle, one piece, TPU 95A |
| [`cad/shin_adapter_petg.step`](cad/shin_adapter_petg.step) / `.stl` | Clamp for the 10 mm carbon shin tube, PETG or PA-CF |
| [`cad/foot_assembly.step`](cad/foot_assembly.step) | Both parts in place |
| [`cad/foot_cad.py`](cad/foot_cad.py) | The parametric source (CadQuery). Change `PARAMS`, re-run, the checks run again. |
| [`electroclimb.pretty/EA_Pad_80x60_F1_G1.kicad_mod`](electroclimb.pretty/) | Adhesion pad: two interdigitated combs, 1 mm fingers and gaps |
| [`electroclimb.pretty/WS_Pad_80x60_S8_W7_G2.kicad_mod`](electroclimb.pretty/) | WallScan test pad: 8 separate strips, 7 mm wide, 2 mm apart, 52 mm long |
| [`fab/EA_Pad_80x60_F1_G1.zip`](fab/EA_Pad_80x60_F1_G1.zip), [`fab/WS_Pad_80x60_S8_W7_G2.zip`](fab/WS_Pad_80x60_S8_W7_G2.zip) | **Order-ready Gerber files** for both pads: copper, coverlay openings, outline. Upload the zip to the PCB maker. |
| [`driver/`](driver/) | The driver board for level 3: KiCad file, Gerbers, parts list, schematic |
| [`../../sensing/bench.py`](../../sensing/bench.py), [`bench_results_template.csv`](../../sensing/bench_results_template.csv) | Predictions for the level 1 test, the sheet to fill in and the script that analyses it |

## Parts

Prices are rough, from distributor listings in November 2026. Check them before ordering.

| # | Part | Qty | For level | About |
|---|---|---|---|---|
| 1 | Flex PCB, 1 layer, polyimide, 80 × 60 mm (adhesion pad and WallScan pad) | 5 of each | 1, 2 | €25–40 per design for 5 pieces, plus shipping. Covered by the AISLER or Seeed sponsorship if agreed. |
| 2 | TPU 95A filament, 25 g | – | all | €1 |
| 3 | PETG or PA-CF filament, 10 g | – | all | €0.50 |
| 4 | Carbon tube 10 mm outer / 8 mm inner, 100 mm | 1 | all | €3 |
| 5 | M4 × 12 socket-head screw + M4 hex nut | 1 | all | €0.30 |
| 6 | M3 × 16 screw + M3 nut (tube clamp) | 1 | all | €0.20 |
| 7 | Adhesive transfer tape, 50 µm (3M 468MP or similar), 80 × 60 mm | 1 | all | €2 |
| 8 | Silicone HV wire rated ≥ 5 kV, 0.5 m red + 0.5 m black | 1 | 2, 3 | €4 |
| 9 | HV resistor 47 MΩ, ≥ 5 kV (current limit) and 1 GΩ, ≥ 5 kV (bleeder) | 1 each | 2, 3 | €6 |
| 10 | Capacitance meter: LCR meter from the lab, or an FDC2214 chip on a breakout (chip about €5–7) | 1 | 1 | lab, or €10–20 |
| 11 | Luggage scale or 5 kg load cell + HX711 | 1 | 2 | €8 |
| 12 | Test surfaces: glass pane, plywood, a concrete paving slab | 1 each | 1, 2 | €10 |
| 13 | XP Power A30P-5, 0–3 kV HV module | 1 | 3 | **about €270** ([Digi-Key](https://www.digikey.de/en/products/detail/xp-power/A30P-5/5873510), 23 in stock on 24 November 2026). |
| 14 | Driver board and its parts: PCB, HV reed relay, HV resistors, XIAO ESP32-S3, small parts (see [`driver/bom.csv`](driver/bom.csv)) | 1 set | 3 | roughly €40–60 without the module (an estimate: the relay price was not checked) |

**The HV module is the expensive part.** Four feet with split pads would need eight of them, which is over €2,000. Finding a cheaper high-voltage source is an open task; one shared supply with a relay per pad half is the first option to test.

## Print

| Part | Material | Settings |
|---|---|---|
| `foot_carrier_tpu` | TPU 95A | Plate flat on the bed. 0.2 mm layers, 3 walls, 100 % infill in the neck (use a modifier or print it all solid: 25 g), slow (25 mm/s), no supports: the cone under the flange is 45°. |
| `shin_adapter_petg` | PETG or PA-CF | Flange on the bed. 0.2 mm layers, 4 walls, 40 % infill, supports under the tube socket where it overhangs the flange. |

After printing, check: the M4 nut slides into the slot in the flange, the two nubs fit the adapter's sockets, and the carbon tube slides 22 mm into the socket.

The neck is the ankle: a **soft pivot**, 4.5 mm thick and 5 mm long. With TPU at E ≈ 26 MPa its bending stiffness is about 0.1 N·m/rad, and the cone under the flange lies down on the plate at about 45° of tilt, which is the end stop. It is this soft on purpose (changed 25 November 2026): with the earlier 14 mm neck (6 N·m/rad) the simulated pads stood on an edge and the robot fell as soon as pads only gripped when flat; see [`sim/README.md`](../../sim/README.md#flat-pads-steering-the-corner-and-the-ceiling-trot). `foot_cad.py` checks the neck by beam theory: 35 % strain at 45°, buckling at 52 N (the largest push in simulation is about 20 N), 0.9 MPa at the full 15 N pull. Real TPU varies by about ±50 %, so measure it (step 6) and change `neck_d` if needed: stiffness grows with the fourth power of the diameter. Not checked: fatigue over many cycles, and how much the thin neck lets the pad twist or shift sideways.

## Assemble

1. **Pad on the carrier.** Stick the transfer tape to the flat underside of the carrier plate and trim it. Lay the flex PCB on a clean glass plate with the electrode side down, and press the carrier onto its back, tabs towards the two notches. Roll it down firmly; no bubbles.
2. **Wires.** Solder the HV wires to the two tabs (level 1: thin hook-up wire to each of the 8 tabs). Keep the joints as flat as you can and cover them with polyimide tape. Route the wires up through the notches.
3. **Nut.** Slide the M4 nut into the slot in the flange.
4. **Adapter.** Seat the adapter on the two nubs and fasten it with the M4 × 12 screw from the top. Snug only; TPU compresses.
5. **Tube.** Push the carbon tube 22 mm into the socket and tighten the M3 clamp screw.
6. **Check the ankle.** Hold the tube and push the pad edge sideways with a scale: tilting the pad 20° (0.35 rad) at 40 mm from the centre should take about 0.9 N (90 g on the scale) if the stiffness is 0.1 N·m/rad. It must stay at or under 0.15 N·m/rad: in simulation the robot climbs walls and ceilings up to that value, only walls up to 0.25, and nothing from 0.3 on. The printed value has just a 50 % margin, which is also how much TPU varies, so do measure it. Note the real value.

## Level 1: WallScan bench test (no high voltage)

Goal: find out whether the pair capacitances really change over a void, water and steel, as the simulation says.

**Order:** upload [`fab/WS_Pad_80x60_S8_W7_G2.zip`](fab/WS_Pad_80x60_S8_W7_G2.zip) to the PCB maker. Options to choose: flexible PCB, 1 layer, polyimide, 18 µm copper, polyimide coverlay, no stiffener, 5 pieces.

**Samples** (about 150 × 150 mm each):

| Name | How to make it |
|---|---|
| `plain` | A concrete paving slab, about 40 mm thick, dry. |
| `void` | A 5 mm fibre-cement or thin concrete tile on the slab with 3 mm spacers at the corners: an air gap 5 mm under the surface. |
| `steel` | A 10 mm steel bar in a groove so its top is 8 mm under the surface, or under an 8 mm tile. Bar under the middle of the pad, parallel to the strips. |
| `wet` | The plain slab with a 30 mm wide strip soaked for an hour, surface wiped dry. Strip under the middle of the pad, parallel to the strips. |

**Measure:**
1. Lay the pad on the sample, strips down, with the carrier and a 1 kg weight on top. Strips are numbered 1 to 8 from one side.
2. Measure the capacitance between strip pairs 1–2, 1–3, 1–5, 1–8 and 4–5 at 10 kHz. Leave the other strips unconnected in the same way every time.
3. Five repeats per sample: lift the pad and put it back each time.
4. Type the values into a copy of [`bench_results_template.csv`](../../sensing/bench_results_template.csv), then run:

```bash
cd sensing
../.venv/bin/python bench.py analyze my_results.csv
```

**What the field model predicts** (`bench.py predict`; 2D, ideal, so expect differences):

| Pair | plain | void | steel | wet |
|---|---|---|---|---|
| 1–2 | 2.00 pF | +1 % | 0 % | −2 % |
| 1–3 | 0.30 pF | **−32 %** | −2 % | +3 % |
| 1–5 | 0.07 pF | **−50 %** | **+22 %** | **+56 %** |
| 1–8 | 0.02 pF | **−33 %** | +7 % | −12 % |
| 4–5 | 1.87 pF | +3 % | −1 % | **+14 %** |

How the pad happens to sit changes the neighbour pairs by about 6 % and the far pairs by about 2 % in the model.

**This is a hard measurement.** The interesting signals are hundredths of a picofarad, and a metre of ordinary cable has about 100 pF. Use short shielded leads, zero the meter with the leads in place and the pad lifted, and keep hands and cables still. Start with pair 1–3 on `void`: 0.3 pF changing by a third is the easiest prediction to check. If the lab meter cannot resolve 0.01 pF, a capacitance-to-digital chip (FDC2214, AD7746) right next to the pad is the next step.

**What counts as a result:** `bench.py analyze` marks a sample/pair as CLEAR when its change against `plain` is more than three times the spread between repeats. If the far pairs come out clear and roughly in the predicted direction, the idea survived its first test. If the repeats scatter more than the samples differ, the contact layer or the cabling dominates, and that is the answer.

## Level 2: hold test (lab high-voltage supply)

Goal: the single most important number in the project. How much shear force does one pad hold?

1. Wire the pad to the lab HV supply **through the 47 MΩ resistor**, with the 1 GΩ bleeder across the pad. Supply off.
2. Clamp the test surface vertically. Press the pad on it and hang the scale from the tube.
3. Supervisor present. Switch on, raise to 1 kV, wait 10 s, then pull down on the scale slowly until the pad slips. Note the force.
4. Repeat at 2 kV and 3 kV, on glass, wood and concrete, five times each. Switch off and wait for the bleeder (about 5 s) before touching the pad.

| Result per pad at 3 kV | Shear stress (48 cm² pad) | What it means |
|---|---|---|
| ≥ 15 N | ≥ 3.1 kPa | Matches the simulation. The robot design stands for this surface. |
| 5–15 N | 1–3 kPa | The robot must get lighter or the pads bigger for this surface. |
| < 5 N | < 1 kPa | This surface is out of reach for a 1.5 kg robot with these pads. |

Put the measured values into `adh_gain` in the simulation; that replaces the 15 N placeholder that every result so far rests on.

## Level 3: the foot's own driver

Build the driver board in [`driver/`](driver/) and work through its list of open points first. Then measure: time to full grip, release time with the reed relay, and whether the grip signal on the sense resistor tells contact from no contact.

## Open questions this build should answer

- Does the transfer tape hold flex PCB to TPU under shear, or is a different adhesive needed?
- Is one flexure neck stiff enough sideways and soft enough to let the pad lie flat?
- **Adhesion needs fine fingers (about 1 mm); WallScan worked with wide strips (5–8 mm).** One pad that does both needs a layout that has not been designed yet, for example fine combs grouped into 8 separately wired zones.
- Can a cheaper high-voltage source replace the €230+ module?
