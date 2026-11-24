# Bill of materials: whole robot

Written on 24 November 2026. **Nothing here has been built.** Part numbers marked ✔ were read in the manufacturer's datasheet or drawing on that day. Prices are what a distributor page or search result showed then, without shipping or VAT; "–" means not priced. Check stock and price before ordering.

Per-board and per-part detail: [driver board](foot/driver/bom.csv), [foot](foot/BUILD.md#parts), [printed parts and fasteners](robot/cad/README.md).

## 1. Motion

| Qty | Part | Notes | Price each | Source / status |
|---|---|---|---|---|
| 12 (+2 spare) | Feetech **STS3215-C018** servo, 12 V, 30 kg·cm ✔ | 45.2 × 24.7 × 35 mm, 55 g, 0.9 A rated, 2.7 A stalled | – | **Sponsored by Feetech** (14 pieces). Shipping of $46 is still to pay. |
| 1 | Feetech **FE-URT-2** USB servo adapter | USB to the one-wire servo bus | – | In the Feetech order |
| 6 | Feetech servo cable, 500 mm | Each servo also comes with a short cable | – | In the Feetech order |
| 1 | Servo splitter (X3) | Stripboard with four 3-pin 2.54 mm headers: 12 V and ground from the 12 V terminals, the data line common | – | Make it |

## 2. Power

| Qty | Part | Notes | Price each | Source / status |
|---|---|---|---|---|
| 1 | 3S LiPo, 11.1 V, 1300 mAh, XT60 | For example Gens ace Soaring 1300 mAh 30C: about 107 g, 72 × 35 × 21 mm | – | Size and weight from a search result, not from the product page |
| 1 | LiPo balance charger and a LiPo bag | Never charge the pack on the robot | – | |
| 1 | Inline blade fuse holder + 15 A fuse | Between battery and switch | – | |
| 1 | Main switch, 15 A DC or more | | – | |
| 4 | Lever connector, 5-way (Wago 221 type) | X1: 12 V + and −. X2: 5 V + and − | – | |
| 1 | 5 V converter, 5 A: **Pololu D36V50F5** (#4091) ✔ | 25.4 × 25.4 × 9.5 mm. Feeds the Pi and the four foot boards. Its current at 12 V input was not read from the graph: measure, and add a second converter for the feet if the Pi browns out. | $44.95 | pololu.com |
| 1 | Key switch (arm) | In series in the armed 5 V line | – | |
| 1 | E-stop button, normally closed | In series in the armed 5 V line | – | |
| 1 m each | Silicone wire 16 AWG red + black | Battery to the 12 V terminals and to the servo splitter | – | |
| 1 m each | Silicone wire 18 AWG red + black | 12 V to the converter, 5 V to the Pi | – | |

## 3. Computer and sensors

| Qty | Part | Notes | Price each | Source / status |
|---|---|---|---|---|
| 1 | **Raspberry Pi 5** ✔ | 85 × 56 mm, holes 58 × 49 mm. Powered through header pins 2, 4 and 6. | – | |
| 1 | microSD card, 64 GB | | – | |
| 1 | **Raspberry Pi Camera Module 3** ✔ + Pi 5 camera cable | 25 × 24 mm, holes 21 × 12.5 mm (the 12.5 was hard to read on the drawing: measure) | – | |
| 1 | IMU breakout on I2C, 3.3 V | **Model not chosen yet** | – | Open |
| 1 | USB-CAN adapter with Linux SocketCAN (gs_usb) support | Candidate: Innomaker USB2CAN (RS 230-5617). Its gs_usb support was not confirmed. | €42.61 | RS Germany, search result |

## 4. Feet (per foot; the robot has 4)

| Qty | Part | Notes | Price each | Source / status |
|---|---|---|---|---|
| 1 | Flexible PCB pad, 80 × 60 mm | [Gerbers](foot/fab/). 1 layer, polyimide, 18 µm copper, coverlay | €25–40 for 5 | AISLER will quote once both pads are uploaded to their site; NextPCB and PCBWay have been asked |
| 1 | [Driver board](foot/driver/) with parts, without the module | 82 × 50 mm, 43 parts | roughly €40–60 | PCB sponsorship agreed by AISLER. Estimate: the relay was not priced. |
| 1 | **XP Power A30P-5** high-voltage module ✔ | 0–3 kV, 1 W | **€268.89** | DigiKey Germany, 23 in stock |
| 1 | Foot carrier, TPU 95A | [STL](foot/cad/) | 25 g filament | Print |
| 1 | Shin adapter, PETG or PA-CF | [STL](foot/cad/) | 10 g filament | Print |
| 1 | Carbon tube 10 mm outer / 8 mm inner, **74.7 mm** | | – | |
| 1 | Microswitch with lever (foot contact) | To J2 on the driver board | – | Mounting on the foot is not designed yet |
| 0.3 m each | Silicone wire rated 5 kV or more, red + black | Driver board to pad | – | |
| 1 | Leg cable: 5 wires, 26 AWG silicone, 35 cm, JST XH 5-pin plug | To J1 | – | |
| 1 | Transfer tape 50 µm (3M 468MP type), 80 × 60 mm | Pad to carrier | – | |
| 1 | M4 × 12 screw + nut, M3 × 16 screw + nut | Foot adapter | – | |

A split pad (two halves driven separately, as in the simulation's best result) needs two driver boards and two modules per foot.

## 5. Frame

| Qty | Part | Notes |
|---|---|---|
| 345 g | PETG or PA-CF filament | [Printed parts](robot/cad/README.md#parts): chassis, top cover, 4 hip brackets, 4 thighs, 4 shin links |
| 96 | M2 × 6 self-tapping pan head | Servo cases to the printed plates |
| 48 + 24 | M3 × 6 button head with washer, M3 × 8 button head | Output horns and idler horns |
| 4 + 4 | M3 × 8 + M3 heat-set insert | Top cover |
| 4 + 4 | M2.5 × 6 + M2.5 heat-set insert | Raspberry Pi |
| 4 + 4 | M2 × 8 + M2 nut | Camera |
| 12 + 12 | M3 × 10 nylon screw + nylon spacer | Driver boards (3 per board). A mount for the board on the leg is not designed yet. |
| 2 | Battery strap, 200 mm | |

## 6. Safety and test equipment

| Qty | Part | Notes |
|---|---|---|
| 1 | Safety tether: 3 mm cord and a small carabiner | From an anchor above the robot to the tether eye. The robot will fall during testing. |
| 1 | High-voltage probe, 1000:1, rated 5 kV or more | Lab equipment. Never a normal multimeter on the pad. |
| 1 | Conformal coating or neutral-cure silicone | For the high-voltage area of each driver board |
| 1 | Load cell 5 kg + HX711, or a luggage scale | Hold test |
| – | Test surfaces: glass, plywood, concrete slab | |

## Cost that is known

| Item | Cost |
|---|---|
| 4 high-voltage modules | about €1,076 |
| 5 V converter | $45 |
| USB-CAN adapter | about €43 |
| Servos, adapter, cables | sponsored, $46 shipping |
| Everything else | not priced |

The four high-voltage modules are the largest cost by far. With split pads it doubles to about €2,150.

## Mass budget

| Item | Mass | Basis |
|---|---|---|
| 12 servos | 660 g | Feetech drawing: 55 g each |
| Printed frame | 345 g | CAD volume, solid PETG |
| Frame fasteners | about 68 g | Estimate |
| Battery, 1300 mAh | about 107 g | Search result |
| Pi 5, camera, converter, adapters, wiring | about 105 g | Estimate |
| 4 feet: carrier, adapter, pad, tube | about 154 g | CAD volume and estimates |
| 4 driver boards with modules | about 128 g | Module 7.1 g from the datasheet; board and parts estimated at 25 g |
| **Total** | **about 1.57 kg** | |

The simulation assumes 1.46 kg. This build comes out about 7 % heavier, and about 0.13 kg more again with split pads. In simulation, 10 % more pad force took the success rate from 88 % to 95 %, so 7 % more weight costs a similar amount in the other direction. Before building the full robot: put the real mass into the simulation and run it again, and measure the real pad force (the hold test in the foot build guide).
