#!/bin/sh
# Regenerates the driver board, checks it with KiCad and exports the files a PCB maker needs.
# Needs KiCad 10 (macOS paths below; on Linux use `python3` with pcbnew and plain `kicad-cli`).
set -e
cd "$(dirname "$0")"
KPY=${KPY:-/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/3.9/bin/python3}
"$KPY" driver_board.py 2>&1 | grep -v "create wxApp"
kicad-cli pcb drc --severity-error --exit-code-violations -o drc.rpt electroclimb_driver.kicad_pcb
rm -rf fab && mkdir fab
kicad-cli pcb export gerbers -o fab/ --layers F.Cu,B.Cu,F.Mask,B.Mask,F.SilkS,B.SilkS,F.Paste,Edge.Cuts electroclimb_driver.kicad_pcb >/dev/null
kicad-cli pcb export drill -o fab/ electroclimb_driver.kicad_pcb >/dev/null
kicad-cli pcb export pos --units mm --format csv -o fab/electroclimb_driver-pos.csv electroclimb_driver.kicad_pcb >/dev/null
(cd fab && zip -q electroclimb_driver.zip *)
kicad-cli pcb render --side top --width 1640 --height 1000 --zoom 1.25 --background opaque -o board_top.png electroclimb_driver.kicad_pcb >/dev/null
rm -f electroclimb_driver.kicad_prl
grep "Found" drc.rpt
echo "ok: board generated, 0 rule errors, Gerbers in fab/electroclimb_driver.zip"
