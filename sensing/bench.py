#!/usr/bin/env python3
"""Level 1 bench test for WallScan: predictions before the test, analysis after it.

    ../.venv/bin/python bench.py predict             # what the field model expects -> bench_prediction.csv
    ../.venv/bin/python bench.py analyze results.csv # compare your measurements with it

The pad is WS_Pad_80x60_S8_W7_G2: 8 strips, 7 mm wide, 2 mm apart, 52 mm long.
Samples (hardware/foot/BUILD.md, level 1):
  plain  40 mm of dry concrete
  void   5 mm of concrete, 3 mm air gap, concrete behind
  steel  10 mm steel bar, 8 mm under the surface, under the middle of the pad
  wet    a wet zone 30 mm wide from 1 to 12 mm deep

results.csv has the columns  sample,repeat,pair,pF  (pair written like 1-5). Lift and replace
the pad between repeats, so the spread includes how the pad happens to sit.
The model is 2D, static and ideal (see wallscan.py); expect the real numbers to differ. What
matters is whether the samples differ by more than the repeats scatter.
"""
import csv
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

import wallscan as w

HERE = Path(__file__).resolve().parent
EPS0 = 8.854e-12
STRIP_LEN = 0.052                      # m
PAIRS = [(1, 2), (1, 3), (1, 5), (1, 8), (4, 5)]
SAMPLES = ["plain", "void", "steel", "wet"]
CONTACT = [3.0, 3.75, 4.5, 5.25, 6.0]  # contact-layer permittivity: how well the pad happens to sit


def setup():
    """Point the field solver at the real test pad: 160 mm wide domain, 60 mm of material."""
    w.NX, w.NAIR, w.NDEPTH = 161, 10, 60
    w.NZ = w.NAIR + 1 + w.NDEPTH
    w.N_EL, w.EL_W, w.EL_GAP = 8, 7, 2
    x0 = (w.NX - (8 * 7 + 7 * 2)) // 2
    w.ELECTRODES = [np.arange(x0 + k * 9, x0 + k * 9 + 8) for k in range(8)]
    w.PAIRS = [(i, j) for i in range(8) for j in range(i, 8)]


def wall(sample, contact):
    eps = np.full((w.NZ, w.NX), 2.0)
    top = w.NAIR + 1
    eps[top:] = 6.5
    eps[top + 40:] = 2.5                                            # wooden bench under the sample
    zz, xx = np.mgrid[0:w.NZ, 0:w.NX]
    depth, xc = zz - top, w.NX // 2
    if sample == "void":
        eps[(depth >= 5) & (depth < 8)] = 1.0
    elif sample == "steel":
        eps[(xx - xc) ** 2 + (depth - 13) ** 2 <= 25] = 1e4
    elif sample == "wet":
        eps[(abs(xx - xc) <= 15) & (depth >= 1) & (depth < 12)] = 20.0
    eps[top] = contact
    return eps


def predict():
    setup()
    index = {p: k for k, p in enumerate(w.PAIRS)}
    table = {}
    for s in SAMPLES:
        runs = np.array([w.capacitances(wall(s, c)) for c in CONTACT]) * EPS0 * STRIP_LEN * 1e12     # pF
        table[s] = {p: runs[:, index[(p[0] - 1, p[1] - 1)]] for p in PAIRS}
    rows = []
    print(f"{'pair':6s}" + "".join(f"{s:>22s}" for s in SAMPLES))
    for p in PAIRS:
        base = table["plain"][p]
        line = f"{p[0]}-{p[1]:<4d}"
        for s in SAMPLES:
            v = table[s][p]
            change = (v.mean() / base.mean() - 1) * 100
            spread = v.std() / v.mean() * 100
            line += f"{v.mean():9.3f} pF {change:+6.0f} %  "
            rows.append({"sample": s, "pair": f"{p[0]}-{p[1]}", "pF": round(float(v.mean()), 4),
                         "change_vs_plain_pct": round(float(change), 1), "contact_spread_pct": round(float(spread), 1)})
        print(line)
    print("\nspread from pad contact alone (std / mean), plain sample: "
          + ", ".join(f"{p[0]}-{p[1]}: {table['plain'][p].std() / table['plain'][p].mean() * 100:.0f} %" for p in PAIRS))
    with open(HERE / "bench_prediction.csv", "w", newline="") as fh:
        wr = csv.DictWriter(fh, fieldnames=list(rows[0]))
        wr.writeheader(), wr.writerows(rows)
    print("saved bench_prediction.csv")


def analyze(path):
    data = defaultdict(list)
    with open(path) as fh:
        for row in csv.DictReader(fh):
            if row["pF"].strip():
                data[(row["sample"].strip(), row["pair"].strip())].append(float(row["pF"]))
    if not data:
        sys.exit("No measurements in the file yet: fill in the pF column.")
    pred = {}
    if (HERE / "bench_prediction.csv").exists():
        with open(HERE / "bench_prediction.csv") as fh:
            pred = {(r["sample"], r["pair"]): float(r["change_vs_plain_pct"]) for r in csv.DictReader(fh)}
    pairs = sorted({k[1] for k in data})
    print(f"{'sample':7s}{'pair':6s}{'mean pF':>9s}{'repeat spread':>15s}{'vs plain':>10s}{'predicted':>11s}  verdict")
    seen = 0
    for s in [x for x in SAMPLES if x != "plain"]:
        for p in pairs:
            if (s, p) not in data or ("plain", p) not in data:
                continue
            a, b = np.array(data[(s, p)]), np.array(data[("plain", p)])
            change = (a.mean() / b.mean() - 1) * 100
            spread = max(a.std(ddof=1) / a.mean(), b.std(ddof=1) / b.mean()) * 100 if min(len(a), len(b)) > 1 else float("nan")
            clear = abs(change) > 3 * spread
            seen += clear
            guess = f"{pred[(s, p)]:+.0f} %" if (s, p) in pred else "-"
            print(f"{s:7s}{p:6s}{a.mean():9.3f}{spread:13.1f} %{change:+8.1f} %{guess:>11s}  "
                  + ("CLEAR: larger than 3x the repeat spread" if clear else "not clear"))
    print(f"\n{seen} sample/pair combinations stand out clearly from the repeat spread.")
    print("If most far pairs (1-5, 1-8) are clear, the idea survived its first test.")


if __name__ == "__main__":
    if sys.argv[1:2] == ["predict"]:
        predict()
    elif sys.argv[1:2] == ["analyze"] and len(sys.argv) > 2:
        analyze(sys.argv[2])
    else:
        print(__doc__)
