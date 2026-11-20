#!/usr/bin/env python3
"""WallScan: can the electroadhesive foot pad also see inside the wall?  (simulation study)

Idea: split each pad's comb into 8 separately wired electrode strips. While the pad holds the
robot, measure the capacitance between every pair of strips. Neighbouring strips sense the
first millimetres; strips far apart sense deeper. A neural network turns the 36 numbers into
what is under the foot: a void (delamination), a wet zone or a rebar, and where.

This file is the first test of that idea, in simulation only:
  1. forward model: 2D finite-difference electrostatics, div(eps grad phi) = 0, through a
     cross-section of pad + concrete with a random defect (1 mm grid)
  2. dataset: random walls (permittivity, contact layer, defect type / size / depth) + sensor noise
  3. inverse model: small network -> defect class, position, depth and a defect mask
  4. baselines: what one total capacitance (today's grip sensing) or only neighbour pairs can do

    ../.venv/bin/python wallscan.py data        # simulate 12,000 footsteps  (~5 min, 8 cores)
    ../.venv/bin/python wallscan.py train       # train + evaluate, writes ../media/wallscan.png

Limits: 2D, static field, ideal electrodes, 1 mm grid (the real insulator is 50 um), rebar as a
high-permittivity disc, no conductivity or frequency effects. It tests whether the information
is in the measurements at all, not whether the electronics can read it next to 3 kV.
"""
import sys
from multiprocessing import Pool
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
DATA = HERE.parent / "data" / "wallscan.npz"
NX, NAIR, NDEPTH = 121, 10, 40           # 1 mm grid: 120 mm wide, 10 mm above the pad, 40 mm of concrete
NZ = NAIR + 1 + NDEPTH
N_EL, EL_W, EL_GAP = 8, 5, 2             # 8 strips, 5 mm wide, 2 mm apart = 54 mm of a 60 mm pad
X0 = (NX - (N_EL * EL_W + (N_EL - 1) * EL_GAP)) // 2
ELECTRODES = [np.arange(X0 + k * (EL_W + EL_GAP), X0 + k * (EL_W + EL_GAP) + EL_W + 1) for k in range(N_EL)]
PAIRS = [(i, j) for i in range(N_EL) for j in range(i, N_EL)]          # 8 self + 28 mutual = 36 numbers
CLASSES = ["sound concrete", "void / delamination", "wet zone", "rebar"]
MASK_X, MASK_Z = slice(30, 90), slice(NAIR + 1, NAIR + 1 + 30)        # 60 mm x 30 mm under the pad
NOISE = 0.01                             # 1 % measurement noise


def random_wall(rng):
    """Permittivity map (NZ x NX) with at most one defect. Returns eps, class, x (mm), depth (mm), mask."""
    eps = np.full((NZ, NX), 2.0)                                   # pad backing / air above the electrodes
    base = rng.uniform(5.0, 8.0)                                   # dry concrete
    tex = rng.normal(0, 1, (NDEPTH // 4 + 1, NX // 4 + 2))
    tex = np.kron(tex, np.ones((4, 4)))[:NDEPTH, :NX]
    eps[NAIR + 1:] = base * (1 + 0.08 * tex)
    eps[NAIR + 1] = rng.uniform(3.0, 6.0)                          # contact layer: insulator + air gaps
    zz, xx = np.mgrid[0:NZ, 0:NX]
    depth_mm = zz - (NAIR + 1)
    cls = int(rng.choice(4, p=[0.25, 0.25, 0.25, 0.25]))
    xc, dc = float(rng.uniform(38, 82)), 0.0
    defect = np.zeros((NZ, NX), bool)
    if cls == 1:                                                   # thin air gap parallel to the surface
        w, t, dc = rng.uniform(8, 30), rng.integers(1, 4), float(rng.uniform(2, 15))
        defect = (abs(xx - xc) <= w / 2) & (depth_mm >= dc) & (depth_mm < dc + t)
        eps[defect] = 1.0
    elif cls == 2:                                                 # moisture
        r, dc = rng.uniform(5, 12), float(rng.uniform(2, 15))
        defect = ((xx - xc) ** 2 + ((depth_mm - dc) * 1.4) ** 2 <= r ** 2) & (depth_mm >= 1)
        eps[defect] = rng.uniform(15, 30)
    elif cls == 3:                                                 # steel bar, seen end-on
        r, dc = rng.uniform(4, 8), float(rng.uniform(10, 25))
        defect = (xx - xc) ** 2 + (depth_mm - dc) ** 2 <= r ** 2
        eps[defect] = 1e4
    return eps, cls, xc, dc, defect[MASK_Z, MASK_X]


def capacitances(eps):
    """All 36 strip-to-strip capacitances (arbitrary units per metre of strip) for one wall."""
    import scipy.sparse as sp
    from scipy.sparse.linalg import splu
    idx = np.arange(NZ * NX).reshape(NZ, NX)
    fixed = np.zeros((NZ, NX), bool)
    fixed[0], fixed[-1], fixed[:, 0], fixed[:, -1] = True, True, True, True       # grounded far boundary
    for e in ELECTRODES:
        fixed[NAIR, e] = True
    face = lambda a, b: 2 * a * b / (a + b)                                        # harmonic mean
    rows, cols, vals = [], [], []
    diag = np.zeros((NZ, NX))
    for dz, dx in ((0, 1), (1, 0)):
        a, b = eps[: NZ - dz, : NX - dx], eps[dz:, dx:]
        w = face(a, b)
        i, j = idx[: NZ - dz, : NX - dx], idx[dz:, dx:]
        rows += [i.ravel(), j.ravel()]
        cols += [j.ravel(), i.ravel()]
        vals += [-w.ravel(), -w.ravel()]
        diag[: NZ - dz, : NX - dx] += w
        diag[dz:, dx:] += w
    L = sp.csr_matrix((np.concatenate(vals + [diag.ravel()]),
                       (np.concatenate(rows + [idx.ravel()]), np.concatenate(cols + [idx.ravel()]))))
    free = ~fixed.ravel()
    lu = splu(L[free][:, free].tocsc())
    out = np.zeros((N_EL, N_EL))
    for i, e in enumerate(ELECTRODES):
        phi = np.zeros(NZ * NX)
        phi[idx[NAIR, e]] = 1.0
        phi[free] = lu.solve(-(L[free][:, ~free] @ phi[~free]))
        q = L @ phi                                                                # net flux out of each node
        out[i] = [q[idx[NAIR, f]].sum() for f in ELECTRODES]
    out = (out + out.T) / 2
    return np.array([abs(out[i, j]) for i, j in PAIRS])


def _sample(seed):
    rng = np.random.default_rng(seed)
    eps, cls, xc, dc, mask = random_wall(rng)
    return capacitances(eps), cls, xc, dc, mask


def make_data(n=12_000):
    with Pool(8) as pool:
        out = pool.map(_sample, range(n), chunksize=50)
    DATA.parent.mkdir(exist_ok=True)
    np.savez_compressed(DATA, C=np.array([o[0] for o in out]), cls=np.array([o[1] for o in out]),
                        x=np.array([o[2] for o in out]), depth=np.array([o[3] for o in out]),
                        mask=np.array([o[4] for o in out]))
    print(f"simulated {n:,} footsteps -> {DATA}")


def train():
    import torch
    from torch import nn
    torch.manual_seed(0)
    d = np.load(DATA)
    rng = np.random.default_rng(0)
    C = d["C"] * (1 + NOISE * rng.normal(size=d["C"].shape))       # sensor noise
    X = np.log(C)
    n = len(X)
    tr, te = slice(0, int(0.8 * n)), slice(int(0.8 * n), n)
    mutual = [k for k, (i, j) in enumerate(PAIRS) if i != j]
    neighbours = [k for k, (i, j) in enumerate(PAIRS) if j == i + 1]
    feats = {"one total capacitance\n(today's grip sensing)": None,
             "7 neighbour pairs": neighbours,
             "all 36 pairs (WallScan)": list(range(len(PAIRS)))}
    T = lambda a, dt=torch.float32: torch.as_tensor(a, dtype=dt)
    cls, mask = T(d["cls"], torch.long), T(d["mask"][:, ::2, ::2])               # mask at 2 mm: 15 x 30
    pos = T(np.column_stack([(d["x"] - 60) / 22, d["depth"] / 25]))
    results, best = {}, None
    for name, cols in feats.items():
        F = np.log(C[:, mutual].sum(1, keepdims=True)) if cols is None else X[:, cols]
        mu, sd = F[tr].mean(0), F[tr].std(0) + 1e-9
        F = T((F - mu) / sd)
        net = nn.Sequential(nn.Linear(F.shape[1], 256), nn.ReLU(), nn.Linear(256, 256), nn.ReLU(),
                            nn.Linear(256, 4 + 2 + 450))
        opt = torch.optim.AdamW(net.parameters(), lr=2e-3, weight_decay=1e-4)
        sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, 300)
        has = (cls > 0).float()
        for _ in range(300):
            for b in torch.randperm(tr.stop).split(512):
                o = net(F[b])
                loss = nn.functional.cross_entropy(o[:, :4], cls[b]) \
                    + 2 * ((o[:, 4:6] - pos[b]) ** 2 * has[b, None]).mean() \
                    + nn.functional.binary_cross_entropy_with_logits(o[:, 6:], mask[b].flatten(1))
                opt.zero_grad()
                loss.backward()
                opt.step()
            sched.step()
        net.train(False)
        with torch.no_grad():
            o = net(F[te])
        pred = o[:, :4].argmax(1)
        truth = cls[te]
        hit = truth > 0
        pm = (o[:, 6:] > 0).view(-1, 15, 30)
        inter, union = (pm & (mask[te] > 0.5)).sum((1, 2)).float(), (pm | (mask[te] > 0.5)).sum((1, 2)).float()
        res = {"accuracy": float((pred == truth).float().mean()),
               "x_mm": float((o[hit, 4] - pos[te][hit, 0]).abs().mean() * 22),
               "depth_mm": float((o[hit, 5] - pos[te][hit, 1]).abs().mean() * 25),
               "iou": float((inter[hit] / union[hit].clamp(min=1)).mean()),
               "per_class": [float((pred[truth == k] == k).float().mean()) for k in range(4)]}
        results[name] = res
        print(f"{name.splitlines()[0]:28s} class accuracy {res['accuracy']:.1%}   x error {res['x_mm']:.1f} mm   "
              f"depth error {res['depth_mm']:.1f} mm   mask IoU {res['iou']:.2f}   per class "
              + " ".join(f"{a:.0%}" for a in res["per_class"]), flush=True)
        best = (pred.numpy(), pm.numpy(), o[:, 4:6].numpy())
    depth = d["depth"][te]
    truth = d["cls"][te]
    curves = {}
    for k in (1, 3):
        bins = np.arange(2, 16, 2.5) if k == 1 else np.arange(10, 26, 3)
        curves[k] = (bins[:-1] + np.diff(bins) / 2,
                     [float((best[0][(truth == k) & (depth >= a) & (depth < b)] == k).mean()) for a, b in zip(bins[:-1], bins[1:])])
        print(CLASSES[k], "found vs depth (mm):", " ".join(f"{x:.0f}:{y:.0%}" for x, y in zip(*curves[k])))
    plot(results, curves, d["mask"][te], truth, best)
    return results


def plot(results, curves, masks, truth, best):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    try:                                              # Apple's system font (San Francisco) when available
        font_manager.fontManager.addfont("/System/Library/Fonts/SFNS.ttf")
        plt.rcParams["font.family"] = "System Font"
    except (OSError, RuntimeError):
        pass
    pred, pm, _ = best
    fig = plt.figure(figsize=(13, 7.2), dpi=110)
    gs = fig.add_gridspec(3, 6, height_ratios=[1.25, 0.8, 0.8], hspace=0.55, wspace=0.5)
    ax = fig.add_subplot(gs[0, :3])
    names = list(results)
    acc = [results[k]["accuracy"] * 100 for k in names]
    bars = ax.barh(names[::-1], acc[::-1], color=["#e08a14", "#8a949c", "#8a949c"])
    ax.bar_label(bars, fmt="%.0f %%", padding=4)
    ax.axvline(25, color="#c2412d", ls="--", lw=1)
    ax.text(26, -0.45, "guessing", color="#c2412d", fontsize=8)
    ax.set_xlim(0, 110), ax.set_xlabel("defect type identified correctly (unseen footsteps)")
    ax.set_title("What the foot can tell about the wall", loc="left", fontweight="bold")
    ax = fig.add_subplot(gs[0, 3:])
    for k, c in ((1, "#2f6fb3"), (3, "#2e8a57")):
        ax.plot(curves[k][0], np.array(curves[k][1]) * 100, "o-", color=c, label=CLASSES[k])
    ax.set_ylim(0, 105), ax.set_xlabel("depth below the surface (mm)"), ax.set_ylabel("found (%)")
    ax.set_title("How deep it sees (all 36 pairs)", loc="left", fontweight="bold"), ax.legend(frameon=False)
    shown = []
    for k in (1, 2, 3):
        shown += list(np.where((truth == k) & (pred == k))[0][:2])
    for n, i in enumerate(shown):
        for row, img, title in ((1, masks[i], f"true: {CLASSES[truth[i]]}"), (2, pm[i], f"network: {CLASSES[pred[i]]}")):
            ax = fig.add_subplot(gs[row, n])
            ax.imshow(img, cmap="Oranges" if row == 1 else "Blues", extent=(0, 60, 30, 0), vmin=0, vmax=1)
            ax.set_title(title, fontsize=8), ax.set_xticks([0, 30, 60]), ax.set_yticks([0, 15, 30])
            ax.tick_params(labelsize=7)
            if n == 0:
                ax.set_ylabel("depth (mm)", fontsize=8)
    fig.text(0.5, 0.02, "Cross-sections under one 60 mm pad. Simulation only (2D field model, 1 % sensor noise).",
             ha="center", fontsize=9, color="#5d676d")
    out = HERE.parent / "media" / "wallscan.png"
    fig.savefig(out, bbox_inches="tight")
    print(f"saved {out}")


if __name__ == "__main__":
    if sys.argv[1:2] == ["data"]:
        make_data()
    elif sys.argv[1:2] == ["train"]:
        train()
    else:
        print(__doc__)
