#!/usr/bin/env python3
"""Crack detection for the robot's camera: a small CNN, trained on concrete photos.

Data: "Concrete Crack Images for Classification", C. F. Ozgenel, Mendeley Data v2,
doi:10.17632/5y9wdsg2zt.2, CC BY 4.0 (20,000 cracked + 20,000 uncracked 227 px photos).
Unpack it to data/cracks/{Positive,Negative}/ (data/ is git-ignored).

    ../.venv/bin/python crack_model.py train                 # -> crack_cnn.pt + test accuracy
    ../.venv/bin/python crack_model.py scan wall.jpg         # -> wall_cracks.png (crack map)

The scan cuts a large photo into overlapping tiles, scores each tile and marks the cracked
ones, so one camera frame of a wall becomes a map of where the cracks are.
"""
import sys
from pathlib import Path

import numpy as np
import torch
from PIL import Image, ImageDraw
from torch import nn
from torch.utils.data import DataLoader, Dataset, random_split

HERE = Path(__file__).resolve().parent
DATA = HERE.parent / "data" / "cracks"
WEIGHTS = HERE / "crack_cnn.pt"
SIZE = 96                      # tiles are resized to 96 x 96
DEVICE = "mps" if torch.backends.mps.is_available() else "cpu"


def to_tensor(img):
    a = np.asarray(img.convert("RGB").resize((SIZE, SIZE)), dtype=np.float32) / 255.0
    return torch.from_numpy((a - 0.5) / 0.25).permute(2, 0, 1)


class Cracks(Dataset):
    def __init__(self):
        self.items = [(p, 1) for p in sorted((DATA / "Positive").glob("*.jpg"))] + \
                     [(p, 0) for p in sorted((DATA / "Negative").glob("*.jpg"))]

    def __len__(self):
        return len(self.items)

    def __getitem__(self, i):
        path, label = self.items[i]
        return to_tensor(Image.open(path)), torch.tensor(label, dtype=torch.float32)


def make_model():
    def block(cin, cout):
        return nn.Sequential(nn.Conv2d(cin, cout, 3, padding=1), nn.BatchNorm2d(cout), nn.ReLU(), nn.MaxPool2d(2))
    return nn.Sequential(block(3, 16), block(16, 32), block(32, 64), block(64, 64),
                         nn.AdaptiveAvgPool2d(1), nn.Flatten(), nn.Dropout(0.2), nn.Linear(64, 1))


def accuracy(model, loader):
    model.train(False)
    right = n = 0
    with torch.no_grad():
        for x, y in loader:
            p = model(x.to(DEVICE)).squeeze(1).cpu() > 0
            right += int((p == (y > 0.5)).sum())
            n += len(y)
    return right / n


def train(epochs=4):
    ds = Cracks()
    assert len(ds) == 40_000, f"expected 40,000 images in {DATA}, found {len(ds)}"
    g = torch.Generator().manual_seed(0)                         # fixed split: 80 / 10 / 10
    tr, va, te = random_split(ds, [32_000, 4_000, 4_000], generator=g)
    loader = lambda d, shuf: DataLoader(d, batch_size=128, shuffle=shuf, num_workers=4)
    model = make_model().to(DEVICE)
    opt = torch.optim.AdamW(model.parameters(), lr=2e-3, weight_decay=1e-4)
    lossf = nn.BCEWithLogitsLoss()
    for ep in range(epochs):
        model.train(True)
        for x, y in loader(tr, True):
            x = torch.where(torch.rand(len(x), 1, 1, 1) < 0.5, x, x.flip(3))   # random mirror
            loss = lossf(model(x.to(DEVICE)).squeeze(1), y.to(DEVICE))
            opt.zero_grad()
            loss.backward()
            opt.step()
        print(f"epoch {ep + 1}: validation accuracy {accuracy(model, loader(va, False)):.2%}", flush=True)
    torch.save(model.state_dict(), WEIGHTS)
    print(f"held-out test accuracy: {accuracy(model, loader(te, False)):.2%} on {len(te):,} unseen photos")


def load_model():
    model = make_model()
    model.load_state_dict(torch.load(WEIGHTS, map_location="cpu"))
    model.train(False)
    return model


def scan(path, tile=227, stride=113, model=None):
    """Crack probability per tile of a large photo. Returns (grid of probabilities, marked image)."""
    model = model or load_model()
    img = Image.open(path).convert("RGB")
    W, H = img.size
    xs = list(range(0, max(1, W - tile + 1), stride))
    ys = list(range(0, max(1, H - tile + 1), stride))
    tiles = [to_tensor(img.crop((x, y, x + tile, y + tile))) for y in ys for x in xs]
    with torch.no_grad():
        prob = torch.sigmoid(model(torch.stack(tiles)).squeeze(1)).numpy().reshape(len(ys), len(xs))
    marked = img.copy()
    d = ImageDraw.Draw(marked, "RGBA")
    for i, y in enumerate(ys):
        for j, x in enumerate(xs):
            if prob[i, j] > 0.5:
                d.rectangle((x + 4, y + 4, x + tile - 4, y + tile - 4), outline=(230, 60, 40, 255), width=3,
                            fill=(230, 60, 40, int(90 * prob[i, j])))
    return prob, marked


if __name__ == "__main__":
    if sys.argv[1:2] == ["train"]:
        train()
    elif sys.argv[1:2] == ["scan"]:
        prob, marked = scan(sys.argv[2])
        out = Path(sys.argv[2]).with_name(Path(sys.argv[2]).stem + "_cracks.png")
        marked.save(out)
        print(f"{(prob > 0.5).sum()} of {prob.size} tiles show cracks -> {out}")
    else:
        print(__doc__)
