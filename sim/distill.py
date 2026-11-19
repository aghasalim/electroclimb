#!/usr/bin/env python3
"""Teacher-student distillation (DAgger).

The teacher (train.py --privileged) also sees the true wall grip and which pads drop out,
which the real robot can't measure. The student sees only what the robot senses
(joints, IMU, pad capacitance, gait phase) and learns to copy the teacher:

  round 0: the teacher drives; every state is labelled with the teacher's action
  round k: the student drives; the teacher labels the states the student actually reaches

    ../.venv/bin/python distill.py runs_teacher/best/best_model.zip      # -> runs_student/

Only load teacher models this project trained itself.
"""
import sys
from multiprocessing import Pool
from pathlib import Path

import numpy as np
import torch
from torch import nn

from rl_env import ACT_DIM, HARD_WALLS, PRIV_DIM, ResidualClimbEnv, evaluate
from train import load_policy

OUT = Path(__file__).resolve().parent / "runs_student"
ROUNDS, EPISODES, SECONDS = 5, 36, 20.0
_teacher = None


class Student(nn.Module):
    def __init__(self, obs_dim, mean, std):
        super().__init__()
        self.register_buffer("mean", torch.as_tensor(mean, dtype=torch.float32))
        self.register_buffer("std", torch.as_tensor(std, dtype=torch.float32))
        self.net = nn.Sequential(nn.Linear(obs_dim, 256), nn.Tanh(), nn.Linear(256, 256), nn.Tanh(),
                                 nn.Linear(256, ACT_DIM), nn.Tanh())

    def forward(self, obs):
        return self.net((obs - self.mean) / self.std)

    def act(self, obs):
        with torch.no_grad():
            return self(torch.as_tensor(obs[None], dtype=torch.float32))[0].numpy()


def _init(teacher_path):
    global _teacher
    torch.set_num_threads(1)
    _teacher = load_policy(teacher_path)[0]


def _rollout(args):
    """One episode on a hard wall. Returns (student observations, teacher actions)."""
    seed, student_state = args
    student = None
    if student_state is not None:
        student = Student(student_state["obs_dim"], student_state["mean"], student_state["std"])
        student.load_state_dict(student_state["weights"])
    env = ResidualClimbEnv(tilt_range=(90, 90), episode_s=SECONDS, seed=seed, privileged=True, **HARD_WALLS)
    obs, _ = env.reset()
    X, Y = [], []
    done = trunc = False
    while not (done or trunc):
        label = _teacher(obs)
        X.append(obs[:-PRIV_DIM])
        Y.append(np.clip(label, -1, 1))
        act = label if student is None else student.act(obs[:-PRIV_DIM])
        obs, _, done, trunc, _ = env.step(act)
    return np.array(X), np.array(Y)


def fit(student, X, Y, epochs=8):
    opt = torch.optim.Adam(student.parameters(), lr=1e-3)
    X, Y = torch.as_tensor(X, dtype=torch.float32), torch.as_tensor(Y, dtype=torch.float32)
    for _ in range(epochs):
        for idx in torch.randperm(len(X)).split(4096):
            loss = nn.functional.mse_loss(student(X[idx]), Y[idx])
            opt.zero_grad()
            loss.backward()
            opt.step()
    return float(loss)


def main(teacher_path):
    OUT.mkdir(exist_ok=True)
    Xs, Ys, student, state = [], [], None, None
    with Pool(6, initializer=_init, initargs=(teacher_path,)) as pool:
        for r in range(ROUNDS):
            seeds = [10_000 + r * EPISODES + k for k in range(EPISODES)]   # never the 5000+ test walls
            for X, Y in pool.map(_rollout, [(s, state) for s in seeds]):
                Xs.append(X)
                Ys.append(Y)
            X, Y = np.concatenate(Xs), np.concatenate(Ys)
            if student is None:
                student = Student(X.shape[1], X.mean(0), X.std(0) + 1e-6)
            loss = fit(student, X, Y)
            state = {"obs_dim": X.shape[1], "mean": student.mean.numpy(), "std": student.std.numpy(),
                     "weights": student.state_dict()}
            print(f"[dagger] round {r}: {len(X):,} labelled states, imitation loss {loss:.4f}", flush=True)
    torch.save(state, OUT / "student.pt")
    for name, fn in (("baseline gait", None), ("student policy", student.act)):
        d, f, w = evaluate(fn, n=40, seconds=20.0, seed0=5000, with_success=True, **HARD_WALLS)
        print(f"hard walls      {name:14s}: SUCCESS {w:.0%}, falls {f:.0%}, climb {d:5.1f} cm  (40 test walls)")


if __name__ == "__main__":
    main(sys.argv[1])
