#!/usr/bin/env python3
"""Train the residual climbing policy with PPO (Stable-Baselines3).

    ../.venv/bin/python train.py --steps 2000000       # CPU-only, parallel envs
    ../.venv/bin/python train.py --eval runs/best/best_model.zip

Training walls: tilt 75..90 deg, grip 55..100 %, humidity 80..100 %, random pad drop-outs.
The best model (by evaluation on random vertical walls) is saved to runs/best/.
Only load models and normalisers that this script produced yourself.
"""
import argparse
import json
import os
from pathlib import Path

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecNormalize

from rl_env import ResidualClimbEnv, evaluate

RUNS = Path(__file__).resolve().parent / "runs"


def make(rank):
    def _init():
        return ResidualClimbEnv(seed=rank)
    return _init


class WallEval(BaseCallback):
    """Every `every` steps: climb 10 random vertical walls with the current policy, keep the best."""

    def __init__(self, vecnorm, every=100_000):
        super().__init__()
        self.vecnorm, self.every, self.next, self.best = vecnorm, every, every, -np.inf
        self.history = []

    def _policy(self, obs):
        o = self.vecnorm.normalize_obs(obs[None])
        return self.model.predict(o, deterministic=True)[0][0]

    def _on_step(self):
        if self.num_timesteps < self.next:
            return True
        self.next += self.every
        dist, fall = evaluate(self._policy, n=10, seconds=20.0)
        score = dist - 20.0 * fall
        self.history.append({"steps": int(self.num_timesteps), "climb_cm": dist, "fall_rate": fall})
        print(f"[eval] {self.num_timesteps:>9,d} steps: climb {dist:5.1f} cm, falls {fall:.0%}", flush=True)
        if score > self.best:
            self.best = score
            (RUNS / "best").mkdir(parents=True, exist_ok=True)
            self.model.save(RUNS / "best" / "best_model")
            self.vecnorm.save(str(RUNS / "best" / "vecnorm.pkl"))
        (RUNS / "eval_history.json").write_text(json.dumps(self.history, indent=1))
        return True


def train(steps, n_envs, eval_every):
    venv = VecNormalize(SubprocVecEnv([make(i) for i in range(n_envs)]), norm_reward=True, clip_obs=10.0)
    model = PPO("MlpPolicy", venv, n_steps=1024, batch_size=4096, n_epochs=5, learning_rate=1e-4,
                gamma=0.99, gae_lambda=0.95, clip_range=0.2, ent_coef=0.0, verbose=0,
                policy_kwargs=dict(net_arch=[128, 128], log_std_init=-2.0), seed=0)
    model.learn(total_timesteps=steps, callback=WallEval(venv, every=eval_every))
    model.save(RUNS / "final_model")


def load_policy(path):
    model = PPO.load(path, device="cpu")
    vn = VecNormalize.load(str(Path(path).parent / "vecnorm.pkl"), DummyVecEnv([lambda: ResidualClimbEnv(seed=0)]))
    vn.training = False
    return lambda obs: model.predict(vn.normalize_obs(obs[None]), deterministic=True)[0][0]


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--steps", type=int, default=2_000_000)
    p.add_argument("--envs", type=int, default=max(2, (os.cpu_count() or 4) - 2))
    p.add_argument("--eval-every", type=int, default=100_000)
    p.add_argument("--eval", help="evaluate a saved model against the baseline instead of training")
    a = p.parse_args()
    if a.eval:
        pol = load_policy(a.eval)
        for name, fn in (("baseline gait", None), ("RL policy", pol)):
            d, f = evaluate(fn, n=30, seconds=20.0, seed0=5000)
            print(f"{name:14s}: climb {d:5.1f} cm in 20 s, falls {f:.0%}  (30 random vertical walls)")
    else:
        RUNS.mkdir(exist_ok=True)
        train(a.steps, a.envs, a.eval_every)
