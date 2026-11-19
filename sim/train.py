#!/usr/bin/env python3
"""Train the residual climbing policy with PPO (Stable-Baselines3).

    ../.venv/bin/python train.py --steps 2000000       # CPU-only, parallel envs
    ../.venv/bin/python train.py --eval runs/best/best_model.zip
    ../.venv/bin/python train.py --algo sac --out runs_sac                     # SAC instead of PPO
    ../.venv/bin/python train.py --curriculum --out runs_curr                  # holdable -> hard walls
    ../.venv/bin/python train.py --privileged --out runs_teacher               # teacher (sees true grip)

Training walls: tilt 75..90 deg, HARD walls: grip 55..100 %, humidity 80..100 %, random pad drop-outs.
The best model (by evaluation on random vertical walls) is saved to runs/best/.
Only load models and normalisers that this script produced yourself.
"""
import argparse
import json
import os
from pathlib import Path

import numpy as np
from stable_baselines3 import PPO, SAC
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecNormalize

from rl_env import HARD_WALLS, ResidualClimbEnv, evaluate

RUNS = Path(__file__).resolve().parent / "runs"
ALGOS = {"ppo": PPO, "sac": SAC}

TRAIN_WALLS = HARD_WALLS   # train where the 90 % target is measured
CURRICULUM_START = 0.75    # curriculum: walls start at >= 75 % surface grip ...
CURRICULUM_END = 0.55      # ... and reach the hard test range halfway through training


def make(rank, seed0=0, privileged=False):
    def _init():
        return ResidualClimbEnv(seed=seed0 + rank, privileged=privileged, **TRAIN_WALLS)
    return _init


class Curriculum(BaseCallback):
    """Lower the weakest wall grip linearly from CURRICULUM_START to CURRICULUM_END over half the run."""

    def __init__(self, venv, steps):
        super().__init__()
        self.venv, self.steps, self.lo = venv, steps, None

    def _on_step(self):
        frac = min(1.0, self.num_timesteps / (0.5 * self.steps))
        lo = round(CURRICULUM_START + frac * (CURRICULUM_END - CURRICULUM_START), 3)
        if lo != self.lo:
            self.lo = lo
            self.venv.env_method("set_grip_floor", lo)
        return True


class WallEval(BaseCallback):
    """Every `every` steps: climb 10 random vertical walls with the current policy, keep the best."""

    def __init__(self, vecnorm, every=100_000, privileged=False):
        super().__init__()
        self.vecnorm, self.every, self.next, self.best = vecnorm, every, every, -np.inf
        self.privileged = privileged
        self.history = []

    def _policy(self, obs):
        o = self.vecnorm.normalize_obs(obs[None])
        return self.model.predict(o, deterministic=True)[0][0]

    def _on_step(self):
        if self.num_timesteps < self.next:
            return True
        self.next += self.every
        dist, fall, win = evaluate(self._policy, n=20, seconds=20.0, with_success=True,
                                   privileged=self.privileged, **TRAIN_WALLS)
        score = win + 0.001 * dist
        self.history.append({"steps": int(self.num_timesteps), "success": win, "climb_cm": dist, "fall_rate": fall})
        print(f"[eval] {self.num_timesteps:>9,d} steps: success {win:.0%}, climb {dist:5.1f} cm, falls {fall:.0%}",
              flush=True)
        if score > self.best:
            self.best = score
            (RUNS / "best").mkdir(parents=True, exist_ok=True)
            self.model.save(RUNS / "best" / "best_model")
            self.vecnorm.save(str(RUNS / "best" / "vecnorm.pkl"))
        (RUNS / "eval_history.json").write_text(json.dumps(self.history, indent=1))
        return True


def train(steps, n_envs, eval_every, lr=1e-4, log_std=-2.0, seed=0, algo="ppo", curriculum=False,
          privileged=False):
    venv = VecNormalize(SubprocVecEnv([make(i, 100 * seed, privileged) for i in range(n_envs)]),
                        norm_reward=True, clip_obs=10.0)
    if algo == "ppo":
        model = PPO("MlpPolicy", venv, n_steps=1024, batch_size=4096, n_epochs=5, learning_rate=lr,
                    gamma=0.99, gae_lambda=0.95, clip_range=0.2, ent_coef=0.0, verbose=0,
                    policy_kwargs=dict(net_arch=[128, 128], log_std_init=log_std), seed=seed)
    else:   # SAC: off-policy, replays old experience
        # No uniform-random warm-up (learning_starts=0) and a small fixed entropy bonus: random actions
        # pause the gait and pick random feet, which made the first SAC run fall on 65-95 % of walls.
        model = SAC("MlpPolicy", venv, learning_rate=lr, buffer_size=500_000, batch_size=512,
                    learning_starts=0, train_freq=8, gradient_steps=1, gamma=0.99,
                    ent_coef=0.001, verbose=0,
                    policy_kwargs=dict(net_arch=[256, 256], log_std_init=log_std), seed=seed)
        for p in model.actor.mu.parameters():          # start at action 0 = the hand-written gait
            p.data.zero_()
    callbacks = [WallEval(venv, every=eval_every, privileged=privileged)]
    if curriculum:
        callbacks.append(Curriculum(venv, steps))
    (RUNS / "best").mkdir(parents=True, exist_ok=True)
    (RUNS / "best" / "config.json").write_text(json.dumps({"algo": algo, "privileged": privileged}))
    model.learn(total_timesteps=steps, callback=callbacks)
    model.save(RUNS / "final_model")


def load_policy(path):
    """policy(obs) -> action for a model saved by this script (reads its algo and observation type)."""
    cfg_file = Path(path).parent / "config.json"
    cfg = json.loads(cfg_file.read_text()) if cfg_file.exists() else {"algo": "ppo", "privileged": False}
    model = ALGOS[cfg["algo"]].load(path, device="cpu")
    vn = VecNormalize.load(str(Path(path).parent / "vecnorm.pkl"),
                           DummyVecEnv([lambda: ResidualClimbEnv(seed=0, privileged=cfg["privileged"])]))
    vn.training = False
    policy = lambda obs: model.predict(vn.normalize_obs(obs[None]), deterministic=True)[0][0]
    return policy, cfg["privileged"]


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--steps", type=int, default=2_000_000)
    p.add_argument("--envs", type=int, default=max(2, (os.cpu_count() or 4) - 2))
    p.add_argument("--eval-every", type=int, default=100_000)
    p.add_argument("--lr", type=float, default=1e-4)
    p.add_argument("--log-std", type=float, default=-2.0)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--algo", choices=sorted(ALGOS), default="ppo")
    p.add_argument("--curriculum", action="store_true", help="start on holdable walls, end on hard ones")
    p.add_argument("--privileged", action="store_true", help="teacher: also observe true grip and drop-outs")
    p.add_argument("--out", default="runs", help="output folder (lets several runs train at once)")
    p.add_argument("--eval", help="evaluate a saved model against the baseline instead of training")
    a = p.parse_args()
    RUNS = Path(__file__).resolve().parent / a.out
    if a.eval:
        pol, priv = load_policy(a.eval)
        for walls, kw in (("hard walls", HARD_WALLS), ("holdable walls", {})):
            for name, fn in (("baseline gait", None), ("RL policy", pol)):
                d, f, w = evaluate(fn, n=40, seconds=20.0, seed0=5000, with_success=True,
                                   privileged=priv and fn is not None, **kw)
                print(f"{walls:15s} {name:14s}: SUCCESS {w:.0%}, falls {f:.0%}, climb {d:5.1f} cm  (40 test walls)")
    else:
        RUNS.mkdir(exist_ok=True)
        train(a.steps, a.envs, a.eval_every, a.lr, a.log_std, a.seed, a.algo, a.curriculum, a.privileged)
