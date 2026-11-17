"""Gymnasium wrapper for residual RL on top of the crawl gait.

The policy does not learn to climb from scratch. It learns corrections:
  - 12 joint offsets added to the gait's joint targets (each up to +-RESIDUAL)
  - 1 gait speed (0.75x .. 1.25x)
Pad voltages stay with the gait (always full on stance feet): in run 1, letting the policy
lower them made random exploration drop the robot.
Everything the gait cannot see (weak grip, a pad failing, humidity) is what RL is for.
"""
import gymnasium as gym
import numpy as np

from crawl_gait import CrawlGait
from env import ClimbEnv

RESIDUAL = 0.15          # max joint correction, in the env's action units
TRAIN_TILTS = (75.0, 90.0)   # train where it is hard (run 1 used 45..90 and learned to rush)
ACT_DIM = 13
FALL_PENALTY = 100.0
ALIVE_BONUS = 0.02


class ResidualClimbEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, tilt_range=TRAIN_TILTS, episode_s=20.0, seed=None, use_policy=True,
                 surface_range=(0.55, 1.0), humidity_range=(0.8, 1.0), fail_prob=3e-3):
        self.env = ClimbEnv(episode_s=episode_s, randomise=True, fail_prob=fail_prob, seed=seed,
                            surface_range=surface_range, humidity_range=humidity_range)
        self.tilt_range, self.use_policy = tilt_range, use_policy
        self.rng = np.random.default_rng(seed)
        obs_dim = 54 + 3
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (obs_dim,), np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (ACT_DIM,), np.float32)

    def _obs(self, base):
        phase = 2 * np.pi * (self.gait.t % self.gait.cycle) / self.gait.cycle
        extra = [np.sin(phase), np.cos(phase), self.env.tilt_deg / 90.0]
        return np.concatenate([base, extra]).astype(np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:                          # make episodes reproducible
            self.rng = np.random.default_rng(seed)
            self.env.rng = np.random.default_rng(seed + 1)
        tilt = float(self.rng.uniform(*self.tilt_range))
        base = self.env.reset(tilt_deg=tilt)
        self.gait = CrawlGait(self.env)
        self.last_x = 0.0
        return self._obs(base), {}

    def step(self, action):
        action = np.clip(action, -1, 1) if self.use_policy else np.zeros(ACT_DIM)
        self.gait.speed = 1.0 + 0.25 * float(action[12])
        a = self.gait(self.env._foot_contacts())
        a[:12] += RESIDUAL * action[:12]
        base, _, done, info = self.env.step(a)
        dx = info["progress"] - self.last_x
        self.last_x = info["progress"]
        reward = 100.0 * dx + ALIVE_BONUS - 0.01 * float(np.square(action[:12]).mean())
        if info["fell"]:
            reward -= FALL_PENALTY
        truncated = done and not info["fell"]
        return self._obs(base), reward, bool(info["fell"]), bool(truncated), info


def evaluate(policy_fn, n=20, tilt=90.0, seconds=20.0, seed0=1000, **kw):
    """Climb distance (cm) and falls over n randomised walls. policy_fn(obs) -> action or None."""
    dists, falls = [], 0
    for k in range(n):
        env = ResidualClimbEnv(tilt_range=(tilt, tilt), episode_s=seconds, seed=seed0 + k,
                               use_policy=policy_fn is not None, **kw)
        obs, _ = env.reset()
        done = trunc = False
        info = {"progress": 0.0, "fell": False}
        while not (done or trunc):
            act = policy_fn(obs) if policy_fn else np.zeros(ACT_DIM)
            obs, _, done, trunc, info = env.step(act)
        dists.append(info["progress"] * 100)
        falls += int(info["fell"])
    return float(np.mean(dists)), falls / n
