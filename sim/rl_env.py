"""Gymnasium wrapper for residual RL on top of the crawl gait.

The policy does not learn to climb from scratch. It learns corrections:
  - 12 joint offsets added to the gait's joint targets (each up to +-RESIDUAL)
  - 1 gait speed: -1 = hold still (pause the gait), 0 = normal, +1 = 1.25x
  - 1 body height: +-1.5 cm (all feet toward / away from the body; lower = less peel moment)
  - 1 press: 0..1 cm extra push of the stance feet while one foot swings
  - 4 leg preferences: which leg steps next (0 = the default crawl order)
Zero action = the hand-written gait exactly.
Pad voltages stay with the gait (always full on stance feet): in run 1, letting the policy
lower them made random exploration drop the robot.
Everything the gait cannot see (weak grip, a pad failing, humidity) is what RL is for.
"""
import gymnasium as gym
import numpy as np

from crawl_gait import CrawlGait
from env import FEET, ClimbEnv

RESIDUAL = 0.05          # max joint correction, in the env's action units
TRAIN_TILTS = (75.0, 90.0)   # train where it is hard (run 1 used 45..90 and learned to rush)
ACT_DIM = 19
PRIV_DIM = 5             # teacher-only observations: true grip + 4 pad drop-out flags
LOWER_M, PRESS_M = 0.015, 0.01
FALL_PENALTY = 100.0
# Grip >= 70 %: 3 stuck pads hold >= 1.3x the robot's weight. Below ~55 % no crawl gait
# can hold a vertical wall (run 2), so those walls only taught the policy noise.
HOLDABLE_GRIP = (0.7, 1.0)
HARD_WALLS = dict(surface_range=(0.55, 1.0), humidity_range=(0.8, 1.0))   # runs 1-2 eval set
ALIVE_BONUS = 0.02


class ResidualClimbEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, tilt_range=TRAIN_TILTS, episode_s=20.0, seed=None, use_policy=True,
                 surface_range=HOLDABLE_GRIP, humidity_range=(0.9, 1.0), fail_prob=3e-3,
                 privileged=False):
        self.env = ClimbEnv(episode_s=episode_s, randomise=True, fail_prob=fail_prob, seed=seed,
                            surface_range=surface_range, humidity_range=humidity_range)
        self.tilt_range, self.use_policy = tilt_range, use_policy
        self.rng = np.random.default_rng(seed)
        # privileged = teacher: also sees the true grip and which pads drop out (the robot can't)
        self.privileged = privileged
        obs_dim = 54 + 3 + 4 + (PRIV_DIM if privileged else 0)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (obs_dim,), np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (ACT_DIM,), np.float32)

    def _obs(self, base):
        phase = 2 * np.pi * (self.gait.t % self.gait.cycle) / self.gait.cycle
        t = self.gait.t % self.gait.cycle
        k = int(t // self.gait.step_s)
        swing = np.zeros(4)
        if k < 4 and k < len(self.gait.cycle_order):            # which leg is stepping now
            swing[FEET.index(self.gait.cycle_order[k])] = 1.0
        extra = [np.sin(phase), np.cos(phase), self.env.tilt_deg / 90.0, *swing]
        if self.privileged:
            extra += [self.env.surface * self.env.humidity, *self.env.last_drop]
        return np.concatenate([base, extra]).astype(np.float32)

    def set_grip_floor(self, lo):
        """Curriculum: lowest surface grip of the walls drawn from now on."""
        self.env.surface_range = (lo, self.env.surface_range[1])

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
        s = float(action[12])
        self.gait.speed = 1.0 + (0.25 * s if s >= 0 else s)   # -1 pauses the gait (hold)
        self.gait.lower = LOWER_M * float(action[13])
        self.gait.press = PRESS_M * max(float(action[14]), 0.0)
        self.gait.leg_pref = action[15:19]
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


SUCCESS_CM = 10.0   # a wall counts as climbed: no fall and >= 10 cm up in 20 s


def evaluate(policy_fn, n=20, tilt=90.0, seconds=20.0, seed0=1000, with_success=False, **kw):
    """Mean climb (cm) and fall rate over n randomised walls (+ success rate if asked).
    policy_fn(obs) -> action, or None for the plain baseline gait."""
    dists, falls, wins = [], 0, 0
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
        wins += int(not info["fell"] and info["progress"] * 100 >= SUCCESS_CM)
    if with_success:
        return float(np.mean(dists)), falls / n, wins / n
    return float(np.mean(dists)), falls / n
