# Simulation (stage 2)

MuJoCo model, climbing environment and a baseline crawl gait for ElectroClimb.

<p align="center"><img src="../media/hero.png" width="320" alt="ElectroClimb robot on a vertical wall in simulation"></p>

```bash
uv venv --python 3.12 .venv && uv pip install --python .venv -r sim/requirements.txt
cd sim
../.venv/bin/python check_physics.py    # holds on the wall with pads on, slides off without
../.venv/bin/python crawl_gait.py       # baseline gait climbs floor, 45 deg and vertical wall
../.venv/bin/python render_media.py     # GIFs + hero image -> ../media/
```

## How climbing is simulated

The robot always stands on one plane. **Gravity is rotated** instead of building walls:

| tilt | surface |
|---|---|
| 0° | floor |
| 90° | vertical wall (climbing up = +x) |
| 135° | overhang |
| 180° | ceiling |

This gives the training curriculum directly: start on the floor and raise the tilt as the policy improves.

## Model ([`electroclimb.xml`](electroclimb.xml))

- 1.46 kg quadruped with 12 servo joints (hip abduction, hip flexion, knee per leg). Servos are modelled on SO-101-class STS3215: kp 25, 3 Nm limit.
- Crouched stance (body ~9 cm off the wall) to keep the **peel moment** small
- 2 passive, spring-loaded ankle joints per foot. The pads are pre-rotated so the spring holds them **flat** in the stance.
- **Electroadhesion = MuJoCo `adhesion` actuator** per foot: action 0–1 is pad voltage 0–100 %, gain 15 N max pull
- IMU: orientation, gyro, accelerometer

## Environment ([`env.py`](env.py))

- **Action (16):** 12 joint offsets around the standing pose plus 4 pad voltages
- **Observation:** joint positions and velocities, IMU, per-foot grip estimate (noisy capacitance proxy), previous action
- **Reward:** progress up the wall, minus effort and pad energy, minus 10 for falling
- **Randomisation per episode:** surface factor (0.4–1.0), humidity factor (0.7–1.0), and a small per-step chance that a pad loses grip

The adhesion model is a **placeholder**. It gets replaced by the data-driven model from the stage 1 test rig (voltage × surface × humidity × peel angle).

## Baseline crawl gait ([`crawl_gait.py`](crawl_gait.py))

The gait plans feet in the body frame and converts them to joint angles with inverse kinematics, solved on a copy of the MuJoCo model. One leg at a time:

**release pad → lift → move up 4 cm → come down until the grip sensor reports contact → re-stick**

Then the body pulls itself up on all four pads.

One safety rule made the difference on the wall: **never lift a foot until every other foot reports contact.** Until then, the gait waits and presses the missing foot down.

| Surface | Climbed in 20 s | Fell |
|---|---|---|
| Floor (0°) | 20.5 cm | no |
| Slope (45°) | 19.0 cm | no |
| **Vertical wall (90°)** | **18.0 cm** | **no** |

This is the baseline the reinforcement-learning policy has to beat. It also shows the real design problems the hardware must solve:

| Problem found in simulation | Fix | What it means for the real robot |
|---|---|---|
| Tall body: gravity peels the upper feet off the wall | Crouched stance, body 16 → 9 cm | Keep the centre of mass low |
| Passive ankle spring held pads tilted, so only the edges stuck | Pads pre-rotated to sit flat in the stance | Foot geometry must keep the pad flat |
| Swung foot landed short, then the next foot released | Reach until contact + wait for all other feet | Capacitance contact sensing is essential, not optional |
| Weak surface (40 % grip) cannot hold a vertical wall | n/a, this is physics | Bigger pads or a lighter robot; the policy must avoid weak patches |

## Reinforcement learning ([`rl_env.py`](rl_env.py), [`train.py`](train.py))

**Residual RL:** PPO (Stable-Baselines3) learns corrections on top of the crawl gait (12 joint offsets plus gait speed) instead of learning to climb from scratch. Training walls are randomised: tilt, grip 55–100 %, humidity, and sudden pad drop-outs.

```bash
../.venv/bin/python train.py --steps 2500000                 # ~20 min on 8 CPU cores
../.venv/bin/python train.py --eval runs/best/best_model.zip  # vs baseline on 30 random walls
../.venv/bin/python render_rl.py runs/best/best_model.zip     # side-by-side GIF
```

**Results so far (30 random vertical walls, 20 s each):**

| | Climbed | Falls |
|---|---|---|
| Baseline crawl gait | 9.2 cm | 13 % |
| RL run 1 (pad voltage in action, tilt 45–90°, fall −20) | −9.6 cm | 40 % |
| RL run 2 (no pad action, tilt 75–90°, fall −100) | 0.6 cm | 30 % |

**RL has not beaten the baseline yet.** What was checked:
- The pipeline is correct: an untrained policy outputs actions under 0.02 and scores exactly like the baseline.
- Run 1 failed because exploration could lower pad voltage (instant falls), and a −20 fall penalty made rushing on easier 45° walls worthwhile.
- Run 2 hovers at baseline level and drifts worse. The likely cause is physics, not tuning: on the weakest training walls (55 % grip), three stuck pads give 3 × 15 N × 0.55 × 0.6 ≈ 14.9 N against a 14.3 N weight. That's no margin. A crawl gait must lift a foot, so joint corrections can't save those walls.

**Next RL steps:** give the policy levers that matter physically (pause/hold, choose which foot to lift, press force), restrict training to feasible walls plus a curriculum, use larger evaluation sets for model selection, and move to GPU-parallel simulation (MuJoCo MJX / Isaac Lab) for 100M+ steps.

## Physics checks ([`check_physics.py`](check_physics.py))

| Case | Result |
|---|---|
| Vertical wall, pads on | holds (0.1 cm creep) ✅ |
| Vertical wall, pads off | slides off ✅ |
| Floor, pads off | stands ✅ |

## Next

1. RL run 3: hold/pause action, feasible-wall curriculum, larger eval sets (see above)
2. Replace the placeholder adhesion with the test-rig model
3. Floor-to-wall transitions
