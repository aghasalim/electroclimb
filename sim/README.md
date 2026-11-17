# Simulation (stage 2 skeleton)

MuJoCo model and climbing environment for ElectroClimb.

```bash
uv venv --python 3.12 .venv && uv pip install --python .venv -r sim/requirements.txt
cd sim && ../.venv/bin/python check_physics.py
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

- 1.46 kg quadruped with 12 servo joints (hip abduction, hip flexion, knee per leg), servo torque limited to 2 Nm
- 2 passive, spring-loaded ankle joints per foot, so the pad lies flat
- **Electroadhesion = MuJoCo `adhesion` actuator** per foot: action 0–1 is pad voltage 0–100 %, gain 15 N max pull
- IMU: orientation, gyro, accelerometer

## Environment ([`env.py`](env.py))

- **Action (16):** 12 joint offsets around the standing pose plus 4 pad voltages
- **Observation:** joint positions and velocities, IMU, per-foot grip estimate (noisy capacitance proxy), previous action
- **Reward:** progress up the wall, minus effort and pad energy, minus 10 for falling
- **Randomisation per episode:** surface factor (0.4–1.0), humidity factor (0.7–1.0), and a small per-step chance that a pad loses grip

The adhesion model is a **placeholder**. It gets replaced by the data-driven model from the stage 1 test rig (voltage × surface × humidity × peel angle).

## Physics checks ([`check_physics.py`](check_physics.py))

| Case | Result |
|---|---|
| Vertical wall, pads on | holds (0.8 cm creep) ✅ |
| Vertical wall, pads off | slides off ✅ |
| Floor, pads off | stands ✅ |

Extra observation: on a weak surface (40 % grip), the robot slides off a vertical wall but still holds on the ceiling. Pad pull (4 × 6 N) times friction (0.6) is about 14 N of shear, just under its 14.3 N weight, while the ceiling only needs the pull itself to exceed the weight. This is exactly the situation the policy must learn to handle: press harder, use more feet, or avoid weak patches.

## Next

1. Hand-written crawl gait as a baseline (one foot moves at a time, pads switched off only on the moving foot)
2. PPO training with the gravity-tilt curriculum (MuJoCo MJX / Isaac Lab for thousands of parallel environments)
3. Replace the placeholder adhesion with the test-rig model
