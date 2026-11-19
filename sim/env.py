"""ElectroClimb climbing environment (MuJoCo), gym-style but dependency-free.

Climbing is simulated by tilting gravity relative to the plane the robot stands on:
    tilt   0 deg -> floor,  90 deg -> vertical wall (up = +x),  180 deg -> ceiling.

Action (16): 12 joint offsets in [-1, 1] around the standing pose (+-1 = half the joint
range) + 4 pad voltages in [0, 1].
Observation: joint pos/vel, torso orientation (quat), gyro, accelerometer,
             per-foot grip estimate (noisy capacitance proxy), previous action.

Adhesion randomisation (per episode): surface factor, humidity factor, and a small
per-step chance that a pad loses grip (one half of a split pad drops out). This is a placeholder for the data-driven
adhesion model from the stage 1 test rig (hardware/foot/README.md section 5).
"""
from pathlib import Path

import mujoco
import numpy as np

XML = Path(__file__).with_name("electroclimb.xml")
FEET = ["fl", "fr", "hl", "hr"]
JOINTS = [f"{f}_{j}" for f in FEET for j in ("abd", "hip", "knee")]
STAND = np.tile([0.0, -0.4, 1.0], 4)  # abd, hip, knee (rad): crouched, foot under hip, body ~9 cm off the wall


def gravity_for_tilt(tilt_deg):
    t = np.radians(tilt_deg)
    return 9.81 * np.array([-np.sin(t), 0.0, -np.cos(t)])


class ClimbEnv:
    def __init__(self, tilt_deg=90.0, frame_skip=10, episode_s=10.0,
                 randomise=True, fail_prob=1e-4, drop_level=0.5, seed=None,
                 surface_range=(0.4, 1.0), humidity_range=(0.7, 1.0)):
        self.model = mujoco.MjModel.from_xml_path(str(XML))
        self.data = mujoco.MjData(self.model)
        self.tilt_deg, self.frame_skip, self.randomise = tilt_deg, frame_skip, randomise
        self.fail_prob = fail_prob
        # Each pad is split into two halves with their own HV driver, so a drop-out (driver fault,
        # arc, dust) only loses half a pad: voltage falls to drop_level. 0.0 = one-piece pad.
        self.drop_level = drop_level
        self.surface_range, self.humidity_range = surface_range, humidity_range
        self.max_steps = int(episode_s / (self.model.opt.timestep * frame_skip))
        self.rng = np.random.default_rng(seed)

        m = self.model
        self.jnt_qadr = np.array([m.jnt_qposadr[m.joint(j).id] for j in JOINTS])
        self.jnt_dadr = np.array([m.jnt_dofadr[m.joint(j).id] for j in JOINTS])
        self.jnt_lo, self.jnt_hi = m.jnt_range[[m.joint(j).id for j in JOINTS]].T
        self.adh_ids = np.array([m.actuator(f"{f}_adh").id for f in FEET])
        self.adh_gain0 = m.actuator_gainprm[self.adh_ids, 0].copy()
        self.pad_geoms = {m.geom(f"{f}_pad").id: i for i, f in enumerate(FEET)}
        self.wall = m.geom("wall").id
        self.torso = m.body("torso").id
        self.act_dim = 16
        self.prev_action = np.zeros(self.act_dim)

    # -- helpers -------------------------------------------------------------
    def _foot_contacts(self):
        touching = np.zeros(4)
        for c in self.data.contact[: self.data.ncon]:
            for g, other in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                if other == self.wall and g in self.pad_geoms:
                    touching[self.pad_geoms[g]] = 1.0
        return touching

    def _grip_estimate(self, touching):
        """Capacitance proxy: contact x surface coupling x pad voltage, plus sensor noise."""
        volts = self.data.ctrl[self.adh_ids]
        est = touching * self.surface * (0.5 + 0.5 * volts)
        return est + self.rng.normal(0, 0.05, 4)

    def _obs(self):
        d = self.data
        return np.concatenate([
            d.qpos[self.jnt_qadr], d.qvel[self.jnt_dadr],
            d.sensor("imu_quat").data, d.sensor("imu_gyro").data, d.sensor("imu_acc").data / 9.81,
            self._grip_estimate(self._foot_contacts()), self.prev_action,
        ]).astype(np.float32)

    # -- API -----------------------------------------------------------------
    def reset(self, tilt_deg=None):
        mujoco.mj_resetData(self.model, self.data)
        self.tilt_deg = self.tilt_deg if tilt_deg is None else tilt_deg
        self.surface = self.rng.uniform(*self.surface_range) if self.randomise else 1.0    # rough..smooth
        self.humidity = self.rng.uniform(*self.humidity_range) if self.randomise else 1.0  # humid..dry
        self.model.actuator_gainprm[self.adh_ids, 0] = self.adh_gain0 * self.surface * self.humidity

        # settle on the floor with pads energised, then rotate gravity to the target tilt
        self.model.opt.gravity[:] = gravity_for_tilt(0.0)
        self.data.ctrl[: 12] = STAND
        self.data.ctrl[self.adh_ids] = 1.0
        mujoco.mj_step(self.model, self.data, 250)
        self.model.opt.gravity[:] = gravity_for_tilt(self.tilt_deg)
        self.start_x = self.data.xpos[self.torso, 0]
        self.steps = 0
        self.last_drop = np.zeros(4)
        self.prev_action = np.concatenate([np.zeros(12), np.ones(4)])
        return self._obs()

    def step(self, action):
        a = np.clip(np.asarray(action, dtype=float), [-1] * 12 + [0] * 4, 1)
        # action 0 = standing pose; +-1 = half the joint range around it
        half = (self.jnt_hi - self.jnt_lo) / 2
        self.data.ctrl[:12] = np.clip(STAND + a[:12] * half, self.jnt_lo, self.jnt_hi)
        volts = a[12:].copy()
        drop = self.rng.random(4) < self.fail_prob         # random grip loss
        volts[drop] *= self.drop_level
        self.last_drop = drop.astype(float)
        self.data.ctrl[self.adh_ids] = volts
        mujoco.mj_step(self.model, self.data, self.frame_skip)
        self.steps += 1
        self.prev_action = a

        x, _, z = self.data.xpos[self.torso]
        fell = z > 0.35 or z < 0.02 or x < self.start_x - 0.3   # left the wall or slid away
        progress = x - self.start_x
        reward = progress - 0.001 * np.square(a[:12]).sum() - 0.0005 * a[12:].sum()
        done = fell or self.steps >= self.max_steps
        return self._obs(), float(reward - 10.0 * fell), done, {"progress": progress, "fell": fell}
