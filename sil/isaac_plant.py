"""
isaac_plant.py -- KinematicPlant with Isaac Sim physics as the integrator.

Everything the firmware sees is produced exactly as in sil.plant.KinematicPlant (valve lag, table
inversion, CalStrkAndSpd joint rates, sensor publishing from the joint state). Only the integration
step differs: instead of q += rate * dt, the rates are sent to the articulation as joint VELOCITY
targets, physics advances 10 ms, and the joint state is read back. Gravity, drive limits and contact
therefore act on the machine, and the firmware closes its loops on what Isaac actually did.

Runs only inside the Isaac container (needs a live World and Articulation); imports nothing from Isaac
itself, so the objects are passed in.

JOINT MAPPING (plant name -> URDF joint; sign identical, verified in sil/tests/test_imu_kinematics.py)
    swing -> swing_joint (UcToChs, +Z CCW)     boom -> boom_joint     arm -> arm_joint
    tilt -> tilt_joint (+X)                    rotator -> rotator_joint (+Z)
    input_link -> bucket_joint THROUGH THE FOUR-BAR: the URDF input_link is a 1:1 mimic placeholder, so
    the firmware's input-link rate is converted to an output-link rate with the firmware's own four-bar
    ratio, and the input-link angle is read back as kinematics.fourbar_input(bucket angle).

JOINT FRICTION is zeroed on the six driven joints (configure_drives): PhysX reads it as a coefficient on the joint
constraint force, and the importer copied the URDF's 10 "N.m" into it, which locks the swing.

VELOCITY DRIVE GAINS (GUESS): force = kd * (v_target - v), capped by the URDF effort. A hydraulic
axis is a flow source, so a stiff velocity loop is the closer analogue than a position drive; the
residual gravity drift is effort/kd (boom ~3e4 N.m / 1e8 -> 3e-4 rad/s), which the firmware's
position loops absorb exactly as they absorb cylinder leakage.
"""
import math

import numpy as np

from . import kinematics as kin
from .plant import JOINTS, WRAPPED, KinematicPlant, wrap
from . import valves as vlv

URDF_JOINT = {"swing": "swing_joint", "boom": "boom_joint", "arm": "arm_joint",
              "input_link": "bucket_joint", "tilt": "tilt_joint", "rotator": "rotator_joint"}
# URDF <mimic> couplings present in the articulation (multiplier, offset). A teleport that moves the leader
# but not the follower is corrected by the mimic constraint in one step, which yanks the whole arm.
MIMIC = {"input_link_joint": ("bucket_joint", 1.0, 0.0)}
KD_DEFAULT = {"swing": 1e7, "boom": 1e8, "arm": 1e8, "input_link": 1e8, "tilt": 1e6, "rotator": 1e6}


class IsaacPlant(KinematicPlant):
    def __init__(self, world, articulation, physics_dt=0.005, kd=None, **kw):
        super().__init__(**kw)
        self.world, self.art = world, articulation
        self.physics_dt = physics_dt
        self.substeps = max(1, int(round(0.01 / physics_dt)))
        self.names = list(articulation.dof_names)
        missing = [j for j in URDF_JOINT.values() if j not in self.names]
        if missing:
            raise KeyError(f"articulation lacks {missing}; has {self.names}")
        self.idx = {k: self.names.index(j) for k, j in URDF_JOINT.items()}
        self.kd = dict(KD_DEFAULT, **(kd or {}))
        self.isaac_q = {}

    def configure_drives(self):
        """Velocity drives on the six actuated joints (stiffness 0, damping kd). The other DOFs (dozer,
        boom swing, the mimic input link) keep the importer's position drives, held at their current angle."""
        kps, kds = (np.asarray(g.numpy() if hasattr(g, "numpy") else g, float).reshape(1, -1).copy()
                    for g in self.art.get_gains())
        for k, i in self.idx.items():
            kps[0, i] = 0.0
            kds[0, i] = self.kd[k]
        self.art.set_gains(kps=kps, kds=kds)
        # PhysX joint friction is a coefficient on the joint's constraint force. USDs built before the asset fix
        # (ecr88_dynamics.xacro dyn_friction) carry 10 and the swing joint, loaded by the whole house, cannot turn.
        fr = self.art.get_friction_coefficients()
        fr = np.asarray(fr.numpy() if hasattr(fr, "numpy") else fr, float).reshape(1, -1).copy()
        self.friction_before = {k: float(fr[0, i]) for k, i in self.idx.items()}
        for i in self.idx.values():
            fr[0, i] = 0.0
        self.art.set_friction_coefficients(fr)

    def push_pose(self):
        """Put the articulation at self.q (after world.reset() and set_hardware())."""
        self._need_hardware(None)
        q = np.asarray(self.art.get_joint_positions()[0], float).copy()
        for k, i in self.idx.items():
            q[i] = kin.fourbar_output(self.hardware, self.q[k]) if k == "input_link" else self.q[k]
        for follower, (leader, mult, off) in MIMIC.items():
            if follower in self.names:
                q[self.names.index(follower)] = mult * q[self.names.index(leader)] + off
        self.art.set_joint_positions(q.reshape(1, -1))
        self.art.set_joint_velocities(np.zeros((1, len(q))))
        self.art.set_joint_position_targets(q.reshape(1, -1))

    def step_kinematics(self, dt=vlv.DT):
        q = self.q
        self.q_used = dict(q)
        self.speeds = vlv.actuator_speeds(self.effective, self.tables, self.deadband, self.vmax)
        t = dict(self.speeds)
        for axis in vlv.CYLINDER_AXES:
            t[axis] = vlv.joint_rate_from_stroke_speed(self.cyls[axis], q[axis], self.speeds[axis], self.min_jacobian)
        for a in vlv.AXES:
            t[a] *= self.axis_sign[a]
        self.targets = t
        hw, eps = self.hardware, 1e-6
        ratio = (kin.fourbar_output(hw, q["input_link"] + eps) - kin.fourbar_output(hw, q["input_link"] - eps)) / (2 * eps)
        v = np.zeros((1, len(self.names)))
        for k, i in self.idx.items():
            v[0, i] = t[k] * ratio if k == "input_link" else t[k]
        self.art.set_joint_velocity_targets(v)
        for _ in range(self.substeps):
            self.world.step(render=False)
        self.sync_from_isaac(dt)

    def sync_from_isaac(self, dt=vlv.DT):
        """self.q <- the articulation; self.qdot <- the finite difference over the tick (what the IMUs see)."""
        qp = np.asarray(self.art.get_joint_positions()[0], float)
        old = dict(self.q)
        for k, i in self.idx.items():
            self.isaac_q[k] = float(qp[i])
            if k == "input_link":
                qi = kin.fourbar_input(self.hardware, float(qp[i]))
                if qi is None:
                    raise RuntimeError(f"bucket_joint {math.degrees(qp[i]):.2f} deg is outside the four-bar branch")
                self.q[k] = qi
            else:
                self.q[k] = wrap(float(qp[i])) if k in WRAPPED else float(qp[i])
        for k in JOINTS:
            d = wrap(self.q[k] - old[k]) if k in WRAPPED else self.q[k] - old[k]
            self.qdot[k] = d / dt
            lim = self.limits.get(k)
            self.at_limit[k] = bool(lim is not None and (self.q[k] <= lim[0] + 1e-3 or self.q[k] >= lim[1] - 1e-3))
