"""
plant.py -- a kinematic actuation plant for the X1Exc SIL harness: y.propVlvCmd in, the sensors the
firmware reads out, 10 ms lockstep.

    from sil.harness import Harness
    from sil.plant import KinematicPlant
    plant = KinematicPlant(q0=dict(boom=-40, arm=90, input_link=-60, tilt=5), degrees=True)
    h = Harness(plant=plant).reset().nominal_inputs()
    h.gnss_rtk_fixed()
    h.tick(3)                                    # plant publishes before every MdlApp_step()
    plant.q["tilt"], plant.firmware_joints()     # ground truth, firmware joint names

WHAT IT IS
  No masses, no forces: every axis moves at the speed the machine's speed table promises for the
  command it received (sil/valves.py), lagged by a first-order valve (GUESS tau), integrated with
  explicit Euler over the 10 ms tick and clamped at joint limits. A closed loop that fails against it
  fails because of signs, geometry or logic, not tuning. Dynamics (inertia, flow sharing, load) are
  deliberately absent.

THE PHYSICAL MACHINE IS NOT THE FIRMWARE'S PARAMETER SET (class Hardware)
  Everything the plant needs from par.* -- IMU mounts, the rotator encoder zero, the speed tables, the
  cylinder / four-bar / tilt-mount geometry, the antenna offsets -- is read ONCE, on the first tick,
  from the COMPILED parameter set (the bytes Firmware.reset() restores), and frozen in plant.hardware.
  The plant never re-reads par.* afterwards: not on a harness reset, not after a test patches par.*,
  not after calibration results are loaded into par.*. So a wrong stored mount, zero offset, table or
  geometry is visible to the firmware exactly as it would be on a real machine (the firmware reads the
  plant's sensors through its own, different, parameters), and loading calibration results cannot
  silently change the plant's physics. hardware={...} overrides individual values (a unit whose IMU
  board sits differently); Hardware.live(fw) + plant.set_hardware() adopts the firmware's CURRENT
  par.* on purpose (e.g. after h.load_imu_mounts('ECR88D_LongArm.m') when the simulated unit really is
  that one). The plant never writes par.*.

ORDER INSIDE ONE CALL (Harness calls plant(h) before each MdlApp_step)
  1. read y.propVlvCmd written by the PREVIOUS step (+ plant.manual, a test/operator fixture), through
     the ECU's tilt/rotator flow-command glue (valves.ecu_glue)
  2. valve lag -> per-port lagged command (plant.valve, drives the pilot pressure) and the command the
     flow model sees (plant.effective: never below the deadband while the raw command is at or above
     it; see valves.effective_commands and test_min_speed_hold_from_rest_needs_the_effective_command_rule)
  3. actuator speed -> joint rate at the current angles (plant.targets), times plant.axis_sign
  4. integrate one tick, clamp at limits; plant.qdot is the rate actually realised (0 against a stop);
     every tick pressed against a stop is appended to plant.limit_hits (strict_limits=True raises)
  5. publish: five IMUs (kin.link_frames + kin.publish_imus with the HARDWARE mounts, gyro = link body
     rate incl. the house yaw rate from swing; accelerometer written here), u.jntAngRaw_Rot,
     u.ehPiPrs.*, u.isSwingAligned, u.blh_Main/Aux
  So the command of step k acts over [k, k+1] and step k+1 sees its result: zero-order hold.

STATE AND CONVENTIONS (plant.q, radians)
  swing       UcToChs: house w.r.t. undercarriage, CCW seen from above positive. Firmware reports
              y.jnts.ChsToUc.q = -swing (chart_2143 l.201). R_chs = R_uc @ Rz(swing).
  boom, arm, input_link, tilt, rotator   firmware joints BmMntToBm1, Bm2ToArm, ArmToInpLink,
              TiltMntToTilt, TiltToRot (== URDF zero and sign for the first three).
  output_link derived: kinematics.fourbar_output(input_link) (the firmware's own branch, hardware geometry).
  R_uc, chs_origin   undercarriage world attitude and the chassis origin (on the swing axis), world
              Z-up, X = grid East. Set R_uc for roll/pitch (CalibChs needs a tilted swing axis).
  plant.firmware_joints() gives the same state under the y.jnts names.

WHAT THE FIRMWARE NEEDS, AND WHERE THE PLANT GETS IT FROM (each from the source, not the spec)
  Joint angles      IMU Euler differences (chart_2143 l.29-32). Published by the shared publisher.
  Joint rates       gyro differences ONLY (chart_2143 l.268-278). Body rates composed down the chain:
                    w_chs = Rz(-swing) w_uc + [0,0,swingDot]; w_bm1 = Ry(-qb) w_chs + [0,qbDot,0];
                    w_arm = Ry(-qa) w_bm1 + [0,qaDot,0]; w_bkt = Ry(-qi) w_arm + [0,qiDot,0];
                    w_tilt = Rx(-qt)(Ry(-(qo+off)) w_arm + [0,qoDot,0]) + [qtDot,0,0].
  Swing angle       NO encoder, NO gyro integration: the antenna baseline's rotation about the chassis Z
                    axis since the isSwingAligned rising edge (chart_2143 l.61-152), advanced ONLY on
                    ticks after the swing counted as "operated" (chart_1167: |propVlvCmd.swing*| > 1e-6
                    OR u.ehPiPrs.swing* > 5 bar, released below 3 bar). So the plant must publish
                    GNSS positions (a site calibration is written on the first tick) AND pilot pressure,
                    or a house that is still moving after the command drops is invisible.
  Swing rate        chassis gyro Z (chart_2143 l.277), independent of the above.
  Swing switch      u.isSwingAligned = |swing| <= swing_switch_halfwidth. ASSUMPTION: a sector centred on
                    0 with half-width SwingZeroProxEdgeOffs = 1 deg -- the firmware latches +-1 deg at
                    the edge by swing direction (chart_2143 l.113-129, SysPar.m:198), so a 1 deg
                    half-width is the geometry it assumes; the real switch drawing is not in any source.
                    Every crossing re-latches the zero, so the firmware's swing differs from plant truth by
                    at most one tick of travel (observed 0.01 deg after Positioning/Swing).
  Heading           baseline direction vs distAntMainToAntAux (chart_2143 l.37-49): same antennas.
  Rotator           absolute sensor: jnts.TiltToRot.q = par.jntAngRotZeroOffs + u.jntAngRaw_Rot
                    (MdlApp.c:42032), rate by linear regression over 10 samples (chart_1179). The plant
                    publishes raw = rotator - hardware["par.jntAngRotZeroOffs"], so a firmware whose
                    stored offset equals the hardware's reads the truth.
  Accelerometer     acc_sign g along world up in each IMU frame, default -1 g (quasi-static; ASSUMPTION:
                    link accelerations neglected), optional white noise acc_noise (g rms). Only
                    calibration reads it (CalcAngDistCalib, chart_2291: the mount rebuild and the leg
                    angles angCalib). SIGN -1 g, from the compiled mounts: CalibArm / CalibLink / CalibChs /
                    CalibTilt on this plant rebuild par.imuArm / imuLink / imuChs / imuTilt exactly only
                    with -1 g at the natural reference poses (arm hanging vertically, input link horizontal,
                    nose-up jack-up, tool at the default pose); with +1 g the arm would have to point
                    straight up at its reference pose, outside the joint ranges
                    (test_valve_plant.test_accelerometer_sign_is_minus_one_g). ASSUMPTION behind that
                    evidence: the par.imu* literals are field calibration results (they carry 9 digits).
                    NOTE the shared kinematics.publish_imus (Harness.set_pose / nominal_inputs) still writes
                    +1 g; a calibration run on that publisher rebuilds the mounts 180 deg off, silently.
                    NOT REPRESENTED: centripetal and tangential acceleration. On a real machine the chassis
                    IMU sees about 0.025 g per metre of lever arm at 0.5 rad/s of swing, against ~0.1 g of
                    in-plane gravity at a 6 deg jack-up -- enough to move the CalibChs leg end points
                    (angCalib.chs) by degrees. The plant's calibration results are therefore cleaner than a
                    real machine's.
  Travel            NOT MODELLED: trvl commands are reported in plant.targets but the undercarriage does
                    not move (no source for track speed per percent or track sign at the ExtY level).
  Dozer / boom swing  not modelled (the firmware pins both, MdlApp.c:12411, :12420).

JOINT LIMITS (firmware convention; override with limits={axis: (lo, hi) rad or None})
  boom -69..-31 deg: OEM-TEAM (Olivia 2026-09-11), already in the firmware's q <= 0 convention (CLAUDE.md).
  arm 31..155 deg: OEM-TEAM numbers taken LITERALLY as Bm2ToArm. ASSUMPTION: CLAUDE.md read them as the
  boom-arm included angle phi (q = 180 - phi would give 25..149 deg); the zero is unconfirmed either way.
  input_link -150..+10 deg: GUESS, below the +14.2 deg cylinder dead centre (valves.dead_centres).
  tilt +-45 deg: ESTIMATE (URDF jntTiltLower/Upper, ecr88_params.xacro:855; resources/
  ECR88_estimated_dynamics.md adopts +-40 CLASS-TYPICAL, no source). The only firmware-derived bound is
  a LOWER bound on the travel: CalibTilt swings the tool 35 deg of gravity angle one way and 70 deg back
  (AngTiltPnt1/2, SysPar.m:124-125); a joint travel of 2*asin(sin(leg/2)/sin(beta)) is needed with beta
  the tilt axis angle from vertical, i.e. at least +-35 deg from a centred start with the tilt axis
  horizontal, +-38.2 at the default pose (axis 21.9 deg below horizontal), +-54.2 at 45 deg -- before the
  post-leg coast (the calibration ramps each _ToPnt command down over 1 s AFTER the leg angle, chart_3055
  l.194-243, so the axis travels ~0.5 s x its speed further). Because the limit has no source, STOP CONTACT
  IS MADE EXPLICIT: plant.limit_hits records it and strict_limits=True raises JointStopError; calibration
  scenarios must run strict (the firmware does not notice a leg that ends on a stop -- it waits out
  CntCalib_timeout = 10 s and carries on, MdlApp.c:36320).
  swing, rotator: none (wrapped to +-pi). q0 outside the limits raises.

USAGE NOTES
  One plant per scenario. On the first tick after Harness.reset() the plant rewrites the site calibration
  (MdlApp_initialize zeroes the u.* site inports); plant.q and plant.hardware are NOT reset.
  The plant owns u.*Imu*, u.jntAngRaw_Rot, u.ehPiPrs.*, u.isSwingAligned (unless
  swing_switch_halfwidth=None) and u.blh_Main/Aux (unless gnss=False); GNSS quality (methodGnss,
  gnssPosStdDevZ), faults and HMI inputs stay with the test (h.nominal_inputs(), h.gnss_rtk_fixed()).
  Plants listed after it may edit those inports in place (noise, faults): publish() re-bases every tick.
  plant.manual = {port: %} drives the plant without the firmware (it is max-ed into y.propVlvCmd, so
  the firmware sees the motion but not a command); the remote lever u.rmtLvrDmd is the firmware path.
  To swing the house off square with swing init intact: boot at swing 0 (switch closed -> latch), then
  move it (remote lever or manual); publishing swing 0 at boot is how the plant satisfies A6.0.

GROUND TRUTH (read after any tick)
  plant.q / plant.qdot     joint angles (rad) / realised rates (rad/s), keys JOINTS
  plant.q_used             the angles the last rates were evaluated at (Jacobians)
  plant.cmd                last y.propVlvCmd after plant.manual and ecu_glue, {port: %}
  plant.valve / .effective lagged command / what the flow model saw, {port: %}
  plant.speeds             signed actuator speed per axis in table units (cylinders: stroke m/s)
  plant.targets            joint-rate targets per axis before limits (valves units, incl. trvl/blade)
  plant.at_limit           {joint: bool} pressed against a stop on the last tick
  plant.limit_hits         [(tick, joint, q)] every tick any joint was pressed against a stop
  plant.hardware           the frozen physical parameter set (Hardware); plant.tables / .cyls from it
  plant.R_chs, .frames, .body_rates   what was published; plant.strokes(); plant.firmware_joints()

MUTATIONS FOR REGRESSION TESTS
  axis_sign={'tilt': -1}   plumbs one axis backwards (the loop must then fail).
  deadband={'rotPosi': 25.0}, vmax={'rotPosi': 0.3}   a plant valve that differs from the stored table
  (calibration steps 20..26 should identify it).
  hardware={'imuArm': M, 'par.jntAngRotZeroOffs': 0.3}   a unit whose sensors differ from par.*.
"""
import ctypes
import math

import numpy as np

from . import kinematics as kin
from . import valves as vlv
from .geodesy import Site, main_antenna_for_chassis, place_antennas

DT = vlv.DT
DEG = math.pi / 180.0

JOINTS = ("swing", "boom", "arm", "input_link", "tilt", "rotator")
DEFAULT_Q0_DEG = dict(swing=0.0, boom=-40.0, arm=90.0, input_link=-60.0, tilt=0.0, rotator=0.0)  # = Harness.NOMINAL_POSE
DEFAULT_LIMITS_DEG = {
    "swing": None,
    "boom": (-69.0, -31.0),         # OEM-TEAM
    "arm": (31.0, 155.0),           # OEM-TEAM, zero ASSUMPTION (see docstring)
    "input_link": (-150.0, 10.0),   # GUESS (dead centre +14.2 deg)
    "tilt": (-45.0, 45.0),          # ESTIMATE (URDF jntTiltLower/Upper), no source: stop contact is recorded
    "rotator": None,
}
FW_JOINT = {"swing": "ChsToUc", "boom": "BmMntToBm1", "arm": "Bm2ToArm", "input_link": "ArmToInpLink",
            "tilt": "TiltMntToTilt", "rotator": "TiltToRot"}
SWING_SWITCH_HALFWIDTH = 1.0 * DEG      # ASSUMPTION = SwingZeroProxEdgeOffs, SysPar.m:198
WRAPPED = ("swing", "rotator")
ACC_SIGN = -1.0                         # g along world up; evidence in the module docstring (Accelerometer)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class JointStopError(AssertionError):
    """A joint was driven against its stop while the plant ran with strict_limits=True."""


# ======================================================================================================
class Hardware:
    """The PHYSICAL machine the plant simulates, as a frozen {par path: value} set.

    It has the firmware's parameter NAMES (so the shared kinematics / geodesy / valves helpers read it in
    place of fw), but it is a separate object: the firmware's par.* can be patched, reset or loaded with
    calibration results without touching it.
        Hardware.compiled(fw)   the compiled parameter set, i.e. what Firmware.reset() restores
        Hardware.live(fw)       the firmware's par.* as they are now
        hw.replace({...})       a copy with some values changed; keys are par paths, or a mount name
                                ('imuArm') with a 3x3 matrix
    """

    def __init__(self, values):
        self._v = {p: (list(v) if isinstance(v, (list, tuple, np.ndarray)) else v) for p, v in values.items()}

    @classmethod
    def compiled(cls, fw):
        """Decode par.* from the parLocalTest bytes captured at library load. firmware.Firmware keeps them
        (_par_initial, restored by reset()); there is no public accessor, so this reads the private fields
        and fails loudly if they change rather than falling back to the live (possibly patched) values."""
        try:
            blob, base, sigs = fw._par_initial, fw._par_addr, fw._sigs
        except AttributeError as e:
            raise RuntimeError("Hardware.compiled needs Firmware._par_initial/_par_addr/_sigs "
                               "(sil/firmware.py changed?); pass hardware=Hardware.live(fw) explicitly") from e
        out = {}
        for p in fw.paths("par."):
            addr, ct, count, code = sigs[p]
            off = addr - base
            if off < 0 or off + ctypes.sizeof(ct) * count > len(blob):
                raise RuntimeError(f"{p} lies outside the captured parLocalTest bytes")
            vals = list((ct * count).from_buffer_copy(blob, off))
            if code == "?":
                vals = [bool(v) for v in vals]
            out[p] = vals[0] if count == 1 else vals
        return cls(out)

    @classmethod
    def live(cls, fw):
        return cls({p: fw[p] for p in fw.paths("par.")})

    def replace(self, overrides):
        v = dict(self._v)
        for key, val in (overrides or {}).items():
            if key in kin.MOUNT_NAMES:
                M = np.asarray(val, dtype=float)
                if M.shape != (3, 3):
                    raise ValueError(f"{key} takes a 3x3 matrix")
                for i in range(3):
                    for j in range(3):
                        v[f"par.{key}.a{i + 1}{j + 1}"] = float(M[i, j])
                continue
            if key not in v:
                raise KeyError(f"{key!r} is not a par.* path or a mount name {kin.MOUNT_NAMES}")
            if isinstance(v[key], list):
                val = list(val)
                if len(val) != len(v[key]):
                    raise ValueError(f"{key} takes {len(v[key])} values, got {len(val)}")
            else:
                val = type(v[key])(val)
            v[key] = val
        return Hardware(v)

    def __getitem__(self, path):
        try:
            return self._v[path]
        except KeyError:
            raise KeyError(f"{path!r} is not in the hardware parameter set") from None

    def __contains__(self, path):
        return path in self._v

    def paths(self, prefix=""):
        return sorted(p for p in self._v if p.startswith(prefix))

    def mounts(self):
        return kin.mounts_from_fw(self)


class _SensorBus:
    """What the shared helpers see in place of fw while the plant publishes: par.* from the Hardware,
    every other signal read from and written to the firmware. Writing par.* is refused."""

    def __init__(self, fw, hardware):
        self.fw, self.hw = fw, hardware

    def __getitem__(self, path):
        return self.hw[path] if path.startswith("par.") else self.fw[path]

    def __setitem__(self, path, value):
        if path.startswith("par."):
            raise PermissionError(f"the plant never writes firmware parameters ({path})")
        self.fw[path] = value


# ======================================================================================================
class KinematicPlant:
    def __init__(self, q0=None, degrees=False, limits=None, tau_valve=vlv.TAU_VALVE, deadband=None, vmax=None,
                 axis_sign=None, R_uc=None, chs_origin=(0.0, 0.0, 0.0), site=None, gnss=True,
                 swing_switch_halfwidth=SWING_SWITCH_HALFWIDTH, acc_noise=0.0, seed=0,
                 rot_quantum=None, min_jacobian=vlv.MIN_JACOBIAN, publish_pilot=True, acc_sign=ACC_SIGN,
                 hardware=None, strict_limits=False, record=False):
        """
        q0              {joint: angle}; missing joints take DEFAULT_Q0_DEG. degrees=True for q0 in degrees.
        limits          {joint: (lo, hi) radians or None}, merged over DEFAULT_LIMITS_DEG.
        tau_valve       s, first-order valve lag (GUESS 0.1); 0 disables.
        deadband, vmax  {port: percent} / {port: table X units}: plant valve differs from the stored table.
        axis_sign       {axis: +1/-1}: mutation, multiplies that axis's realised rate.
        R_uc            undercarriage world attitude (3x3); chs_origin world position of the chassis origin.
        site            sil.geodesy.Site to write on the first tick (default Site()); gnss=False publishes
                        neither site nor antennas (the firmware's swing angle then cannot move).
        swing_switch_halfwidth  rad; None = do not publish u.isSwingAligned (the test owns it).
        acc_noise       g rms white noise added to every accelerometer axis; seed for its RNG.
        rot_quantum     rad; quantise u.jntAngRaw_Rot (None = exact).
        publish_pilot   False leaves u.ehPiPrs alone (what a plant without pilot sensors looks like).
        acc_sign        g along world up at rest: -1 (default, the sign the compiled mounts were calibrated
                        with) or +1 (the shared kinematics.publish_imus convention).
        hardware        None: the compiled parameter set, frozen on the first tick. A Hardware: that one.
                        A dict: overrides (par path or mount name -> value) over the compiled set.
        strict_limits   True: raise JointStopError the first tick any joint is pressed against a stop.
        record          keep one dict per tick in plant.log.
        """
        self.limits = {k: (None if v is None else (v[0] * DEG, v[1] * DEG)) for k, v in DEFAULT_LIMITS_DEG.items()}
        for k, v in (limits or {}).items():
            if k not in JOINTS:
                raise KeyError(f"unknown joint {k!r}")
            self.limits[k] = None if v is None else (float(v[0]), float(v[1]))
        q = {k: v * DEG for k, v in DEFAULT_Q0_DEG.items()}
        for k, v in (q0 or {}).items():
            if k not in JOINTS:
                raise KeyError(f"unknown joint {k!r}; joints are {JOINTS}")
            q[k] = v * DEG if degrees else float(v)
        for k in JOINTS:
            if k in WRAPPED:
                q[k] = wrap(q[k])
            elif self._clamp(k, q[k]) != q[k]:
                lo, hi = self.limits[k]
                raise ValueError(f"q0 {k} = {math.degrees(q[k]):.3f} deg is outside its limits "
                                 f"[{math.degrees(lo):.3f}, {math.degrees(hi):.3f}] deg")
        self.q = q
        self.qdot = {k: 0.0 for k in JOINTS}
        self.tau_valve = tau_valve
        self.deadband = dict(deadband or {})
        self.vmax = dict(vmax or {})
        for d in (self.deadband, self.vmax):
            for p in d:
                if p not in vlv.PORTS:
                    raise KeyError(f"unknown port {p!r}")
        self.axis_sign = {a: 1.0 for a in vlv.AXES}
        for a, s in (axis_sign or {}).items():
            if a not in vlv.AXES:
                raise KeyError(f"unknown axis {a!r}; axes are {vlv.AXES}")
            self.axis_sign[a] = float(s)
        self.R_uc = np.eye(3) if R_uc is None else np.asarray(R_uc, dtype=float)
        self.w_uc = np.zeros(3)                 # undercarriage body rate (link coords); static ground
        self.chs_origin = np.asarray(chs_origin, dtype=float)
        self.site = site if site is not None else (Site() if gnss else None)
        self.gnss = gnss
        self.swing_switch_halfwidth = swing_switch_halfwidth
        self.acc_noise = float(acc_noise)
        self.rng = np.random.default_rng(seed)
        self.rot_quantum = rot_quantum
        self.min_jacobian = min_jacobian
        self.publish_pilot = publish_pilot
        if acc_sign not in (1, -1):
            raise ValueError("acc_sign is +1 or -1")
        self.acc_sign = float(acc_sign)
        self.strict_limits = strict_limits
        self.manual = {}                        # {port: percent} added (max) to the firmware command
        self.cmd = {p: 0.0 for p in vlv.PORTS}
        self.valve = {p: 0.0 for p in vlv.PORTS}
        self.effective = {p: 0.0 for p in vlv.PORTS}
        self.q_used = dict(self.q)
        self.targets = {a: 0.0 for a in vlv.AXES}
        self.speeds = {a: 0.0 for a in vlv.AXES}
        self.at_limit = {k: False for k in JOINTS}
        self.limit_hits = []
        self.frames = None
        self.record = record
        self.log = []
        self.hardware = None
        self._hardware_overrides = None
        self.tables = None
        self.cyls = None
        if isinstance(hardware, Hardware):
            self.set_hardware(hardware)
        elif hardware is not None:
            self._hardware_overrides = dict(hardware)
        self.ticks = 0
        self._tick = None
        self.R_chs = self.R_uc @ kin.Rz(self.q["swing"])
        self.body_rates = None
        self.invalidate()

    # -- setup -----------------------------------------------------------------------------------
    def set_ground(self, roll=0.0, pitch=0.0, yaw=0.0):
        """Undercarriage world attitude from URDF fixed-axis roll/pitch/yaw (radians)."""
        self.R_uc = kin.rpy_to_R(roll, pitch, yaw)
        return self

    def set_hardware(self, hardware):
        """Freeze `hardware` (a Hardware) as the physical machine: valve tables, deadbands, cylinders, rotator
        zero, tilt-mount offset and IMU mounts are derived from it here and never from the firmware."""
        self.hardware = hardware
        self.tables = vlv.read_tables(hardware)
        self._dbs = vlv.deadbands(self.tables, self.deadband)
        self.cyls = vlv.cylinders(hardware)
        self._mounts = hardware.mounts()
        self._rot_offs = float(hardware["par.jntAngRotZeroOffs"])
        self._tilt_off = float(hardware["par.parKin.angOutpLinkToTiltMnt"]) + float(hardware["par.parKin.angTiltMntToTilt"])
        self.invalidate()
        return self

    def _clamp(self, k, v):
        lim = self.limits[k]
        if lim is None:
            return v
        return min(max(v, lim[0]), lim[1])

    # -- the plant hook ----------------------------------------------------------------------------
    def __call__(self, h):
        fw = h.fw
        if self.hardware is None:
            self.set_hardware(Hardware.compiled(fw).replace(self._hardware_overrides))
        first = h.tick_count == 0
        if first:
            if self.gnss:
                self.site.write(fw)
            self.valve = {p: 0.0 for p in vlv.PORTS}
        self._tick = h
        cmds = vlv.read_commands(fw)
        for p, v in self.manual.items():
            cmds[p] = max(cmds[p], float(v))
        cmds = vlv.ecu_glue(cmds)
        self.cmd = cmds
        self.valve = vlv.lag_step(self.valve, cmds, DT, self.tau_valve)
        self.effective = vlv.effective_commands(cmds, self.valve, self._dbs)
        self.step_kinematics(DT)
        self.publish(fw, force=first)
        if self.record:
            self.log.append(dict(tick=h.tick_count, q=dict(self.q), qdot=dict(self.qdot), cmd=dict(self.cmd),
                                 valve=dict(self.valve), effective=dict(self.effective),
                                 targets=dict(self.targets)))
        self.ticks += 1

    def step_kinematics(self, dt=DT):
        """Valve state -> rates -> integrate one step (no firmware access). Needs the hardware set."""
        if self.hardware is None:
            raise RuntimeError("step_kinematics before the hardware is known: tick once or call set_hardware()")
        q = self.q
        self.q_used = dict(q)                   # the angles the Jacobians / targets were evaluated at
        self.speeds = vlv.actuator_speeds(self.effective, self.tables, self.deadband, self.vmax)
        t = dict(self.speeds)
        for axis in vlv.CYLINDER_AXES:
            t[axis] = vlv.joint_rate_from_stroke_speed(self.cyls[axis], q[axis], self.speeds[axis], self.min_jacobian)
        for a in vlv.AXES:
            t[a] *= self.axis_sign[a]
        self.targets = t
        hit = []
        for k in JOINTS:
            old = q[k]
            new = old + t[k] * dt
            if k in WRAPPED:
                new = wrap(new)
                d = wrap(new - old)
            else:
                clamped = self._clamp(k, new)
                self.at_limit[k] = clamped != new
                if self.at_limit[k]:
                    hit.append(k)
                new = clamped
                d = new - old
            q[k] = new
            self.qdot[k] = d / dt
        if hit:
            tick = self._tick.tick_count if self._tick is not None else self.ticks
            self.limit_hits.extend((tick, k, q[k]) for k in hit)
            if self.strict_limits:
                where = self._tick.describe() if self._tick is not None else f"plant tick {self.ticks}"
                raise JointStopError(
                    "joint stop reached: " + ", ".join(f"{k} at {math.degrees(q[k]):.2f} deg, commanded "
                                                       f"{math.degrees(t[k]):+.3f} deg/s" for k in hit)
                    + f" -- {where}")

    def invalidate(self):
        """Force a full re-publish on the next tick."""
        self._pub_key = None
        self._snapshot = {}

    def _owned_paths(self):
        paths = [f"u.{port}Imu{k}" for port in kin.PORT_MOUNT for k in ("Quat", "AngRate", "Acc")]
        paths.append("u.jntAngRaw_Rot")
        if self.gnss:
            paths += ["u.blh_Main", "u.blh_Aux"]
        return paths

    def publish(self, fw, force=False):
        """Write the sensors for the current state, through the HARDWARE parameters. The IMU / rotator /
        antenna writes (~0.3 ms) are skipped when the plant state is bit-identical to the last publish AND
        every one of those inports still holds what was written (read back), so a plant listed after this
        one that edits them in place (noise, faults) is re-based every tick instead of accumulating. A reset
        zeroes MdlApp_U; tick 0 is always a full publish."""
        q, qd = self.q, self.qdot
        if self.publish_pilot:
            for p in vlv.PILOT_PORTS:
                bar = float(np.float32(vlv.pilot_pressure_bar(self.valve[p])))
                if force or fw[f"u.ehPiPrs.{p}"] != bar:
                    fw[f"u.ehPiPrs.{p}"] = bar
        if self.swing_switch_halfwidth is not None:
            fw["u.isSwingAligned"] = int(abs(q["swing"]) <= self.swing_switch_halfwidth)
        key = (tuple(q[k] for k in JOINTS), tuple(qd[k] for k in JOINTS), self.R_uc.tobytes(),
               self.w_uc.tobytes(), self.chs_origin.tobytes())
        if (not force and self.acc_noise == 0.0 and key == self._pub_key and self._snapshot
                and all(fw[p] == v for p, v in self._snapshot.items())):
            return
        self._pub_key = key
        bus = _SensorBus(fw, self.hardware)
        R_chs = self.R_uc @ kin.Rz(q["swing"])
        frames = kin.link_frames(bus, R_chs, q["boom"], q["arm"], q["input_link"], q["tilt"])
        self.frames = frames
        q_outp = kin.fourbar_output(bus, q["input_link"])
        ratio = 0.0
        if qd["input_link"] != 0.0:
            eps = 1e-6
            ratio = (kin.fourbar_output(bus, q["input_link"] + eps)
                     - kin.fourbar_output(bus, q["input_link"] - eps)) / (2 * eps)
        w_chs = kin.Rz(-q["swing"]) @ self.w_uc + np.array([0.0, 0.0, qd["swing"]])
        w_bm1 = kin.Ry(-q["boom"]) @ w_chs + np.array([0.0, qd["boom"], 0.0])
        w_arm = kin.Ry(-q["arm"]) @ w_bm1 + np.array([0.0, qd["arm"], 0.0])
        w_bkt = kin.Ry(-q["input_link"]) @ w_arm + np.array([0.0, qd["input_link"], 0.0])
        w_tilt = (kin.Rx(-q["tilt"]) @ (kin.Ry(-(q_outp + self._tilt_off)) @ w_arm
                                         + np.array([0.0, ratio * qd["input_link"], 0.0]))
                  + np.array([qd["tilt"], 0.0, 0.0]))
        self.body_rates = {"chs": w_chs, "bm1": w_bm1, "arm": w_arm, "bkt": w_bkt, "tilt": w_tilt}
        kin.publish_imus(bus, frames, rates=self.body_rates, accel=False)
        for port, R_link in frames.items():
            R_imu = R_link @ self._mounts[kin.PORT_MOUNT[port]].T
            a = self.acc_sign * (R_imu.T @ kin.UP)             # specific force convention, sign ACC_SIGN
            if self.acc_noise > 0.0:
                a = a + self.rng.normal(0.0, self.acc_noise, 3)
            fw[f"u.{port}ImuAcc"] = kin.mirror(a)              # the firmware's input mirror, as the gyro
        raw = wrap(q["rotator"] - self._rot_offs)
        if self.rot_quantum:
            raw = round(raw / self.rot_quantum) * self.rot_quantum
        fw["u.jntAngRaw_Rot"] = raw
        if self.gnss:
            main = main_antenna_for_chassis(bus, self.chs_origin, R_chs)
            place_antennas(bus, self.site, main, R_chs)
        self.R_chs = R_chs
        self._snapshot = {p: fw[p] for p in self._owned_paths()}

    # -- ground truth ------------------------------------------------------------------------------
    def output_link(self, fw=None):
        """ArmToOutpLink of the hardware four-bar (fw is accepted and ignored)."""
        self._need_hardware(fw)
        return kin.fourbar_output(self.hardware, self.q["input_link"])

    def firmware_joints(self, fw=None):
        """Plant state under the y.jnts names (ChsToUc = -swing), incl. ArmToOutpLink of the hardware
        four-bar. fw is only used to freeze the compiled hardware if the plant has not ticked yet."""
        out = {FW_JOINT[k]: (-self.q[k] if k == "swing" else self.q[k]) for k in JOINTS}
        if self.hardware is not None or fw is not None:
            out["ArmToOutpLink"] = self.output_link(fw)
        return out

    def strokes(self):
        """{axis: cylinder length m} for boom/arm/input_link (compare y.cyls.bm1/arm/bkt.strk)."""
        self._need_hardware(None)
        return {a: vlv.stroke(self.cyls[a], self.q[a]) for a in vlv.CYLINDER_AXES}

    def _need_hardware(self, fw):
        if self.hardware is None:
            if fw is None:
                raise RuntimeError("the plant's hardware is frozen on its first tick; tick once or pass fw")
            self.set_hardware(Hardware.compiled(fw).replace(self._hardware_overrides))
