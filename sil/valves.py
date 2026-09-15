"""
valves.py -- y.propVlvCmd (percent per port) -> actuator speed -> joint rate, numpy only.

Pure functions (no firmware state, no plant state). sil/plant.py integrates what these return;
sil/tests/test_valve_plant.py pins every number below against hand calculation and the loop
against the running firmware.

WHERE EACH PIECE COMES FROM
  Port pairing and sign   chart_2463 AutoVelDmdToVlvCmd lines 28-47 (generated MdlApp.c:50496-50553):
                          actuatorSpdReq.<first port> = max(actuatorVelReq.<axis>, 0), <second port> =
                          |min(actuatorVelReq.<axis>, 0)|, EXCEPT blade (bladeDown takes the positive
                          part). AXIS_PORTS lists (positive port, negative port) in that sense.
  Speed table             par.reqSpdToActCmd.<port>_X / _Y (= parLocalTest, MdlApp.c:50334ff), the table
                          the firmware itself interpolates speed -> percent with (chart_2463 lines 93-112).
                          3 points: X = [0, 0.001, vmax], Y = [0, deadband %, cmdmax %].
                          Units of X: joint rad/s for swing/tilt/rot; CYLINDER STROKE m/s for bm1/arm/link
                          (SetTarActuators builds [q, qDot] = CalStrkAndSpd(...), chart_1076 lines 13-43);
                          track demand % for trvl; blade is open loop.
                          The table a CALIBRATION writes has a different shape (GenCorrTblSetReqSpdToActCmd,
                          chart_2338 l.68-116): X = [0, 0.01, max(MinTblReqSpd 0.002, peak speed)],
                          Y = [0, motion-onset command - 0.5, PropVlvRefCmd] -- the knee sits at 0.01, not the
                          compiled 0.001, and Y[2] is the fixed reference command of the _ToPnt leg
                          (SysPar.m:136-156: 60 swing/trvl, 70 bm1/arm/link/tilt, 100 rot), not a saturation
                          point. Comparing an identified table with this plant's valve therefore checks two
                          numbers only (the onset command and the speed AT PropVlvRefCmd); it does not
                          validate the line between them, which here is the GUESSed linear knee below.
  Inversion              cmd < deadband -> 0; deadband .. Y[2] linear from (deadband, X[1]) to (Y[2], X[2]);
                          >= Y[2] clamped at X[2]. With the table's own deadband (Y[1]) and vmax (X[2])
                          this is exactly the inverse of the firmware's interp1 on the open interval.
                          ASSUMPTION: clamping above Y[2] (the table has no information there; auto caps at
                          70 %, calibration _ToPnt commands up to 100 % for rot, chart_3055 / SysPar.m:153).
                          ASSUMPTION: an overridden deadband/vmax keeps the same 2-segment shape (a spool
                          overlap change shifts the start point, the saturation speed stays at Y[2]).
  Opposite ports          The firmware never opens both ports of one axis: auto splits ONE signed request
                          (chart_2463 l.28-47) and "Keep motion in task space" zeroes the other port when a
                          min-output hold is active (l.136-155); remote splits one signed lever (chart_2327
                          l.4-23); calibration writes exactly one port per calibStep (chart_3055 l.100-186)
                          and every change of direction passes through a _stb state that commands nothing
                          for 8 s, longer than the 1 s SmoothPropVlvCmd ramp (l.194-243, CntCalib_ramp 100);
                          arbitration picks ONE source (chart_2258). For the case that cannot happen:
                          tilt / rotator follow the ECU glue (ecu_glue), which sends the tilt-rotator
                          controller ONE flow command posi + nega with direction Posi if posi > 0 else Nega
                          (AppCtrlIf.c:692-699, OEM) -- applied to the RAW command, before any lag; the EH
                          axes use ASSUMPTION: both pilots act on one double-pilot spool, so the NET command
                          (positive - negative) is looked up on the port it favours (also what a lagged
                          port still decaying after a reversal does).
  Cylinder Jacobian       CalStrkAndSpd (chart_1076 l.106-125, MdlApp.c:1953):
                            angIncluded = s*(q + angCylSml - angCylLrg), s = +1 if isCyl*TopMnt else -1
                            L = sqrt(a^2 + b^2 - 2ab cos(angIncluded))
                            dL/dt = s * a*b*sin(angIncluded)/L * qDot
                          link passes angCylSml = single(0) (chart_1076 l.38-42, the literal, not a par).
                          Positive request = positive stroke speed = the first port (bm1Up, armIn, linkIn):
                          boom q DEcreasing (raise), arm q INcreasing, input link q INcreasing below the
                          +14.2 deg dead centre (test_imu_kinematics TestPhysicalSignConvention).
                          GUESS: |dL/dq| is floored at MIN_JACOBIAN so a plant driven onto a dead centre
                          does not divide by zero; the default joint limits keep it far from there.
  Pilot pressure          u.ehPiPrs.<port> (bar) is the valve-output pilot pressure (AppCtrlIf.c:201-214,
                          PrePostProcArgs.ip.vlvOutPrs*_bar). The firmware reads it ONLY in
                          ValidateSwingAngCalc (chart_1167; MdlApp.c:51884-51928): swing/trvl pressure > 5 bar
                          = operated, < 3 bar = not. PILOT CURVE is OEM platform code, not a guess: the ECU
                          turns percent into EPPR current linearly, 0 % -> 150 mA, 100 % -> 800 mA
                          (PrePostProc_If.c:41-45, 1247-1251, IntrpnFloat clamps), and expects the output
                          pressure from current through the rise curve manJstHystComp_Param.hystUpY/X
                          (PrePostProc_If.c:153-158, used as LookUpTblFloat(current, hystUpY, hystUpX) at
                          :1264-1268 for stuck-valve detection; LookUpTblFloat clamps, ApplCmnFcts.c:1089).
                          Consistency check: VALVE_CURVE_START_MA 360 mA (:44) = 32.3 %, the swing deadband.
  Lag                     GUESS: first-order lag on the percent command per port (EPPR + spool), TAU_VALVE.
                          The firmware's own filters (3 Hz target LPF, 1.2 s smooth step) sit upstream of
                          propVlvCmd and are NOT repeated here.
                          ASSUMPTION (effective_commands): a port whose RAW command is at or above its
                          deadband is open -- its lagged command reads at least the deadband -- while closing
                          follows the lag. Reason: the firmware's minimum-speed hold commands EXACTLY the
                          table deadband (actuatorSpdReq = max(req, X(2)), chart_2463 l.50-69) to keep an
                          axis creeping at X[1]. Approached from ABOVE (an axis already moving) a plain lag
                          stays above the deadband and the rule changes nothing; approached from REST, a
                          lag that only tends to the deadband asymptotically never reaches it and the axis
                          never moves. That case is a firmware scenario, not a corner: PreparePick with the
                          rotator 0.8 deg off its target (outside the 0.5 deg tolerance, inside the
                          min-speed region) commands rotNega = 17.0 % from rest, and without this rule the
                          rotator stays at 0.8 deg forever (test_min_speed_hold_from_rest_needs_the_
                          effective_command_rule). The physical reading: a real spool reaches its commanded
                          position in finite time; only the model's exponential does not. Closing through
                          the lag keeps the short coast after a command drops, which is what u.ehPiPrs
                          exists to cover.

UNITS OF THE RETURNED TARGETS (velocity_targets / joint_velocity_targets)
  swing       rad/s of UcToChs = house w.r.t. undercarriage, counter-clockwise seen from above.
              y.jnts.ChsToUc.qDot = -swing (chart_2143 l.201, 278). swingLe -> positive.
  boom, arm, input_link   rad/s of BmMntToBm1 / Bm2ToArm / ArmToInpLink (firmware == URDF zero and sign).
  tilt, rotator           rad/s of TiltMntToTilt / TiltToRot.
  trvlLe, trvlRi          signed track demand %, forward positive (table X unit). No undercarriage model.
  blade                   signed table units, bladeDown positive (firmware sense). No dozer model.
"""
import math
from collections import namedtuple

import numpy as np

DT = 0.01

PORTS = ("trvlLeFwd", "trvlLeRev", "trvlRiFwd", "trvlRiRev", "swingLe", "swingRi", "bm1Up", "bm1Down",
         "bm2Up", "bm2Down", "armIn", "armOut", "linkIn", "linkOut", "tiltPosi", "tiltNega",
         "rotPosi", "rotNega", "bladeUp", "bladeDown")

# axis -> (port a POSITIVE actuatorVelReq opens, port a negative one opens); chart_2463 l.28-47.
AXIS_PORTS = {
    "swing": ("swingLe", "swingRi"),
    "boom": ("bm1Up", "bm1Down"),
    "arm": ("armIn", "armOut"),
    "input_link": ("linkIn", "linkOut"),
    "tilt": ("tiltPosi", "tiltNega"),
    "rotator": ("rotPosi", "rotNega"),
    "trvlLe": ("trvlLeFwd", "trvlLeRev"),
    "trvlRi": ("trvlRiFwd", "trvlRiRev"),
    "blade": ("bladeDown", "bladeUp"),          # inverted in the firmware: bladeDown = max(req, 0)
}
AXES = tuple(AXIS_PORTS)
# firmware actuator field names (actuatorVelReq.*, ctrlMode.*, isTarActuatorReached.*)
FW_ACTUATOR = {"swing": "swing", "boom": "bm1", "arm": "arm", "input_link": "link", "tilt": "tilt",
               "rotator": "rotate", "trvlLe": "trvlLe", "trvlRi": "trvlRi", "blade": "blade"}
# bm2Up/bm2Down exist in propVlvCmd and the table but have no output channel (AppCtrlIf.c:671-699)
# and hasBm2 = false: no axis.

# u.ehPiPrs has these 16 fields; tilt/rot go to the tilt-rotator controller as a flow command
# (AppCtrlIf.c:692-699) and have no pressure sensor.
FLOW_COMMAND_AXES = ("tilt", "rotator")
PILOT_PORTS = ("trvlLeFwd", "trvlLeRev", "trvlRiFwd", "trvlRiRev", "swingLe", "swingRi", "bm1Up", "bm1Down",
               "bm2Up", "bm2Down", "armIn", "armOut", "linkIn", "linkOut", "bladeUp", "bladeDown")

# OEM platform valve curve (PrePostProc_If.c; see module docstring)
VALVE_CURVE_STANDBY_MA = 150.0
VALVE_CURVE_MAX_MA = 800.0
HYST_UP_MA = (150.0, 243.7, 333.7, 515.8, 675.3, 768.0, 829.2)
HYST_UP_BAR = (0.0, 1.5, 5.0, 14.5, 24.0, 28.5, 31.0)

TAU_VALVE = 0.1      # s. GUESS: EPPR + main spool first-order lag; no source in X1Exc or resources/.
MIN_JACOBIAN = 0.05  # m/rad. GUESS: floor on |dL/dq| (a numerical guard, not a physical value).

Cylinder = namedtuple("Cylinder", "side1 side2 ang_lrg ang_sml top_mount")

# CalStrkAndSpd arguments per axis, chart_1076 l.64-67. None = the single(0) literal for the link.
_CYL_PAR = {
    "boom": ("lenToCylLrgBm1", "lenToCylSmlBm1", "angCylLrgBm1", "angCylSmlBm1", "isCylBm1TopMnt"),
    "arm": ("lenToCylLrgArm", "lenToCylSmlArm", "angCylLrgArm", "angCylSmlArm", "isCylArmTopMnt"),
    "input_link": ("lenToCylLrgBkt", "lenToCylSmlBkt", "angCylLrgBkt", None, "isCylBktTopMnt"),
}
CYLINDER_AXES = tuple(_CYL_PAR)
FW_CYLINDER = {"boom": "bm1", "arm": "arm", "input_link": "bkt"}      # y.cyls.<name>.strk / .spd


# -- reading the firmware ------------------------------------------------------------------------
def read_tables(fw, prefix="par.reqSpdToActCmd"):
    """{port: (X, Y)} as float64 arrays. prefix 'y.tblReqSpdToActCmd' reads a calibration result (a
    different shape, see the module docstring). fw: the firmware, or a plant.Hardware (par.* only)."""
    return {p: (np.array(fw[f"{prefix}.{p}_X"], dtype=float), np.array(fw[f"{prefix}.{p}_Y"], dtype=float))
            for p in PORTS}


def read_commands(fw):
    """{port: percent} from y.propVlvCmd."""
    return {p: float(fw[f"y.propVlvCmd.{p}"]) for p in PORTS}


def ecu_glue(cmds):
    """What leaves the ECU for the tilt-rotator controller (AppCtrlIf.c:692-699): one flow command
    posi + nega, direction Posi if posi > 0 else Nega if nega > 0. Returned as an equivalent port pair with
    one side zero; every other port unchanged. A no-op for anything this firmware outputs."""
    out = dict(cmds)
    for axis in FLOW_COMMAND_AXES:
        pos, neg = AXIS_PORTS[axis]
        cp, cn = float(cmds.get(pos, 0.0)), float(cmds.get(neg, 0.0))
        out[pos], out[neg] = ((cp + cn), 0.0) if cp > 0.0 else (0.0, (cp + cn) if cn > 0.0 else 0.0)
    return out


def cylinders(fw):
    """{axis: Cylinder} from par.parKin, in the argument order CalStrkAndSpd receives."""
    out = {}
    for axis, (a, b, lrg, sml, top) in _CYL_PAR.items():
        k = lambda n: fw[f"par.parKin.{n}"]
        out[axis] = Cylinder(float(k(a)), float(k(b)), float(k(lrg)), 0.0 if sml is None else float(k(sml)),
                             bool(k(top)))
    return out


# -- table inversion -------------------------------------------------------------------------------
def table_speed_to_cmd(speed, X, Y):
    """The firmware's own forward map for ONE port (chart_2463 l.72-112 without the min-speed hold l.50-69, the
    70 % auto cap l.114-133 and the min-output hold l.136-155): clamp speed to [X[0], X[end]], interp1 linear."""
    s = min(max(float(speed), float(X[0])), float(X[-1]))
    return float(np.interp(s, X, Y))


def port_speed(cmd, X, Y, deadband=None, vmax=None):
    """Actuator speed (>= 0, table X units) for a percent command on one port.
    deadband / vmax override Y[1] / X[2] (see the module docstring for the shape)."""
    cmd = float(cmd)
    db = float(Y[1]) if deadband is None else float(deadband)
    top = float(Y[2])
    v1 = float(X[1])
    v2 = float(X[2]) if vmax is None else float(vmax)
    if cmd <= 0.0 or cmd < db:
        return 0.0
    if cmd >= top or top <= db:
        return v2
    return v1 + (cmd - db) * (v2 - v1) / (top - db)


def axis_actuator_speed(cmds, tables, axis, deadband=None, vmax=None):
    """Signed actuator speed of one axis (positive = the first port of AXIS_PORTS[axis]).
    deadband / vmax: optional {port: value} overrides."""
    pos, neg = AXIS_PORTS[axis]
    net = float(cmds.get(pos, 0.0)) - float(cmds.get(neg, 0.0))
    port, sign = (pos, 1.0) if net >= 0.0 else (neg, -1.0)
    db = None if deadband is None else deadband.get(port)
    vm = None if vmax is None else vmax.get(port)
    return sign * port_speed(abs(net), *tables[port], deadband=db, vmax=vm)


# -- cylinder geometry -----------------------------------------------------------------------------
def included_angle(cyl, q):
    s = 1.0 if cyl.top_mount else -1.0
    return s * (q + cyl.ang_sml - cyl.ang_lrg)


def stroke(cyl, q):
    """Cylinder pin-to-pin length L(q), metres (CalStrkAndSpd sideOpp)."""
    a, b = cyl.side1, cyl.side2
    return math.sqrt(max(a * a + b * b - 2.0 * a * b * math.cos(included_angle(cyl, q)), 0.0))


def stroke_jacobian(cyl, q):
    """dL/dq, m/rad (CalStrkAndSpd sideOppRate / jntAngVel). 0 where L <= 1e-6, as the firmware."""
    L = stroke(cyl, q)
    if L <= 1e-6:
        return 0.0
    s = 1.0 if cyl.top_mount else -1.0
    return s * cyl.side1 * cyl.side2 * math.sin(included_angle(cyl, q)) / L


def joint_rate_from_stroke_speed(cyl, q, stroke_speed, min_jacobian=MIN_JACOBIAN):
    """qDot = dL/dt / (dL/dq), with |dL/dq| floored at min_jacobian (GUESS guard, sign kept)."""
    J = stroke_jacobian(cyl, q)
    if abs(J) < min_jacobian:
        J = math.copysign(min_jacobian, J)
    return stroke_speed / J


def dead_centres(cyl):
    """Joint angles in (-pi, pi] where dL/dq = 0 (angIncluded = 0 or pi): over-centre limits."""
    out = []
    for inc in (0.0, math.pi):
        q = (inc / (1.0 if cyl.top_mount else -1.0)) - cyl.ang_sml + cyl.ang_lrg
        out.append(math.atan2(math.sin(q), math.cos(q)))
    return sorted(out)


# -- lag and pilot pressure -------------------------------------------------------------------------
def lag_alpha(dt=DT, tau=TAU_VALVE):
    """Exact discrete first-order step fraction; tau <= 0 means no lag."""
    return 1.0 if tau is None or tau <= 0.0 else 1.0 - math.exp(-dt / tau)


def lag_step(state, cmds, dt=DT, tau=TAU_VALVE):
    """New lagged per-port command dict. state None = start from the command (no lag history)."""
    if state is None:
        return {p: float(cmds.get(p, 0.0)) for p in PORTS}
    a = lag_alpha(dt, tau)
    return {p: state[p] + a * (float(cmds.get(p, 0.0)) - state[p]) for p in PORTS}


def deadbands(tables, deadband=None):
    """{port: deadband %}: the table's Y[1] unless overridden."""
    return {p: float(deadband[p]) if deadband and p in deadband else float(tables[p][1][1]) for p in PORTS}


def effective_commands(raw, lagged, dbs):
    """{port: percent} the flow model sees: the lagged command, but never below the deadband while the RAW
    command is at or above it (module docstring, Lag). dbs from deadbands()."""
    out = {}
    for p in PORTS:
        r, l, db = float(raw.get(p, 0.0)), float(lagged.get(p, 0.0)), dbs[p]
        out[p] = max(l, db) if (r > 0.0 and r >= db) else l
    return out


def pilot_pressure_bar(cmd):
    """Valve output pilot pressure (bar) the ECU expects for a percent command (OEM curve, see docstring)."""
    ma = min(max(VALVE_CURVE_STANDBY_MA + (VALVE_CURVE_MAX_MA - VALVE_CURVE_STANDBY_MA) * float(cmd) / 100.0,
                 VALVE_CURVE_STANDBY_MA), VALVE_CURVE_MAX_MA)
    if ma <= HYST_UP_MA[0]:
        return HYST_UP_BAR[0]
    for i in range(1, len(HYST_UP_MA)):
        if ma < HYST_UP_MA[i]:
            x0, x1, y0, y1 = HYST_UP_MA[i - 1], HYST_UP_MA[i], HYST_UP_BAR[i - 1], HYST_UP_BAR[i]
            return y0 + (y1 - y0) * (ma - x0) / (x1 - x0)
    return HYST_UP_BAR[-1]


# -- targets ----------------------------------------------------------------------------------------
def actuator_speeds(cmds, tables, deadband=None, vmax=None):
    """{axis: signed actuator speed in table units} (cylinder axes in stroke m/s)."""
    return {axis: axis_actuator_speed(cmds, tables, axis, deadband, vmax) for axis in AXES}


def joint_velocity_targets(cmds, tables, cyls, q, deadband=None, vmax=None, min_jacobian=MIN_JACOBIAN):
    """{axis: target rate} in the units of the module docstring. q: {'boom','arm','input_link': rad},
    the joint angles the cylinder Jacobians are evaluated at."""
    spd = actuator_speeds(cmds, tables, deadband, vmax)
    out = dict(spd)
    for axis in CYLINDER_AXES:
        out[axis] = joint_rate_from_stroke_speed(cyls[axis], q[axis], spd[axis], min_jacobian)
    return out


def velocity_targets(fw, q=None, deadband=None, vmax=None, min_jacobian=MIN_JACOBIAN, cmds=None, hardware=None):
    """Axis velocity targets straight from y.propVlvCmd. Tables and cylinder geometry come from `hardware`
    (anything indexable by par path, e.g. plant.hardware) or, by default, the firmware's CURRENT par.*.
    q defaults to the FIRMWARE's joint estimate (y.jnts), so the default is what the firmware's command
    would do to a plant that agrees with its sensors and its parameters; a plant passes its true angles
    and its hardware."""
    if q is None:
        q = {"boom": fw["y.jnts.BmMntToBm1.q"], "arm": fw["y.jnts.Bm2ToArm.q"],
             "input_link": fw["y.jnts.ArmToInpLink.q"]}
    src = fw if hardware is None else hardware
    return joint_velocity_targets(read_commands(fw) if cmds is None else cmds, read_tables(src), cylinders(src),
                                  q, deadband, vmax, min_jacobian)
