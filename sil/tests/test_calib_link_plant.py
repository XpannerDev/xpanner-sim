"""
CalibLink (AutoCtrlStep 24) end to end: the compiled X1Exc firmware calibrates sil.plant.KinematicPlant.

WHAT CALIBLINK DOES (chart_1210 CalibStepMgr, generated MdlApp.c <S179>; every number from SysPar.m)
  LinkPntRef_stb 8 s -> LinkPntRef_log 1 s -> LinkInMin -> LinkInMin_stb 8 s -> LinkInToPnt1 -> LinkPnt1_stb 8 s
  -> LinkPnt1_log 1 s -> LinkOutMin -> LinkOutMin_stb 8 s -> LinkOutToPnt2 -> LinkPnt2_stb 8 s -> LinkPnt2_log 1 s
  -> Link_save -> CalibStandby (CntCalib_stb 800 / _log 100, SysPar.m:102-103).
  _log      1 s running mean of the RAW input-link (bktImu) accelerometer, renormalised (chart_2291 l.74-76, 402-413).
  _Min      propVlvCmd.<port> = 20 % + 0.2 % every 200 ticks, unfiltered (chart_3055 l.58-67, 76, 91-92, 136-141,
            196-197; SysPar.m:105, 110, 172-173), until isMotionOnset: the gravity angle of the board since the
            last log, |angle(acc_log, acc_raw)|, moved > 0.5 deg (chart_2291 l.155-161, chart_2316 l.76-77, 91-104,
            SysPar.m:131). No timeout. Stores propVlvCmd - 0.5 (chart_2338 l.61-62, SysPar.m:134).
  _ToPnt    PropVlvRefCmd.link* = 70 % through the 1 s cubic SmoothPropVlvCmd ramp (chart_3055 l.138-143, 194-243;
            SysPar.m:149-150); ends on angCalib.link > 40 / 80 deg OR cnt > 1000 (MdlApp.c:30491, :30710;
            SysPar.m:122-123). Records the peak of |y.cyls.bkt.spd| -- the firmware's CalStrkAndSpd CYLINDER STROKE
            speed, not a joint rate (MdlApp.c:45061, chart_2383 l.60-63, 83-84).
  table     X = [0, 0.01, max(0.002, peak)], Y = [0, onset - 0.5, 70] (chart_2338 l.42-43, 98-101).
  mount     vy = -cross(accPnt1 - accRef, accPnt2 - accRef), vx = -cross(vy, accRef), vz = cross(vx, vy),
            M = [vx vy vz] (chart_2291 l.303-329). Built from raw accelerometer means only.
  Nothing checks the direction of motion (angCalib is an unsigned angle), the reference posture, or the result.

THE PLANT (sil/plant.py, sil/valves.py; their GUESS/ASSUMPTION tags apply)
  Kinematic: stroke speed = the valve line (deadband, 0.001 m/s) -> (Y[2], vmax) behind a 0.1 s lag, turned into a
  joint rate through the firmware's own cylinder Jacobian. Accelerometer = -1 g quasi-static (no link acceleration).
  Runs strict about joint stops (the firmware would time a leg out on a stop without any alarm, MdlApp.c:30491).
  The plant's valve is deliberately NOT the stored table (PLANT_VALVE), so identification cannot be an echo:
      stored  linkIn  deadband 30.5 %, 0.588 m/s at 70 %     linkOut 31.5 %, 0.85 m/s at 90 %
      plant   linkIn  deadband 24.0 %, 0.20 m/s at 70 %      linkOut 23.0 %, 0.16 m/s at 90 % (0.1125 m/s at 70 %)
  ASSUMPTION vmax: the stored link speeds swing the link ~53 deg past the 40 deg leg during the 1 s ramp-down
  (TestCalibLinkRun.test_the_stored_link_table_is_not_what_calibrating_its_own_valve_returns), which with the GUESS
  link stops (-150 / +10 deg) and the +14.2 deg dead centre leaves few start poses; every scenario here stays 20 deg
  or more inside the stops.

POSTURE (the operator deck, resources/Sensor_and_Valve_Calibration_ProductionV1.md slides 10-11)
  "Pitch angle > 5 deg, Roll angle = 0 deg", "Use the laser level to check ... (horizontal)". Every scenario stands on
  a 6 deg nose-up jack-up (URDF pitch -6 deg, as test_valve_plant's CalibChs) with the input-link chord level to
  GRAVITY: boom + arm + input link = +6 deg. ASSUMPTION u.isMachCalib = 1 (the tablet's service-mode flag; it only
  arms the calibration inhibit mask, MdlApp.c:39678-39680, so a healthy plant runs either way).

RUN TIME  Each run is simulated in lockstep: ~126 s of machine time (~2.5 s wall) for the plant valve, ~270 s
  (~4.5 s wall) for a valve at the stored deadbands. Five runs, each cached and shared by the tests that read it:
  the whole file takes ~18 s.

Run:  cd xpanner-sim && python3 -m unittest sil.tests.test_calib_link_plant -v
"""
import functools
import math
import unittest
from types import SimpleNamespace

import numpy as np

from sil import kinematics as kin
from sil import valves as vlv
from sil.harness import Harness, SaveHandshake
from sil.plant import Hardware, KinematicPlant

DEG = math.pi / 180.0
DT = 0.01


def f32(x):
    return float(np.float32(x))


LINK_SEQUENCE = ("LinkPntRef_stb", "LinkPntRef_log", "LinkInMin", "LinkInMin_stb", "LinkInToPnt1", "LinkPnt1_stb",
                 "LinkPnt1_log", "LinkOutMin", "LinkOutMin_stb", "LinkOutToPnt2", "LinkPnt2_stb", "LinkPnt2_log",
                 "Link_save")
STB_TICKS = 801             # entry tick + 800 (test_calibration_entry STB_TICKS)
LOG_TICKS = 101
TIMEOUT_TICKS = 1002        # a _ToPnt leg that dwells this long ended on cnt > 1000, not on angle
SAVE_TICKS = 2              # with the main.c save handshake (SaveHandshake); 102 without it
CNT_STEP = 200              # CntCalibStepFindingMin, SysPar.m:105
STEP_PCT = 0.2              # StepFindingMin_size, SysPar.m:110
INIT_PCT = 20.0             # PropVlvCmdInitOffs.link*, SysPar.m:172-173
ONSET_CMP = 0.5             # PropVlvCmdMotionOnsetDlyCmp, SysPar.m:134
REF_CMD = 70.0              # PropVlvRefCmd.linkIn/linkOut, SysPar.m:149-150
ONSET_ANG = 0.5 * DEG       # AngMotionOnsetThld, SysPar.m:131
LEG1, LEG2 = 40.0 * DEG, 80.0 * DEG
IDENTIFIED_X1 = f32(0.01)   # chart_2338 l.98-101

DECK_PITCH = -6.0 * DEG     # URDF pitch: 6 deg nose UP (deck "Pitch angle > 5 deg")
PLANT_VALVE = dict(deadband={"linkIn": 24.0, "linkOut": 23.0}, vmax={"linkIn": 0.20, "linkOut": 0.16})
STORED_VALVE = dict(deadband={}, vmax={})   # the plant valve IS the stored table (compiled parameter set)
PORT_SIGN = {"linkIn": 1.0, "linkOut": -1.0}
LEG_OF = {"linkIn": "LinkInToPnt1", "linkOut": "LinkOutToPnt2"}
MIN_OF = {"linkIn": "LinkInMin", "linkOut": "LinkOutMin"}


# ----------------------------------------------------------------------------------------------------------------
# one scenario = one calibration run, everything read back from the firmware before the next run resets it
# ----------------------------------------------------------------------------------------------------------------
class _Probe:
    """Called after the plant, before MdlApp_step(): y.* are the outputs of the PREVIOUS step, plant.* the state that
    step's command produced (the plant's own zero-order hold)."""

    COLS = ("tick", "step", "linkIn", "linkOut", "spd", "q_fw", "qd_fw", "q", "v")

    def __init__(self, plant):
        self.plant, self.rows = plant, []

    def __call__(self, h):
        fw, p = h.fw, self.plant
        self.rows.append((h.tick_count, h.calib_step(), fw["y.propVlvCmd.linkIn"], fw["y.propVlvCmd.linkOut"],
                          fw["y.cyls.bkt.spd"], fw["y.jnts.ArmToInpLink.q"], fw["y.jnts.ArmToInpLink.qDot"],
                          p.q["input_link"], p.speeds["input_link"]))


def _segments(rows):
    """[(state, first row index, last row index)] in order."""
    out = []
    for i, r in enumerate(rows):
        if not out or out[-1][0] != r[1]:
            out.append([r[1], i, i])
        else:
            out[-1][2] = i
    return [tuple(s) for s in out]


def mount_from(src, prefix):
    return np.array([[src[f"{prefix}.a{r}{c}"] for c in (1, 2, 3)] for r in (1, 2, 3)], dtype=float)


def run_calib_link(q0, valve=PLANT_VALVE, pitch=DECK_PITCH, hardware=None, axis_sign=None, after=None):
    plant = KinematicPlant(q0=q0, degrees=True, hardware=hardware, axis_sign=axis_sign, strict_limits=True, **valve)
    plant.set_ground(pitch=pitch)
    probe, emu = _Probe(plant), SaveHandshake()
    h = Harness(plant=[plant, probe, emu]).reset().nominal_inputs()
    h.gnss_rtk_fixed()
    h.fw["u.isMachCalib"] = 1                   # ASSUMPTION, module docstring
    h.tick(3)
    fw = h.fw
    r = SimpleNamespace(plant=plant)
    r.q_start = dict(plant.q)
    r.fw_link_err_before = fw["y.jnts.ArmToInpLink.q"] - plant.q["input_link"]
    r.inhibit_before = h.inhibit_status()
    r.stored_mount = kin.mounts_from_fw(fw)["imuLink"]
    r.compiled_mount = Hardware.compiled(fw).mounts()["imuLink"]
    r.unit_mount = plant.hardware.mounts()["imuLink"]
    r.stored_tables = vlv.read_tables(fw)
    start = h.tick_count
    h.jump_to_step("CalibLink")
    h.run_until(lambda h: h.curr_step() == "NoTarget", 600.0, "CalibLink back in NoTarget")
    r.ticks = h.tick_count - start
    r.final = dict(curr_step=h.curr_step(), calib_step=h.calib_step(), calibrating=fw["y.isCalibrating"],
                   inhibit=h.inhibit_status())
    r.saves = [snap for _, snap in emu.saved]
    r.rows = probe.rows
    r.segs = [s for s in _segments(probe.rows) if s[0] != "CalibStandby"]
    r.limit_hits = list(plant.limit_hits)
    r.q_range = (min(row[7] for row in probe.rows), max(row[7] for row in probe.rows))
    if after is not None:
        r.after = after(h, plant, r)
    return r


def rows_of(r, state):
    seg = next(s for s in r.segs if s[0] == state)
    return r.rows[seg[1]:seg[2] + 1]


def dwell(r, state):
    seg = next(s for s in r.segs if s[0] == state)
    return seg[2] - seg[1] + 1


def identified(r, port):
    snap = r.saves[0]
    return snap[f"y.tblReqSpdToActCmd.{port}_X"], snap[f"y.tblReqSpdToActCmd.{port}_Y"]


def load_identified_mount(h, plant, r, ticks=100):
    """Write the identified mount into par.imuLink. In THIS build nothing does that (calibration results are outports
    only, parLocalTest is never written outside MdlApp.c; test_calibration_entry): this is what a wired NVM restore
    would do. Returns the firmware's input-link reading error after the joint LPF settles."""
    kin.write_mounts(h.fw, {"imuLink": mount_from(r.saves[0], "y.imuMntOri_link")})
    h.tick(ticks)
    return h.fw["y.jnts.ArmToInpLink.q"] - plant.q["input_link"]


def predicted_onset(plant, port, q_start):
    """Firmware-free prediction of the _Min exit: replay the staircase through the SAME valve lag / effective-command /
    table-inversion / Jacobian functions the plant integrates with, and return (staircase level at which the input
    link has moved > AngMotionOnsetThld, ticks the level had been applied). The level is float32 20 + single(k)*0.2
    (chart_3055 l.76, 91-92); the command reaches the plant one tick after the step that wrote it."""
    X, Y = plant.tables[port]
    db, vm = plant.deadband.get(port), plant.vmax.get(port)
    dbs = vlv.deadbands(plant.tables, plant.deadband)
    cyl = plant.cyls["input_link"]
    a = vlv.lag_alpha(DT, plant.tau_valve)
    lag, q = 0.0, q_start                       # the preceding _log state commanded nothing
    for i in range(400 * CNT_STEP):             # i = row index inside _Min; row i's command is integrated in row i
        k = (i + 1) // CNT_STEP
        level = staircase(k)
        lag = lag + a * (level - lag)
        eff = vlv.effective_commands({port: level}, {port: lag}, dbs)[port]
        v = PORT_SIGN[port] * vlv.port_speed(eff, X, Y, db, vm)
        q += vlv.joint_rate_from_stroke_speed(cyl, q, v, plant.min_jacobian) * DT
        if abs(q - q_start) > ONSET_ANG:
            return level, i - (k * CNT_STEP - 1)
    raise AssertionError("no onset within 800 s")


def staircase(k):
    """float32 command of staircase level k: PropVlvCmdInitOffs + single(k) * StepFindingMin_size (chart_3055 l.76, 91)."""
    return f32(np.float32(INIT_PCT) + np.float32(k) * np.float32(STEP_PCT))


# The deck posture: chord level to gravity on the 6 deg jack-up.
Q_BASE = dict(boom=-40.0, arm=130.0, input_link=-84.0)


@functools.lru_cache(maxsize=None)
def scenario_base():
    return run_calib_link(Q_BASE)


@functools.lru_cache(maxsize=None)
def scenario_stored_valve():
    # The stored link speeds need a lower start: observed range -125.6 .. -11.1 deg (GUESS stops -150 / +10).
    return run_calib_link(dict(boom=-40.0, arm=150.0, input_link=-104.0), valve=STORED_VALVE)


R_BOARD = kin.Rz(4.0 * DEG) @ kin.Rx(-7.0 * DEG) @ kin.Ry(9.0 * DEG)   # 11.9 deg about a skew axis, link coordinates


@functools.lru_cache(maxsize=None)
def scenario_unit_board():
    def after(h, plant, r):
        return SimpleNamespace(err_after_load=load_identified_mount(h, plant, r))

    hw = Hardware.compiled(Harness().reset().fw)
    return run_calib_link(Q_BASE, hardware={"imuLink": hw.mounts()["imuLink"] @ R_BOARD}, after=after)


@functools.lru_cache(maxsize=None)
def scenario_machine_level_chord():
    # Same jack-up, chord level to the MACHINE (boom + arm + input link = 0): what setting it by the tablet's joint
    # readout instead of a laser level does.
    def after(h, plant, r):
        err = load_identified_mount(h, plant, r)
        return SimpleNamespace(err_after_load=err,
                               outp_err=h.fw["y.jnts.ArmToOutpLink.q"] - plant.output_link())

    return run_calib_link(dict(boom=-40.0, arm=130.0, input_link=-90.0), after=after)


@functools.lru_cache(maxsize=None)
def scenario_backwards_hose():
    def after(h, plant, r):
        fw = h.fw
        err = load_identified_mount(h, plant, r)
        out = SimpleNamespace(err_after_load=err, q_fw=fw["y.jnts.ArmToInpLink.q"], q=plant.q["input_link"],
                              outp_fw=fw["y.jnts.ArmToOutpLink.q"], outp=plant.output_link(),
                              inhibit=h.inhibit_names())
        plant.manual = {"linkIn": 40.0}           # outside the firmware: the plant alone moves
        q0_fw, q0 = out.q_fw, out.q
        h.tick(60)                                 # > 10 time constants of the 3 Hz joint LPF
        out.manual = SimpleNamespace(dq=plant.q["input_link"] - q0, dq_fw=fw["y.jnts.ArmToInpLink.q"] - q0_fw,
                                     qd=plant.qdot["input_link"], qd_fw=fw["y.jnts.ArmToInpLink.qDot"])
        plant.manual = {}
        return out

    return run_calib_link(dict(boom=-40.0, arm=120.0, input_link=-74.0), axis_sign={"input_link": -1}, after=after)


def rotation_angle_axis(R):
    ang = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0)))
    ax = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    n = np.linalg.norm(ax)
    return ang, (ax / n if n > 1e-12 else ax)


# ================================================================================================================
class TestCalibLinkRun(unittest.TestCase):
    """The plant valve is not the stored table; the firmware must find the plant's."""

    def test_completes_through_the_thirteen_substeps_saves_once_and_no_leg_times_out(self):
        # FW: sequence and guards chart_1210 (transitions listed in the module docstring); success = calibStep back to
        # CalibStandby and autoCtrl_CurrStep back to NoTarget (spec A7 step 5); the exit through J260
        # [cntSave > CntCalib_save || ~isCalibrating || hasChanged(isCalibDataSaved) && isCalibDataSaved] is shared by
        # every Link state, so ONE save and a 2-tick Link_save show the handshake ended it, not an abort.
        r = scenario_base()
        self.assertEqual(tuple(s[0] for s in r.segs), LINK_SEQUENCE)
        self.assertEqual(r.final, dict(curr_step="NoTarget", calib_step="CalibStandby", calibrating=False,
                                       inhibit=r.inhibit_before))
        self.assertEqual(len(r.saves), 1)
        self.assertEqual(r.limit_hits, [], "strict plant: no stop contact")
        for s in LINK_SEQUENCE:
            with self.subTest(state=s):
                if s.endswith("_stb"):
                    self.assertEqual(dwell(r, s), STB_TICKS)
                elif s.endswith("_log"):
                    self.assertEqual(dwell(r, s), LOG_TICKS)
        self.assertEqual(dwell(r, "Link_save"), SAVE_TICKS)
        for leg in LEG_OF.values():
            self.assertLess(dwell(r, leg), TIMEOUT_TICKS, f"{leg} ended on the 10 s timeout")
        # ... and on the plant's true angle: on the jack-up the pin axis is level, so the board's gravity angle is the
        # joint travel since the log the leg is measured from (observed +40.45 / -80.37 deg at the last leg tick,
        # 2-3 ticks of travel past the threshold)
        for leg, log, thld, sign in (("LinkInToPnt1", "LinkPntRef_log", LEG1, 1.0), ("LinkOutToPnt2", "LinkPnt1_log", LEG2, -1.0)):
            travel = sign * (rows_of(r, leg)[-1][7] - rows_of(r, log)[-1][7])
            self.assertTrue(thld < travel < thld + 1.0 * DEG, f"{leg}: {math.degrees(travel):.2f} deg")
        # observed: LinkInToPnt1 202 ticks, LinkOutToPnt2 511 ticks; the link never came within 30 deg of a stop
        lo, hi = r.plant.limits["input_link"]
        self.assertGreater(r.q_range[0] - lo, 30 * DEG)
        self.assertGreater(hi - r.q_range[1], 30 * DEG)

    def test_duration_is_the_min_staircase_two_seconds_per_fifth_of_a_percent_per_direction(self):
        # WHY: the budget question. Everything but the two _Min states is fixed or short: 5 x 8 s dwell + 3 x 1 s log
        # + two legs (1-6 s together) + save. Each _Min state lasts 2 s per 0.2 % from 20 % to the level at which motion is
        # detected, plus the time into that level: on the plant valve (24 / 23 %) the run is ~126 s, on a valve at
        # the stored deadbands (30.5 / 31.5 %) ~270 s, of which the staircase is 83 %. There is no _Min timeout: a
        # valve that never opens keeps it for 800 s and then forever at 100 % (test_calibration_entry).
        # FW: staircase chart_3055 l.58-67, 76, 91-92 (raw command during _Min, l.196-197); SysPar.m:105, 110.
        for label, r in (("plant valve", scenario_base()), ("stored valve", scenario_stored_valve())):
            with self.subTest(valve=label):
                fixed = 5 * STB_TICKS + 3 * LOG_TICKS + SAVE_TICKS
                legs = sum(dwell(r, s) for s in LEG_OF.values())
                mins = sum(dwell(r, s) for s in MIN_OF.values())
                # plus 3 CalibStandby ticks: the step request, the press, the tick back to NoTarget (harness idiom)
                self.assertEqual(r.ticks, fixed + legs + mins + 3, "every other tick of the run is in a Link state")
                for port, state in MIN_OF.items():
                    rows = rows_of(r, state)
                    # the command during _Min is exactly the float32 staircase (row index i = entry tick + i)
                    for i, row in enumerate(rows):
                        want = staircase((i + 1) // CNT_STEP)
                        if row[2 if port == "linkIn" else 3] != want:
                            self.fail(f"{state} row {i}: {row[2 if port == 'linkIn' else 3]} != staircase {want}")
                    level = rows[-1][2 if port == "linkIn" else 3]
                    n = round((level - INIT_PCT) / STEP_PCT)
                    self.assertTrue(n * CNT_STEP - 1 <= len(rows) < (n + 1) * CNT_STEP - 1, (state, level, len(rows)))
                seconds = r.ticks * DT
                if label == "plant valve":
                    self.assertTrue(120.0 < seconds < 132.0, seconds)          # observed 125.0 s
                    self.assertGreater(mins / r.ticks, 0.55)                  # observed 0.60
                else:
                    self.assertTrue(260.0 < seconds < 280.0, seconds)          # observed 270.5 s
                    self.assertGreater(mins / r.ticks, 0.80)                  # observed 0.83

    def test_identified_minimum_command_is_the_onset_level_minus_half_a_percent_so_below_the_valve_opening(self):
        # WHY: this is the "identified deadband versus the plant's" check, done three ways.
        #  1. firmware bookkeeping: Y[1] == float32(command on the last _Min tick - 0.5) (chart_2338 l.61-62);
        #  2. that level is the one a firmware-free replay of the staircase through the plant's own valve model
        #     predicts (predicted_onset), and the plant's recorded input link crossed 0.5 deg on that level at most
        #     2 ticks before the firmware left _Min (IMU -> angCalib -> onset edge -> Delay4 -> chart);
        #  3. the level is the first or second staircase level at or above the plant deadband, never below it (the
        #     plant does not move below its deadband: the recorded travel before that level is exactly 0).
        # FINDING (PropVlvCmdMotionOnsetDlyCmp): the staircase dwells 2 s per level, and at its opening command the
        # valve already moves the link 0.27-0.8 deg per level (X[1] = 0.001 m/s for 2 s through J = 0.42 m/rad at the
        # linkIn start, 0.14-0.21 m/rad where linkOut starts after the coast), so motion is detected on the opening
        # level or the next one (<= 0.2 % late) -- yet the firmware subtracts 0.5 %. The identified minimum is BELOW the valve's opening point in every run:
        # 23.7 for 24.0, 22.5 for 23.0 (plant valve); 30.1 for 30.5, 31.1 for 31.5 (stored-deadband valve). A
        # minimum-speed hold that commands exactly Y[1] (chart_2463 l.50-69) then commands a closed valve. On this
        # plant PreparePick still converges (the link reaches tolerance while the command is above Y[1]; checked in
        # development, not asserted here), so the consequence is not demonstrated -- the bias is. It depends on the
        # GUESSed valve line near the deadband (sil/valves.py): a valve whose flow rises more slowly would push the
        # onset further up and could hide it.
        for label, r in (("plant valve", scenario_base()), ("stored valve", scenario_stored_valve())):
            for port in ("linkIn", "linkOut"):
                with self.subTest(valve=label, port=port):
                    col = 2 if port == "linkIn" else 3
                    rows = rows_of(r, MIN_OF[port])
                    level = rows[-1][col]
                    X, Y = identified(r, port)
                    self.assertEqual(Y[1], f32(np.float32(level) - np.float32(ONSET_CMP)))            # 1
                    q_entry = rows[0][7]
                    moved = [i for i, row in enumerate(rows) if abs(row[7] - q_entry) > ONSET_ANG]
                    self.assertTrue(moved, "the plant never moved 0.5 deg inside _Min")
                    self.assertLessEqual(len(rows) - 1 - moved[0], 2, "onset detected > 2 ticks after the plant moved")
                    self.assertEqual(rows[moved[0]][col], level)
                    pred, into = predicted_onset(r.plant, port, q_entry)                              # 2
                    self.assertGreater(into, 3, "prediction too close to a level boundary to be exact")
                    self.assertEqual(pred, level)
                    db = vlv.deadbands(r.plant.tables, r.plant.deadband)[port]                        # 3
                    k_open = next(k for k in range(400) if staircase(k) >= db)
                    self.assertIn(level, (staircase(k_open), staircase(k_open + 1)))
                    before = [row for row in rows if row[col] < db]
                    self.assertTrue(all(row[7] == q_entry for row in before), "plant moved below its deadband")
                    self.assertLess(Y[1], db, "identified minimum is below the valve opening (FINDING)")
                    self.assertGreaterEqual(Y[1], db - ONSET_CMP - 1e-4)
                    self.assertNotIn(Y[1], (f32(INIT_PCT - ONSET_CMP),), "19.5 = the axis never moved")
                    if label == "plant valve":
                        self.assertNotAlmostEqual(Y[1], r.stored_tables[port][1][1], delta=5.0, msg="not an echo")

    def test_identified_speed_is_the_plant_stroke_speed_at_the_reference_command(self):
        # WHY: X[2] is the peak of the FIRMWARE's cylinder stroke speed (J(q_fw) * qDot_fw from gyro differences,
        # MdlApp.c:45061) over the 70 % leg; the plant's is the valve line at 70 % turned into a joint rate through
        # the same Jacobian. Plant valve: linkIn 70 % = Y[2] -> vmax 0.20 m/s, identified 0.1995 (-0.3 %); linkOut
        # on a 23 -> 90 % line -> 0.11254 m/s, identified 0.11287 (+0.3 %). The table's shape is the calibration's,
        # not the stored one: X[1] 0.01 (stored 0.001), Y[2] 70 (stored linkOut 90).
        # Stored-deadband valve (the compiled table as the plant): the leg is short (126 ticks) and the link fast, so
        # the firmware's 3 Hz joint LPF lag shows: linkIn 0.5784 against the plant's 0.5845 peak (-1.0 %) and the
        # stored 0.5881 (-1.6 %); linkOut 0.5620 against 0.5596 (+0.4 %).
        # FW: chart_2383 l.60-63, 83-84 (reset to 0 on leg entry, peak while in the leg); chart_2338 l.98-101.
        for label, r, tol_truth in (("plant valve", scenario_base(), 0.005), ("stored valve", scenario_stored_valve(), 0.02)):
            for port in ("linkIn", "linkOut"):
                with self.subTest(valve=label, port=port):
                    X, Y = identified(r, port)
                    self.assertEqual((X[0], X[1], Y[0], Y[2]), (0.0, IDENTIFIED_X1, 0.0, REF_CMD))
                    leg = rows_of(r, LEG_OF[port])
                    # firmware bookkeeping: the peak of |y.cyls.bkt.spd| over the leg (one-tick output skew allowed)
                    seg = next(s for s in r.segs if s[0] == LEG_OF[port])
                    window = r.rows[seg[1] - 1:seg[2] + 2]
                    self.assertIn(X[2], [abs(f32(row[4])) for row in window])
                    self.assertLessEqual(X[2], max(abs(row[4]) for row in window))
                    truth_peak = max(abs(row[8]) for row in leg)
                    Xp, Yp = r.plant.tables[port]
                    at_ref = vlv.port_speed(REF_CMD, Xp, Yp, r.plant.deadband.get(port), r.plant.vmax.get(port))
                    self.assertLessEqual(truth_peak, at_ref + 1e-9)
                    self.assertAlmostEqual(X[2], truth_peak, delta=tol_truth * truth_peak)
                    if label == "plant valve":
                        self.assertAlmostEqual(truth_peak, at_ref, delta=1e-4 * at_ref, msg="valve settled in the leg")
                        self.assertAlmostEqual(X[2], at_ref, delta=0.005 * at_ref)
                        self.assertLess(X[2], 0.5 * r.stored_tables[port][0][2], "not the stored speed")

    def test_the_stored_link_table_is_not_what_calibrating_its_own_valve_returns(self):
        # WHY / CONTRADICTS spec A4.3 ("bm1/arm/link Xmax are byte-identical between LongArm and ShortArm and round
        # 2-figure numbers ... demand clamps"). In the compiled ShortArm set (ECR88D_ShortArm.m:326-329) linkIn is
        # X = [0; 0.001; 0.588065445], Y = [0; 30.5; 70]: nine digits, Y[2] = PropVlvRefCmd, Y[1] + 0.5 = 31.0 on the
        # 20 + 0.2 k staircase -- the fingerprint of a CalibLink result with its knee edited back to 0.001 (LongArm has
        # 0.28). linkOut is Y[1] 31.5 (also staircase + 0.5) but X[2] 0.85 at Y[2] 90: not calibration-shaped.
        # Calibrating a plant whose valve IS that table does not return it: linkIn 30.1 % / 0.578 m/s (the onset bias
        # and the estimator lag above), linkOut 31.1 % / 0.562 m/s at 70 % replacing 0.85 at 90 %.
        # The same run shows the LEG COAST (PLANT-DEPENDENT: constant stroke speed, no load): the 1 s ramp-down after
        # the 40 deg leg carries the link a further ~53 deg at the stored speed (observed 93 deg from the reference),
        # to 25 deg below its +14.2 deg dead centre from this start; started at -90 deg it ran into the dead centre
        # (development run). On the plant valve the coast is ~20 deg.
        r = scenario_stored_valve()
        st = r.stored_tables
        self.assertEqual((list(st["linkIn"][1]), list(st["linkOut"][1][[0, 2]])), ([0.0, 30.5, 70.0], [0.0, 90.0]))
        self.assertAlmostEqual(st["linkIn"][0][2], 0.588065445, places=6)
        for port in ("linkIn", "linkOut"):
            onset = st[port][1][1] + ONSET_CMP
            self.assertAlmostEqual((onset - INIT_PCT) / STEP_PCT, round((onset - INIT_PCT) / STEP_PCT), places=4)
        for port, want_min, want_speed in (("linkIn", 30.1, 0.578), ("linkOut", 31.1, 0.562)):
            with self.subTest(port=port):
                X, Y = identified(r, port)
                self.assertAlmostEqual(Y[1], want_min, places=4)
                self.assertAlmostEqual(X[2], want_speed, delta=0.002)
                self.assertNotAlmostEqual(Y[1], st[port][1][1], places=2)
                self.assertNotAlmostEqual(X[2], st[port][0][2], delta=0.005)
        ref = rows_of(r, "LinkPntRef_log")[-1][7]
        coast = max(row[7] for row in rows_of(r, "LinkPnt1_stb")) - ref
        self.assertGreater(coast, LEG1 + 45 * DEG)
        base = scenario_base()
        ref_b = rows_of(base, "LinkPntRef_log")[-1][7]
        self.assertLess(max(row[7] for row in rows_of(base, "LinkPnt1_stb")) - ref_b, LEG1 + 25 * DEG)
        self.assertEqual(r.limit_hits, [])


# ================================================================================================================
class TestCalibLinkMount(unittest.TestCase):
    """par.imuLink (the stored mount, = compiled here) is what the firmware believes; plant.hardware is the unit."""

    def test_deck_posture_returns_the_units_mount_control(self):
        # The control for the tests below: chord level to gravity on the jack-up, unit board = compiled mount.
        # FW: chart_2291 l.303-329 with -1 g (plant ACC_SIGN): vy = pin axis, vx = level direction across it.
        r = scenario_base()
        np.testing.assert_array_equal(r.unit_mount, r.compiled_mount)
        self.assertLess(abs(r.fw_link_err_before), 1e-4 * DEG)
        self.assertLess(np.abs(mount_from(r.saves[0], "y.imuMntOri_link") - r.unit_mount).max(), 1e-4)

    def test_a_unit_board_that_is_not_the_stored_mount_is_recovered_and_the_speed_table_is_corrupted(self):
        # WHY: proves the mount is REBUILT from gravity, not echoed. The unit's board is turned 11.9 deg about a skew
        # axis away from the stored (compiled) mount, so the firmware reads the input link 8.6 deg low before the
        # run. The rebuild uses raw accelerometer means only, so it returns the UNIT's mount (to float32 precision), and
        # loaded into par.imuLink the firmware reads the true link (2e-5 deg).
        # The _Min onset is accelerometer-only too: the identified minimum commands are bit-identical to the run
        # with the correct mount.
        # FINDING: the speed table is identified through the WRONG mount -- the peak is CalStrkAndSpd(q_fw, qDot_fw)
        # at the misread angle and misprojected gyro rate: linkIn +9.3 %, linkOut +11.4 % against the same valve with
        # the right mount. The mount is fixed by the calibration; the table it wrote in the same run is not, until
        # CalibLink is run again with the corrected mount loaded (same mechanism as the arm, test_valve_plant).
        r, base = scenario_unit_board(), scenario_base()
        identified_mount = mount_from(r.saves[0], "y.imuMntOri_link")
        self.assertLess(np.abs(identified_mount - r.unit_mount).max(), 1e-4)
        self.assertGreater(np.abs(identified_mount - r.stored_mount).max(), 0.15, "not the stored mount echoed")
        self.assertAlmostEqual(math.degrees(rotation_angle_axis(r.compiled_mount.T @ r.unit_mount)[0]), 11.89, delta=0.01)
        self.assertTrue(-9.0 * DEG < r.fw_link_err_before < -8.0 * DEG, math.degrees(r.fw_link_err_before))
        self.assertLess(abs(r.after.err_after_load), 1e-3 * DEG)
        for port in ("linkIn", "linkOut"):
            with self.subTest(port=port):
                X, Y = identified(r, port)
                Xb, Yb = identified(base, port)
                self.assertEqual(Y[1], Yb[1])
                ratio = X[2] / Xb[2]
                self.assertTrue(1.05 < ratio < 1.15, f"{port}: {ratio:.4f}")

    def test_the_reference_is_gravity_level_so_a_machine_level_chord_on_the_jack_up_bakes_the_pitch_in(self):
        # WHY: vx = -cross(vy, accRef) is the LEVEL direction across the pin at the reference log, so the rebuilt
        # mount equals the unit's only if the input-link chord is level to gravity then. The deck asks for a jack-up
        # (Pitch > 5 deg) AND a laser level: consistent (scenario_base, exact). Setting the chord level to the MACHINE
        # instead (boom + arm + input link = 0, e.g. from the tablet's joint readout) on the same 6 deg jack-up gives
        # a mount turned exactly 6.00 deg about the pin axis, and once loaded the firmware reads the input link
        # +6.00 deg off and the four-bar output link / tool attitude off by more, with no check anywhere.
        # FW: chart_2291 l.303-329; nothing in chart_1210 or the main chart reads machine pitch for step 24.
        r = scenario_machine_level_chord()
        self.assertEqual(len(r.saves), 1)
        E = r.compiled_mount.T @ mount_from(r.saves[0], "y.imuMntOri_link")
        ang, axis = rotation_angle_axis(E)
        self.assertAlmostEqual(math.degrees(ang), 6.0, delta=0.01)
        self.assertGreater(abs(axis[1]), 0.9999, f"about the pin axis: {axis}")
        self.assertAlmostEqual(math.degrees(r.after.err_after_load), 6.0, delta=0.01)
        self.assertGreater(abs(r.after.outp_err), 6.0 * DEG)
        # the same valve numbers as the gravity-level run: the posture error is invisible in the saved table
        base = scenario_base()
        for port in ("linkIn", "linkOut"):
            self.assertAlmostEqual(identified(r, port)[0][2], identified(base, port)[0][2], delta=0.01)

    def test_a_backwards_link_hose_calibrates_silently_into_a_mount_turned_180_deg(self):
        # WHY / FINDING: angCalib.link is an UNSIGNED gravity angle (CalcAccVecAngle, chart_2291 l.489-496) and no
        # state checks which way the link went. With linkIn and linkOut plumbed backwards (plant axis_sign) the run
        # completes, both legs end on angle, the same per-port minimum commands and speeds are saved, no inhibit bit
        # changes -- and the rebuilt mount is the unit's turned 180 deg about the board-to-link Z axis:
        # vy = -cross(v1, v2) flips with the rotation sense, vx follows, vz does not (M @ diag(-1, -1, 1)).
        # Loaded, the firmware's input-link angle is the true one MIRRORED about the level chord,
        # q_fw = -2 (pitch + boom + arm) - q (observed -108.9 deg for a true -39.1 deg; the four-bar output link 94 deg
        # off), and its joint RATE is mirrored too (+0.227 rad/s for a true -0.226 rad/s). So after the load linkIn
        # moves both estimates the way the firmware's model of linkIn expects: the reversed hose now closes every loop
        # sign, around an absolute link angle that is wrong by twice the chord's pitch (70 deg here) and a tool
        # attitude that is wrong by more. In this build the mount is an outport only (never loaded,
        # test_calibration_entry), so this is latent until the NVM restore is wired. The boom and arm rebuilds have
        # the same structure (chart_2291 l.219-301) and should share the defect (not run here).
        r = scenario_backwards_hose()
        self.assertEqual(tuple(s[0] for s in r.segs), LINK_SEQUENCE)
        self.assertEqual(r.final["curr_step"], "NoTarget")
        self.assertEqual(r.final["inhibit"], r.inhibit_before)
        self.assertEqual(len(r.saves), 1)
        for leg in LEG_OF.values():
            self.assertLess(dwell(r, leg), TIMEOUT_TICKS)
        # the link really went the other way on the linkIn leg, and the firmware (still on the stored mount) saw it
        ref, end = rows_of(r, "LinkPntRef_log")[-1], rows_of(r, "LinkInToPnt1")[-1]
        self.assertLess(end[7] - ref[7], -LEG1 + 1 * DEG)                 # plant: -40.5 deg
        self.assertLess(end[5] - ref[5], -LEG1 + 3 * DEG)                 # firmware, 3 Hz LPF behind: -38.6 deg
        base = scenario_base()
        for port in ("linkIn", "linkOut"):
            with self.subTest(port=port):
                X, Y = identified(r, port)
                self.assertLess(Y[1], PLANT_VALVE["deadband"][port])
                self.assertGreaterEqual(Y[1], PLANT_VALVE["deadband"][port] - ONSET_CMP - 1e-4)
                self.assertAlmostEqual(X[2], identified(base, port)[0][2], delta=0.01 * X[2])
        M = mount_from(r.saves[0], "y.imuMntOri_link")
        self.assertLess(np.abs(M - r.unit_mount @ np.diag([-1.0, -1.0, 1.0])).max(), 1e-4)
        a = r.after
        chord_offset = DECK_PITCH + (r.q_start["boom"] + r.q_start["arm"])
        self.assertAlmostEqual(a.q_fw, -2.0 * chord_offset - a.q, delta=0.01 * DEG)
        self.assertGreater(abs(a.outp_fw - a.outp), 45 * DEG)
        m = a.manual
        self.assertLess(m.dq, -3.0 * DEG, "linkIn moves the backwards link down")
        self.assertGreater(m.dq_fw, 3.0 * DEG, "the mirrored angle estimate moves up")
        self.assertLess(m.qd, 0.0)
        self.assertAlmostEqual(m.qd_fw, -m.qd, delta=0.02 * abs(m.qd), msg="the rate estimate is mirrored too")


# ================================================================================================================
class TestCalibLinkDeckGeometry(unittest.TestCase):

    def test_the_decks_connecting_rod_cannot_be_the_link_that_must_be_level(self):
        # UNEXPLAINED (question for Olivia/David): deck slide 11 says "check connecting rod joints position
        # (horizontal)"; the rebuild needs the INPUT-LINK chord level (the bktImu frame is R_arm * Ry(ArmToInpLink),
        # MdlApp.c:11670-11684; the compiled imuLink comes back with that chord level, TestCalibLinkMount). Through the
        # firmware's own four-bar the connecting rod is 88-148 deg from the input link over the GUESS joint range
        # (input link -150 .. +10 deg), at least 32 deg from parallel, and 88-135 deg (>= 45 deg from parallel) over
        # the -130 .. -10 deg these calibrations sweep: the two are never level together, and read literally the deck
        # posture would bake 45-90 deg into the mount. The deck text is an extraction of a photo slide; its
        # "connecting rod" most likely names the input link.
        fw = Harness().reset().fw
        k = lambda n: fw[f"par.parKin.{n}"]
        a, c, d, g = k("lenInpLink"), k("lenOutpLink"), k("lenGndLink"), k("angArmToGndLink")
        rel = {}
        for qd in range(-150, 11):
            qi = qd * DEG
            t2, t4 = qi - g, kin.fourbar_output(fw, qi) - g
            rod = np.array([d + c * math.cos(t4) - a * math.cos(t2), c * math.sin(t4) - a * math.sin(t2)])
            self.assertAlmostEqual(np.linalg.norm(rod), k("lenConnRod"), places=5)   # the loop closes
            rel[qd] = abs(math.degrees(math.atan2(rod[1], rod[0]) + g - qi))
        from_parallel = lambda qs: min(min(rel[q], 180.0 - rel[q]) for q in qs)
        self.assertGreater(min(rel.values()), 85.0)
        self.assertGreater(from_parallel(range(-150, 11)), 30.0)
        self.assertGreater(from_parallel(range(-130, -9)), 44.0)


if __name__ == "__main__":
    unittest.main()
