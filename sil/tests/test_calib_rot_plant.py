"""
CalibRot (AutoCtrlStep 26) end to end: the compiled X1Exc firmware calibrates the rotator of
sil.plant.KinematicPlant, and every number it saves is checked against the PLANT, not against the
stored table.

WHAT CalibRot IS (chart_1210 CalibStepMgr, 11 sub-steps, no gravity triple, no IMU mount)
    RotPosiPntRef_stb 8 s -> RotPosiPntRef_log 1 s (angRefRot = mean of RAW u.jntAngRaw_Rot, chart_2291 l.80-81)
    -> RotPosiMin (staircase on rotPosi until a 0.5 deg motion onset, chart_3055 l.53-99 / chart_2316)
    -> RotPosiMin_stb 8 s -> RotPosi (rotPosi = PropVlvRefCmd 100 % through a 1 s cubic ramp, until
    |wrap(raw - angRefRot)| > 60 deg or cnt > 1000) -> the same four for Nega with the reference RE-LATCHED
    (n restarts at 0, UpdateAngleAvg's first sample replaces the mean) and a 120 deg leg -> Rot_save.
    Identified table (chart_2338 GenCorrTblSetReqSpdToActCmd l.108-111):
        X = [0, 0.01, peak |y.jnts.TiltToRot.qDot| during the leg]   (chart_2383 l.87-88, MdlApp.c:45063)
        Y = [0, command at the onset - 0.5 (PropVlvCmdMotionOnsetDlyCmp), 100]

RUN TIME BUDGET (measured on ROT_HW, pinned in test_run_time_budget_on_this_plant -- every variable part depends
    on the plant's valve model, so the pins are kept out of the chart-order test; ~4000 ticks/s wall on this server)
    Fixed part 2 x (801 + 101 + 801) = 3406 ticks = 34 s. Each _Min staircase costs 2 s per 0.2 % of
    deadband above PropVlvCmdInitOffs = 20 % (SysPar.m:105, 110, 176-177), plus up to 2 s. The main plant here
    (deadbands 21.0 / 22.3 %) takes 15.3 s + 28.5 s of staircase and 4.1 s + 7.1 s of legs: 89 s simulated,
    ~2.3 s wall. A 32 % deadband (the stored swing value) would cost ~120 s of staircase per direction.
    The compiled rotator deadband (17 %) is below the staircase start, so a machine like the compiled one
    finishes CalibRot in ~50 s (test_valve_plant.test_calib_rot_cannot_identify_a_deadband_below_the_staircase_start).

FINDINGS (numbers are asserted below). The tag is the class reported to the firmware owner:
    [firmware] shown by the source whatever the plant   [latent] source-level, but nothing in this build reaches it
    [plant]    depends on a plant GUESS / ASSUMPTION (valve line near the deadband, TR controller behaviour, ...)
  F1 (not a defect) identification works and is exact on this plant: onset command = the staircase value at which
     the plant has turned 0.5 deg, predicted in closed form from the plant's valve line (predict_onset) and equal
     to the firmware's for 3 deadband pairs; speed at 100 % within 1e-4 relative (7e-7 on the reference run). Both
     are independent of the LIVE stored rotator table (test_identification_ignores_the_live_stored_table...).
  F2 [firmware] CalibRot is sign-blind: a backwards-plumbed rotator (equivalently a wrong INTP.trRotDir,
     PrePostProc_If.c:1043) saves the identical table. It cannot validate the rotator direction.
  F3 [firmware] a leg accepts its timeout as completion: RotNega -> Rot_save on [|angCalib.rot| > 120 deg ||
     cnt > 1000] (chart_1210), no alarm, the save proceeds. [plant] On this plant the 120 deg leg needs >= 0.223
     rad/s at 100 % to end on angle (the pre-leg travel is the staircase's and its ramp-down coast's, both from
     the GUESSed valve model); a 0.195 rad/s rotator ends on the timeout at 105 deg and still saves the right
     speed. The stored rotNega X[2], 0.271 rad/s at 80 %, is 22 % above that.
  F4 [firmware] Rot_save follows the last full-flow leg directly. The motion calibrations Chs/Bm1/Bm2/Arm/Link/Tilt
     end <Axis>Pnt2_log -> <Axis>_save (their last 1 s ramp-down finishes during Pnt2_stb), the Fork saves follow
     a _log with no motion, Trvl_save follows TrvlCcwMin (an unsmoothed _Min staircase command), step 32's
     SwingStopCoeff_save is unreachable (chart_1210). With the main.c save handshake the 1 s ramp-down of the
     100 % command is cut after 3 ticks: y.propVlvCmd.rotNega goes from 99.7 % to 0 in one tick (at full rotator
     speed on this plant).
  F5 [latent] the save writes EVERY table, mount and fork value (AppCtrlIf.c:801-1018 is unconditional), the other
     axes' ones from GenCorrTbl's / CalcImuMntOri's persistents, initialised on the first step from parLocalTest
     (MdlApp.c:45077-45127, 45404-45476; no ECU code writes parLocalTest, so: the compiled set) in the calibration
     shape: knee 0.001 -> 0.01, Y[2] -> PropVlvRefCmd (travel 100 -> 60 %, bm1 80 -> 70 %, blade deadband
     25 -> 0.01 %). Shown also on a plant whose IMU boards and other valves differ from the compiled set. After a
     power cycle, any calibration (even step 28) overwrites CalibRot's result with the compiled rotator table.
     Latent: the NVM tables reach only the u.tblReqSpdToActCmdStored inport, which this build never reads because
     EnTestPar = true (SysPar.m:5) folds Switch17 to parLocalTest. Live in an EnTestPar = false build.
  F6 [plant, latent] the identified minimum is the onset command - 0.5 (PropVlvCmdMotionOnsetDlyCmp). Whether that
     lies below the valve's real deadband is set by the valve line just above its opening -- a GUESS here
     (valves.port_speed: 0.001 rad/s at the deadband, linear above). Closed form, identified - deadband over
     deadbands 20.00..25.99 % in 0.01 steps, by the line's slope at the opening: 0.00047 rad/s/% -> +0.10..+0.30,
     never below; 0.0038 (this plant's rotPosi, 20.9 vs 21.0 here) -> -0.10..+0.10, below for 48 % (0.483; 5 % land
     exactly on the deadband, 50.5 % below on deadbands between those ties); 0.019 -> -0.33..-0.13 and
     0.096 -> -0.46..-0.27, always below. IF such a table were loaded -- in this build nothing loads it: the auto
     control interpolates parLocalTest (MdlApp.c:50328-50480, the model's tblReqSpdToActCmdStored folded to
     parLocalTest by the constant Switch17), and no ECU code writes parLocalTest -- the minimum-speed hold
     (chart_2463 l.66) commands a closed valve on this plant: PreparePick stalls with the rotator 0.8 deg off
     (no ApproachPanel in 30 s), or at the 2.95 deg error where the error->demand table reaches the deadband.
     The firmware's own knob u.parMotionOnsetCmp fixes it and is a To-do in AppCtrlIf.c:562.
     CAN: [firmware] the rotator command leaves the ECU as a uint8 at 0.4 %, truncated (CanCtrl.c:1090): the
     staircase's odd 0.2 % steps never reach the TR controller while the firmware records its own value, and the
     transmitted hold is always >= 0.4 % below the transmitted onset command. [plant] On this valve line that
     transmitted hold is below the deadband for every deadband off the 0.4 % grid.
  F7 [firmware] a lost rotator angle (PrePostProc_If.c:1049 substitutes 0.0 deg) reaches MdlApp as no fault:
     u.isJntAngRotFault has no writer, and what the ECU does detect (lostComm_CAN4TrCtrlDiagL, TrRotSensFault_DiagL,
     isTrAutoOk) goes to service diagnostics or to an inport MdlApp.c never reads. RotPosiMin has no timeout, so
     the command climbs 0.2 % per 2 s. [plant] ASSUMPTION: the TR controller keeps executing flow commands while
     its BLADE_ANG is lost, so the rotator turns (92 deg in 100 s here). The MdlApp guard (isJntAngRotFault ->
     BIT_JNT_ANG_SNSR_ERR, calib mask) works when written.
  Also: [firmware] the zero offset CalibRot measures is never what gets saved -- the save carries the LIVE
  par.jntAngRotZeroOffs (MdlApp.c:43359-43371; primary test in test_valve_plant, asserted again here).
  [plant] the real sensor's 1/128 deg resolution and 20 ms period bias the identified speed by < 0.5 % (the 3 Hz
  joint-rate filter, LPF call MdlApp.c:13336 / TiltToRot output :13361, absorbs the 2-tick hold); a start across
  +-180 deg is handled.

PLANT CHOICES THAT ARE NOT FIRMWARE (each tagged where used)
  ROT_HW valve lines, the 0.001 rad/s speed at the deadband (valves.port_speed GUESS), quasi-static rotator,
  isMachCalib = 1 during calibration (ASSUMPTION, open question in test_calibration_entry), the CAN plant
  subclass and the sensor sample-and-hold below.

Charts cited as chart_NNNN are the Stateflow XML of MdlApp.slx (script line numbers); MdlApp.c:N is
X1Exc/Asw/GeneratedCode/MdlApp_ert_rtw/MdlApp.c.

Run:  cd xpanner-sim && python3 -m unittest sil.tests.test_calib_rot_plant -v
"""
import math
import unittest
from collections import namedtuple

import numpy as np

from sil import kinematics as kin
from sil import valves as vlv
from sil.harness import Harness, SaveHandshake
from sil.plant import DT, Hardware, KinematicPlant

DEG = math.pi / 180.0
f32 = np.float32

# ---- firmware constants (SysPar.m, compiled into MdlApp.c) -----------------------------------------------
STB_TICKS = 801                 # CntCalib_stb 800 (SysPar.m:102): entry tick + 800 during ticks
LOG_TICKS = 101                 # CntCalib_log 100 (SysPar.m:103)
TIMEOUT_TICKS = 1002            # [... || cnt > CntCalib_timeout 1000] (SysPar.m:104)
STEP_TICKS = 200                # CntCalibStepFindingMin (SysPar.m:105)
STEP_SIZE = 0.2                 # StepFindingMin_size (SysPar.m:110)
INIT_OFFS = 20.0                # PropVlvCmdInitOffs.rotPosi/rotNega (SysPar.m:176-177)
RAMP_TICKS = 100                # CntCalib_ramp (SysPar.m:106), SmoothPropVlvCmd chart_3055 l.205-243
ONSET_DLY_CMP = 0.5             # PropVlvCmdMotionOnsetDlyCmp (SysPar.m:134)
ANG_ONSET = float(f32(math.pi / 180 * 0.5))    # AngMotionOnsetThld (SysPar.m:131)
ANG_ROT_POSI, ANG_ROT_NEGA = 60.0, 120.0        # deg, AngRotPosi / AngRotNega (SysPar.m:126-127)
REF_CMD = 100.0                 # PropVlvRefCmd.rotPosi/rotNega (SysPar.m:153-154)
IDENTIFIED_X1 = float(f32(0.01))                # chart_2338 l.108-111
PROP_VLV_REF_CMD = dict(trvlLeFwd=60.0, trvlLeRev=60.0, trvlRiFwd=60.0, trvlRiRev=60.0, swingLe=60.0,
                        swingRi=60.0, bm1Up=70.0, bm1Down=70.0, bm2Up=60.0, bm2Down=60.0, armIn=70.0,
                        armOut=70.0, linkIn=70.0, linkOut=70.0, tiltPosi=70.0, tiltNega=70.0,
                        rotPosi=100.0, rotNega=100.0)    # SysPar.m:136-156
# Tbl_ErrToDmdRot (SysPar.m:296-297 = MdlApp.c:1267-1283), rad -> rad/s, positive half
ERR_TO_DMD_ROT_X = [0.0, 0.5, 1.0, 2.0, 6.0, 15.0, 16.0]
ERR_TO_DMD_ROT_Y = [0.0, 0.0, 0.001, 0.25, 1.7, 14.5, 15.0]
# OEM platform CAN encodings (not MdlApp)
CAN_ROT_CMD_SCALE = 2.5         # (uint8)(cmd * 2.5f), "factor 0.4", CanCtrl.c:1090 (tilt :1100); CAN4_X1Exc.dbc:189
CAN_QUANTUM = 0.4               # percent per uint8 count
BLADE_ANG_QUANTUM = 0.0078125 * DEG             # 1/128 deg, CanCtrl.c:3501, CAN4_X1Exc.dbc:259 (then *DEG_TO_RAD)

ROT_PORTS = ("rotPosi", "rotNega")
OTHER_PORTS = tuple(p for p in vlv.PORTS if p not in ROT_PORTS)

# ---- the plant's rotator valve ---------------------------------------------------------------------------
# ASSUMPTION (test design, not a machine value): the plant's rotator valve lines. Deadbands 21.0 / 22.3 %
# differ from the stored 17 % and from each other; Y[2] above 100 % puts PropVlvRefCmd = 100 % on the
# valve's LINEAR segment (valves.port_speed clamps at Y[2]), so the identified speed is a point of the line,
# not a saturation clamp, and differs from the stored X[2] (0.257 / 0.271 rad/s).
ROT_HW = {"par.reqSpdToActCmd.rotPosi_X": [0.0, 0.001, 0.40], "par.reqSpdToActCmd.rotPosi_Y": [0.0, 21.0, 125.0],
          "par.reqSpdToActCmd.rotNega_X": [0.0, 0.001, 0.45], "par.reqSpdToActCmd.rotNega_Y": [0.0, 22.3, 130.0]}
Q0_ROT_DEG = 10.0

# Deadband sweeps (closed form, predict_onset). SWEEP_DBS is the 0.01 % grid the F6 statistic is quoted on; on it
# 5 % of deadbands sit exactly 0.5 below a staircase value and the identified minimum TIES the deadband.
# TIE_FREE_DBS lies between those grid points (and off the 0.2 % staircase and 0.4 % CAN grids), where a tie with
# the valve model's strict `cmd < deadband` test is impossible.
SWEEP_DBS = np.arange(20.0, 26.0, 0.01)
TIE_FREE_DBS = 20.005 + 0.01 * np.arange(600)
TIE_TOL = 1e-4                  # % ; >> float32 rounding near 20 % (1e-6), << the 0.01 % grid


def rot_hw(db_posi=21.0, db_nega=22.3, vmax_nega=0.45):
    hw = dict(ROT_HW)
    hw["par.reqSpdToActCmd.rotPosi_Y"] = [0.0, db_posi, 125.0]
    hw["par.reqSpdToActCmd.rotNega_Y"] = [0.0, db_nega, 130.0]
    hw["par.reqSpdToActCmd.rotNega_X"] = [0.0, 0.001, vmax_nega]
    return hw


def staircase(k):
    """The k-th _Min command, as the generated code computes it (chart_3055 l.76, 95-96)."""
    return float(f32(INIT_OFFS) + f32(k) * f32(STEP_SIZE))


def can_received(cmd):
    """What the tilt-rotator controller receives for a percent command: uint8 truncation of cmd * 2.5f,
    decoded at 0.4 % (CanCtrl.c:1090, CAN4_X1Exc.dbc:189). ASSUMPTION: the controller applies the decoded
    value as is."""
    return int(f32(cmd) * f32(CAN_ROT_CMD_SCALE)) * CAN_QUANTUM


def predict_onset(X, Y, can=False):
    """Closed-form _Min result for a plant valve line (X, Y) (valves.port_speed): walk the staircase, integrate
    the travel at each step's constant speed, stop where it exceeds AngMotionOnsetThld. Returns (the command
    the firmware records at the onset, step index k, ticks into step k). Ignores the valve lag (tau 0.1 s,
    a few ticks) and the one-tick detection skew, so callers keep a margin from step boundaries.
    can=True applies the CAN4 truncation to the command the valve sees (the firmware still records its own)."""
    travel = 0.0
    for k in range(400):
        cmd = staircase(k)
        v = vlv.port_speed(can_received(cmd) if can else cmd, X, Y)
        if v > 0.0:
            need = (ANG_ONSET - travel) / (v * DT)
            if need <= STEP_TICKS:
                return cmd, k, need
            travel += v * DT * STEP_TICKS
    return None


def identified_min(onset_cmd):
    return float(f32(onset_cmd) - f32(ONSET_DLY_CMP))


def sweep_min_minus_deadband(dbs, vmax, top=125.0, knee=0.001, can=False):
    """identified minimum (as transmitted when can=True) minus deadband, for each deadband, on the valve line
    X = [0, knee, vmax], Y = [0, deadband, top]."""
    out = []
    for db in dbs:
        onset = predict_onset([0.0, knee, vmax], [0.0, float(db), top], can=can)[0]
        m = identified_min(onset)
        out.append((can_received(m) if can else m) - float(db))
    return np.array(out)


def smooth_step(t):
    """SmoothPropVlvCmd's cubic, t ticks into a 100-tick ramp (chart_3055 l.229-238)."""
    r = min(t, RAMP_TICKS) / RAMP_TICKS
    return r * r * (3.0 - 2.0 * r)


# ---- recording and running --------------------------------------------------------------------------------
Row = namedtuple("Row", "tick calib step calibrating posi nega other rot qdot turned fw_qdot raw zero save_req")


class Recorder:
    """One Row per tick, taken after the plant (and any sensor plants) published and before MdlApp_step:
    y.* are the previous step's outputs, plant fields the state that command produced. turned is the plant's
    unwrapped rotator travel since boot."""

    def __init__(self, plant):
        self.plant, self.rows, self.turned = plant, [], 0.0

    def __call__(self, h):
        fw, p = h.fw, self.plant
        self.turned += p.qdot["rotator"] * DT
        self.rows.append(Row(h.tick_count, h.calib_step(), h.curr_step(), bool(fw["y.isCalibrating"]),
                             fw["y.propVlvCmd.rotPosi"], fw["y.propVlvCmd.rotNega"],
                             max(abs(fw[f"y.propVlvCmd.{q}"]) for q in OTHER_PORTS),
                             p.q["rotator"], p.qdot["rotator"], self.turned, fw["y.jnts.TiltToRot.qDot"],
                             fw["u.jntAngRaw_Rot"], fw["y.jntAngRotZeroOffs"], bool(fw["y.isCalibDataSaveReq"])))


Run = namedtuple("Run", "h plant emu rows blocks end")


def blocks_of(rows):
    """[(calibStep, first row index, row count)] for consecutive rows."""
    out = []
    for i, r in enumerate(rows):
        if out and out[-1][0] == r.calib:
            out[-1][2] += 1
        else:
            out.append([r.calib, i, 1])
    return [tuple(b) for b in out]


def block(run, name, nth=0):
    found = [b for b in run.blocks if b[0] == name]
    return found[nth]


def boot(plant, extra=(), setup=None, sensors=(), mach_calib=1):
    """Healthy machine, plant publishing every sensor (house at swing 0: the swing switch is closed, the latch
    set), sensor plants after it, then extra. NoTarget. setup(fw) patches par.* after the reset and BEFORE the
    first step, so first-step persistents (GenCorrTbl, CalcImuMntOri) initialise from the patched values.
    ASSUMPTION: u.isMachCalib = 1 as the tablet's service mode would hold it while calibrating (open question,
    test_calibration_entry); on a healthy machine it changes nothing, it only arms the calibration inhibit mask
    (MdlApp.c:39679)."""
    h = Harness(plant=[plant, *sensors, *extra]).reset()
    if setup is not None:
        setup(h.fw)
    h.nominal_inputs()
    h.gnss_rtk_fixed()
    h.fw["u.isMachCalib"] = mach_calib
    h.tick(3)
    return h


def run_calib_rot(plant, sensors=(), setup=None, after=None):
    """CalibRot to NoTarget with SaveHandshake (main.c:394-419) and strict joint stops. after(h) runs in the
    same power cycle once the valve has settled."""
    plant.strict_limits = True
    rec, emu = Recorder(plant), SaveHandshake()
    h = boot(plant, extra=[rec, emu], setup=setup, sensors=sensors)
    h.jump_to_step("CalibRot")
    assert h.fw["y.isCalibrating"], h.describe()
    h.run_until(lambda h: h.curr_step() == "NoTarget", 400.0, "CalibRot done")
    end = len(rec.rows)
    h.tick(150)
    if after is not None:
        after(h)
    return Run(h, plant, emu, rec.rows, blocks_of(rec.rows[:end + 5]), end)


_CACHE = {}


def main_run():
    """The reference scenario, once per process: CalibRot on ROT_HW from rotator 10 deg, then step 28 in the
    SAME power cycle (for the NVM persistence test)."""
    if "main" not in _CACHE:
        def step28(h):
            h.jump_to_step("CalibForkRefPose")
            h.run_until(lambda h: h.curr_step() == "NoTarget", 30.0, "step 28 done")
        plant = KinematicPlant(q0=dict(rotator=Q0_ROT_DEG), degrees=True, hardware=ROT_HW)
        _CACHE["main"] = run_calib_rot(plant, after=step28)
    return _CACHE["main"]


def compiled(fw):
    """The compiled parameter set (what Firmware.reset() restores), independent of any live par.* patch another
    test in this process left behind."""
    return Hardware.compiled(fw)


def table(snap, port):
    return snap[f"y.tblReqSpdToActCmd.{port}_X"], snap[f"y.tblReqSpdToActCmd.{port}_Y"]


def true_speed_at_ref(plant, port, can=False):
    X, Y = plant.tables[port]
    return vlv.port_speed(can_received(REF_CMD) if can else REF_CMD, X, Y)


EXPECTED_ORDER = ["CalibStandby", "RotPosiPntRef_stb", "RotPosiPntRef_log", "RotPosiMin", "RotPosiMin_stb", "RotPosi",
                  "RotNegaPntRef_stb", "RotNegaPntRef_log", "RotNegaMin", "RotNegaMin_stb", "RotNega", "Rot_save",
                  "CalibStandby"]


# ===========================================================================================================
class TestCalibRotEndToEnd(unittest.TestCase):
    """The reference run (main_run): plant rotator valve ROT_HW, rotator starting at 10 deg.
    Checks are split into subtests by what they rest on: the firmware against a closed-form prediction from the
    plant ("prediction"), plant ground truth ("plant truth"), and numbers documented for ROT_HW ("documented"),
    so a changed plant fails the documented pins without hiding whether the firmware still matches the prediction."""

    @classmethod
    def setUpClass(cls):
        cls.calib = main_run()

    def test_completes_in_the_chart_order_with_the_firmware_dwell_times(self):
        # FW: chart_1210 transitions (RotPosiPntRef_stb [cnt >= CntCalib_stb] ... RotNega [|angCalib.rot| >
        # AngRotNega || cnt > CntCalib_timeout] -> Rot_save); Rot_save -> Standby on the save ack
        # (hasChanged(isCalibDataSaved) && isCalibDataSaved); main chart back to NoTarget.
        run = self.calib
        self.assertEqual([b[0] for b in run.blocks], EXPECTED_ORDER)
        d = {b[0]: b[2] for b in run.blocks[1:-1]}
        for s in ("RotPosiPntRef_stb", "RotPosiMin_stb", "RotNegaPntRef_stb", "RotNegaMin_stb"):
            self.assertEqual(d[s], STB_TICKS, s)
        for s in ("RotPosiPntRef_log", "RotNegaPntRef_log"):
            self.assertEqual(d[s], LOG_TICKS, s)
        # scenario precondition (plant speed), not a firmware property: both legs end on angle, not on the timeout
        self.assertLess(d["RotPosi"], TIMEOUT_TICKS - 100, "the 60 deg leg ended on angle")
        self.assertLess(d["RotNega"], TIMEOUT_TICKS - 100, "the 120 deg leg ended on angle")
        self.assertEqual(d["Rot_save"], 2, "SaveHandshake acks on the next tick")
        # ends like a success, once
        first_idle = run.rows[run.end]                     # the row after the step that reported NoTarget
        self.assertEqual((first_idle.step, first_idle.calibrating), ("NoTarget", False))
        self.assertEqual(len(run.emu.saved), 2, "CalibRot + the step 28 that follows in main_run")
        self.assertEqual(run.h.inhibit_names(), ["BIT_NO_TARGET"])

    def test_run_time_budget_on_this_plant(self):
        # PLANT-DEPENDENT documentation of the module docstring's budget: the staircase and leg durations follow
        # ROT_HW, TAU_VALVE and the quasi-static rotator. Kept apart from the chart-order test so a valve-model
        # change (sil/valves.py) fails here and not there.
        d = {b[0]: b[2] for b in self.calib.blocks[1:-1]}
        fixed = 4 * STB_TICKS + 2 * LOG_TICKS
        self.assertEqual(fixed, 3406)
        total = sum(d.values())
        with self.subTest("every tick is in a sub-step the budget names"):
            self.assertEqual(total, fixed + d["RotPosiMin"] + d["RotNegaMin"] + d["RotPosi"] + d["RotNega"] + d["Rot_save"])
        with self.subTest("documented"):
            self.assertTrue(1500 < d["RotPosiMin"] < 1560 and 2830 < d["RotNegaMin"] < 2880, d)   # 15.3 s / 28.5 s
            self.assertTrue(400 < d["RotPosi"] < 420 and 700 < d["RotNega"] < 725, d)           # 4.1 s / 7.1 s
            self.assertTrue(8870 < total < 8970, total)                                          # 89 s simulated

    def test_staircase_is_raw_on_one_port_and_the_legs_ramp_over_one_second(self):
        # FW: chart_3055 l.53-67 (cnt/step reset on a calibStep change, step + 1 every 200 ticks), l.76-96
        # (20 + step * 0.2), l.154-161 (exactly one rot port per sub-step), l.196-199 (_Min bypasses the smoothing,
        # every other sub-step goes through the 1 s cubic SmoothPropVlvCmd).
        run, rows = self.calib, self.calib.rows
        for name, port in (("RotPosiMin", "posi"), ("RotNegaMin", "nega")):
            _, i0, n = block(run, name)
            for j in range(n):
                self.assertEqual(getattr(rows[i0 + j], port), staircase((j + 1) // STEP_TICKS), (name, j))
        for name, port in (("RotPosi", "posi"), ("RotNega", "nega")):
            _, i0, n = block(run, name)
            for j in range(min(n, RAMP_TICKS + 20)):
                self.assertAlmostEqual(getattr(rows[i0 + j], port), REF_CMD * smooth_step(j + 1), delta=2e-3)
        for r in rows[:run.end]:
            self.assertEqual(r.other, 0.0, r)
            self.assertFalse(r.posi > 0.0 and r.nega > 0.0, r)
        # the 60 deg leg's ramp-down runs in full inside RotNegaPntRef_stb (100 ticks, cubic from 100 %)
        _, i0, _ = block(run, "RotNegaPntRef_stb")
        self.assertEqual(sum(1 for r in rows[i0:i0 + STB_TICKS] if r.posi > 0.0), RAMP_TICKS - 1)

    def test_identified_minimum_is_the_plant_deadband_the_staircase_finds_not_the_stored_17(self):
        # WHY: proves identification rather than echo. The stored rotator deadband is 17 % on both ports
        # (ECR88D_ShortArm.m:335-337); the plant's are 21.0 / 22.3 %. The firmware stores the staircase command
        # at the onset minus 0.5 (chart_2338 l.65-66); predict_onset derives that command from the plant's
        # valve line alone.
        # FW: onset = |angCalib.rot - ref| > 0.5 deg, a rising edge (chart_2316 l.80-81, 91-104); angCalib.rot =
        # wrap(raw - angRefRot) (chart_2291 l.171-172, MdlApp.c:43544).
        run, snap = self.calib, self.calib.emu.saved[0][1]
        plant = run.plant
        stored_min = compiled(run.h.fw)["par.reqSpdToActCmd.rotPosi_Y"][1]
        self.assertEqual(stored_min, 17.0)
        for port, name, db, want in (("rotPosi", "RotPosiMin", 21.0, 20.9), ("rotNega", "RotNegaMin", 22.3, 22.3)):
            X, Y = plant.tables[port]
            got = table(snap, port)[1][1]
            onset, k, need = predict_onset(X, Y)
            _, i0, n = block(run, name)
            with self.subTest(port=port, check="prediction"):
                self.assertTrue(10 < need < STEP_TICKS - 10, f"prediction {need:.0f} ticks into step {k}: too close to a step edge")
                self.assertAlmostEqual(got, identified_min(onset), places=5)
                # the onset the firmware saw is the command of the last _Min tick; the Min dwell is the prediction
                self.assertEqual(getattr(run.rows[i0 + n - 1], "posi" if port == "rotPosi" else "nega"), onset)
                self.assertLessEqual(abs(n - (k * STEP_TICKS + need)), 15, (n, k, need))
            with self.subTest(port=port, check="plant truth"):
                # 0.5 deg turned since the staircase started when the firmware detected it, not yet 12 ticks before
                start = run.rows[i0].turned
                self.assertGreater(abs(run.rows[i0 + n - 1].turned - start), ANG_ONSET)
                self.assertLess(abs(run.rows[i0 + n - 12].turned - start), ANG_ONSET)
            with self.subTest(port=port, check="not the stored table"):
                self.assertNotEqual(Y[1], stored_min, "precondition: the plant valve is not the stored one")
                self.assertNotAlmostEqual(got, stored_min - ONSET_DLY_CMP, places=3)
            with self.subTest(port=port, check="documented"):
                self.assertEqual(Y[1], db)
                self.assertAlmostEqual(got, want, places=5)

    def test_identified_speed_is_the_plant_speed_at_the_reference_command(self):
        # FW: peak |y.jnts.TiltToRot.qDot| over RotPosi / RotNega (chart_2383 l.69-70, 87-88; input MdlApp.c:45063),
        # the qDot being a 10-sample regression on the raw angle (chart_1179) through the 3 Hz joint-rate filter
        # (LPF call MdlApp.c:13336, TiltToRot output :13361); stored as X[2] = max(MinTblReqSpd, peak)
        # (chart_2338 l.46-47, 108-111).
        run, snap = self.calib, self.calib.emu.saved[0][1]
        cp = compiled(run.h.fw)
        stored = {p: cp[f"par.reqSpdToActCmd.{p}_X"][2] for p in ROT_PORTS}
        for port in ROT_PORTS:
            X, Y = table(snap, port)
            truth = true_speed_at_ref(run.plant, port)
            with self.subTest(port=port, check="plant truth"):
                self.assertEqual((X[0], X[1], Y[0], Y[2]), (0.0, IDENTIFIED_X1, 0.0, REF_CMD))
                self.assertAlmostEqual(X[2], truth, delta=1e-4 * truth)
            with self.subTest(port=port, check="not the stored speed"):
                self.assertGreater(abs(X[2] - stored[port]), 0.03)
            with self.subTest(port=port, check="documented"):
                self.assertLess(abs(X[2] / truth - 1.0), 2e-6)            # 7e-7 (posi) / 3e-7 (nega)
        with self.subTest(check="documented"):
            self.assertAlmostEqual(true_speed_at_ref(run.plant, "rotPosi"), 0.30409, places=5)
            self.assertAlmostEqual(true_speed_at_ref(run.plant, "rotNega"), 0.32493, places=5)
        with self.subTest(check="firmware rate = plant rate at the end of the leg"):
            _, i0, n = block(run, "RotNega")
            r = run.rows[i0 + n - 1]
            self.assertAlmostEqual(r.fw_qdot, r.qdot, delta=1e-5)

    def test_legs_end_on_angle_from_their_own_reference_and_sweep_more_than_120_deg(self):
        # FW: posi reference = mean raw over RotPosiPntRef_log; nega reference RE-LATCHED over RotNegaPntRef_log
        # (UpdateAngleAvg with n restarting at 0, chart_2291 l.58-60, 80-81, 419-427). The leg angle therefore
        # includes the _Min travel and its ramp-down coast. Detection lags the plant by <= 3 ticks (angCalib is
        # computed after CalibStepMgr in the step, MdlApp.c:42087 vs :43544).
        run, rows = self.calib, self.calib.rows
        spd = {"posi": true_speed_at_ref(run.plant, "rotPosi"), "nega": true_speed_at_ref(run.plant, "rotNega")}
        for leg, ref_state, end_state, thld, key in (("RotPosi", "RotPosiPntRef_log", "RotNegaPntRef_stb", ANG_ROT_POSI, "posi"),
                                                     ("RotNega", "RotNegaPntRef_log", "Rot_save", ANG_ROT_NEGA, "nega")):
            with self.subTest(leg=leg, check="prediction"):
                _, i_ref, n_ref = block(run, ref_state)
                refs = {rows[i].turned for i in range(i_ref, i_ref + n_ref)}
                self.assertEqual(len(refs), 1, "the rotator is still during the log")
                ref = refs.pop()
                _, i_end, _ = block(run, end_state)
                swept = abs(rows[i_end].turned - ref) / DEG
                self.assertGreater(swept, thld)
                self.assertLess(swept, thld + 4 * spd[key] * DT / DEG, f"{swept:.3f} deg")
        start = rows[0].turned
        excursion = [(r.turned - start) / DEG for r in rows[:run.end]]
        with self.subTest(check="plant truth"):
            self.assertGreater(max(excursion) - min(excursion), 120.0)
        with self.subTest(check="documented"):
            # the nega reference sits where the posi coast ended: 69.0 deg above the start, the run ends 52 deg below
            _, i_ref, _ = block(run, "RotNegaPntRef_log")
            self.assertAlmostEqual((rows[i_ref].turned - start) / DEG, 69.0, delta=0.5)
            self.assertAlmostEqual(min(excursion), -52.0, delta=1.0)
        with self.subTest(check="zero offset, in passing"):
            # test_valve_plant finding: -angRefRot is output only right after the posi log (MdlApp.c:43359-43361),
            # the saved offset is par.jntAngRotZeroOffs (:43371) although the reference pose read 10 deg
            _, i_log, n_log = block(run, "RotPosiPntRef_log")
            self.assertAlmostEqual(rows[i_log + n_log].zero, -Q0_ROT_DEG * DEG, delta=1e-5)
            self.assertEqual(run.emu.saved[0][1]["y.jntAngRotZeroOffs"], 0.0)

    def test_the_last_leg_is_saved_at_full_flow_and_the_valve_is_cut_in_one_tick(self):
        # WHY/FINDING F4 [firmware]: in chart_1210 the motion calibrations Chs/Bm1/Bm2/Arm/Link/Tilt end
        # <Axis>Pnt2_log -> <Axis>_save, so their last _ToPnt ramp-down (1 s) finishes inside Pnt2_stb; the Fork
        # saves follow a _log with no motion; Trvl_save follows TrvlCcwMin (an unsmoothed _Min staircase command, not
        # a full-flow leg). CalibRot alone goes from a full-flow leg straight to its save: RotNega -> Rot_save. With
        # the main.c save handshake (ack on the next tick) the run leaves isCalibrating 3 ticks later, the
        # arbitration stops routing the calibration command (MdlApp.c:51711-51735), and rotNega drops from 99.7 %
        # to 0 in one tick. Without the handshake, Rot_save would wait out its 1 s fallback and the ramp would
        # complete. The rotator speed at that tick is the plant's (plant truth subtest).
        run, rows = self.calib, self.calib.rows
        _, i_save, _ = block(run, "Rot_save")
        last_cal = max(i for i in range(i_save, run.end + 5) if rows[i].calibrating)
        with self.subTest(check="firmware command"):
            self.assertGreater(rows[i_save].nega, 99.9)
            self.assertEqual(last_cal, i_save + 2)
            self.assertGreater(rows[last_cal].nega, 99.5)
            self.assertEqual(rows[last_cal + 1].nega, 0.0)
            self.assertFalse(rows[last_cal + 1].calibrating)
        with self.subTest(check="plant truth"):
            self.assertLess(rows[last_cal].qdot, -0.99 * true_speed_at_ref(run.plant, "rotNega"),
                            "full rotator speed at the moment the command is cut")


# ===========================================================================================================
class TestCalibRotVariants(unittest.TestCase):
    """The same step on other machines. Each is one full CalibRot (~2.3 s wall)."""

    def test_off_grid_deadbands_follow_the_same_prediction(self):
        # 21.1 / 22.55 % sit between staircase values: identified 21.1 (= deadband) and 22.5 (-0.05).
        plant = KinematicPlant(q0=dict(rotator=Q0_ROT_DEG), degrees=True, hardware=rot_hw(21.1, 22.55))
        run = run_calib_rot(plant)
        snap = run.emu.saved[0][1]
        for port, want in (("rotPosi", 21.1), ("rotNega", 22.5)):
            X, Y = plant.tables[port]
            onset, k, need = predict_onset(X, Y)
            with self.subTest(port=port, check="prediction"):
                self.assertTrue(10 < need < STEP_TICKS - 10, (port, k, need))
                self.assertAlmostEqual(table(snap, port)[1][1], identified_min(onset), places=5)
            with self.subTest(port=port, check="plant truth"):
                self.assertAlmostEqual(table(snap, port)[0][2], true_speed_at_ref(plant, port), delta=1e-4)
            with self.subTest(port=port, check="documented"):
                self.assertAlmostEqual(table(snap, port)[1][1], want, places=5)

    def test_identification_ignores_the_live_stored_table_and_the_save_carries_the_live_zero_offset(self):
        # WHY: the other "not an echo" checks compare with the COMPILED set, which is also the live par.* there.
        # Here the live par.* differ from both the compiled set and the plant, patched before the first step so
        # every first-step persistent (GenCorrTbl's propVlvCmdMin_Rot*/actSpdRef_Rot*, MdlApp.c:45118-45121,
        # 45460-45464) starts from them.
        #   identification: the saved rotator table is the plant's, bit-identical to the reference run -- not the live
        #   table (deadband 30 / 28 %) and not the compiled one (17 %).
        #   zero offset [firmware]: y.jntAngRotZeroOffs = -angRefRot only while calibStep_prev == RotPosiPntRef_log
        #   (MdlApp.c:43359-43361), else parLocalTest.jntAngRotZeroOffs (:43371), which AppCtrlIf.c:882 copies on
        #   every calibrating tick -- so the save carries the LIVE par (0.3 rad), not the measurement (-10 deg) and
        #   not the compiled value (0.0). (u.jntAngRotZeroOffsStored, AppCtrlIf.c:244, is folded away by
        #   Switch16 because EnTestPar = true, SysPar.m:5; with false the measurement would latch through INTP.)
        live = {"par.reqSpdToActCmd.rotPosi_X": [0.0, 0.001, 0.12], "par.reqSpdToActCmd.rotPosi_Y": [0.0, 30.0, 90.0],
                "par.reqSpdToActCmd.rotNega_X": [0.0, 0.001, 0.15], "par.reqSpdToActCmd.rotNega_Y": [0.0, 28.0, 95.0],
                "par.jntAngRotZeroOffs": 0.3}

        def setup(fw):
            for path, value in live.items():
                fw[path] = value

        plant = KinematicPlant(q0=dict(rotator=Q0_ROT_DEG), degrees=True, hardware=ROT_HW)
        run = run_calib_rot(plant, setup=setup)
        snap, ref = run.emu.saved[0][1], main_run().emu.saved[0][1]
        cp = compiled(run.h.fw)
        with self.subTest(check="scenario"):
            self.assertEqual([b[0] for b in run.blocks], EXPECTED_ORDER)
            self.assertEqual(run.h.inhibit_names(), ["BIT_NO_TARGET"])
            self.assertEqual(run.h.fw["par.reqSpdToActCmd.rotPosi_Y"][1], 30.0, "the live patch is in place")
        for port in ROT_PORTS:
            with self.subTest(port=port, check="identified from the plant, not the live table"):
                X, Y = table(snap, port)
                self.assertEqual(list(Y), list(table(ref, port)[1]))
                self.assertAlmostEqual(X[2], table(ref, port)[0][2], delta=1e-6 * X[2])
                self.assertAlmostEqual(X[2], true_speed_at_ref(plant, port), delta=1e-4 * X[2])
                self.assertNotAlmostEqual(Y[1], live[f"par.reqSpdToActCmd.{port}_Y"][1] - ONSET_DLY_CMP, places=3)
                self.assertNotAlmostEqual(X[2], live[f"par.reqSpdToActCmd.{port}_X"][2], places=3)
        with self.subTest(check="the saved zero offset is the live par"):
            _, i_log, n_log = block(run, "RotPosiPntRef_log")
            self.assertAlmostEqual(run.rows[i_log + n_log].zero, -Q0_ROT_DEG * DEG, delta=1e-5, msg="the measurement")
            self.assertEqual(snap["y.jntAngRotZeroOffs"], float(f32(live["par.jntAngRotZeroOffs"])))
            self.assertNotEqual(snap["y.jntAngRotZeroOffs"], cp["par.jntAngRotZeroOffs"])

    def test_real_sensor_resolution_and_can_period_bias_the_speed_by_less_than_half_a_percent(self):
        # WHY: the rotator angle is the only absolute joint sensor and the speed peak is a max over a regression
        # of it -- a max of a jittery estimate is biased up. Real sensor: BLADE_ANG 1/128 deg resolution
        # (CanCtrl.c:3501, CAN4_X1Exc.dbc:259), 20 ms cycle (CAN4_X1Exc.dbc:592) into a 10 ms task. A 2-tick hold
        # makes the raw regression alternate 0.970 / 1.030 x the true rate; the 3 Hz joint-rate filter
        # (LPF call MdlApp.c:13336, output :13361) absorbs most of it. Measured bias +0.36 % (posi) / +0.28 % (nega);
        # the minimum is unchanged.
        # ASSUMPTION: the TR controller samples synchronously with the ECU (a 2-tick hold, never 1 or 3) and rounds.
        class Hold20ms:
            def __init__(self):
                self.held = None

            def __call__(self, h):
                if h.tick_count % 2 == 0 or self.held is None:
                    self.held = h.fw["u.jntAngRaw_Rot"]
                else:
                    h.fw["u.jntAngRaw_Rot"] = self.held

        plant = KinematicPlant(q0=dict(rotator=Q0_ROT_DEG), degrees=True, hardware=ROT_HW, rot_quantum=BLADE_ANG_QUANTUM)
        run = run_calib_rot(plant, sensors=[Hold20ms()])
        snap, ref = run.emu.saved[0][1], main_run().emu.saved[0][1]
        for port in ROT_PORTS:
            with self.subTest(port=port):
                self.assertEqual(table(snap, port)[1][1], table(ref, port)[1][1])
                truth = true_speed_at_ref(plant, port)
                bias = table(snap, port)[0][2] / truth - 1.0
                self.assertGreater(bias, 0.001)
                self.assertLess(bias, 0.005)

    def test_a_backwards_rotator_saves_the_identical_table(self):
        # FINDING F2 [firmware]: CalibRot uses |angCalib.rot| for both the onset (chart_2316 l.98) and the leg guards
        # (chart_1210), and |qDot| for the speed (chart_2383 l.39). A rotator plumbed backwards -- or, what the
        # firmware sees identically, a wrong INTP.trRotDir sensor sign (PrePostProc_If.c:1043, open in spec
        # B2.8) -- turns the other way on rotPosi and saves the same table, no inhibit. The positioning loop is what
        # breaks (test_valve_plant.test_a_backwards_axis_breaks_its_loop); calibration cannot catch it.
        plant = KinematicPlant(q0=dict(rotator=Q0_ROT_DEG), degrees=True, hardware=ROT_HW, axis_sign={"rotator": -1})
        run = run_calib_rot(plant)
        snap, ref = run.emu.saved[0][1], main_run().emu.saved[0][1]
        for port in ROT_PORTS:
            X, Y = table(snap, port)
            Xr, Yr = table(ref, port)
            self.assertEqual(list(Y), list(Yr), port)
            self.assertAlmostEqual(X[2], Xr[2], delta=1e-5 * Xr[2])
        _, i0, n = block(run, "RotPosi")
        r = run.rows[i0 + n - 1]
        self.assertGreater(r.posi, 99.0)
        self.assertLess(r.fw_qdot, -0.95 * true_speed_at_ref(plant, "rotPosi"),
                        "rotPosi open, the firmware reads the rotator turning negative")
        self.assertEqual([b[0] for b in run.blocks], EXPECTED_ORDER)
        self.assertEqual(run.h.inhibit_names(), ["BIT_NO_TARGET"])

    def test_a_start_near_180_deg_crosses_the_wrap_and_identifies_the_same_table(self):
        # FW: angCalib.rot = atan2(sin, cos) of the difference (chart_2291 l.172), the reference mean wraps
        # (UpdateAngleAvg l.424-426), the rate regression unwraps (chart_1179). Plant raw = wrap(rot - offset).
        plant = KinematicPlant(q0=dict(rotator=175.0), degrees=True, hardware=ROT_HW)
        run = run_calib_rot(plant)
        wrapped = [r.rot for r in run.rows[:run.end]]
        self.assertGreater(max(wrapped), 179.0 * DEG)
        self.assertLess(min(wrapped), -170.0 * DEG)
        snap, ref = run.emu.saved[0][1], main_run().emu.saved[0][1]
        for port in ROT_PORTS:
            self.assertEqual(table(snap, port)[1][1], table(ref, port)[1][1], port)
            self.assertAlmostEqual(table(snap, port)[0][2], table(ref, port)[0][2], delta=1e-4)
        self.assertEqual([b[0] for b in run.blocks], EXPECTED_ORDER)

    def test_a_slow_rotator_times_out_the_120_deg_leg_and_saves_without_an_alarm(self):
        # FINDING F3. [firmware] RotNega ends on [|angCalib.rot| > 120 deg || cnt > 1000] (chart_1210, SysPar.m:104,
        # 127): a timeout is a completion -- save, NoTarget, no inhibit, no alarm output.
        # [plant] At 0.195 rad/s at 100 % the leg covers only 105 deg in 10 s, and the saved speed is still right.
        # From this run the effective leg time is (swept - pre-leg travel) / speed; the speed needed to end on angle
        # is (120 deg - pre-leg travel) / that time = 0.223 rad/s at 100 %. The pre-leg travel (staircase + ramp
        # coast) comes from the GUESSed valve model, so 0.223 is this plant's number. The stored rotNega X[2] is
        # 0.271 rad/s at 80 % (ECR88D_ShortArm.m:336-337): 22 % above it.
        plant = KinematicPlant(q0=dict(rotator=Q0_ROT_DEG), degrees=True, hardware=rot_hw(vmax_nega=0.27))
        run = run_calib_rot(plant)
        rows, snap = run.rows, run.emu.saved[0][1]
        v = true_speed_at_ref(plant, "rotNega")
        _, i0, n = block(run, "RotNega")
        _, i_ref, _ = block(run, "RotNegaPntRef_log")
        ref = rows[i_ref].turned
        pre = abs(rows[i0].turned - ref)
        swept = abs(rows[i0 + n].turned - ref)
        with self.subTest(check="firmware: the timeout ends the leg and the save proceeds"):
            self.assertEqual(n, TIMEOUT_TICKS)
            self.assertTrue(swept < ANG_ROT_NEGA * DEG, math.degrees(swept))
            self.assertEqual([b[0] for b in run.blocks], EXPECTED_ORDER)
            self.assertEqual(len(run.emu.saved), 1)
            self.assertEqual(run.h.inhibit_names(), ["BIT_NO_TARGET"])
        with self.subTest(check="plant truth: the saved speed is still the plant's"):
            self.assertAlmostEqual(table(snap, "rotNega")[0][2], v, delta=1e-4)
        with self.subTest(check="documented (plant-dependent threshold)"):
            stored = compiled(run.h.fw)["par.reqSpdToActCmd.rotNega_X"][2]
            v_needed = (ANG_ROT_NEGA * DEG - pre) / ((swept - pre) / v)
            self.assertAlmostEqual(v, 0.19507, places=5)
            self.assertAlmostEqual(math.degrees(swept), 105.0, delta=1.0)
            self.assertTrue(0.21 < v_needed < 0.23, v_needed)
            self.assertAlmostEqual(stored / v_needed - 1.0, 0.22, delta=0.01)


# ===========================================================================================================
class TestWhatTheSavedTableDoes(unittest.TestCase):

    @staticmethod
    def plant_differs_hardware(fw):
        """ROT_HW plus a unit whose five IMU boards sit 5 deg off the compiled mounts (about the link's pin or
        vertical axis, so no inhibit trips) and whose other valves differ from the compiled tables (vmax x 0.8,
        deadband + 3 %). None of it is identifiable by CalibRot; the save must still carry the compiled values."""
        cp = compiled(fw)
        M, a = cp.mounts(), 5.0 * DEG
        hw = dict(ROT_HW)
        hw.update(imuChs=M["imuChs"] @ kin.Rz(a), imuBm1=M["imuBm1"] @ kin.Ry(a), imuArm=M["imuArm"] @ kin.Ry(a),
                  imuLink=M["imuLink"] @ kin.Ry(a), imuTilt=M["imuTilt"] @ kin.Rx(a))
        for port in OTHER_PORTS:
            X, Y = cp[f"par.reqSpdToActCmd.{port}_X"], cp[f"par.reqSpdToActCmd.{port}_Y"]
            hw[f"par.reqSpdToActCmd.{port}_X"] = [X[0], X[1], 0.8 * X[2]]
            hw[f"par.reqSpdToActCmd.{port}_Y"] = [Y[0], Y[1] + 3.0, Y[2]]
        return hw

    def test_the_save_rewrites_every_table_mount_and_fork_value_from_the_compiled_set(self):
        # FINDING F5 (a) [latent]: AppCtrlIf.c:801-1018 copies ALL y.imuMntOri_*, jntAngRotZeroOffs, 20 table triples
        # and the fork kin fields into the NVM buffers on every calibrating tick, whichever step runs; main.c:394-419
        # writes them at the save request. The ports CalibRot did not identify come from GenCorrTbl's persistents,
        # initialised on the first step (isempty(isInitiated), MdlApp.c:45077-45127, 45404-45476) from parLocalTest
        # -- in the CALIBRATION shape: knee 0.01, Y[2] = PropVlvRefCmd (chart_2338 l.68-116). Mounts come from
        # CalcImuMntOri's init (parLocalTest). No ECU code writes parLocalTest, so on a machine these are the
        # compiled values.
        # WHY two plants: on the reference plant the physical mounts and other valves ARE the compiled set, so "echoed
        # from compiled" and "identified from the plant" look the same. The second plant's boards and other valves
        # differ, and the save still carries the compiled values.
        ref = main_run()
        fw = ref.h.fw
        cp = compiled(fw)
        plant = KinematicPlant(q0=dict(rotator=Q0_ROT_DEG), degrees=True, hardware=self.plant_differs_hardware(fw))
        differs = run_calib_rot(plant)
        self.assertEqual(differs.h.inhibit_names(), ["BIT_NO_TARGET"])
        for label, run in (("reference plant", ref), ("plant boards and other valves differ", differs)):
            snap, hw = run.emu.saved[0][1], run.plant.hardware
            changed_top = {}
            for port in OTHER_PORTS:
                with self.subTest(plant=label, port=port):
                    sX, sY = cp[f"par.reqSpdToActCmd.{port}_X"], cp[f"par.reqSpdToActCmd.{port}_Y"]
                    X, Y = table(snap, port)
                    self.assertEqual((X[0], X[1], Y[0]), (0.0, IDENTIFIED_X1, 0.0))
                    if run is differs:
                        pX, pY = hw[f"par.reqSpdToActCmd.{port}_X"], hw[f"par.reqSpdToActCmd.{port}_Y"]
                        self.assertNotEqual((list(X), list(Y)), (list(pX), list(pY)), "not the plant's valve")
                    if port.startswith("blade"):
                        self.assertEqual((X[2], Y[1], Y[2]), (100.0, IDENTIFIED_X1, 100.0))   # l.113-116 literals
                        continue
                    self.assertEqual(Y[1], sY[1], "the compiled minimum")
                    self.assertEqual(X[2], 100.0 if port.startswith("trvl") else sX[2])
                    self.assertEqual(Y[2], PROP_VLV_REF_CMD[port])
                    if Y[2] != sY[2]:
                        changed_top[port] = (sY[2], Y[2])
            with self.subTest(plant=label, check="Y[2] rewritten to PropVlvRefCmd"):
                self.assertEqual(changed_top, {"trvlLeFwd": (100.0, 60.0), "trvlLeRev": (100.0, 60.0),
                                               "trvlRiFwd": (100.0, 60.0), "trvlRiRev": (100.0, 60.0),
                                               "bm1Up": (80.0, 70.0), "bm1Down": (80.0, 70.0), "bm2Up": (70.0, 60.0),
                                               "bm2Down": (70.0, 60.0), "armOut": (80.0, 70.0), "linkOut": (90.0, 70.0),
                                               "tiltPosi": (80.0, 70.0), "tiltNega": (80.0, 70.0)})
            with self.subTest(plant=label, check="mounts"):
                for board, par_name in (("chs", "imuChs"), ("bm1", "imuBm1"), ("bm2", "imuBm2"), ("arm", "imuArm"),
                                        ("link", "imuLink"), ("tilt", "imuTilt")):
                    saved = np.array([[snap[f"y.imuMntOri_{board}.a{i}{j}"] for j in (1, 2, 3)] for i in (1, 2, 3)])
                    for i in (1, 2, 3):
                        for j in (1, 2, 3):
                            self.assertEqual(saved[i - 1, j - 1], cp[f"par.{par_name}.a{i}{j}"], (board, i, j))
                    if run is differs and par_name in kin.MOUNT_NAMES:
                        self.assertGreater(np.abs(saved - hw.mounts()[par_name]).max(), 0.05, f"{board}: not the plant's board")
            with self.subTest(plant=label, check="fork"):
                for k in ("angForkUpLimit", "distUcToForkBack", "distForkBackToPanelTop"):
                    self.assertEqual(snap[f"y.parKin.{k}"], cp[f"par.parKin.{k}"], k)

    def test_after_a_power_cycle_any_calibration_erases_the_identified_rotator_table(self):
        # FINDING F5 (b) [latent]: the identification lives in GenCorrTbl's persistent DW (chart_2338 l.4-32). In the
        # same power cycle a later save (step 28 here) still carries it. A power cycle re-initialises DW from
        # parLocalTest -- which no ECU code writes (the NVM restore goes to u.tblReqSpdToActCmdStored,
        # AppCtrlIf.c:432ff, an inport folded away by EnTestPar = true, SysPar.m:5) -- so the next save of ANY calibration writes the
        # compiled rotator table over CalibRot's result.
        run = main_run()
        saves = run.emu.saved
        self.assertEqual(len(saves), 2)
        for port in ROT_PORTS:
            self.assertEqual(table(saves[1][1], port), table(saves[0][1], port), "same power cycle: kept")
        emu = SaveHandshake()
        plant = KinematicPlant(q0=dict(rotator=Q0_ROT_DEG), degrees=True, hardware=ROT_HW)
        h = boot(plant, extra=[emu])
        h.jump_to_step("CalibForkRefPose")
        h.run_until(lambda h: h.curr_step() == "NoTarget", 30.0, "step 28 done")
        self.assertEqual(len(emu.saved), 1)
        for port in ROT_PORTS:
            with self.subTest(port=port):
                X, Y = table(emu.saved[0][1], port)
                cp = compiled(h.fw)
                sX, sY = cp[f"par.reqSpdToActCmd.{port}_X"], cp[f"par.reqSpdToActCmd.{port}_Y"]
                self.assertEqual(list(X), [0.0, IDENTIFIED_X1, sX[2]])
                self.assertEqual(list(Y), [0.0, 17.0, REF_CMD])
                self.assertNotEqual(list(Y), list(table(saves[0][1], port)[1]))

    @staticmethod
    def prepare_pick(tables, q_rot_deg, onset_cmp=0.0, budget_s=30.0):
        """PreparePick (joint-space bm1/arm/link/rotator to the pose above the stack) with par.reqSpdToActCmd.rot*
        replaced by `tables` -- what the NVM reload would do if the Stored path were wired. In this build nothing
        loads an identified table: the auto control interpolates parLocalTest (MdlApp.c:50328-50480), which no ECU
        code writes. Returns (reached ApproachPanel, plant, nonzero rotPosi commands seen)."""
        plant = KinematicPlant(q0=dict(boom=-50.0, arm=70.0, input_link=-90.0, rotator=q_rot_deg), degrees=True,
                               hardware=ROT_HW, strict_limits=True)
        seen = set()
        probe = lambda h: seen.add(h.fw["y.propVlvCmd.rotPosi"]) if h.fw["y.propVlvCmd.rotPosi"] else None

        def setup(fw):
            for port, (X, Y) in tables.items():
                fw[f"par.reqSpdToActCmd.{port}_X"] = list(X)
                fw[f"par.reqSpdToActCmd.{port}_Y"] = list(Y)

        h = boot(plant, extra=[probe], setup=setup, mach_calib=0)       # the panel cycle, not a calibration
        h.fw["u.parMotionOnsetCmp.rotPosi"] = onset_cmp
        h.set_target_panel(panel_id=7)
        h.tick(3)
        h.request_step("Standby")
        h.run_until(lambda h: h.curr_step() == "Standby", 0.5, "Standby")
        h.tick(90)
        h.jump_to_step("Picking")
        assert h.picking_step() == "PickingStep_PreparePick", h.describe()
        try:
            h.run_until(lambda h: h.picking_step() == "PickingStep_ApproachPanel", budget_s, "ApproachPanel")
            return True, plant, seen
        except AssertionError:
            return False, plant, seen

    def test_the_identified_minimum_below_the_deadband_stalls_the_min_speed_hold(self):
        # FINDING F6 [plant, latent] -- a conditional scenario, NOT a present defect of this build.
        #   LATENT: nothing loads an identified table. chart_2463 interpolates parLocalTest.reqSpdToActCmd
        #   (MdlApp.c:50328-50480; the model reads tblReqSpdToActCmdStored, folded to parLocalTest by the constant
        #   Switch17), no ECU code writes parLocalTest, and the NVM tables reach only the unread
        #   u.tblReqSpdToActCmdStored. prepare_pick loads the table into par.* by hand.
        #   PLANT-DEPENDENT: whether onset - 0.5 falls below the real deadband is decided by the valve line just above
        #   its opening -- here the GUESS of valves.port_speed (a sharp deadband, 0.001 rad/s at it, linear above).
        #   A valve that creeps below its nominal opening, or a flatter line, gives a minimum at or above it.
        # On this plant: rotPosi identified 20.9 %, the valve opens at 21.0 %. The minimum-speed hold asks for
        # max(req, X(2)) (chart_2463 l.66) -> exactly Y(2) = 20.9 % -> closed valve. PreparePick needs the rotator
        # within 0.5 deg (PreparePoseCtrlTol.rotate, SysPar.m:460; MdlApp.c:40540-40548); no timeout fired in 30 s.
        #   -0.8 deg start: the firmware only ever commands 20.9 %, the rotator never moves, no ApproachPanel in 30 s.
        #   control, the same table with Y(2) = 21.0: ApproachPanel in ~6 s.
        #   -4 deg start: the rotator stops where the demand from Tbl_ErrToDmdRot (SysPar.m:296-297) maps through the
        #   identified line to exactly the deadband: 2.95 deg from the target.
        #   u.parMotionOnsetCmp.rotPosi = +0.5 (added to Y(2), chart_2463 l.23 / MdlApp.c:50477) fixes it;
        #   AppCtrlIf.c:562 leaves that inport "// To-do".
        snap = main_run().emu.saved[0][1]
        ident = {p: table(snap, p) for p in ROT_PORTS}
        self.assertAlmostEqual(ident["rotPosi"][1][1], 20.9, places=5)
        with self.subTest(case="-0.8 deg, identified table"):
            ok, plant, seen = self.prepare_pick(ident, -0.8)
            self.assertFalse(ok)
            self.assertEqual(plant.q["rotator"], -0.8 * DEG, "not a single tick of rotation")
            self.assertEqual(seen, {float(f32(20.9))})
        with self.subTest(case="-0.8 deg, control with Y(2) = the plant deadband"):
            fixed = dict(ident)
            fixed["rotPosi"] = (ident["rotPosi"][0], [0.0, 21.0, REF_CMD])
            ok, plant, _ = self.prepare_pick(fixed, -0.8)
            self.assertTrue(ok)
            self.assertLessEqual(abs(plant.q["rotator"]), 0.5 * DEG)
        with self.subTest(case="-0.8 deg, the firmware's own onset compensation input"):
            ok, plant, _ = self.prepare_pick(ident, -0.8, onset_cmp=0.5)
            self.assertTrue(ok)
        with self.subTest(case="-4 deg, stalls where the demand maps to the deadband"):
            ok, plant, _ = self.prepare_pick(ident, -4.0)
            self.assertFalse(ok)
            X, Y = ident["rotPosi"]
            demand = X[1] + (21.0 - Y[1]) * (X[2] - X[1]) / (Y[2] - Y[1])          # rad/s at which the line hits 21.0 %
            stall_err = float(np.interp(math.degrees(demand), ERR_TO_DMD_ROT_Y, ERR_TO_DMD_ROT_X))
            self.assertAlmostEqual(stall_err, 2.95, delta=0.01)
            self.assertAlmostEqual(-math.degrees(plant.q["rotator"]), stall_err, delta=0.1)

        # How often, and why it is the valve model that decides: closed-form identified minimum - deadband over
        # deadbands 20.00..25.99 % (predict_onset equals the firmware in every run of this file), for valve lines
        # X = [0, 0.001, vmax], Y = [0, deadband, 125]. Slope at the opening = (vmax - 0.001) / (125 - deadband).
        # Above -0.5 throughout: none of these lines turns 0.5 deg within one 2 s step AT its deadband (0.001 rad/s),
        # so the onset command is above the deadband.
        X, Y = ROT_HW["par.reqSpdToActCmd.rotPosi_X"], ROT_HW["par.reqSpdToActCmd.rotPosi_Y"]
        self.assertEqual((X[1], X[2], Y[2]), (0.001, 0.40, 125.0), "this plant's rotPosi line is the 0.40 row")
        for vmax, below, at, lo, hi in ((0.05, 0.0, 0.0, 0.10, 0.30),       # 0.00047 rad/s per %: never below
                                        (0.40, 0.4833, 0.05, -0.10, 0.10),  # 0.0038 (this plant): 48 %, 5 % tie
                                        (2.0, 1.0, 0.0, -0.33, -0.13),      # 0.019: always below
                                        (10.0, 1.0, 0.0, -0.46, -0.27)):    # 0.096: always below
            with self.subTest(sweep="0.01 % grid", vmax=vmax):
                d = sweep_min_minus_deadband(SWEEP_DBS, vmax)
                self.assertAlmostEqual(float(np.mean(d < -TIE_TOL)), below, delta=0.002)
                self.assertAlmostEqual(float(np.mean(np.abs(d) <= TIE_TOL)), at, delta=0.002)
                self.assertAlmostEqual(float(d.min()), lo, delta=0.006)
                self.assertAlmostEqual(float(d.max()), hi, delta=0.006)
                self.assertGreater(float(d.min()), -ONSET_DLY_CMP)
        with self.subTest(sweep="between the ties", vmax=0.40):
            d = sweep_min_minus_deadband(TIE_FREE_DBS, 0.40)
            self.assertAlmostEqual(float(np.mean(d < -TIE_TOL)), 0.505, delta=0.002)
            self.assertEqual(float(np.mean(np.abs(d) <= TIE_TOL)), 0.0)

    def test_can_truncation_of_the_rotator_command_pushes_the_hold_below_the_deadband(self):
        # FINDING F6 (CAN). [firmware] ecu_glue sends rotPosi + rotNega to the tilt-rotator controller as a uint8 at
        # 0.4 %, truncated (CanCtrl.c:1090, CAN4_X1Exc.dbc:189). The staircase's odd 0.2 % steps never leave the ECU
        # (20.2 -> 20.0, 20.6 -> 20.4, ...) while the firmware records its own value, and the hold command
        # (onset - 0.5) is truncated again: the transmitted hold is always at least one quantum (0.4 %) below the
        # transmitted onset command, whatever the valve.
        # [plant, latent] Whether that lands below the deadband depends on the valve line (as F6), and nothing loads
        # the table in this build (F6). On this plant: identified rotPosi 21.1 (transmitted as a hold: 20.8 < 21.0),
        # rotNega 22.3 (22.0 < 22.3); over deadbands 20..26 % between the grid ties the transmitted hold is below the
        # deadband every time. (On a deadband exactly on the 0.4 % grid the hold can equal it, and whether the GUESSed
        # valve then opens is a float tie, so those deadbands are not swept.)
        # ASSUMPTION: the controller applies the decoded value with no dithering, the valve deadband is on that
        # value, and the 20 ms message period is ignored (2 s staircase steps).
        class CanTiltRotatorPlant(KinematicPlant):
            QUANTISED = ("rotPosi", "rotNega", "tiltPosi", "tiltNega")

            def __init__(self, *a, **k):
                super().__init__(*a, **k)
                self._prev_valve = {p: 0.0 for p in vlv.PORTS}

            def step_kinematics(self, dt=DT):
                # __call__ already lagged the unquantised command; redo lag and effective on what the controller receives
                if self._tick is not None and self._tick.tick_count == 0:
                    self._prev_valve = {p: 0.0 for p in vlv.PORTS}
                cmd = dict(self.cmd)
                for p in self.QUANTISED:
                    cmd[p] = can_received(cmd[p])
                self.cmd = cmd
                self.valve = vlv.lag_step(self._prev_valve, cmd, dt, self.tau_valve)
                self.effective = vlv.effective_commands(cmd, self.valve, self._dbs)
                super().step_kinematics(dt)
                self._prev_valve = dict(self.valve)

        with self.subTest(check="encoding, plant-independent"):
            np.testing.assert_allclose([can_received(staircase(k)) for k in range(6)], [20.0, 20.0, 20.4, 20.4, 20.8, 20.8],
                                       atol=1e-9)
            for k in range(401):                                     # the whole staircase, 20 .. 100 %
                s = staircase(k)
                self.assertTrue(-1e-5 <= s - can_received(s) < CAN_QUANTUM + 1e-5, (k, s))
                self.assertLessEqual(can_received(identified_min(s)), can_received(s) - CAN_QUANTUM + 1e-6, (k, s))
        plant = CanTiltRotatorPlant(q0=dict(rotator=Q0_ROT_DEG), degrees=True, hardware=ROT_HW)
        run = run_calib_rot(plant)
        snap = run.emu.saved[0][1]
        for port, db, want, hold in (("rotPosi", 21.0, 21.1, 20.8), ("rotNega", 22.3, 22.3, 22.0)):
            X, Y = plant.tables[port]
            onset, k, need = predict_onset(X, Y, can=True)
            got = table(snap, port)[1][1]
            with self.subTest(port=port, check="prediction"):
                self.assertTrue(10 < need < STEP_TICKS - 10, (k, need))
                self.assertAlmostEqual(got, identified_min(onset), places=5)
                self.assertAlmostEqual(table(snap, port)[0][2], true_speed_at_ref(plant, port, can=True), delta=1e-4)
            with self.subTest(port=port, check="documented"):
                self.assertAlmostEqual(got, want, places=5)
                self.assertAlmostEqual(can_received(got), hold, places=9)
                self.assertLess(can_received(got), db)
        with self.subTest(sweep="between the ties, this valve line"):
            d = sweep_min_minus_deadband(TIE_FREE_DBS, ROT_HW["par.reqSpdToActCmd.rotPosi_X"][2], can=True)
            self.assertTrue(np.all(d < -TIE_TOL), float(d.max()))


# ===========================================================================================================
class TestLostRotatorAngle(unittest.TestCase):

    def test_a_frozen_angle_drives_an_unbounded_staircase_and_the_guard_input_is_never_written(self):
        # FINDING F7. [firmware] When the TR controller's BLADE_ANG is lost, PrePostProc_If.c:1041-1050 substitutes
        # 0.0 deg ("safe value") and figureOfMerit 2. No fault reaches the model: u.isJntAngRotFault has no writer
        # anywhere in the ECU code (declared in MdlApp.h:1069 only). The ECU does detect the loss, but only for
        # others: lostComm_CAN4TrCtrlDiagL (InpHndlr.c:685) and TrRotSensFault_DiagL (non-zero figure of merit,
        # InpHndlr.c:693) go to the service diagnostics (ServiceCANDiag.c:422, 453), and tr_isAutoEnabled -- which
        # needs figureOfMerit == 0 (PrePostProc_If.c:579-587) -- drops into u.isTrAutoOk (AppCtrlIf.c:245, "// To-do"),
        # an inport MdlApp.c never references. CalibRot then logs a reference of 0, and RotPosiMin waits for an onset
        # that cannot come: _Min has no timeout (chart_1210 [isMotionOnset.rotPosi] only), the command climbs 0.2 %
        # every 2 s, to PropVlvCmdMinCalib_limit 100 % (SysPar.m:109) after 800 s. 100 s in: 30.0 %, no inhibit.
        # [plant] ASSUMPTION: the TR controller keeps executing the RotExtn flow commands while its own BLADE_ANG is
        # timed out or reports figureOfMerit 2 -- its fail-safe is not in the sources. Under it the plant rotator
        # turns faster and faster: ~92 deg in those 100 s.
        # The guard exists: isJntAngRotFault && hasRot -> BIT_JNT_ANG_SNSR_ERR (MdlApp.c:39612-39618), in
        # CALIB_INHIBIT_MASK 13976 when isMachCalib (MdlApp.c:39679): written, it stops the run at once.
        class LostBladeAngle:
            def __call__(self, h):
                h.fw["u.jntAngRaw_Rot"] = 0.0

        plant = KinematicPlant(q0=dict(rotator=Q0_ROT_DEG), degrees=True, hardware=ROT_HW, strict_limits=True)
        rec = Recorder(plant)
        h = boot(plant, sensors=[LostBladeAngle()], extra=[rec, SaveHandshake()])
        fw = h.fw
        h.jump_to_step("CalibRot")
        h.run_until(lambda h: h.calib_step() == "RotPosiMin", 20.0, "RotPosiMin")
        t0, turned0 = h.tick_count, rec.turned
        h.run_seconds(100.0)
        elapsed = h.tick_count - t0
        with self.subTest(check="firmware: no fault, no timeout"):
            self.assertEqual(h.calib_step(), "RotPosiMin")
            self.assertTrue(fw["y.isCalibrating"])
            self.assertEqual(h.inhibit_names(), ["BIT_NO_TARGET"])
            self.assertEqual(fw["y.jnts.TiltToRot.q"], 0.0, "the firmware sees a rotator that never moved")
            self.assertEqual(fw["y.propVlvCmd.rotPosi"], staircase(elapsed // STEP_TICKS))
            self.assertAlmostEqual(fw["y.propVlvCmd.rotPosi"], 30.0, places=4)
        with self.subTest(check="plant (ASSUMPTION: the TR controller keeps executing)"):
            turned = (rec.turned - turned0) / DEG
            self.assertGreater(turned, ANG_ROT_POSI, "more than the whole leg the procedure intends")
            self.assertAlmostEqual(turned, 92.0, delta=5.0)
            self.assertGreater(plant.qdot["rotator"], 0.02, "and still accelerating")
        # the guard, when its input is written
        fw["u.isJntAngRotFault"] = 1
        h.run_until(lambda h: not h.fw["y.isCalibrating"], 0.05, "calibration stopped by the sensor fault")
        self.assertEqual(h.main_state(), "CalibRot_Inhibited")
        self.assertIn("BIT_JNT_ANG_SNSR_ERR", h.inhibit_names())
        self.assertEqual(h.valves(), {})


if __name__ == "__main__":
    unittest.main()
