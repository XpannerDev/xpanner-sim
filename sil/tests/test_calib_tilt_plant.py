"""
CalibTilt (AutoCtrlStep 25) end to end: the compiled X1Exc firmware calibrates sil.plant.KinematicPlant.

WHAT THE STEP DOES (chart_1210 CalibStepMgr, chart_3055 SetPropVlvCmdCalib, chart_2316 DetectMotionOnsetFlags,
chart_2291 CalcImuMntOri, chart_2383 / chart_2338 SetTblReqSpdToActCmd; generated MdlApp.c line numbers below)
  TiltPntRef_stb 8 s -> TiltPntRef_log 1 s (accRef = unit mean of the raw tilt accelerometer)
  -> TiltPosiMin     staircase tiltPosi = 20 % + 0.2 % every 200 ticks, RAW (no smoothing), until a motion-onset
                     pulse: angle(accRef, acc) > 0.5 deg on ONE sample (chart_2316 l.91-104); stores cmd - 0.5
  -> TiltPosiMin_stb 8 s (the 1 s cubic ramp-down of the onset command runs inside it)
  -> TiltPosiToPnt1  tiltPosi = PropVlvRefCmd 70 % through a 1 s cubic ramp; peak |y.jnts.TiltMntToTilt.qDot|;
                     ends on angle(accRef, acc) > 35 deg OR cnt > 1000 (10 s), no alarm either way
  -> TiltPnt1_stb 8 s -> TiltPnt1_log 1 s (accPnt1)
  -> TiltNegaMin / TiltNegaMin_stb / TiltNegaToPnt2 (angle measured from accPnt1, > 70 deg) -> TiltPnt2_stb 8 s
  -> TiltPnt2_log 1 s (accPnt2) -> Tilt_save (isCalibDataSaveReq; harness.SaveHandshake = main.c:394-419)
  Mount rebuild on leaving TiltPnt2_log: vx = -cross(Pnt1 - Ref, Pnt2 - Ref), vy = cross(vx, accRef),
  vz = cross(vx, vy), all normalised; imuMntOri_tilt = [vx vy vz] (chart_2291 l.331-357, MdlApp.c:43953-44010).
  So vx is the tilt axis in board coordinates (any three distinct points on the gravity circle give it) and vy is
  the horizontal perpendicular at the REFERENCE pose: the rebuilt mount is the true board only if the tilt link's
  Y axis was horizontal when the calibration started (TestTiltZeroIsTheStartPose).

RUN TIME (sim seconds; wall ~2.5 s per run at ~0.25 ms per tick)
  fixed: 5 x 8.01 s (_stb) + 3 x 1.01 s (_log) = 43.1 s
  + each _Min staircase 2 s per 0.2 % from 20 % to the valve's onset: (deadband - 20) / 0.2 x 2 s, i.e. 10 s per
    percent of deadband above 20 % -- the dominant variable term
  + each leg 1 s ramp + travel / speed (at most 10.02 s, then it ends on the timeout).
  This file's main plant (deadbands 22.3 / 21.1 %): 101.7 s. The unit's own stored valve (19.5 / 25.7 %) at the
  default pose: 128.3 s, of which 63 s is the tiltNega staircase. Total file wall time ~20 s.

WHY THE PLANT DIFFERS FROM THE STORED PARAMETERS
  The plant's tilt valve (deadband, speed) and, in most scenarios, its tilt IMU board are deliberately NOT the
  compiled par.* (the plant freezes its hardware from the compiled set plus overrides and never reads par.*
  again, sil/plant.py Hardware), so "identified == plant" cannot be the stored numbers echoed back.

ACCELEROMETER SIGN: every run uses the plant default acc_sign = -1 g (sil/plant.py ACC_SIGN; with +1 g vy/vz of
the rebuilt tilt mount flip, test_valve_plant.test_calib_tilt_identifies_the_valve_and_reproduces_the_compiled_mount).

Every calibration here runs with strict joint stops (plant.strict_limits): the +-45 deg tilt limit is an ESTIMATE
(sil/plant.py) and a leg that ended on it would still "succeed" in the firmware.

Run:  cd xpanner-sim && python3 -m unittest sil.tests.test_calib_tilt_plant -v
"""
import math
import unittest

import numpy as np

from sil import kinematics as kin
from sil import valves as vlv
from sil.harness import Harness, SaveHandshake
from sil.plant import KinematicPlant

DEG = math.pi / 180.0
F32 = lambda v: float(np.float32(v))

# --- firmware constants (SysPar.m; generated as literals) -----------------------------------------------------
STB_TICKS = 801             # CntCalib_stb 800 (SysPar.m:102): entry tick + 800 during ticks (test_calibration_entry)
LOG_TICKS = 101             # CntCalib_log 100 (SysPar.m:103)
TIMEOUT_TICKS = 1002        # _ToPnt [... || cnt > CntCalib_timeout 1000] (SysPar.m:104, MdlApp.c:36318, :35493)
STAIR_TICKS = 200           # CntCalibStepFindingMin (SysPar.m:105, MdlApp.c:44565)
STAIR_STEP = 0.2            # StepFindingMin_size, % (SysPar.m:110)
STAIR_START = 20.0          # PropVlvCmdInitOffs.tilt* (SysPar.m:174-175)
ONSET_DLY_CMP = 0.5         # PropVlvCmdMotionOnsetDlyCmp (SysPar.m:134, MdlApp.c:45610/45618 literal 0.5F)
REF_CMD = 70.0              # PropVlvRefCmd.tiltPosi/tiltNega (SysPar.m:151-152)
ANG_PNT1, ANG_PNT2 = 35.0 * DEG, 70.0 * DEG     # AngTiltPnt1/2 (SysPar.m:124-125; MdlApp.c:36318 0.610865235F)
IDENTIFIED_X1 = F32(0.01)   # knee a calibration writes (chart_2338 l.103-106, MdlApp.c:45777)
MIN_TBL_REQ_SPD = F32(0.002)  # SysPar.m:112, MdlApp.c:45778 fmaxf(0.002F, ...)
# PropVlvRefCmd of every port (SysPar.m:136-156): Y[2] of every table any calibration save writes.
REF_CMD_ALL = dict(trvlLeFwd=60.0, trvlLeRev=60.0, trvlRiFwd=60.0, trvlRiRev=60.0, swingLe=60.0, swingRi=60.0,
                   bm1Up=70.0, bm1Down=70.0, bm2Up=60.0, bm2Down=60.0, armIn=70.0, armOut=70.0, linkIn=70.0,
                   linkOut=70.0, tiltPosi=70.0, tiltNega=70.0, rotPosi=100.0, rotNega=100.0)

TILT_STATES = ["TiltPntRef_stb", "TiltPntRef_log", "TiltPosiMin", "TiltPosiMin_stb", "TiltPosiToPnt1",
               "TiltPnt1_stb", "TiltPnt1_log", "TiltNegaMin", "TiltNegaMin_stb", "TiltNegaToPnt2",
               "TiltPnt2_stb", "TiltPnt2_log", "Tilt_save"]       # chart_1210 SSIDs 584, 609 ... 602

# --- scenario choices (NOT from the firmware) ------------------------------------------------------------------
# ASSUMPTION (pose): arm 70 instead of the harness rest pose's 90 puts the tilt axis 1.9 deg below horizontal, so
# the 35 / 70 deg GRAVITY legs are ~35 / 70 deg of joint travel; at the rest pose (axis 21.9 deg down) they are
# 37.8 / 76.4 deg and the post-leg coast approaches the +-45 deg tilt ESTIMATE. Chosen for stop margin only.
POSE = dict(boom=-40.0, arm=70.0, input_link=-60.0, tilt=0.0)
# GUESS (test fixture): a tilt valve that is NOT the stored table (stored tiltPosi_Y[1] 19.5 / tiltNega_Y[1] 25.7 %,
# X[2] 0.1158 / 0.1647 rad/s at Y[2] 80 %; ECR88D_ShortArm.m:330-333). Posi above, nega below the stored deadband.
# Deadbands close to the 20 % staircase start keep the run short (10 s per % above 20); vmax sized so both legs end
# on angle (not the 10 s timeout) and the coast stays inside +-45 deg (measured -35.6 .. +39.0 deg).
VALVE = dict(deadband={"tiltPosi": 22.3, "tiltNega": 21.1}, vmax={"tiltPosi": 0.18, "tiltNega": 0.20})


def mount(src, prefix):
    """3x3 from the a11..a33 fields under `prefix` of the firmware, a plant Hardware or a SaveHandshake snapshot."""
    return np.array([[src[f"{prefix}.a{r}{c}"] for c in (1, 2, 3)] for r in (1, 2, 3)], dtype=float)


def plant_speed_at_ref(plant, port):
    """The plant's true tilt speed (rad/s) at PropVlvRefCmd -- what _ToPnt should identify."""
    X, Y = plant.tables[port]
    return vlv.port_speed(REF_CMD, X, Y, deadband=plant.deadband.get(port), vmax=plant.vmax.get(port))


def tilt_travel_for_leg(leg, pitch):
    """Joint travel that turns gravity by `leg` about a tilt axis `pitch` off horizontal (cone chord)."""
    s = math.sin(leg / 2) / math.cos(pitch)
    return math.inf if s > 1 else 2 * math.asin(s)


def reference_roll(frame_tilt):
    """phi = rotation about the tilt link's X that would bring its Y axis horizontal: atan2 of up expressed in
    the link frame. Derived from chart_2291 l.341 (vy = cross(vx, accRef)): the rebuilt mount is M @ Rx(-phi)."""
    g = np.asarray(frame_tilt).T @ kin.UP
    return math.atan2(g[1], g[2])


class Recorder:
    """Listed AFTER the KinematicPlant: each tick it sees the calibStep / propVlvCmd the firmware produced on the
    previous step together with the plant state that command has just moved."""

    def __init__(self, plant):
        self.plant = plant
        self.entries = []               # (tick, calibStep name, plant tilt rad)
        self.last_min_cmd = {}          # port -> propVlvCmd on the last tick labelled <dir>Min = the onset command
        self.ref_frame = None           # plant tilt-link world attitude while the reference is logged
        self.tilt_range = [math.inf, -math.inf]

    def __call__(self, h):
        fw, s = h.fw, h.calib_step()
        if not self.entries or self.entries[-1][1] != s:
            self.entries.append((h.tick_count, s, self.plant.q["tilt"]))
        if s == "TiltPosiMin":
            self.last_min_cmd["tiltPosi"] = fw["y.propVlvCmd.tiltPosi"]
        elif s == "TiltNegaMin":
            self.last_min_cmd["tiltNega"] = fw["y.propVlvCmd.tiltNega"]
        elif s == "TiltPntRef_log" and self.ref_frame is None:
            self.ref_frame = np.array(self.plant.frames["tilt"])
        t = self.plant.q["tilt"]
        self.tilt_range = [min(self.tilt_range[0], t), max(self.tilt_range[1], t)]

    def sequence(self):
        return [s for _, s, _ in self.entries]

    def dwell(self):
        """{state: ticks} for states visited once (the sequence here visits each Tilt state once)."""
        return {s: t1 - t0 for (t0, s, _), (t1, _, _) in zip(self.entries, self.entries[1:])}

    def tilt_at_entry(self, state):
        return next(q for _, s, q in self.entries if s == state)


def run_calib_tilt(plant, setup=None, extra=()):
    """Boot a healthy machine with `plant` publishing every sensor, run CalibTilt from NoTarget to completion.
    setup(h) runs after nominal_inputs, before the first tick. Returns (h, rec, emu, ticks from the jump)."""
    plant.strict_limits = True
    rec, emu = Recorder(plant), SaveHandshake()
    h = Harness(plant=[plant, rec, *extra, emu]).reset().nominal_inputs()
    h.gnss_rtk_fixed()                       # not a CalibTilt gate (spec A7: steps 20-26 have none); healthy machine
    if setup is not None:
        setup(h)
    h.tick(3)
    start = h.tick_count
    h.jump_to_step("CalibTilt")
    h.run_until(lambda h: h.curr_step() == "NoTarget", 400.0, "CalibTilt done")
    return h, rec, emu, h.tick_count - start


# ==============================================================================================================
class TestCalibTiltEndToEnd(unittest.TestCase):
    """One run, several claims. The plant's tilt board is rolled 10 deg about the tilt axis relative to the
    compiled par.imuTilt -- the one board error that leaves the firmware's tilt RATE exact (gyro x is invariant
    under a rotation about x), so the speed identification can be checked against the plant exactly while the
    mount identification is still not an echo (columns y and z differ from the compiled mount by 0.17)."""

    BOARD_ROLL = 10.0 * DEG          # GUESS (test fixture)

    @classmethod
    def setUpClass(cls):
        h0 = Harness().reset()
        cls.compiled_mount = kin.mounts_from_fw(h0.fw)["imuTilt"]
        cls.stored_tables = vlv.read_tables(h0.fw)
        cls.hw_mount = cls.compiled_mount @ kin.Rx(cls.BOARD_ROLL)
        plant = KinematicPlant(q0=POSE, degrees=True, hardware={"imuTilt": cls.hw_mount}, **VALVE)
        # the tablet procedure's service flag (test_calibration_entry: isCalibInhibited ANDs it, MdlApp.c:39679)
        boot = {}

        def setup(h):
            h.fw["u.isMachCalib"] = 1

        def before(h):
            if h.tick_count == 3 and not boot:
                boot["tilt_err"] = h.fw["y.jnts.TiltMntToTilt.q"] - plant.q["tilt"]

        h, rec, emu, ticks = run_calib_tilt(plant, setup=setup, extra=[before])
        fw = h.fw
        cls.plant, cls.rec, cls.emu, cls.ticks = plant, rec, emu, ticks
        cls.boot_tilt_err = boot["tilt_err"]
        cls.final = dict(curr=h.curr_step(), calibrating=fw["y.isCalibrating"], calib=h.calib_step(),
                         inhibit=h.inhibit_names(), valves=h.valves(), limit_hits=list(plant.limit_hits))
        cls.snap = emu.saved[0][1] if emu.saved else None
        identified = mount(cls.snap, "y.imuMntOri_tilt")
        # The ECU's NVM boot path writes the saved mount into u.tiltImu_MntOriStored (AppCtrlIf.c:630-638) ...
        for r in (1, 2, 3):
            for c in (1, 2, 3):
                fw[f"u.tiltImu_MntOriStored.a{r}{c}"] = float(identified[r - 1, c - 1])
        h.tick(100)
        cls.tilt_err_nvm_inport = fw["y.jnts.TiltMntToTilt.q"] - plant.q["tilt"]
        # ... but the model reads parLocalTest.imuTilt (MdlApp.c:41966). Patching par is what a rebuild would do.
        kin.write_mounts(fw, {"imuTilt": identified})
        h.tick(100)
        cls.tilt_err_par = fw["y.jnts.TiltMntToTilt.q"] - plant.q["tilt"]

    def test_completes_through_all_thirteen_states_in_about_100_s(self):
        # FW: chart_1210 SSIDs 584 -> 609 -> 640 -> 610 -> 598 -> 599 -> 595 -> 635 -> 588 -> 639 -> 624 -> 603 -> 602,
        # exit J260 -> Standby on the save ack (SSID 346); spec A7 success = calibStep back to CalibStandby and
        # autoCtrl_CurrStep back to NoTarget.
        self.assertEqual(self.rec.sequence(), ["CalibStandby"] + TILT_STATES + ["CalibStandby"])
        self.assertEqual(self.final["curr"], "NoTarget")
        self.assertFalse(self.final["calibrating"])
        self.assertEqual(self.final["valves"], {})
        self.assertEqual(self.final["inhibit"], ["BIT_NO_TARGET"])
        self.assertEqual(len(self.emu.saved), 1)
        self.assertEqual(self.final["limit_hits"], [], "strict stops: the coast never reached +-45 deg")
        d = self.rec.dwell()
        for s in ("TiltPntRef_stb", "TiltPosiMin_stb", "TiltPnt1_stb", "TiltNegaMin_stb", "TiltPnt2_stb"):
            self.assertEqual(d[s], STB_TICKS, s)
        for s in ("TiltPntRef_log", "TiltPnt1_log", "TiltPnt2_log"):
            self.assertEqual(d[s], LOG_TICKS, s)
        self.assertEqual(d["Tilt_save"], 2, "handshake acks on the next tick (SaveHandshake)")
        # both legs ended on ANGLE, not on the silent 10 s timeout (MdlApp.c:36318, :35493) ...
        self.assertLess(d["TiltPosiToPnt1"], TIMEOUT_TICKS - 1)
        self.assertLess(d["TiltNegaToPnt2"], TIMEOUT_TICKS - 1)
        # ... at the joint angle the gravity legs predict: Pnt1 is 35 deg of gravity from the REFERENCE (not from where
        # the staircase left the tool), Pnt2 70 deg from the Pnt1 LOG pose; tiltPosi raises q, tiltNega lowers it
        # (chart_3055 l.147-152 -> valves.AXIS_PORTS). Overshoot <= 4 ticks of travel at the leg speed: the sensors
        # lag the plant by one tick, the guard reads angCalib through Delay4 (MdlApp.c:36318), the state changes one
        # tick later, and this recorder samples one tick after that (measured 0.17 / 0.27 deg = 1.8 / 2.8 ticks).
        fw_like = self.plant.hardware
        off = fw_like["par.parKin.angOutpLinkToTiltMnt"] + fw_like["par.parKin.angTiltMntToTilt"]
        q = self.plant.q
        pitch = -(q["boom"] + q["arm"] + kin.fourbar_output(fw_like, q["input_link"]) + off)
        self.assertAlmostEqual(math.degrees(pitch), -1.9, delta=0.1)
        ref = self.rec.tilt_at_entry("TiltPntRef_log")
        slack = {p: 4 * 0.01 * plant_speed_at_ref(self.plant, p) for p in ("tiltPosi", "tiltNega")}
        end1 = self.rec.tilt_at_entry("TiltPnt1_stb") - ref
        self.assertGreaterEqual(end1, tilt_travel_for_leg(ANG_PNT1, pitch))
        self.assertLessEqual(end1, tilt_travel_for_leg(ANG_PNT1, pitch) + slack["tiltPosi"])
        end2 = self.rec.tilt_at_entry("TiltPnt2_stb") - self.rec.tilt_at_entry("TiltPnt1_log")
        self.assertLessEqual(end2, -tilt_travel_for_leg(ANG_PNT2, pitch))
        self.assertGreaterEqual(end2, -tilt_travel_for_leg(ANG_PNT2, pitch) - slack["tiltNega"])
        self.assertLess(self.rec.tilt_range[1], 40.0 * DEG)           # measured +39.0 / -35.6 deg incl. the coast
        self.assertGreater(self.rec.tilt_range[0], -36.0 * DEG)
        # staircase timing (chart_3055 l.53-67, 76, 93-94; MdlApp.c:44565, :44597): onset command 20 + 0.2 k is
        # on the valve from tick 200 k to 200 (k + 1) of the state, plus the one-tick delay into CalibStepMgr.
        for port, state in (("tiltPosi", "TiltPosiMin"), ("tiltNega", "TiltNegaMin")):
            k = round((self.rec.last_min_cmd[port] - STAIR_START) / STAIR_STEP)
            self.assertAlmostEqual(self.rec.last_min_cmd[port], F32(STAIR_START + F32(k) * F32(STAIR_STEP)), places=4)
            self.assertGreaterEqual(d[state], STAIR_TICKS * k, state)
            self.assertLessEqual(d[state], STAIR_TICKS * (k + 1) + 1, state)
        fixed = 5 * STB_TICKS + 3 * LOG_TICKS
        variable = d["TiltPosiMin"] + d["TiltNegaMin"] + d["TiltPosiToPnt1"] + d["TiltNegaToPnt2"] + d["Tilt_save"]
        self.assertEqual(sum(d[s] for s in TILT_STATES), fixed + variable)
        # from the jump: request tick + StartPause pulse (2 ticks) precede TiltPntRef_stb
        self.assertLessEqual(abs(self.ticks - (fixed + variable)), 3)
        # measured 10171 ticks from the jump = 101.7 s: 43.1 s fixed, 45.8 s of staircase (onset 22.8 % after 28.95 s,
        # 21.6 % after 16.83 s), 12.8 s of legs (4.77 s / 8.03 s), 3 ticks of request + button
        self.assertGreater(self.ticks, 95 * 100)
        self.assertLess(self.ticks, 110 * 100)

    def test_identified_minimum_command_is_the_plants_deadband_not_the_stored_one(self):
        # FW: onset = |angle(accRef or accPnt1, raw acc)| > 0.5 deg on one sample, rising edge (chart_2316 l.78-79,
        # 91-104; MdlApp.c:44276); chart_2338 l.63-64 stores propVlvCmd - 0.5 on the SAME tick (MdlApp.c:45606-45618)
        # while CalibStepMgr sees the pulse one tick later through Delay1 (MdlApp.c:36152, :46182).
        # PLANT: no motion below the deadband (valves.port_speed), 0.001 rad/s at it, so the 0.5 deg onset needs a
        # few 0.2 % steps of creep -- measured 2.5 steps, which the 0.5 % compensation cancels exactly here.
        for port in ("tiltPosi", "tiltNega"):
            with self.subTest(port=port):
                db = self.plant.deadband[port]
                stored = self.stored_tables[port][1][1]
                Y = self.snap[f"y.tblReqSpdToActCmd.{port}_Y"]
                onset = self.rec.last_min_cmd[port]
                self.assertAlmostEqual(Y[1], F32(onset - ONSET_DLY_CMP), places=5)
                self.assertGreaterEqual(onset, db - 1e-4, "the valve cannot move below its deadband")
                self.assertGreaterEqual(Y[1], db - ONSET_DLY_CMP - 1e-4)
                self.assertLessEqual(Y[1], db + ONSET_DLY_CMP + 1e-4)
                self.assertGreater(abs(Y[1] - stored), 1.0, f"not the stored {stored} %")
        self.assertAlmostEqual(self.snap["y.tblReqSpdToActCmd.tiltPosi_Y"][1], 22.3, places=4)   # measured
        self.assertAlmostEqual(self.snap["y.tblReqSpdToActCmd.tiltNega_Y"][1], 21.1, places=4)

    def test_identified_speed_is_the_plants_speed_at_the_reference_command(self):
        # FW: chart_2383 l.64-67 zeroes actSpdRef at leg entry, l.85-86 keeps max |y.jnts.TiltMntToTilt.qDot|
        # (MdlApp.c:45062, :45318-45323) -- the 3 Hz filtered gyro-difference rate; chart_2338 l.103-106 writes
        # X = [0, 0.01, max(0.002, peak)], Y = [0, onset - 0.5, 70].
        # Tilt is a direct joint (no cylinder Jacobian): the steady rate at 70 % is the peak. Measured ratio 1.00000.
        for port in ("tiltPosi", "tiltNega"):
            with self.subTest(port=port):
                X = self.snap[f"y.tblReqSpdToActCmd.{port}_X"]
                Y = self.snap[f"y.tblReqSpdToActCmd.{port}_Y"]
                true = plant_speed_at_ref(self.plant, port)
                self.assertEqual((X[0], X[1], Y[0], Y[2]), (0.0, IDENTIFIED_X1, 0.0, REF_CMD))
                self.assertAlmostEqual(X[2], true, delta=1e-3 * true)
                # the stored table's own speed at 70 % (its X[2] belongs to Y[2] = 80 %, not comparable directly)
                stored = vlv.port_speed(REF_CMD, *self.stored_tables[port])
                self.assertGreater(abs(X[2] - stored), 0.15 * stored, f"not the stored table's {stored:.4f} rad/s")
        self.assertAlmostEqual(plant_speed_at_ref(self.plant, "tiltPosi"), 0.14898, places=5)
        self.assertAlmostEqual(plant_speed_at_ref(self.plant, "tiltNega"), 0.16621, places=5)

    def test_identified_mount_is_the_plants_board_and_only_a_rebuild_makes_the_firmware_use_it(self):
        # FW: chart_2291 l.331-357 (MdlApp.c:43953-44010). Before: the firmware reads tilt through the compiled mount,
        # 10 deg off (MdlApp.c:41966 parLocalTest.imuTilt).
        identified = mount(self.snap, "y.imuMntOri_tilt")
        self.assertLess(np.abs(identified - self.hw_mount).max(), 1e-4)
        self.assertGreater(np.abs(identified - self.compiled_mount).max(), 0.15, "not the compiled mount echoed")
        self.assertAlmostEqual(self.boot_tilt_err, -self.BOARD_ROLL, delta=0.01 * DEG)
        # FINDING (firmware): the saved mount never reaches the running model. AppCtrlIf.c:630-638 copies NVM into
        # u.tiltImu_MntOriStored at boot, but MdlApp.c reads MdlApp_U.tiltImu_MntOriStored 0 times (grep -ac) and
        # nothing outside the model writes parLocalTest. With the inport loaded the tilt still reads 10 deg off.
        self.assertAlmostEqual(self.tilt_err_nvm_inport, -self.BOARD_ROLL, delta=0.01 * DEG)
        self.assertAlmostEqual(self.tilt_err_par, 0.0, delta=0.01 * DEG)

    def test_the_save_rewrites_every_other_axis_speed_table_in_the_calibration_shape(self):
        # FINDING (firmware): chart_2338 rebuilds ALL 18 tables every tick (l.68-116): X[1] = 0.01 literal, Y[2] =
        # PropVlvRefCmd, X[2] = the persistent peak initialised from the stored X[2], Y[1] from the stored Y[1]
        # (l.11-32); AppCtrlIf.c:802-1012 copies every table into the NVM buffers on every isCalibrating tick. So a
        # CalibTilt save also rewrites the tables of axes it never moved: every knee 0.001 -> 0.01, and Y[2]
        # 80 -> 70 (bm1Up/Down, armOut), 90 -> 70 (linkOut), 80 -> 100 (rotPosi/Nega), 100 -> 60 (travel), 70 -> 60
        # (bm2), blade [0 .001 80]/[0 25 80] -> [0 .01 100]/[0 .01 100] -- same X[2], different Y[2], i.e. a
        # different speed per percent. Latent in this build only because the Stored inports are never read.
        changed_y2 = set()
        for port in vlv.PORTS:
            X, Y = self.snap[f"y.tblReqSpdToActCmd.{port}_X"], self.snap[f"y.tblReqSpdToActCmd.{port}_Y"]
            Xs, Ys = self.stored_tables[port]
            with self.subTest(port=port):
                if port.startswith("blade"):
                    self.assertEqual((X, Y), ([0.0, IDENTIFIED_X1, 100.0], [0.0, IDENTIFIED_X1, 100.0]))
                    self.assertNotEqual(list(Ys), Y)
                    continue
                self.assertEqual(X[1], IDENTIFIED_X1)
                self.assertEqual(Xs[1], F32(0.001))
                self.assertEqual(Y[2], REF_CMD_ALL[port])
                if not port.startswith("tilt"):
                    self.assertEqual(X[2], F32(Xs[2]))
                    self.assertEqual(Y[1], F32(Ys[1]))
                    if Y[2] != Ys[2]:
                        changed_y2.add(port)
        self.assertEqual(changed_y2, {"trvlLeFwd", "trvlLeRev", "trvlRiFwd", "trvlRiRev", "bm1Up", "bm1Down",
                                      "bm2Up", "bm2Down", "armOut", "linkOut", "rotPosi", "rotNega"})


# ==============================================================================================================
class TestCalibTiltBoard(unittest.TestCase):

    def test_a_general_board_rotation_is_recovered_but_the_speed_is_read_through_the_stored_mount(self):
        # WHY: the mount rebuild uses raw accelerometer vectors only, so any board orientation comes back. The speed
        # leg does not: its peak is y.jnts.TiltMntToTilt.qDot, the x component of M_stored^T * gyro (chart_2143 rate
        # chain), so with a board error R = M_stored^T M_board the firmware reads R[0,0] x the true rate.
        # FINDING: the identified tilt speeds are low by 1 - R[0,0] (1.5 % here); the mount is right after the
        # rebuild, the table stays wrong until CalibTilt is run AGAIN with the corrected mount (as for the arm,
        # test_valve_plant.test_calib_arm_recovers_the_units_mount_from_a_wrong_stored_one_but_not_its_speed_table).
        compiled = kin.mounts_from_fw(Harness().reset().fw)["imuTilt"]
        R = kin.Rz(8.0 * DEG) @ kin.Ry(-6.0 * DEG) @ kin.Rx(12.0 * DEG)      # GUESS (test fixture)
        board = compiled @ R
        plant = KinematicPlant(q0=POSE, degrees=True, hardware={"imuTilt": board}, **VALVE)
        h, rec, emu, _ = run_calib_tilt(plant)
        fw = h.fw
        self.assertEqual(len(emu.saved), 1)
        snap = emu.saved[0][1]
        identified = mount(snap, "y.imuMntOri_tilt")
        for col in range(3):
            self.assertGreater(np.abs(board[:, col] - compiled[:, col]).max(), 0.1, "every column differs")
        self.assertLess(np.abs(identified - board).max(), 1e-4)
        kappa = (compiled.T @ board)[0, 0]
        self.assertAlmostEqual(kappa, math.cos(8 * DEG) * math.cos(6 * DEG), places=6)   # par mounts are float32
        for port in ("tiltPosi", "tiltNega"):
            with self.subTest(port=port):
                true = plant_speed_at_ref(plant, port)
                got = snap[f"y.tblReqSpdToActCmd.{port}_X"][2]
                self.assertAlmostEqual(got / true, kappa, delta=1e-3)
                self.assertLess(got / true, 0.99)
        err_before = fw["y.jnts.TiltMntToTilt.q"] - plant.q["tilt"]
        self.assertGreater(abs(err_before), 10.0 * DEG, "precondition: the stored mount misreads the tilt")
        kin.write_mounts(fw, {"imuTilt": identified})
        h.tick(100)
        self.assertAlmostEqual(fw["y.jnts.TiltMntToTilt.q"], plant.q["tilt"], delta=0.01 * DEG)


# ==============================================================================================================
class TestTiltZeroIsTheStartPose(unittest.TestCase):
    """vy = cross(vx, accRef) (chart_2291 l.341, MdlApp.c:43985) is horizontal at the REFERENCE pose, so the rebuilt
    mount is M_board @ Rx(-phi), phi = the tilt link's roll about its own X at the reference (up in link coordinates),
    and after the rebuild the firmware reads tilt - phi. The deck's CalibTilt preconditions (resources/
    Sensor_and_Valve_Calibration_ProductionV1.md slides 12-13: "Roll angle = 0", "Pitch angle > 5", "Set tilt to
    Horizontal") are therefore half load-bearing -- roll and the start tilt define the zero -- and the firmware checks
    none of them (no machine attitude or tilt-angle guard in chart_1210 / chart_2537). The jack-up pitch changes
    nothing: a pitch about the machine's Y keeps the tilt link's Y horizontal."""

    CASES = (   # GUESS (test fixtures): start tilt deg, ground roll deg, ground pitch deg (URDF; -6 = nose up)
        ("tool 6 deg off horizontal", -6.0, 0.0, 0.0),
        ("machine rolled 3 deg", 0.0, 3.0, 0.0),
        ("deck jack-up: nose up 6 deg, roll 0, tilt level", 0.0, 0.0, -6.0),
    )

    def test_roll_and_start_tilt_become_the_tilt_zero_but_the_jack_up_pitch_does_not(self):
        compiled = kin.mounts_from_fw(Harness().reset().fw)["imuTilt"]
        for label, tilt0, roll, pitch in self.CASES:
            with self.subTest(case=label):
                q0 = dict(POSE, tilt=tilt0)
                plant = KinematicPlant(q0=q0, degrees=True, **VALVE)
                plant.set_ground(roll=roll * DEG, pitch=pitch * DEG)
                h, rec, emu, _ = run_calib_tilt(plant)
                fw = h.fw
                self.assertEqual(len(emu.saved), 1)
                self.assertEqual(plant.limit_hits, [])
                phi = reference_roll(rec.ref_frame)
                expect_phi = {"tool 6 deg off horizontal": -6.0, "machine rolled 3 deg": 3.0}.get(label, 0.0)
                self.assertAlmostEqual(math.degrees(phi), expect_phi, delta=0.01)
                snap = emu.saved[0][1]
                identified = mount(snap, "y.imuMntOri_tilt")
                self.assertLess(np.abs(identified - compiled @ kin.Rx(-phi)).max(), 1e-4)
                # the valve identification does not care where the zero is
                for port in ("tiltPosi", "tiltNega"):
                    true = plant_speed_at_ref(plant, port)
                    self.assertAlmostEqual(snap[f"y.tblReqSpdToActCmd.{port}_X"][2], true, delta=1e-3 * true)
                kin.write_mounts(fw, {"imuTilt": identified})
                h.tick(100)
                self.assertAlmostEqual(fw["y.jnts.TiltMntToTilt.q"] - plant.q["tilt"], -phi, delta=0.01 * DEG)
                if expect_phi == 0.0:
                    self.assertLess(np.abs(identified - compiled).max(), 1e-4)
                else:
                    self.assertGreater(np.abs(identified - compiled).max(), 0.05, "the zero moved, silently")


# ==============================================================================================================
class TestCalibTiltFindings(unittest.TestCase):

    def test_the_units_own_valve_times_out_the_70_degree_leg_at_the_rest_pose_and_still_saves(self):
        # WHY: the plant here IS the compiled unit (valve, board) at the harness rest pose (tilt axis 21.9 deg below
        # horizontal, what an operator gets without re-posing the arm). The 70 deg gravity leg is 76.4 deg of joint
        # travel; the stored tiltNega speed at 70 % is 0.135 rad/s, so it needs ~10.4 s incl. the 1 s ramp.
        # FINDING: TiltNegaToPnt2 ends on the 10 s timeout (cnt > 1000, MdlApp.c:35493) with no alarm and the
        # calibration saves as usual. Harmless for the mount in a noise-free plant (any three distinct points on the
        # gravity circle give the axis; measured 1.6e-7) but the Pnt1-Pnt2 chord is shorter than designed.
        # FINDING: the stored tiltPosi_Y[1] = 19.5 is exactly what the staircase writes when motion starts on its
        # FIRST step (20 - 0.5); a deadband below 20 % cannot be identified -- the same 19.5 % valve reads 19.9 here
        # (onset needs 0.5 deg of creep at 20 %, 2 steps). Run time 128 s, 63 s of it the tiltNega staircase.
        plant = KinematicPlant()
        h, rec, emu, ticks = run_calib_tilt(plant)
        fw = h.fw
        d = rec.dwell()
        self.assertEqual(len(emu.saved), 1)
        self.assertEqual(h.curr_step(), "NoTarget")
        self.assertEqual(plant.limit_hits, [])
        snap = emu.saved[0][1]
        off = fw["par.parKin.angOutpLinkToTiltMnt"] + fw["par.parKin.angTiltMntToTilt"]
        pitch = -(plant.q["boom"] + plant.q["arm"] + kin.fourbar_output(fw, plant.q["input_link"]) + off)
        self.assertAlmostEqual(math.degrees(pitch), -21.9, delta=0.1)
        need = tilt_travel_for_leg(ANG_PNT2, pitch)
        self.assertLess(d["TiltPosiToPnt1"], TIMEOUT_TICKS - 1, "the 35 deg leg still ends on angle")
        self.assertEqual(d["TiltNegaToPnt2"], TIMEOUT_TICKS, "the 70 deg leg ends on the 10 s timeout")
        travelled = rec.tilt_at_entry("TiltNegaToPnt2") - rec.tilt_at_entry("TiltPnt2_stb")
        self.assertLess(travelled, need - 2.0 * DEG, f"{math.degrees(travelled):.1f} of {math.degrees(need):.1f} deg")
        self.assertLess(np.abs(mount(snap, "y.imuMntOri_tilt") - mount(fw, "par.imuTilt")).max(), 1e-4)
        stored = vlv.read_tables(fw)
        self.assertEqual(stored["tiltPosi"][1][1], 19.5)
        self.assertEqual(F32(STAIR_START - ONSET_DLY_CMP), 19.5)
        posi = snap["y.tblReqSpdToActCmd.tiltPosi_Y"][1]
        self.assertGreater(posi, 19.5 + 0.1, "the unit's own 19.5 % valve does not reproduce 19.5")
        self.assertLessEqual(posi, 19.5 + ONSET_DLY_CMP + 1e-4)
        nega = snap["y.tblReqSpdToActCmd.tiltNega_Y"][1]
        self.assertAlmostEqual(nega, stored["tiltNega"][1][1], delta=ONSET_DLY_CMP + 1e-4)
        for port in ("tiltPosi", "tiltNega"):
            true = plant_speed_at_ref(plant, port)
            self.assertAlmostEqual(snap[f"y.tblReqSpdToActCmd.{port}_X"][2], true, delta=1e-3 * true)
        self.assertGreater(d["TiltNegaMin"], 30 * STAIR_TICKS)
        self.assertGreater(ticks, 125 * 100)
        self.assertLess(ticks, 132 * 100)

    def test_accelerometer_noise_of_a_few_mg_makes_every_tilt_deadband_read_19_5(self):
        # FINDING (firmware robustness): the motion-onset detector compares ONE raw accelerometer sample against the
        # 1 s mean reference with a 0.5 deg threshold (chart_2291 l.163-166 raw acc, chart_2316 l.97-100, MdlApp.c:44276
        # 0.00872664619F): 0.5 deg = 8.7 mg perpendicular. With per-axis white noise sigma the per-sample false-onset
        # probability is exp(-(8.7 mg)^2 / (2 sigma^2)): 1 mg -> never, 5 mg -> 22 % per tick. Then the onset fires on
        # the staircase's first step and BOTH deadbands are saved as 20 - 0.5 = 19.5 %, whatever the valve. The mount
        # (1 s means) is barely affected. ASSUMPTION: white, isotropic noise (sil/plant.py acc_noise); real machine
        # vibration is coloured, but the detector has no filter so any broadband content above ~3 mg rms trips it.
        # (Measured on this plant with seed 3: 2 mg fires one step early, 22.1 / 20.9; 3 mg fires within 0.7 s.)
        results = {}
        for sigma in (0.001, 0.005):
            plant = KinematicPlant(q0=POSE, degrees=True, acc_noise=sigma, seed=7, **VALVE)
            h, rec, emu, _ = run_calib_tilt(plant)
            self.assertEqual(len(emu.saved), 1, sigma)
            snap = emu.saved[0][1]
            err = np.abs(mount(snap, "y.imuMntOri_tilt") - plant.hardware.mounts()["imuTilt"]).max()
            results[sigma] = (plant, snap, rec.dwell(), err)
        _, snap, d, mount_err = results[0.001]
        for port in ("tiltPosi", "tiltNega"):
            db = VALVE["deadband"][port]
            self.assertLessEqual(abs(snap[f"y.tblReqSpdToActCmd.{port}_Y"][1] - db), ONSET_DLY_CMP + 1e-4, port)
        self.assertLess(mount_err, 2e-3)
        plant, snap, d, mount_err = results[0.005]
        for port, state in (("tiltPosi", "TiltPosiMin"), ("tiltNega", "TiltNegaMin")):
            self.assertEqual(snap[f"y.tblReqSpdToActCmd.{port}_Y"][1], 19.5, port)
            self.assertLess(d[state], STAIR_TICKS, f"{state} ended on the first staircase step")
            true = plant_speed_at_ref(plant, port)
            self.assertAlmostEqual(snap[f"y.tblReqSpdToActCmd.{port}_X"][2], true, delta=5e-3 * true)
        self.assertLess(mount_err, 3e-3)

    def test_an_aborted_speed_leg_leaves_a_non_monotonic_table_that_the_next_save_persists(self):
        # FINDING (firmware): chart_2383 l.64-67 zeroes actSpdRef_TiltPosi on entering TiltPosiToPnt1 and chart_2338
        # l.44 writes X = [0, 0.01, max(MinTblReqSpd, peak)]. Pausing a few ticks into the leg (Auto Mode button)
        # leaves peak ~ 0, so X = [0, 0.01, 0.002]: X[2] < X[1]. MinTblReqSpd's own comment says it exists "to have
        # monotonous change of the input array" (SysPar.m:112) -- it was sized for the old 0.001 knee, the knee is now
        # the 0.01 literal. The abort raises no save, but the persistent tables survive and the next calibration of
        # ANY step saves them (AppCtrlIf.c:802-1012 copies all tables each calibrating tick): here step 28, which does
        # not move anything. Same pattern as test_calibration_entry.test_aborted_step28_value_is_persisted_by_the_next_save.
        plant = KinematicPlant(q0=POSE, degrees=True, strict_limits=True, **VALVE)
        emu = SaveHandshake()
        h = Harness(plant=[plant, emu]).reset().nominal_inputs()
        h.gnss_rtk_fixed()
        h.tick(3)
        fw = h.fw
        h.jump_to_step("CalibTilt")
        h.run_until(lambda h: h.calib_step() == "TiltPosiToPnt1", 100.0, "speed leg")
        h.tick(3)
        fw["u.jstAutoReq_StartPause"] = 1
        h.tick(1)
        fw["u.jstAutoReq_StartPause"] = 0
        h.tick(1)
        self.assertEqual(h.main_state(), "CalibTilt_Paused")
        self.assertEqual(h.calib_step(), "CalibStandby")
        self.assertEqual(h.valves(), {}, "the pause drops the valve at once, no ramp")
        self.assertEqual(emu.saved, [])
        X = fw["y.tblReqSpdToActCmd.tiltPosi_X"]
        self.assertEqual(X, [0.0, IDENTIFIED_X1, MIN_TBL_REQ_SPD])
        self.assertLess(X[2], X[1], "non-monotonic breakpoints")
        h.request_step("CalibForkRefPose")               # paused calibration steps switch directly (chart_2537)
        h.tick(1)
        self.assertEqual(h.main_state(), "CalibForkRefPose_Paused")
        h.pulse("u.jstAutoReq_StartPause")               # fork steps need the live swing switch: plant at swing 0
        h.run_until(lambda h: h.curr_step() == "NoTarget", 12.0, "step 28 done")
        self.assertEqual(len(emu.saved), 1)
        snap = emu.saved[0][1]
        self.assertEqual(snap["y.tblReqSpdToActCmd.tiltPosi_X"], [0.0, IDENTIFIED_X1, MIN_TBL_REQ_SPD])
        self.assertAlmostEqual(snap["y.tblReqSpdToActCmd.tiltPosi_Y"][1], 22.3, places=4)   # the aborted run's onset
        self.assertEqual(snap["y.tblReqSpdToActCmd.tiltNega_X"][2], F32(vlv.read_tables(fw)["tiltNega"][0][2]))
        self.assertLess(np.abs(mount(snap, "y.imuMntOri_tilt") - mount(fw, "par.imuTilt")).max(), 1e-6)


if __name__ == "__main__":
    unittest.main()
