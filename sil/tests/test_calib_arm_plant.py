"""
CalibArm (AutoCtrlStep 23) end to end: the compiled X1Exc firmware calibrates sil.plant.KinematicPlant's arm, and every
number it saves is checked against what the plant really is.

THE SEQUENCE (chart_1210 CalibStepMgr, states 204-248; generated MdlApp.c:25608-26482, entered from Standby at :32276)
  ArmPntRef_stb 8 s -> ArmPntRef_log 1 s -> ArmInMin (valve staircase) -> ArmInMin_stb 8 s -> ArmInToPnt1 (armIn 70 % until
  the arm gravity vector turned > AngArmPnt1 = 20 deg, or 10 s) -> ArmPnt1_stb 8 s -> ArmPnt1_log 1 s -> ArmOutMin ->
  ArmOutMin_stb 8 s -> ArmOutToPnt2 (armOut 70 %, > AngArmPnt2 = 40 deg from Pnt1, or 10 s) -> ArmPnt2_stb 8 s ->
  ArmPnt2_log 1 s -> Arm_save -> CalibStandby.  Guards: T559/T564 [isMotionOnset.armIn/armOut] (MdlApp.c:25656, :25862),
  T533/T547 [angCalib.arm > AngArmPnt1/2 || cnt > CntCalib_timeout] (MdlApp.c:25823, :26040); SysPar.m:102-121.

WHAT THE FIRMWARE IDENTIFIES, AND FROM WHAT
  minimum command  _Min: armIn/armOut = 20 % + 0.2 % every 200 ticks, raw (not smoothed) (chart_3055 l.53-99, 127-134,
                   196-197; SysPar.m:105, 110, 170-171). Onset = |angle(accRef, accRaw)| > 0.5 deg since the _Min entry
                   (chart_2316 l.74-75, 91-104; chart_2291 l.147-153, CalcAccVecAngle l.489-496; SysPar.m:131), a rising
                   edge. On the onset tick the table stores propVlvCmd - 0.5 (chart_2338 l.59-60, SysPar.m:134).
  speed            peak |y.cyls.arm.spd| -- the firmware's CYLINDER STROKE speed J(q_fw) * qDot_fw -- over the _ToPnt leg
                   (chart_2383 l.56-58, 81-82; actSpdArm = fabsf(MdlApp_Y.cyls.arm.spd), MdlApp.c:45060), saved as X[2]
                   next to Y[2] = PropVlvRefCmd = 70 % and the fixed knee X[1] = 0.01 (chart_2338 l.93-96).
  IMU mount        rebuilt on the tick leaving ArmPnt2_log from three 1 s accelerometer means (chart_2291 l.275-301):
                   vy = -(v1 x v2) (pin axis), vz = -(vy x accRef), vx = vy x vz. Only raw accelerometers enter. Its
                   output starts from parLocalTest.imuArm (one-time init, MdlApp.c:42964-42972).
  Nothing is fed back: the arm attitude reads parLocalTest.imuArm (MdlApp.c:41930-41934) and the valve map reads
  parLocalTest.reqSpdToActCmd (MdlApp.c:50340ff); the results only reach the NVM image (AppCtrlIf.c:802-1012, while
  isCalibrating). Tests that "load" a result write it into par.* themselves, standing for a reflash.

THE PLANT THIS FILE CALIBRATES (every choice is a test fixture, not firmware data)
  UNIT_VALVE   armIn deadband 23.1 % / vmax 0.11 m/s, armOut 22.39 % / 0.14 m/s. GUESS, chosen so that (1) neither equals
               the stored table (armIn 32.5 % / 0.417, armOut 31.5 % / 0.5 with Y[2] 80, ECR88D_ShortArm.m:322-325), so a
               pass cannot be an echo; (2) the staircase is short (10 s per 1 % above 20 %); (3) the post-leg coast stays
               below the arm stop from a plumb start (at the stored 0.417 m/s it reaches 155 deg, test_valve_plant);
               (4) the plant's 0.5 deg crossing falls well inside a staircase step (asserted, not assumed).
  UNIT_BOARD   the unit's arm IMU board = compiled par.imuArm @ Rx(6 deg) @ Rz(-4 deg). GUESS: a plausible crooked board,
               so the stored mount is another unit's and the rebuilt one can be told apart from it.
  DECK_POSTURE the deck's arm procedure (resources/Sensor_and_Valve_Calibration_ProductionV1.md slides 8-9: "Pitch angle
               > 5 deg, Roll = 0", "Use the laser level to check arm joints position (vertical)"): undercarriage pitched
               6 deg, boom -31, arm 115, so boom + arm + pitch = 90 deg and the arm chord hangs plumb. ASSUMPTION: URDF pitch
               +6 deg; the deck's sign convention is not in any source, and CalibArm only sees the arm's attitude to gravity
               (TestReferencePostureIsPartOfTheMount), so the sign does not matter here. Nose-up with a plumb arm would need
               arm 127 deg and leave too little room to the 155 deg OEM-TEAM stop.
  Accelerometer -1 g along world up and no link accelerations (plant defaults; the sign is pinned in
               test_valve_plant.test_accelerometer_sign_is_minus_one_g). Strict joint stops in every run.
  u.isMachCalib = 1: ASSUMPTION that the tablet raises the service-mode flag while calibrating (question for David,
               test_calibration_entry); on a healthy machine it blocks nothing (asserted).

FINDINGS PINNED HERE
  F1 The rebuilt mount encodes the arm's attitude at ArmPntRef: it is the true mount turned so that link x points along
     gravity. Off plumb by delta at the reference -> saved mount = true @ Ry(delta) -> once loaded the arm reads delta high,
     forever, with no alarm. The laser-level plumb in the deck is load-bearing; squaring the arm to the jacked-up chassis
     (or trusting the tablet's angle) costs the jack-up pitch.
  F2 A backwards-plumbed arm valve calibrates without complaint into mount @ diag(1, -1, -1): once loaded the firmware reads
     the arm mirrored about the reference posture AND its arm rate sign follows the backwards valve, so its loops would
     converge onto the mirrored geometry. No guard checks the direction (all angle tests use |angle|).
  F3 The onset detector compares single raw accelerometer samples against 0.5 deg (8.7 mg). At 5 mg rms per axis the first
     staircase sample fires it: both minima are saved as 19.5 % = PropVlvCmdInitOffs - 0.5 while the valve never opened.
  F4 Arm_save rewrites every OTHER axis's table into the calibration shape: knee 0.001 -> 0.01, Y[2] -> PropVlvRefCmd (boom
     80 -> 70 %, travel 100 -> 60 %, rotator 80 -> 100 %), blade -> identity with a 0.01 % deadband (chart_2338 l.68-116
     has no per-axis gating). Latent in this build (par.* is what runs), live the day the NVM tables are used.
  F5 A unit that matches its stored arm deadbands spends ~4 of its ~5 minutes of CalibArm in the two staircases.

TICK BOOKKEEPING (Recorder): row t is recorded inside the plant call before MdlApp_step() number t, after the plant integrated:
  plant state in row t is what step t reads; firmware outputs in row t were written by step t-1.

Run:  cd xpanner-sim && python3 -m unittest sil.tests.test_calib_arm_plant -v        (~25 s, seven CalibArm runs)
"""
import math
import unittest

import numpy as np

from sil import kinematics as kin
from sil import valves as vlv
from sil.harness import Harness, SaveHandshake
from sil.plant import KinematicPlant

DEG = math.pi / 180.0


def f32(x):
    return float(np.float32(x))


# -- firmware constants (compiled values; SysPar.m line numbers) ---------------------------------------------------------
STB_TICKS = 801             # CntCalib_stb 800 (:102): entry tick + 800 during ticks (test_calibration_entry STB_TICKS)
LOG_TICKS = 101             # CntCalib_log 100 (:103)
TIMEOUT_TICKS = 1002        # CntCalib_timeout 1000 (:104), [cnt > 1000]
SAVE_TICKS = 2              # Arm_save with the main.c handshake: request on the first during tick, ack one step later
STAIR_TICKS = 200           # CntCalibStepFindingMin (:105)
STAIR_FIRST = 199           # the entry tick counts as the first of 200 (test_calibration_entry, staircase test)
STAIR_START = 20.0          # PropVlvCmdInitOffs.armIn/armOut (:170-171)
STAIR_PCT = 0.2             # StepFindingMin_size (:110)
ONSET = 0.5 * DEG           # AngMotionOnsetThld (:131)
ONSET_CMP = 0.5             # PropVlvCmdMotionOnsetDlyCmp (:134)
REF_CMD = 70.0              # PropVlvRefCmd.armIn/armOut (:147-148)
ANG_PNT1, ANG_PNT2 = 20.0 * DEG, 40.0 * DEG     # AngArmPnt1/2 (:120-121)
IDENTIFIED_X1 = f32(0.01)   # chart_2338 l.93-96
FIXED_TICKS = 5 * STB_TICKS + 3 * LOG_TICKS + SAVE_TICKS     # 4310 ticks = 43.1 s of unconditional dwell

ARM_SEQUENCE = ("ArmPntRef_stb", "ArmPntRef_log", "ArmInMin", "ArmInMin_stb", "ArmInToPnt1", "ArmPnt1_stb",
                "ArmPnt1_log", "ArmOutMin", "ArmOutMin_stb", "ArmOutToPnt2", "ArmPnt2_stb", "ArmPnt2_log", "Arm_save",
                "CalibStandby")

# chart_2338 l.68-116: what EVERY saved table's Y[2] becomes (PropVlvRefCmd, SysPar.m:136-156), blade excepted.
PROP_VLV_REF_CMD = dict(trvlLeFwd=60.0, trvlLeRev=60.0, trvlRiFwd=60.0, trvlRiRev=60.0, swingLe=60.0, swingRi=60.0,
                        bm1Up=70.0, bm1Down=70.0, bm2Up=60.0, bm2Down=60.0, armIn=70.0, armOut=70.0, linkIn=70.0,
                        linkOut=70.0, tiltPosi=70.0, tiltNega=70.0, rotPosi=100.0, rotNega=100.0)

# -- the plant (module docstring, THE PLANT THIS FILE CALIBRATES) ----------------------------------------------------------
UNIT_DEADBAND = {"armIn": 23.1, "armOut": 22.39}          # GUESS, see docstring
UNIT_VMAX = {"armIn": 0.11, "armOut": 0.14}               # GUESS, see docstring
UNIT_BOARD = kin.Rx(6.0 * DEG) @ kin.Rz(-4.0 * DEG)        # GUESS: unit board = compiled mount @ UNIT_BOARD
JACK_UP_PITCH = 6.0 * DEG                                 # deck "Pitch angle > 5 deg"; sign ASSUMPTION (docstring)
DECK_POSTURE = dict(boom=-31.0, arm=115.0)                 # boom + arm + pitch = 90 deg: arm chord plumb


def mount(src, prefix):
    return np.array([[src[f"{prefix}.a{r}{c}"] for c in (1, 2, 3)] for r in (1, 2, 3)], dtype=float)


def compiled_arm_mount():
    return mount(Harness().reset().fw, "par.imuArm")


def plumb_error(R_link):
    """delta such that the gravity direction, seen in the link frame and projected onto its x-z plane, is Ry(delta) @ x.
    That is the direction CalibArm's rebuild takes as link x (chart_2291 l.279-297 with accRef = gravity): for
    vy = y, vz = -(y x g) = (-g_z, 0, g_x), vx = y x vz = (g_x, 0, g_z). Accelerometer -1 g: accRef = world down."""
    g = np.asarray(R_link).T @ -kin.UP
    return math.atan2(-g[2], g[0])


class Recorder:
    """Plant listed right after the KinematicPlant: one row per tick (see TICK BOOKKEEPING)."""

    def __init__(self, plant):
        self.plant = plant
        self.cols = {k: [] for k in ("t", "step", "cmd_in", "cmd_out", "q", "speed", "y1_in", "y1_out", "mount")}

    def __call__(self, h):
        fw, c = h.fw, self.cols
        c["t"].append(h.tick_count)
        c["step"].append(h.calib_step())
        c["cmd_in"].append(fw["y.propVlvCmd.armIn"])
        c["cmd_out"].append(fw["y.propVlvCmd.armOut"])
        c["q"].append(self.plant.q["arm"])
        c["speed"].append(self.plant.speeds["arm"])
        c["y1_in"].append(fw["y.tblReqSpdToActCmd.armIn_Y"][1])
        c["y1_out"].append(fw["y.tblReqSpdToActCmd.armOut_Y"][1])
        c["mount"].append(tuple(fw[f"y.imuMntOri_arm.a{r}{k}"] for r in (1, 2, 3) for k in (1, 2, 3)))


class ArmRun:
    """One CalibArm run and its record. Index helpers take ROW TICKS (Recorder.t)."""

    def __init__(self, h, plant, rec, emu, ref_frame, inhibit_before):
        self.h, self.fw, self.plant, self.emu = h, h.fw, plant, emu
        self.ref_frame = ref_frame              # plant arm frame at the reference posture (world <- link)
        self.inhibit_before = inhibit_before    # inhibit bits in NoTarget just before the step jump
        c = rec.cols
        self.t = np.array(c["t"])
        self.step = c["step"]
        self.cmd = {"armIn": np.array(c["cmd_in"]), "armOut": np.array(c["cmd_out"])}
        self.q = np.array(c["q"])
        self.speed = np.array(c["speed"])
        self.y1 = {"armIn": np.array(c["y1_in"]), "armOut": np.array(c["y1_out"])}
        self.mounts = c["mount"]
        self.entry = {}
        for i, s in enumerate(self.step):
            if s in ARM_SEQUENCE and s not in self.entry and (s != "CalibStandby" or "ArmPntRef_stb" in self.entry):
                self.entry[s] = int(self.t[i])
        self.snap = emu.saved[0][1] if emu.saved else None

    def i(self, tick):
        return int(tick - self.t[0])

    def dwell(self, state):
        return self.entry[self.next_state(state)] - self.entry[state]

    @staticmethod
    def next_state(state):
        return ARM_SEQUENCE[ARM_SEQUENCE.index(state) + 1]

    def first_tick_after(self, start, pred):
        for k in range(self.i(start), len(self.t)):
            if pred(k):
                return int(self.t[k])
        raise AssertionError(f"predicate never true after tick {start}")

    def table(self, port):
        return self.snap[f"y.tblReqSpdToActCmd.{port}_X"], self.snap[f"y.tblReqSpdToActCmd.{port}_Y"]

    def identified_mount(self):
        return mount(self.snap, "y.imuMntOri_arm")


def calibrate_arm(plant, ground=None, timeout_s=600.0):
    """Boot healthy in NoTarget with `plant` publishing every sensor, jump to CalibArm, run to NoTarget (strict stops)."""
    plant.strict_limits = True
    if ground:
        plant.set_ground(**ground)
    rec, emu = Recorder(plant), SaveHandshake()
    h = Harness(plant=[plant, rec, emu]).reset()
    h.nominal_inputs()
    h.gnss_rtk_fixed()
    h.fw["u.isMachCalib"] = 1
    h.tick(3)
    inhibit_before = h.inhibit_names()
    ref_frame = plant.frames["arm"].copy()
    h.jump_to_step("CalibArm")
    h.run_until(lambda h: h.curr_step() == "NoTarget", timeout_s, "CalibArm back in NoTarget")
    h.tick(1)
    return ArmRun(h, plant, rec, emu, ref_frame, inhibit_before)


def unit_plant(**kw):
    args = dict(q0=DECK_POSTURE, degrees=True, deadband=UNIT_DEADBAND, vmax=UNIT_VMAX)
    args.update(kw)
    return KinematicPlant(**args)


def settle_at(h, plant, q_arm, ticks=300):
    """Put the plant arm at q_arm (at rest) and let the firmware's 3 Hz joint LPF settle (chart_2143 l.188)."""
    plant.q["arm"] = q_arm
    plant.qdot["arm"] = 0.0
    plant.invalidate()
    h.tick(ticks)
    return h.fw["y.jnts.Bm2ToArm.q"]


# =======================================================================================================================
class TestCalibArmOnTheDeckPosture(unittest.TestCase):
    """The deck procedure on a unit whose valve and arm IMU board are NOT what the firmware stores."""

    @classmethod
    def setUpClass(cls):
        cls.compiled = compiled_arm_mount()
        cls.unit_mount = cls.compiled @ UNIT_BOARD
        plant = unit_plant(hardware={"imuArm": cls.unit_mount})
        cls.arm = run = calibrate_arm(plant, ground=dict(pitch=JACK_UP_PITCH))
        h, fw = run.h, run.fw
        cls.stored = {p: (fw[f"par.reqSpdToActCmd.{p}_X"], fw[f"par.reqSpdToActCmd.{p}_Y"]) for p in vlv.PORTS}
        cls.inhibit_after = h.inhibit_names()
        cls.calibrating_after = fw["y.isCalibrating"]
        # static reads with the STORED (compiled) mount, then with the identified one loaded (a reflash)
        poses = (60.0 * DEG, 150.0 * DEG)
        cls.err_stored = [settle_at(h, plant, q) - q for q in poses]
        kin.write_mounts(fw, {"imuArm": run.identified_mount()})
        cls.err_loaded = [settle_at(h, plant, q) - q for q in poses]
        cls.links_arm_err = np.abs(np.array(fw["y.links.arm.R"]).reshape(3, 3, order="F") - plant.frames["arm"]).max()

    def test_runs_the_13_substates_in_order_and_takes_109_s(self):
        # WHY: success = calibStep back at CalibStandby and autoCtrl_CurrStep back at NoTarget (spec A7 step 5), and the
        # machine time is the budget an Isaac run has to pay: 43.1 s of unconditional dwell + the two staircases + legs.
        # FW: chart_1210 transitions (module docstring); save handshake main.c:394-419 (SaveHandshake), Arm_save exit on
        # hasChanged(isCalibDataSaved) && isCalibDataSaved (MdlApp.c:25628-25640, '<S179>:346').
        run = self.arm
        self.assertEqual(run.inhibit_before, ["BIT_NO_TARGET"], "isMachCalib = 1 blocks nothing on a healthy machine")
        self.assertEqual([s for s in ARM_SEQUENCE if s in run.entry], list(ARM_SEQUENCE))
        self.assertEqual(sorted(run.entry.values()), [run.entry[s] for s in ARM_SEQUENCE], "entered in order, once")
        for s in ("ArmPntRef_stb", "ArmInMin_stb", "ArmPnt1_stb", "ArmOutMin_stb", "ArmPnt2_stb"):
            self.assertEqual(run.dwell(s), STB_TICKS, s)
        for s in ("ArmPntRef_log", "ArmPnt1_log", "ArmPnt2_log"):
            self.assertEqual(run.dwell(s), LOG_TICKS, s)
        self.assertEqual(run.dwell("Arm_save"), SAVE_TICKS)
        for s in ("ArmInToPnt1", "ArmOutToPnt2"):
            self.assertLess(run.dwell(s), TIMEOUT_TICKS, f"{s} ended on angle")
        # completion
        self.assertEqual(run.h.calib_step(), "CalibStandby")
        self.assertEqual(run.h.curr_step(), "NoTarget")
        self.assertFalse(self.calibrating_after)
        self.assertEqual(self.inhibit_after, ["BIT_NO_TARGET"])
        self.assertEqual(len(run.emu.saved), 1)
        self.assertEqual(run.emu.reloads, 1)
        self.assertEqual(run.emu.saved[0][0], run.entry["Arm_save"] + 1, "request raised on the first during tick")
        # machine time = fixed dwell + staircases + legs. Observed 4310 + 3488 + 2633 + 195 + 308 = 10934 ticks = 109.3 s
        # (ArmPntRef_stb entry to CalibStandby); NoTarget and isCalibrating = false follow one tick later.
        variable = sum(run.dwell(s) for s in ("ArmInMin", "ArmOutMin", "ArmInToPnt1", "ArmOutToPnt2"))
        total = run.entry["CalibStandby"] - run.entry["ArmPntRef_stb"]
        self.assertEqual(total, FIXED_TICKS + variable)
        self.assertTrue(10500 < total < 11500, total)

    def test_staircase_is_raw_20_percent_plus_0p2_every_200_ticks_and_the_plant_waits_for_its_own_deadband(self):
        # WHY: the _Min result can only mean "the plant's deadband" if the firmware commands exactly the documented
        # staircase and the plant does not move before the staircase reaches its valve's deadband.
        # FW: chart_3055 l.53-66 (cnt resets on step change, step += 1 every CntCalibStepFindingMin), l.76-92
        # (cmdStep = single(step) * 0.2 + 20), l.196-197 (isFindingMotionOnset -> raw command, no cubic smoothing).
        run = self.arm
        for port, state in (("armIn", "ArmInMin"), ("armOut", "ArmOutMin")):
            with self.subTest(port=port):
                start, end = run.entry[state], run.entry[state + "_stb"]
                cmd = run.cmd[port]
                other = run.cmd["armOut" if port == "armIn" else "armIn"]
                for t in range(start, end):
                    k = 0 if t - start < STAIR_FIRST else 1 + (t - start - STAIR_FIRST) // STAIR_TICKS
                    self.assertEqual(cmd[run.i(t)], f32(np.float32(k) * np.float32(STAIR_PCT) + np.float32(STAIR_START)),
                                     f"tick {t}")
                    self.assertEqual(other[run.i(t)], 0.0)
                q0 = run.q[run.i(start)]
                db = UNIT_DEADBAND[port]
                first_open = run.first_tick_after(start, lambda k: cmd[k] >= db)
                # row t: the plant has already integrated the command step t-1 wrote (TICK BOOKKEEPING)
                np.testing.assert_array_equal(run.q[run.i(start):run.i(first_open)], q0)
                self.assertNotEqual(run.q[run.i(first_open)], q0, "moves on the first command at its deadband")

    def test_identified_minimum_is_the_staircase_command_at_the_plants_half_degree_crossing(self):
        # WHY: the core identification. The plant's arm first turns 0.5 deg away from the logged reference on a known
        # tick; the firmware must see it on that very step (the IMU is read and the table updated in the same step),
        # leave _Min one step later (Delay1 on isMotionOnset), and store the command of that tick minus 0.5 %.
        # Deadband here: 23.1 / 22.39 % (stored 32.5 / 31.5): what comes back is the plant, not the table.
        # FW: chart_2316 l.74-75 + DetectAngMotion l.91-104; chart_2338 l.59-60; chart_1210 T559/T564 via Delay1
        # (MdlApp.c:25656, :25862). Gravity angle == arm travel here: the pin axis is horizontal (pitch only).
        run = self.arm
        for port, state, y1_stored in (("armIn", "ArmInMin", 32.5), ("armOut", "ArmOutMin", 31.5)):
            with self.subTest(port=port):
                start = run.entry[state]
                q0 = run.q[run.i(start)]
                cross = run.first_tick_after(start, lambda k: abs(run.q[k] - q0) > ONSET)
                cmd = run.cmd[port]
                c = cmd[run.i(cross)]
                # precondition: the crossing is not on a staircase edge (the plant integrates the command of step t-1)
                self.assertTrue(all(cmd[run.i(cross) + d] == c for d in (-2, -1, 0, 1)), "crossing on a step edge")
                self.assertEqual(run.y1[port][run.i(cross)], f32(y1_stored), "before the onset: stored value")
                self.assertEqual(run.y1[port][run.i(cross) + 1], f32(np.float32(c) - np.float32(ONSET_CMP)))
                self.assertEqual(run.entry[state + "_stb"], cross + 2)
                identified = run.table(port)[1][1]
                self.assertEqual(identified, run.y1[port][run.i(cross) + 1], "the saved value")
                db = UNIT_DEADBAND[port]
                self.assertGreaterEqual(identified + ONSET_CMP, db - 1e-5)
                self.assertLessEqual(identified + ONSET_CMP, db + 2 * STAIR_PCT + 1e-5, "within two staircase steps")
                self.assertGreater(y1_stored - identified, 9.0, "not the stored value")
        # observed: armIn 22.9 (crossing 87 ticks into the 23.4 % step), armOut 22.1 (into the 22.6 % step): the
        # quasi-static plant needs 0.5 deg at ~X[1] = 0.001 m/s, i.e. 1-2 steps above the deadband, and the fixed -0.5 %
        # compensation then lands 0.2-0.3 % BELOW the true deadband.
        self.assertAlmostEqual(run.table("armIn")[1][1], 22.9, places=4)
        self.assertAlmostEqual(run.table("armOut")[1][1], 22.1, places=4)

    def test_identified_speed_is_the_plant_stroke_speed_at_the_reference_command(self):
        # WHY: X[2] is what the auto valve map would scale every arm request with. The plant's stroke speed at 70 % is
        # armIn: vmax 0.11 (Y[2] of the plant table is 70, so 70 % is the top of its line); armOut: its line at 70 %
        # (Y[2] 80). The firmware measures J(q_fw) * qDot_fw through its STORED mount, i.e. another unit's: the board
        # error biases qDot (gyro projected through the wrong mount) and J (static arm error, -0.06..-0.36 deg below)
        # -- observed -0.35 % / +0.05 % for this board; a 10 deg pitch error costs 17 % (test_valve_plant). Tolerance 1 %.
        # FW: chart_2383 l.56-58, 81-82, MdlApp.c:45060; CalStrkAndSpd MdlApp.c:13453-13483; chart_2338 l.93-96.
        run = self.arm
        for port, state in (("armIn", "ArmInToPnt1"), ("armOut", "ArmOutToPnt2")):
            with self.subTest(port=port):
                X, Y = run.plant.tables[port]
                truth = vlv.port_speed(REF_CMD, X, Y, deadband=UNIT_DEADBAND[port], vmax=UNIT_VMAX[port])
                seg = run.speed[run.i(run.entry[state]):run.i(run.entry[run.next_state(state)])]
                # precondition: the lagged valve settled at 70 % before the leg ended (1 s ramp + tau 0.1 s GUESS)
                self.assertAlmostEqual(np.abs(seg).max(), truth, delta=1e-4 * truth, msg="the plant reached it")
                Xs, Ys = run.table(port)
                self.assertEqual((Xs[0], Xs[1], Ys[0], Ys[2]), (0.0, IDENTIFIED_X1, 0.0, REF_CMD))
                self.assertAlmostEqual(Xs[2], truth, delta=0.01 * truth)
                self.assertGreater(abs(Xs[2] - self.stored[port][0][2]), 0.25, "not the stored speed")
        self.assertAlmostEqual(run.table("armIn")[0][2], 0.11, delta=0.0011)
        self.assertAlmostEqual(vlv.port_speed(REF_CMD, *run.plant.tables["armOut"], deadband=22.39, vmax=0.14),
                               0.001 + (70.0 - 22.39) * 0.139 / (80.0 - 22.39), delta=1e-9)   # 0.11587 m/s

    def test_legs_end_on_the_gravity_angle_one_step_after_the_plant_passes_it(self):
        # WHY: a leg that ends on the 10 s timeout still "succeeds" (MdlApp.c:25823 angle OR cnt), so the exit must be
        # shown to be the angle. angCalib.arm is measured from accRef for leg 1 and from accPnt1 for leg 2 (chart_2291
        # l.147-153); the guard reads it through Delay4 (MdlApp.c:25823, :26040). The arm then coasts through the 1 s
        # cubic ramp-down (chart_3055 l.194-243): observed 9.4 deg after leg 1, 6.3 deg after leg 2 at these speeds.
        run = self.arm
        for leg, state, ref_state, thld in (("in", "ArmInToPnt1", "ArmPntRef_log", ANG_PNT1),
                                            ("out", "ArmOutToPnt2", "ArmPnt1_log", ANG_PNT2)):
            with self.subTest(leg=leg):
                q_ref = run.q[run.i(run.entry[ref_state])]
                np.testing.assert_array_equal(run.q[run.i(run.entry[ref_state]):run.i(run.entry[ref_state]) + LOG_TICKS],
                                              q_ref, "the arm is at rest while the reference is logged")
                start = run.entry[state]
                cross = run.first_tick_after(start, lambda k: abs(run.q[k] - q_ref) > thld)
                nxt = run.next_state(state)
                self.assertEqual(run.entry[nxt], cross + 2)
                coast = abs(run.q[run.i(run.entry[nxt]) + STB_TICKS - 1] - run.q[run.i(cross)])
                self.assertTrue(5.0 * DEG < coast < 10.0 * DEG, math.degrees(coast))
        self.assertGreater(run.q[run.i(run.entry["ArmPnt1_log"])], run.q[0], "armIn raised the arm angle")
        # top of the leg-1 coast: 144.4 deg, 10.6 deg below the 155 deg OEM-TEAM stop (strict stops would have raised)
        self.assertLess(run.q.max(), 145.0 * DEG)

    def test_rebuilt_mount_is_the_units_board_not_the_stored_one(self):
        # WHY: the non-echo mount check. The output starts as the stored (compiled) mount, stays bit-identical until the
        # tick leaving ArmPnt2_log, and then equals the UNIT's board -- the plant published with a mount 0.107 away from
        # the stored one. Loaded into par, the arm reads the plant to 3e-5 deg and y.links.arm.R matches the plant frame.
        # FW: chart_2291 l.275-301 (rebuild on calibStep_prev == ArmPnt2_log), init MdlApp.c:42964-42972, attitude
        # MdlApp.c:41930-41934.
        run = self.arm
        ident = run.identified_mount()
        self.assertLess(np.abs(ident - self.unit_mount).max(), 1e-6)
        self.assertGreater(np.abs(ident - self.compiled).max(), 0.1, "the stored mount is another unit's")
        np.testing.assert_allclose(ident @ ident.T, np.eye(3), atol=1e-6)
        k = run.i(run.entry["Arm_save"])
        before = {run.mounts[j] for j in range(run.i(run.entry["ArmPntRef_stb"]), k)}
        self.assertEqual(len(before), 1)
        np.testing.assert_array_equal(np.array(before.pop()).reshape(3, 3), self.compiled.astype(np.float32))
        np.testing.assert_array_equal(np.array(run.mounts[k]).reshape(3, 3), ident)
        # the arm reading: wrong through the stored mount away from plumb, right once the result is loaded
        self.assertGreater(abs(self.err_stored[1]), 0.3 * DEG)
        self.assertGreater(abs(self.err_stored[0]), 0.04 * DEG)
        for e in self.err_loaded:
            self.assertLess(abs(e), 1e-3 * DEG)
        self.assertLess(self.links_arm_err, 2e-6)

    def test_save_rewrites_every_other_axis_table_into_the_calibration_shape(self):
        # FINDING F4. GenCorrTblSetReqSpdToActCmd writes all 20 tables every tick with no per-axis gating: X = [0, 0.01,
        # peak or stored X(3)], Y = [0, onset or stored Y(2), PropVlvRefCmd]; travel X = [0, 0.01, 100]; blade = identity
        # [0 0.01 100]/[0 0.01 100]. AppCtrlIf.c:884-1012 copies all of them to the actuator NVM image while
        # isCalibrating. So a CalibArm save changes the boom's top command 80 -> 70 %, travel 100 -> 60 %, rotator 80 ->
        # 100 %, the blade deadband 25 -> 0.01 %, and every knee 0.001 -> 0.01 (the min-speed hold, chart_2463 l.50-69,
        # would then creep 10x faster). Latent: this build's valve map reads parLocalTest (MdlApp.c:50340ff), unchanged.
        run = self.arm
        for port in vlv.PORTS:
            if port in ("armIn", "armOut"):
                continue
            with self.subTest(port=port):
                (X, Y), (Xs, Ys) = self.stored[port], run.table(port)
                self.assertEqual(Xs[1], IDENTIFIED_X1)
                self.assertNotEqual(Xs[1], X[1])
                self.assertEqual(run.fw[f"par.reqSpdToActCmd.{port}_Y"], Y, "par untouched")
                if port.startswith("blade"):
                    self.assertEqual((list(Xs), list(Ys)), ([0.0, IDENTIFIED_X1, 100.0], [0.0, IDENTIFIED_X1, 100.0]))
                    continue
                self.assertEqual(Ys[1], Y[1], "untouched onset command")
                self.assertEqual(Ys[2], PROP_VLV_REF_CMD[port])
                self.assertEqual(Xs[2], 100.0 if port.startswith("trvl") else X[2])
        changed = {p: (self.stored[p][1][2], run.table(p)[1][2]) for p in ("bm1Up", "trvlLeFwd", "rotPosi")}
        self.assertEqual(changed, {"bm1Up": (80.0, 70.0), "trvlLeFwd": (100.0, 60.0), "rotPosi": (80.0, 100.0)})
        self.assertEqual((self.stored["bladeUp"][1][1], run.table("bladeUp")[1][1]), (25.0, IDENTIFIED_X1))


# =======================================================================================================================
class TestReferencePostureIsPartOfTheMount(unittest.TestCase):
    """FINDING F1: the rebuilt mount is the true one turned until link x points along gravity AT ArmPntRef."""

    def assertMountOffBy(self, run, delta):
        true = run.plant.hardware.mounts()["imuArm"]
        self.assertLess(np.abs(run.identified_mount() - true @ kin.Ry(delta)).max(), 2e-4)

    def test_arm_off_plumb_at_the_reference_becomes_a_constant_arm_angle_offset(self):
        # WHY: the deck's "laser level ... arm joints position (vertical)" is the only thing that ties the mount to the
        # arm. Start level with boom -40, arm 100: the arm chord is 30 deg short of plumb. The run completes and saves
        # like the plumb one; the saved mount is true @ Ry(30 deg), and once loaded the firmware reads EVERY arm angle
        # 30 deg high (R_link_fw = R_link @ Ry(delta)) -- a constant offset, no alarm, no inhibit.
        # FW: chart_2291 l.279-297 (vz = -(vy x accRef) makes link z horizontal at the reference); attitude
        # MdlApp.c:41930-41934; joint angle = euAng(2) difference, chart_2143 l.31.
        plant = unit_plant(q0=dict(boom=-40.0, arm=100.0), hardware=None)
        run = calibrate_arm(plant)
        delta = plumb_error(run.ref_frame)
        self.assertAlmostEqual(delta, 30.0 * DEG, delta=1e-9)
        self.assertEqual(len(run.emu.saved), 1)
        self.assertEqual(run.h.curr_step(), "NoTarget")
        self.assertMountOffBy(run, delta)
        kin.write_mounts(run.fw, {"imuArm": run.identified_mount()})
        for q in (100.0, 60.0, 140.0):
            with self.subTest(arm=q):
                self.assertAlmostEqual(settle_at(run.h, plant, q * DEG) - q * DEG, delta, delta=0.01 * DEG)
        self.assertEqual(run.h.inhibit_names(), ["BIT_NO_TARGET"])

    def test_arm_square_to_the_jacked_up_chassis_is_off_plumb_by_the_jack_up_pitch(self):
        # WHY: jacked up for the calibration (pitch 6 deg, here also rolled 4 deg against the deck's "Roll = 0"), an arm
        # set square to the chassis (boom + arm = 90 deg, e.g. checked against the upper frame instead of with the laser
        # plumb) hangs 6 deg off plumb. The saved mount absorbs it: true @ Ry(-6.01 deg). Roll only enters through the gravity
        # projection, delta = -atan(tan(pitch) / cos(roll)) = -6.0145 deg (-0.0145 deg for 4 deg of roll): the deck's
        # "Roll = 0" is second order for the arm, the plumb line is first order.
        plant = unit_plant(q0=dict(boom=-31.0, arm=121.0), hardware=None)
        run = calibrate_arm(plant, ground=dict(pitch=JACK_UP_PITCH, roll=4.0 * DEG))
        delta = plumb_error(run.ref_frame)
        self.assertAlmostEqual(delta, -math.atan(math.tan(JACK_UP_PITCH) / math.cos(4.0 * DEG)), delta=1e-9)
        self.assertAlmostEqual(math.degrees(delta), -6.0145, delta=1e-3)
        self.assertEqual(len(run.emu.saved), 1)
        self.assertMountOffBy(run, delta)


# =======================================================================================================================
class TestReversedArmPlumbing(unittest.TestCase):

    def test_backwards_arm_valve_calibrates_into_a_mirrored_mount_and_a_consistent_rate_sign(self):
        # FINDING F2. Plumb the arm valve backwards (plant axis_sign: armIn now LOWERS the arm angle). Nothing in CalibArm
        # checks the direction: the legs test |angle| (CalcAccVecAngle, chart_2291 l.489-496), the speed is |cyls.arm.spd|
        # (chart_2383 l.36). The valve numbers come out as for the correct plumbing, and the rebuild's vy = -(v1 x v2)
        # (chart_2291 l.279) flips with the direction of travel, so the saved mount is true @ diag(1, -1, -1) (a
        # 180 deg turn about link x). Loaded, the firmware reads q_fw = 2 q_ref - q (mirror about the plumb reference)
        # and its arm rate is POSITIVE while armIn physically lowers the arm: the firmware's model "armIn raises q"
        # (valves.py AXIS_PORTS) holds again, so a closed loop would converge -- onto the mirrored geometry.
        plant = unit_plant(axis_sign={"arm": -1.0}, hardware=None)
        run = calibrate_arm(plant, ground=dict(pitch=JACK_UP_PITCH))
        fw = run.fw
        self.assertEqual(len(run.emu.saved), 1)
        self.assertEqual(run.h.curr_step(), "NoTarget")
        self.assertLess(run.q[run.i(run.entry["ArmPnt1_log"])], run.q[0], "precondition: armIn lowered the arm")
        for port in ("armIn", "armOut"):
            X, Y = run.table(port)
            truth = vlv.port_speed(REF_CMD, *plant.tables[port], deadband=UNIT_DEADBAND[port], vmax=UNIT_VMAX[port])
            self.assertAlmostEqual(X[2], truth, delta=0.01 * truth, msg=port)
            self.assertTrue(UNIT_DEADBAND[port] <= Y[1] + ONSET_CMP <= UNIT_DEADBAND[port] + 0.4 + 1e-5, (port, Y[1]))
        true = plant.hardware.mounts()["imuArm"]
        self.assertLess(np.abs(run.identified_mount() - true @ np.diag([1.0, -1.0, -1.0])).max(), 2e-4)
        q_ref = run.q[0]
        kin.write_mounts(fw, {"imuArm": run.identified_mount()})
        for q in (90.0 * DEG, 135.0 * DEG):
            self.assertAlmostEqual(settle_at(run.h, plant, q), 2 * q_ref - q, delta=0.01 * DEG)
        plant.strict_limits = False
        plant.manual = {"armIn": 45.0}
        run.h.tick(100)
        plant.manual = {}
        self.assertLess(plant.qdot["arm"], -0.05)
        self.assertGreater(fw["y.jnts.Bm2ToArm.qDot"], 0.05)
        self.assertAlmostEqual(fw["y.jnts.Bm2ToArm.qDot"], -plant.qdot["arm"], delta=0.02 * abs(plant.qdot["arm"]))
        self.assertGreater(fw["y.cyls.arm.spd"], 0.0, "the firmware sees its cylinder extending, as armIn should")


# =======================================================================================================================
class TestAccelerometerNoise(unittest.TestCase):
    """FINDING F3. The onset detector has no filter: one raw sample against 0.5 deg of gravity angle (8.73 mg).

    With white noise sigma per axis the two components across gravity make the sample's angle Rayleigh-distributed:
    P(angle > 0.5 deg) per tick = exp(-(8.73 mg)^2 / (2 sigma^2)); the 1 s reference mean is 10x quieter and ignored here.
      sigma 1 mg: 3e-17 per tick -- never in a calibration.   sigma 5 mg: 0.22 per tick -- P(no onset in 50 ticks) = 4e-6.
    The noise is ASSUMPTION-level: the IMU's own filtering before CAN is not in any source; engine vibration on an arm is
    commonly tens of mg. Seeds are fixed; the bounds above say why the assertions do not depend on them."""

    def test_5mg_fires_the_onset_on_the_first_staircase_samples_and_saves_19p5_percent(self):
        # WHY: the same unit as the deck run, plus accelerometer noise. The valve never opens (20 % < 23.1 / 22.39 %), the
        # arm never moves, and both minima are still "identified" -- as 19.5 %, the value that also comes out of a valve
        # whose deadband is below 20 % (test_valve_plant CalibRot). Observed dwell 1 and 3 ticks. Nothing flags it.
        # FW: rawMotion = |angCalib - angCalibRef| > thld on the current sample (chart_2316 l.97-100); angCalib.arm from
        # the raw accelerometer of this tick (chart_2291 l.54, 147-153); stored value chart_2338 l.59-60.
        plant = unit_plant(hardware=None, acc_noise=5e-3, seed=11)
        run = calibrate_arm(plant, ground=dict(pitch=JACK_UP_PITCH))
        self.assertEqual(len(run.emu.saved), 1)
        for port, state in (("armIn", "ArmInMin"), ("armOut", "ArmOutMin")):
            with self.subTest(port=port):
                self.assertLessEqual(run.dwell(state), 50)
                seg = slice(run.i(run.entry[state]), run.i(run.entry[state + "_stb"]))
                self.assertEqual(set(run.cmd[port][seg]), {STAIR_START})
                self.assertEqual(len(set(run.q[seg])), 1, "the valve never opened: 20 % < its deadband")
                self.assertEqual(run.table(port)[1][1], STAIR_START - ONSET_CMP)
                # the speed comes from gyros and survives
                truth = vlv.port_speed(REF_CMD, *plant.tables[port], deadband=UNIT_DEADBAND[port], vmax=UNIT_VMAX[port])
                self.assertAlmostEqual(run.table(port)[0][2], truth, delta=0.01 * truth)
        # the mount is built from 1 s means and is only mildly disturbed (observed 2e-4 with this seed; 0.012 with another
        # seed at another posture while exploring -- hence the loose bound)
        err = np.abs(run.identified_mount() - plant.hardware.mounts()["imuArm"]).max()
        self.assertLess(err, 0.05)

    def test_1mg_leaves_the_minimum_command_inside_the_staircase_window(self):
        # Control for the test above: no false onset, but the noise can move the 0.5 deg crossing by a step (observed armIn
        # 22.9 as without noise, armOut 21.9 instead of 22.1: onset on the first open step). Inside the two-step window.
        plant = unit_plant(hardware=None, acc_noise=1e-3, seed=11)
        run = calibrate_arm(plant, ground=dict(pitch=JACK_UP_PITCH))
        self.assertEqual(len(run.emu.saved), 1)
        for port in ("armIn", "armOut"):
            y1 = run.table(port)[1][1]
            self.assertTrue(UNIT_DEADBAND[port] <= y1 + ONSET_CMP <= UNIT_DEADBAND[port] + 0.4 + 1e-5, (port, y1))
        self.assertLess(np.abs(run.identified_mount() - plant.hardware.mounts()["imuArm"]).max(), 0.01)


# =======================================================================================================================
class TestStoredValveTiming(unittest.TestCase):

    def test_a_unit_matching_its_stored_arm_deadbands_spends_most_of_calibarm_in_the_staircase(self):
        # FINDING F5 / run-time budget. The plant's arm valve opens at the STORED deadbands (32.5 / 31.5 %: no deadband
        # override); vmax stays at UNIT_VMAX (the stored 0.417 m/s coasts into the 155 deg stop from a plumb start).
        # Each _Min state climbs from 20 % to one or two steps above the deadband: 10 s of machine time per 1 %.
        # Observed: onset at 32.8 / 31.8 % (64 / 59 steps), ArmInMin 128.7 s, ArmOutMin 118.0 s, whole run 295.1 s,
        # 84 % of it in the two staircases; saved 32.3 / 31.3 %, i.e. 0.2 % below the valve (same bias as above).
        # The stored values sit on the grid a CalibArm result lands on, 20 + 0.2 k - 0.5 (32.5 = k 65, 31.5 = k 60) --
        # as do swingRi 30.5, linkIn 30.5, linkOut 31.5, tiltNega 25.7 and tiltPosi 19.5 (the floor: onset on the first
        # staircase sample), while swingLe 32, bm1 32/33, rot 17, travel 35/36 are round numbers. An inference about the
        # table's history, asserted only for the arm; the knee X[1] = 0.001 is not calibration-shaped.
        plant = unit_plant(deadband=None, hardware=None)
        run = calibrate_arm(plant, ground=dict(pitch=JACK_UP_PITCH))
        self.assertEqual(len(run.emu.saved), 1)
        self.assertEqual(run.h.curr_step(), "NoTarget")
        for port, state, stored in (("armIn", "ArmInMin", 32.5), ("armOut", "ArmOutMin", 31.5)):
            with self.subTest(port=port):
                self.assertEqual(plant.tables[port][1][1], stored, "precondition: the plant valve is the stored one")
                y1 = run.table(port)[1][1]
                self.assertTrue(stored <= y1 + ONSET_CMP <= stored + 2 * STAIR_PCT + 1e-5, y1)
                steps = round((y1 + ONSET_CMP - STAIR_START) / STAIR_PCT)
                self.assertAlmostEqual((y1 + ONSET_CMP - STAIR_START) / STAIR_PCT, steps, delta=1e-4)
                # onset during step `steps`, which starts STAIR_FIRST + 200 (steps - 1) ticks after entry; +2 for the
                # same-step detection and the Delay1 on the transition
                self.assertTrue(STAIR_FIRST + STAIR_TICKS * (steps - 1) < run.dwell(state)
                                <= STAIR_FIRST + STAIR_TICKS * steps + 2, (steps, run.dwell(state)))
                k_stored = (stored + ONSET_CMP - STAIR_START) / STAIR_PCT
                self.assertAlmostEqual(k_stored, round(k_stored), delta=1e-9)
        stair = run.dwell("ArmInMin") + run.dwell("ArmOutMin")
        total = run.entry["CalibStandby"] - run.entry["ArmPntRef_stb"]
        self.assertEqual(total, FIXED_TICKS + stair + run.dwell("ArmInToPnt1") + run.dwell("ArmOutToPnt2"))
        self.assertTrue(28500 < total < 30500, total)
        self.assertGreater(stair / total, 0.8)


if __name__ == "__main__":
    unittest.main()
