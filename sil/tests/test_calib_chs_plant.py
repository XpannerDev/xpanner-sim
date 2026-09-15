"""
CalibChs (AutoCtrlStep 20) end to end: the compiled X1Exc firmware calibrates the swing valves and the chassis IMU
mount of sil.plant.KinematicPlant, on a jacked-up undercarriage and, as the control, on level ground.

WHAT THE STEP DOES (chart_1210 CalibStepMgr, state labels; guards generated in MdlApp.c:28725-29700)
    ChsPntRef_stb 8 s -> ChsPntRef_log 1 s (accRef)
    ChsLeMin      swingLe staircase 20 % + 0.2 % per 2 s until a motion onset, no timeout (MdlApp.c:28768-28770)
    ChsLeMin_stb  8 s
    ChsLeMoveToPnt1  swingLe at PropVlvRefCmd 60 % until angCalib.chs > 170 deg or 10 s (MdlApp.c:28887)
    ChsPnt1_stb 8 s -> ChsPnt1_log 1 s (accPnt1)
    ChsRiMin (onset only, no timeout, MdlApp.c:29448-29449) / ChsRiMin_stb / ChsRiMoveToPnt2 (angCalib.chs < 80 deg
    or 10 s, MdlApp.c:29662)
    ChsPnt2_stb 8 s -> ChsPnt2_log 1 s (accPnt2) -> Chs_save -> CalibStandby -> NoTarget
  angCalib.chs = |planar angle of the RAW chassis accelerometer sample vs accRef| in the board plane normal to accRef's
  largest axis (chart_2291 l.107-129; accRaw is the mirrored inport with no filter, MdlApp.c:11102-11105). The onset is
  a > 0.5 deg change of that angle within the _Min state (chart_2316 l.91-104, SysPar.m:131). The mount is rebuilt from
  the three 1 s means: vz = (accPnt1 - accRef) x (accPnt2 - accRef), vy = accRef x vz, vx = vy x vz (chart_2291
  l.191-217). The identified table is X = [0, 0.01, peak |y.jnts.ChsToUc.qDot| during the _ToPnt leg], Y = [0, onset
  command - 0.5, 60] (chart_2383 l.75-76 with MdlApp.c:42071/45131; chart_2338 l.53-54, 78-81).

THE PLANT IS NOT THE STORED PARAMETER SET (so a pass is identification, not echo)
  UNIT_MOUNT     the plant's chassis board = compiled par.imuChs @ Rz(25 deg) @ Ry(2 deg). ASSUMPTION: an invented unit,
                 25 deg of yaw and 2 deg of tilt away from the compiled calibration; the tilt is kept below the 6 deg
                 jack-up (a board tilt larger than the ground tilt puts the planar-angle origin outside the gravity
                 circle and angCalib.chs never reaches 170 deg -- measured at 1 deg pitch: the Le leg times out).
  PLANT_DEADBAND swingLe 23.0 / swingRi 21.8 %, PLANT_VMAX 0.42 / 0.36 rad/s. GUESS: invented, deliberately not the
                 stored table (32.0 / 30.5 %, 0.5889 / 0.5196 rad/s, ECR88D_ShortArm.m:310-313) and above the 20 %
                 staircase start so identification has to climb.
  Accelerometer  quasi-static: gravity only, no centripetal or tangential term (sil/plant.py docstring, Accelerometer).
                 Sign kinematics.ACC_SIGN = -1 g. ASSUMPTION: inferred from the compiled mounts, which CalibArm / Link /
                 Chs / Tilt rebuild only with -1 g at natural reference poses, on the premise that the par.imu*
                 literals are field calibration results (test_valve_plant.test_accelerometer_sign_is_minus_one_g).
  Jack-up        6 deg nose UP = URDF pitch -6 deg ("Roll = 0, Pitch > 5 deg", CLAUDE.md step table / deck). The
                 firmware alone only fixes that ONE lean direction at the reference pose rebuilds the unit's mount
                 (TestJackUpDirection); that this direction is nose up rests on the -1 g sign above -- with +1 g it is
                 nose down.
  isMachCalib    set to 1 in every run. ASSUMPTION: the tablet holds the service-mode flag during calibration (open
                 question for David, test_calibration_entry.test_calib_inhibit_mask_only_bites_when_isMachCalib_is_set);
                 with it set, any in-mask inhibit bit during the run would stop the calibration, and none occurs.
  Accelerometer noise (plant acc_noise, white, per axis): 1 mg is ASSUMPTION (order of a MEMS part's broadband noise; the
                 unit's IMU part and filtering are not in any source), 30 mg is GUESS (engine vibration, the level
                 test_calibration_entry uses).

RESULTS (measured on this build, pinned below). Rotation angles use a well-conditioned atan2 (rot_deg).
  Jacked up 6 deg nose up, noise free: completes in 11,276 ticks = 112.8 s, of which the two staircases are 56.7 s.
    swingLe onset 23.4 % -> Y[1] 22.9 (plant 23.0); swingRi 22.2 % -> 21.7 (plant 21.8); peak speeds 0.41974 / 0.35978
    = plant vmax x cos(2 deg): the firmware reads the swing rate through its STORED mount. Mount = UNIT_MOUNT to 1.9e-6
    (25.08 deg from the compiled one). Loading it corrects the firmware's chassis attitude from 4.56 deg to 2e-5 deg off
    and the boom angle from 4.33 deg to 1e-5 deg off. The house ends 88.4 deg left of where it started.
  At the stored deadbands (32 / 30.5 %) the same run takes 28,880 ticks = 288.8 s; the staircases are 232.7 s of it.
  Jack-up direction decides the mount, silently: nose DOWN gives UNIT_MOUNT @ Rz(180), roll +-6 gives Rz(+-90) (with the
    plant's -1 g); with +1 g nose up gives Rz(180) and nose down gives UNIT_MOUNT.
  Level ground, noise-free quasi-static accelerometer: never leaves ChsLeMin. The staircase reaches the plant deadband
    after 30 s and the house swings on with a rising command (no timeout; the limit is 100 %, SysPar.m:109), under the
    normal calibration beacon: after 100 s in ChsLeMin, 30 % and 158 deg of swing. This needs the idealised
    accelerometer: at 0.01 / 0.03 mg rms all 3 seeds still run away (probe), at 0.1 mg 1 of 3 leaves, at 0.2 mg all 3
    leave within 0.5 s with the valve still closed (pinned).
  Level ground, 1 mg noise: completes and saves in 53 s with the STORED mount bit-identical (spec A7's "silently
    unchanged"), both deadbands 19.5 % (noise onsets, the valves never opened), swingRi X = [0, 0.01, 0.002]
    (MinTblReqSpd; breakpoints no longer monotonic), and the house left ~234 deg from where it started.
  Jacked up with 1 mg noise: both staircases fire on noise (19.5 %) while the mount stays within 0.005-0.076 deg; with
    30 mg the mount is off by 11-34 deg AND the swingRi speed is corrupted (0.002 / 0.356 / 0.313 against 0.36): the
    noisy planar angle ends the Ri leg after 9-127 ticks instead of 495.

FINDINGS FOR THE FIRMWARE OWNER (classification; the report to David takes these, not the plant numbers)
  firmware_defect (the source shows it, any plant)
    - ChsLeMin / ChsRiMin have no timeout and no plausibility exit: only isMotionOnset (one raw accelerometer sample)
      or the shared abort leaves them; the gyro swing rate and the GNSS swing estimate are not consulted; the staircase
      limit is 100 %.
    - The rebuilt mount's rotation about the swing axis is set by the lean direction of gravity at the reference pose
      (vy = accRef x vz); any other lean saves a mount turned by that direction (180 / +-90 deg) and nothing checks it
      (SanityCheckImuCalib generates no code).
    - _ToPnt legs accept the 10 s timeout exactly like reaching 170 / 80 deg, and ChsRiMoveToPnt2's "< 80 deg" is
      already true when the Le leg did not pass 80 deg, so that leg exits on its first evaluation.
    - A leg whose peak is below 0.01 rad/s saves X[2] = max(0.002, peak) below the fixed X[1] = 0.01 knee, i.e.
      non-monotonic breakpoints: MinTblReqSpd exists "to have monotonous change" (SysPar.m:112) but the knee is the
      0.01 literal (chart_2338 l.34-35, 78-81). The level-ground and 30 mg runs below save such a table.
    - Cosmetic: ChsRiMin sets calibStep in its during action, so ChsPnt1_log reads 102 ticks.
  procedure_dependency
    - Roll = 0 / Pitch > 5 deg is not checked; on level ground the |v1 x v2| > 1e-6 gate keeps the stored mount without
      a word and the run still saves.
    - The identified swing speed is read through the stored mount; exact only after reload and a second run.
  plant_dependent
    - The level-ground runaway (needs < ~0.1-0.2 mg rms and no centripetal / tangential term).
    - That nose UP is the right jack-up (ACC_SIGN = -1 ASSUMPTION).
    - Noise onsets at 19.5 % (1 mg, 0.2 mg) and the 30 mg mount / speed errors (the unit's IMU noise and filtering are
      in no source).

Run: cd xpanner-sim && python3 -m unittest sil.tests.test_calib_chs_plant -v     (~55 s; the firmware runs ~5000 ticks/s)
"""
import collections
import math
import unittest

import numpy as np

from sil import kinematics as kin
from sil import valves as vlv
from sil.harness import Harness, SaveHandshake
from sil.plant import DT, Hardware, KinematicPlant

DEG = math.pi / 180.0


def f32(x):
    return float(np.float32(x))


SWING_PORTS = ("swingLe", "swingRi")
CALIB_INHIBIT_MASK = 13976                  # isCalibInhibited = (status & 13976) != 0 & isMachCalib, MdlApp.c:39679
STB_TICKS = 801                             # CntCalib_stb 800 (SysPar.m:102); entry tick + 800 during ticks
LOG_TICKS = 101                             # CntCalib_log 100 (SysPar.m:103)
TIMEOUT_TICKS = 1002                        # _ToPnt [... || cnt > CntCalib_timeout 1000] (SysPar.m:104)
STAIR_START = 20.0                          # PropVlvCmdInitOffs.swingLe/Ri (SysPar.m:164-165)
STAIR_STEP = 0.2                            # StepFindingMin_size (SysPar.m:110)
STAIR_TICKS = 200                           # CntCalibStepFindingMin (SysPar.m:105)
ONSET_DLY_CMP = 0.5                         # PropVlvCmdMotionOnsetDlyCmp (SysPar.m:134)
PROP_VLV_REF_CMD = 60.0                     # PropVlvRefCmd.swingLe/Ri (SysPar.m:141-142)
MIN_TBL_REQ_SPD = f32(0.002)                # SysPar.m:112
IDENTIFIED_X1 = f32(0.01)                   # chart_2338 l.78-81
NO_MOTION_ONSET = f32(STAIR_START - ONSET_DLY_CMP)   # 19.5: an onset on the staircase's first step

EXPECTED_SUBSTEPS = ["CalibStandby", "ChsPntRef_stb", "ChsPntRef_log", "ChsLeMin", "ChsLeMin_stb", "ChsLeMoveToPnt1",
                     "ChsPnt1_stb", "ChsPnt1_log", "ChsRiMin", "ChsRiMin_stb", "ChsRiMoveToPnt2", "ChsPnt2_stb",
                     "ChsPnt2_log", "Chs_save", "CalibStandby"]

PLANT_DEADBAND = {"swingLe": 23.0, "swingRi": 21.8}     # GUESS, see module docstring
PLANT_VMAX = {"swingLe": 0.42, "swingRi": 0.36}         # GUESS, see module docstring
JACK_UP_PITCH_DEG = -6.0                                # nose up, URDF pitch convention


# ------------------------------------------------------------------------------------------------------------------
def compiled_chs_mount():
    return Hardware.compiled(Harness().fw).mounts()["imuChs"]


def unit_mount():
    """ASSUMPTION (module docstring): the plant's chassis board, 25 deg yaw and 2 deg tilt off the compiled mount."""
    return compiled_chs_mount() @ kin.Rz(25.0 * DEG) @ kin.Ry(2.0 * DEG)


def unit_plant(pitch_deg=JACK_UP_PITCH_DEG, roll_deg=0.0, deadband=None, acc_noise=0.0, seed=0, acc_sign=kin.ACC_SIGN):
    plant = KinematicPlant(deadband=dict(deadband or PLANT_DEADBAND), vmax=dict(PLANT_VMAX),
                           hardware={"imuChs": unit_mount()}, acc_noise=acc_noise, seed=seed, strict_limits=True,
                           acc_sign=acc_sign)
    plant.set_ground(roll=roll_deg * DEG, pitch=pitch_deg * DEG)
    return plant


def mount(src, prefix):
    return np.array([[src[f"{prefix}.a{r}{c}"] for c in (1, 2, 3)] for r in (1, 2, 3)], dtype=float)


def table(src, prefix, port):
    return list(src[f"{prefix}.{port}_X"]), list(src[f"{prefix}.{port}_Y"])


def rot_deg(A, B):
    """Angle of the rotation A^T B, degrees, as atan2(|skew part|, (trace - 1) / 2). WHY not acos((trace - 1) / 2):
    that is ill-conditioned near 0 and, on float32 firmware matrices (~1e-7 per element), bottoms out at ~0.03 deg --
    it printed 0.0 and 0.019 deg for rotations this form measures as 0.005 and 2e-5 deg."""
    R = A.T @ B
    s = math.hypot(R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]) / 2.0
    return math.degrees(math.atan2(s, (np.trace(R) - 1.0) / 2.0))


def mat(fw, path):
    return np.array(fw[path], dtype=float).reshape(3, 3, order="F")       # MATLAB column-major


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Recorder:
    """Listed after the KinematicPlant: sees the outputs of the previous MdlApp_step and the plant state that step's
    command produced."""

    def __init__(self, plant):
        self.plant = plant
        self.entries = []                           # (tick, calibStep name, plant swing rad) at each change
        self.inhibit_or = 0
        self.save_req_ticks = 0
        self.swing_travel = 0.0                     # signed, unwrapped, rad (plant truth)
        self.cab_alarm_or_led = False
        self.ticks_in = collections.Counter()
        self.ext_alarm_on = collections.Counter()

    def __call__(self, h):
        fw = h.fw
        s = h.calib_step()
        if not self.entries or self.entries[-1][1] != s:
            self.entries.append((h.tick_count, s, self.plant.q["swing"]))
        self.inhibit_or |= int(fw["y.autoCtrl_InhibitSts"])
        self.save_req_ticks += bool(fw["y.isCalibDataSaveReq"])
        self.swing_travel += self.plant.qdot["swing"] * DT
        self.cab_alarm_or_led |= bool(fw["y.cabAlarmCmd"]) or bool(fw["y.cabWarningLedCmd"])
        self.ticks_in[s] += 1
        self.ext_alarm_on[s] += bool(fw["y.extAlarmCmd"])

    def names(self):
        return [e[1] for e in self.entries]

    def dwell(self):
        """{calibStep: ticks} from each entry to the next (CalibStandby: the first visit)."""
        out = {}
        for a, b in zip(self.entries, self.entries[1:]):
            out.setdefault(a[1], b[0] - a[0])
        return out

    def swing_at(self, name):
        return next(e[2] for e in self.entries if e[1] == name)

    def in_state_for(self, name, ticks):
        return lambda h: self.entries[-1][1] == name and h.tick_count - self.entries[-1][0] >= ticks


def run_calib_chs(plant, timeout_s=400.0, stop=None, setup=None):
    """Boot with the plant publishing every sensor (house at swing 0: switch closed, swing latch set), isMachCalib = 1,
    tablet step 20 + Auto press from NoTarget, run to NoTarget (or until stop(rec)(h)). setup(fw) runs after the reset
    and before the first tick (patch par.* there). Returns (h, rec, emu)."""
    rec, emu = Recorder(plant), SaveHandshake()
    h = Harness(plant=[plant, rec, emu]).reset()
    if setup is not None:
        setup(h.fw)
    h.nominal_inputs()
    h.gnss_rtk_fixed()
    h.fw["u.isMachCalib"] = 1
    h.tick(3)
    h.jump_to_step("CalibChs")
    h.run_until(stop(rec) if stop else (lambda h: h.curr_step() == "NoTarget"), timeout_s, "CalibChs")
    return h, rec, emu


def staircase_steps(identified_min):
    """Staircase steps k whose command 20 + 0.2 k was the onset command recorded as identified_min + 0.5."""
    return round((identified_min + ONSET_DLY_CMP - STAIR_START) / STAIR_STEP)


# ==================================================================================================================
class TestJackedUpNoseUp(unittest.TestCase):
    """The procedure as the deck describes it: undercarriage pitched 6 deg nose up, noise-free plant."""

    @classmethod
    def setUpClass(cls):
        cls.plant = plant = unit_plant()
        h, cls.rec, cls.emu = run_calib_chs(plant)
        fw = h.fw
        cls.final_step, cls.calibrating = h.curr_step(), bool(fw["y.isCalibrating"])
        cls.compiled = mount(fw, "par.imuChs")
        cls.unit = plant.hardware.mounts()["imuChs"]
        cls.stored = {p: table(fw, "par.reqSpdToActCmd", p) for p in SWING_PORTS}
        cls.snap = cls.emu.saved[0][1] if cls.emu.saved else None
        cls.fw_swing_end = -fw["y.jnts.ChsToUc.q"]
        cls.swing_end = plant.q["swing"]
        # What the NVM reload would do: the identified mount into par.imuChs, then read the same pose again.
        cls.att_err_before = rot_deg(mat(fw, "y.links.chs.R"), plant.R_chs)
        cls.boom_err_before = fw["y.jnts.BmMntToBm1.q"] - plant.q["boom"]
        if cls.snap is not None:
            kin.write_mounts(fw, {"imuChs": mount(cls.snap, "y.imuMntOri_chs")})
            h.tick(300)
        cls.att_err_after = rot_deg(mat(fw, "y.links.chs.R"), plant.R_chs)
        cls.boom_err_after = fw["y.jnts.BmMntToBm1.q"] - plant.q["boom"]

    def test_completes_every_substep_in_order_and_saves_once(self):
        # WHY: the success signature a tablet sees (calibStep back to CalibStandby, CurrStep NoTarget) must come with one
        # save and with the machine having done both legs. isMachCalib = 1 makes CALIB_INHIBIT_MASK bite; a healthy plant
        # raises none of its bits over 113 s of swinging on a tilted undercarriage.
        # FW: sub-step order chart_1210 (MdlApp.c:28725-29700); main chart [calibStep == CalibStandby] -> NoTarget
        # through Delay5 (MdlApp.c:19014 for the fork step, same pattern); save handshake main.c:394-419.
        self.assertIsNotNone(self.snap, "no save request was raised")
        self.assertEqual(self.rec.names(), EXPECTED_SUBSTEPS)
        self.assertEqual(self.final_step, "NoTarget")
        self.assertFalse(self.calibrating)
        self.assertEqual(len(self.emu.saved), 1)
        self.assertEqual(self.emu.reloads, 1)
        self.assertEqual(self.rec.inhibit_or & CALIB_INHIBIT_MASK, 0, hex(self.rec.inhibit_or))
        self.assertFalse(self.rec.cab_alarm_or_led)
        self.assertEqual(self.plant.limit_hits, [])
        # The procedure leaves the house off square: Pnt2 is 80 deg of planar gravity angle plus the 1 s ramp-down coast
        # (measured 88.37 deg; the firmware's own GNSS swing estimate reads 88.55 -- the chassis Z it rotates about is
        # misread through the not-yet-reloaded mount). 88.4 is a regression pin (measured), not a physics bound.
        self.assertAlmostEqual(math.degrees(self.swing_end), 88.4, delta=1.0)
        self.assertAlmostEqual(self.fw_swing_end, self.swing_end, delta=0.5 * DEG)

    def test_run_time_is_dominated_by_the_two_deadband_staircases(self):
        # WHY: budget. Every fixed state is exact to the tick; the _Min dwell is set by how far above 20 % the plant's
        # deadband sits (0.2 % per 2 s), plus the travel needed for a 0.5 deg planar-angle change.
        # QUIRK: ChsPnt1_log reads 102 ticks: ChsRiMin assigns calibStep in its DURING action, not its entry (chart_1210
        # state SSID 183 label; MdlApp.c:29018-29020 entry sets only cnt = 0), so the first ChsRiMin tick still reports
        # ChsPnt1_log. Its only effect here is one more accPnt1 sample (chart_2291 l.63) with the valve still closed.
        d = self.rec.dwell()
        for s in ("ChsPntRef_stb", "ChsLeMin_stb", "ChsPnt1_stb", "ChsRiMin_stb", "ChsPnt2_stb"):
            self.assertEqual(d[s], STB_TICKS, s)
        self.assertEqual(d["ChsPntRef_log"], LOG_TICKS)
        self.assertEqual(d["ChsPnt1_log"], LOG_TICKS + 1)
        self.assertEqual(d["ChsPnt2_log"], LOG_TICKS)
        self.assertEqual(d["Chs_save"], 2)
        self.assertLess(d["ChsLeMoveToPnt1"], TIMEOUT_TICKS, "Le leg ended on the 10 s timeout, not on 170 deg")
        self.assertLess(d["ChsRiMoveToPnt2"], TIMEOUT_TICKS, "Ri leg ended on the 10 s timeout, not on 80 deg")
        for port, state in (("swingLe", "ChsLeMin"), ("swingRi", "ChsRiMin")):
            Y1 = self.snap[f"y.tblReqSpdToActCmd.{port}_Y"][1]
            k = staircase_steps(Y1)
            self.assertAlmostEqual(STAIR_START + STAIR_STEP * k, Y1 + ONSET_DLY_CMP, delta=1e-4, msg="off the staircase grid")
            self.assertGreaterEqual(d[state], STAIR_TICKS * k - 2, state)
            self.assertLessEqual(d[state], STAIR_TICKS * (k + 1), state)
        start = next(t for t, s, _ in self.rec.entries if s == "ChsPntRef_stb")
        total = self.rec.entries[-1][0] - start
        self.assertEqual(total, sum(v for k, v in d.items() if k != "CalibStandby"))
        # measured: 11,276 ticks = 112.8 s; ChsLeMin 3,460 + ChsRiMin 2,214 ticks
        self.assertGreater(total, 10800)
        self.assertLess(total, 11800)
        self.assertGreater((d["ChsLeMin"] + d["ChsRiMin"]) / total, 0.45)

    def test_identified_onset_command_brackets_the_plant_deadband_per_direction(self):
        # WHY: the table's Y[1] is what auto control commands for the slowest swing; it must be the PLANT's valve. The
        # plant's valve is closed below its deadband (valves.port_speed), so an onset command below it cannot be
        # motion; above it the axis starts at X[1] = 0.001 rad/s and needs a few 0.2 % steps to turn the gravity vector's
        # planar angle by 0.5 deg. Measured: swingLe onset 23.4 % (plant 23.0) -> 22.9; swingRi 22.2 % (21.8) -> 21.7.
        # The exact (22.9, 21.7) pin is a regression pin (measured, deterministic); the bracket is the physics check.
        # FW: onset chart_2316 l.68-69 / l.91-104; stored value propVlvCmd - 0.5 (chart_2338 l.53-54, SysPar.m:134).
        ids = {}
        for port in SWING_PORTS:
            X, Y = table(self.snap, "y.tblReqSpdToActCmd", port)
            db = PLANT_DEADBAND[port]
            onset = Y[1] + ONSET_DLY_CMP
            with self.subTest(port=port):
                self.assertGreaterEqual(onset, db - 1e-4, "onset below the plant deadband cannot be motion")
                self.assertLessEqual(onset, db + 5 * STAIR_STEP + 1e-4, "more than 5 staircase steps late")
                self.assertGreater(abs(Y[1] - (self.stored[port][1][1] - ONSET_DLY_CMP)), 5.0, "stored table echoed")
                self.assertNotEqual(Y[1], NO_MOTION_ONSET)
            ids[port] = Y[1]
        self.assertAlmostEqual(ids["swingLe"] - ids["swingRi"], PLANT_DEADBAND["swingLe"] - PLANT_DEADBAND["swingRi"],
                               delta=0.4, msg="per-direction deadbands not resolved")
        self.assertEqual((ids["swingLe"], ids["swingRi"]), (f32(22.9), f32(21.7)))
        # the house really moved inside each staircase, in the leg's direction (swingLe = CCW = +swing)
        le = wrap(self.rec.swing_at("ChsLeMin_stb") - self.rec.swing_at("ChsLeMin"))
        ri = wrap(self.rec.swing_at("ChsRiMin_stb") - self.rec.swing_at("ChsRiMin"))
        self.assertTrue(0.1 * DEG < le < 2.0 * DEG, math.degrees(le))
        self.assertTrue(-2.0 * DEG < ri < -0.1 * DEG, math.degrees(ri))

    def test_identified_speed_is_the_plant_speed_at_60_percent_read_through_the_stored_mount(self):
        # WHY: X[2] is the speed auto control expects at 60 %. The stored swing tables have Y[2] = 60 = PropVlvRefCmd, so
        # the plant's true speed at the reference command is its vmax. The firmware's peak is |y.jnts.ChsToUc.qDot| =
        # chassis gyro z through the STORED mount (chart_2143 l.277, MdlApp.c:42071 -> chart_2383 l.33, 75-76): it reads
        # (M_stored^T M_unit)[2,2] = cos(2 deg) of the true rate, 0.06 % low (0.41974 / 0.35978). The next test shows the
        # reloaded mount removes it.
        scale = (self.compiled.T @ self.unit)[2, 2]
        self.assertAlmostEqual(scale, math.cos(2.0 * DEG), places=5)
        for port in SWING_PORTS:
            X, Y = table(self.snap, "y.tblReqSpdToActCmd", port)
            with self.subTest(port=port):
                self.assertEqual((X[0], X[1], Y[0], Y[2]), (0.0, IDENTIFIED_X1, 0.0, PROP_VLV_REF_CMD))
                true = vlv.port_speed(PROP_VLV_REF_CMD, *self.plant.tables[port], deadband=PLANT_DEADBAND[port],
                                      vmax=PLANT_VMAX[port])
                self.assertEqual(true, PLANT_VMAX[port])
                self.assertAlmostEqual(X[2], true * scale, delta=1e-5)
                self.assertLess(abs(X[2] - true) / true, 1e-3)
                self.assertGreater(abs(X[2] - self.stored[port][0][2]), 0.1, "stored table echoed")

    def test_identified_mount_is_the_units_board_not_the_stored_one(self):
        # WHY: the chassis mount turns every IMU reading of the chassis into link attitude, and the chassis is the base of
        # every joint angle (the joint angle is an IMU Euler difference). Three gravity vectors on a cone about the tilted
        # swing axis give that axis exactly (cross of two chords), and accRef x axis gives chassis Y when the reading's
        # horizontal part at the reference pose points along chassis -X -- with the plant's -1 g sign (ASSUMPTION, module
        # docstring) that is a pure nose-up pitch -- so the rebuild is exact for ANY board orientation, which is why a
        # unit 25 deg away comes back to float32 resolution (measured 1.9e-6).
        # FW: chart_2291 l.191-217 (MdlApp.c:43585ff), gated on |v1 x v2| > 1e-6; NVM copy AppCtrlIf.c:802-813.
        M = mount(self.snap, "y.imuMntOri_chs")
        self.assertLess(np.abs(M - self.unit).max(), 1e-5)
        self.assertAlmostEqual(rot_deg(self.compiled, M), rot_deg(self.compiled, self.unit), delta=0.01)
        self.assertGreater(rot_deg(self.compiled, M), 20.0, "precondition: not the stored mount")
        np.testing.assert_allclose(M @ M.T, np.eye(3), atol=1e-5)
        self.assertAlmostEqual(np.linalg.det(M), 1.0, places=5)

    def test_loading_the_identified_mount_corrects_the_chassis_attitude_and_the_boom_angle(self):
        # WHY: the point of the calibration. Before the reload the firmware reads this unit's chassis through the stored
        # mount: 4.56 deg of attitude error and the boom angle 4.33 deg off (boom = chassis IMU vs boom IMU). After it,
        # 2e-5 deg (float32 resolution; the old acos form read 0.019 here) and 1e-5 deg.
        # FW: link attitude from parLocalTest.imuChs (MdlApp.c:41894-41898; linkOri = imuOri * mntOri, MdlApp.c:10940).
        self.assertGreater(self.att_err_before, 3.0)
        self.assertGreater(abs(self.boom_err_before), 3.0 * DEG)
        self.assertLess(self.att_err_after, 1e-3)
        self.assertLess(abs(self.boom_err_after), 0.001 * DEG)

    def test_a_second_run_with_the_identified_mount_loaded_is_a_fixed_point(self):
        # WHY: closes both loops. With the first run's mount in par.imuChs the swing rate is read without the cos(2 deg)
        # loss, so the identified speed IS the plant's vmax, and the rebuilt mount is the loaded one again.
        M1 = mount(self.snap, "y.imuMntOri_chs")
        _, rec, emu = run_calib_chs(unit_plant(), setup=lambda fw: kin.write_mounts(fw, {"imuChs": M1}))
        self.assertEqual(rec.names(), EXPECTED_SUBSTEPS)
        self.assertEqual(len(emu.saved), 1)
        snap = emu.saved[0][1]
        for port in SWING_PORTS:
            self.assertAlmostEqual(snap[f"y.tblReqSpdToActCmd.{port}_X"][2], PLANT_VMAX[port], delta=2e-5, msg=port)
        self.assertLess(np.abs(mount(snap, "y.imuMntOri_chs") - self.unit).max(), 1e-5)


# ==================================================================================================================
class TestJackUpDirection(unittest.TestCase):
    """The deck asks for Pitch > 5 deg and Roll = 0; the firmware checks neither (spec A7). The DIRECTION of the lean at
    the reference pose decides the mount, silently: exactly one direction rebuilds the unit's board. Which physical tilt
    that is depends on the accelerometer sign, not on the firmware: nose up under the plant's -1 g ASSUMPTION, nose down
    under +1 g."""

    def test_any_other_lean_rotates_the_mount_about_the_swing_axis_and_nothing_notices(self):
        # WHY/FINDING (firmware_defect: the silent direction dependence): vz (the swing axis) is right for any lean, but
        # vy = accRef x vz is chassis +Y only when the reference reading's horizontal part points along chassis -X; a lean
        # in another direction turns vy, vx about vz by that direction's angle. With the plant's -1 g (ASSUMPTION) the
        # right lean is nose up, so: nose down -> 180 deg, left side up (URDF roll +6) -> +90 deg, right side up -> -90
        # deg, measured to 1.8e-6 / 2.1e-6 / 6.3e-6 per element. The valve identification does not depend on it and
        # completes normally.
        # FW: chart_2291 l.195-213 (vz = v1 x v2, vy = accRef x vz, vx = vy x vz); no plausibility check on the result
        # (SanityCheckImuCalib, chart_2516, generates no code: 'AngImuCalibErr' occurs 0 times in MdlApp.c,
        # test_calibration_entry).
        unit = unit_mount()
        cases = (("nose down", dict(pitch_deg=6.0), 180.0),
                 ("left side up", dict(pitch_deg=0.0, roll_deg=6.0), 90.0),
                 ("right side up", dict(pitch_deg=0.0, roll_deg=-6.0), -90.0))
        for label, ground, yaw in cases:
            with self.subTest(ground=label):
                plant = unit_plant(**ground)
                h, rec, emu = run_calib_chs(plant)
                self.assertEqual(rec.names(), EXPECTED_SUBSTEPS)
                self.assertEqual(len(emu.saved), 1)
                self.assertEqual(rec.inhibit_or & CALIB_INHIBIT_MASK, 0)
                self.assertFalse(rec.cab_alarm_or_led)
                self.assertLess(max(rec.dwell()[s] for s in ("ChsLeMoveToPnt1", "ChsRiMoveToPnt2")), TIMEOUT_TICKS)
                snap = emu.saved[0][1]
                for port in SWING_PORTS:
                    X, Y = table(snap, "y.tblReqSpdToActCmd", port)
                    onset = Y[1] + ONSET_DLY_CMP
                    self.assertGreaterEqual(onset, PLANT_DEADBAND[port] - 1e-4, port)
                    self.assertLessEqual(onset, PLANT_DEADBAND[port] + 5 * STAIR_STEP + 1e-4, port)
                    self.assertLess(abs(X[2] - PLANT_VMAX[port]) / PLANT_VMAX[port], 1e-3, port)
                M = mount(snap, "y.imuMntOri_chs")
                self.assertLess(np.abs(M - unit @ kin.Rz(yaw * DEG)).max(), 1e-5)
                self.assertAlmostEqual(rot_deg(unit, M), abs(yaw), delta=0.01)

    def test_which_lean_is_right_is_set_by_the_accelerometer_sign_not_by_the_firmware(self):
        # WHY: keeps "nose up" out of the firmware finding. Flipping the sign flips accRef, v1 and v2 together, so
        # vz = v1 x v2 is unchanged while vy = accRef x vz and vx flip: the same firmware saves the unit's board for
        # nose DOWN and UNIT_MOUNT @ Rz(180) for nose up (measured 1.8e-6 / 1.9e-6 per element). Nose up is right only
        # under the -1 g sign inferred from the compiled mounts (test_valve_plant.test_accelerometer_sign_is_minus_one_g).
        # FW: chart_2291 l.192-213.
        unit = unit_mount()
        for label, pitch, yaw in (("nose up", JACK_UP_PITCH_DEG, 180.0), ("nose down", -JACK_UP_PITCH_DEG, 0.0)):
            with self.subTest(ground=label, acc_sign=+1):
                _, rec, emu = run_calib_chs(unit_plant(pitch_deg=pitch, acc_sign=+1.0))
                self.assertEqual(rec.names(), EXPECTED_SUBSTEPS)
                self.assertEqual(len(emu.saved), 1)
                M = mount(emu.saved[0][1], "y.imuMntOri_chs")
                self.assertLess(np.abs(M - unit @ kin.Rz(yaw * DEG)).max(), 1e-5)


# ==================================================================================================================
class TestStoredDeadbandRunTime(unittest.TestCase):

    def test_a_machine_with_the_stored_deadbands_needs_almost_five_minutes(self):
        # WHY: operator and SIL budget. With the stored 32.0 / 30.5 % deadbands the staircases climb 60 and 52 steps from
        # 20 % at 2 s each. Measured: 28,880 ticks = 288.8 s; ChsLeMin 124.3 s, ChsRiMin 108.5 s; identified 31.9 / 30.3 %.
        # FW: chart_3055 l.58-66, 76, 83-84 (SysPar.m:105, 110, 164-165); ECR88D_ShortArm.m:310-313.
        stored_db = {"swingLe": 32.0, "swingRi": 30.5}
        h0 = Harness().reset()
        for p, db in stored_db.items():
            self.assertEqual(h0.fw[f"par.reqSpdToActCmd.{p}_Y"][1], db)
        plant = unit_plant(deadband=stored_db)
        _, rec, emu = run_calib_chs(plant, timeout_s=600.0)
        self.assertEqual(rec.names(), EXPECTED_SUBSTEPS)
        self.assertEqual(len(emu.saved), 1)
        d = rec.dwell()
        total = rec.entries[-1][0] - next(t for t, s, _ in rec.entries if s == "ChsPntRef_stb")
        self.assertGreater(total, 28000)
        self.assertLess(total, 30000)
        self.assertGreater(d["ChsLeMin"] + d["ChsRiMin"], 22000)
        snap = emu.saved[0][1]
        for port, db in stored_db.items():
            onset = snap[f"y.tblReqSpdToActCmd.{port}_Y"][1] + ONSET_DLY_CMP
            self.assertGreaterEqual(onset, db - 1e-4, port)
            self.assertLessEqual(onset, db + 5 * STAIR_STEP + 1e-4, port)


# ==================================================================================================================
class TestLevelGroundControl(unittest.TestCase):
    """Same plant on level ground. Gravity is then parallel to the swing axis and the chassis accelerometer cannot see the
    house turn; spec A7 calls the result 'silently unchanged'."""

    def test_ideal_accelerometer_level_machine_never_leaves_the_staircase_and_keeps_swinging(self):
        # FINDING (firmware_defect, code fact): ChsLeMin has no timeout and no plausibility exit. Its only exits are
        # isMotionOnset.swingLe, which only the accelerometer's planar angle can raise, and the shared abort; the gyro
        # swing rate and the GNSS swing estimate are not consulted, and the staircase limit is
        # PropVlvCmdMinCalib_limit = 100 % (reached after 800 s, pinned in test_calibration_entry without a plant).
        # PLANT-DEPENDENT: the endless swing below. It needs this plant's idealised accelerometer -- noise-free (next
        # test: 0.2 mg rms already ends the staircase) and quasi-static (no centripetal / tangential term: at the
        # 0.078 rad/s reached here the centripetal term is 0.6 mg per metre of lever arm, the IMU position is in no
        # source, against ~35 mg of in-plane gravity on this 2 deg board -- up to ~1 deg of planar angle, past the
        # 0.5 deg threshold). The plant's own 1 mg control below completes and saves; on a real machine that garbage
        # save, not a runaway, is the likely outcome.
        # What the plant shows: the staircase reaches the plant's 23 % deadband after 30 s and keeps climbing while the
        # house swings faster and faster. The firmware's own swing estimate (GNSS baseline) follows the house the whole
        # time; the only annunciation is the normal calibration beacon (extAlarmCmd 0.5 s on / 1 s off) -- no cab alarm,
        # no warning LED, no inhibit. Measured after 100 s in ChsLeMin: 30.0 %, 0.078 rad/s, the house 158 deg round
        # (firmware estimate 157.8).
        # FW: ChsLeMin guard MdlApp.c:28768, read through Delay1 (:28770), plus only the shared abort on cntSave /
        # ~isCalibrating / save ack (:28742-28744); ChsRiMin the same (:29448-29449); staircase
        # chart_3055 l.58-66, 83, 100-101, SysPar.m:109; beacon <S130> AlarmOff after(1, sec) / AlarmOn after(0.5, sec)
        # while isCalibrating (MdlApp.c:49190-49410).
        plant = unit_plant(pitch_deg=0.0)
        in_min = 10000
        h, rec, emu = run_calib_chs(plant, timeout_s=130.0, stop=lambda rec: rec.in_state_for("ChsLeMin", in_min))
        fw = h.fw
        self.assertEqual(h.calib_step(), "ChsLeMin")
        self.assertTrue(fw["y.isCalibrating"])
        self.assertEqual(h.main_state(), "CalibChs")
        self.assertEqual(rec.save_req_ticks, 0)
        self.assertEqual(emu.saved, [])
        cmd = fw["y.propVlvCmd.swingLe"]
        self.assertAlmostEqual(cmd, STAIR_START + STAIR_STEP * (in_min // STAIR_TICKS), delta=STAIR_STEP + 1e-4)
        self.assertGreater(cmd, PLANT_DEADBAND["swingLe"] + 5.0)
        self.assertGreater(rec.swing_travel, 90.0 * DEG, "the house swings while the firmware waits for an onset")
        self.assertGreater(plant.qdot["swing"], 0.05)
        self.assertAlmostEqual(-fw["y.jnts.ChsToUc.q"], plant.q["swing"], delta=1.0 * DEG)
        self.assertTrue(np.array_equal(mount(fw, "y.imuMntOri_chs"), mount(fw, "par.imuChs")))
        self.assertEqual(rec.inhibit_or & CALIB_INHIBIT_MASK, 0)
        self.assertFalse(rec.cab_alarm_or_led)
        self.assertEqual(plant.limit_hits, [])
        # the same beacon as in a normal dwell: 1/3 on (measured 250/801 in ChsPntRef_stb, which starts with 1 s off,
        # and 3302/10000 in the endless ChsLeMin)
        for state, tol in (("ChsPntRef_stb", 0.03), ("ChsLeMin", 0.01)):
            self.assertAlmostEqual(rec.ext_alarm_on[state] / rec.ticks_in[state], 1.0 / 3.0, delta=tol, msg=state)

    def test_the_level_ground_runaway_needs_an_accelerometer_quieter_than_0_2_mg(self):
        # WHY: bounds the test above to what it is, a plant idealisation. The onset compares ONE raw sample's planar
        # angle with a latched one (chart_2316 l.97-98 on unfiltered accRaw, MdlApp.c:11102-11105); on this board
        # (~35 mg in-plane gravity) 0.2 mg rms leaves ChsLeMin on noise within the first staircase step on every seed
        # (measured 13 / 42 / 5 ticks for seeds 1-3), the valve at 20 % still below the plant's 23 % deadband, so the
        # house has not moved. Probe, not pinned
        # (scratch calpolish-chs/probe_level.py): 0.01 and 0.03 mg still run away on all 3 seeds, 0.1 mg on 2 of 3.
        # With the compiled board's 3.1 mg in-plane component even one CAN LSB (2^-14 g) ends it
        # (test_calibration_entry.test_stationary_level_machine_with_accel_noise_completes_and_saves).
        for seed in (1, 2, 3):
            with self.subTest(seed=seed):
                plant = unit_plant(pitch_deg=0.0, acc_noise=2e-4, seed=seed)
                stop = lambda rec: (lambda h: "ChsLeMin_stb" in rec.names() or rec.in_state_for("ChsLeMin", 3000)(h))
                h, rec, emu = run_calib_chs(plant, timeout_s=60.0, stop=stop)
                self.assertIn("ChsLeMin_stb", rec.names(), "still in the staircase after 30 s")
                self.assertLess(rec.dwell()["ChsLeMin"], STAIR_TICKS)
                self.assertEqual(rec.swing_at("ChsLeMin_stb"), rec.swing_at("ChsLeMin"), "valve never opened")
                self.assertEqual(rec.swing_travel, 0.0)

    def test_noisy_level_machine_completes_saves_and_silently_keeps_the_stored_mount(self):
        # WHY: the spec's level-ground signature, on a plant whose board is NOT the stored one -- so "unchanged" is visibly
        # wrong. 1 mg of noise (ASSUMPTION) on a board with ~35 mg of in-plane gravity (the 2 deg tilt) jitters the planar
        # angle by ~1.6 deg, so each staircase fires on its first step (the valve at 20 % never opens: 19.5 % stored for
        # both directions against plant 23.0 / 21.8), the Le leg runs its full 10 s at 60 % without ever reaching 170 deg
        # (the house turns ~234 deg), the Ri leg ends on its first evaluation (its "< 80 deg" guard is already true), and
        # the 1 s means differ by noise only: |v1 x v2| < 1e-6 keeps the stored mount bit-identical. Saved once, NoTarget,
        # no alarm, no in-mask inhibit. Collateral: swingLe's speed is right (the timeout leg is long enough to peak),
        # swingRi's is MinTblReqSpd, so its breakpoints [0, 0.01, 0.002] are not monotonic although SysPar.m:112 says the
        # clamp exists "to have monotonous change of the input array" (the knee is the 0.01 literal, chart_2338 l.80).
        # Classification: the silent mount gate, the accepted timeout leg, the 1-tick Ri leg and the non-monotonic table
        # are source facts; that level ground produces exactly these values depends on the plant's noise.
        # Asserted per seed.
        # FW: _ToPnt guards MdlApp.c:28887, :29662; mount gate chart_2291 l.198; tables chart_2338 l.34-35, 53-54, 78-81.
        unit, compiled = unit_mount(), compiled_chs_mount()
        for seed in (1, 2, 3):
            with self.subTest(seed=seed):
                plant = unit_plant(pitch_deg=0.0, acc_noise=1e-3, seed=seed)
                h, rec, emu = run_calib_chs(plant)
                self.assertEqual(rec.names(), EXPECTED_SUBSTEPS)
                self.assertEqual(h.curr_step(), "NoTarget")
                self.assertEqual(len(emu.saved), 1)
                self.assertFalse(rec.cab_alarm_or_led)
                self.assertEqual(rec.inhibit_or & CALIB_INHIBIT_MASK, 0, hex(rec.inhibit_or))
                self.assertEqual(plant.limit_hits, [])
                snap = emu.saved[0][1]
                M = mount(snap, "y.imuMntOri_chs")
                self.assertTrue(np.array_equal(M, compiled), "stored mount kept bit-identical")
                self.assertGreater(rot_deg(unit, M), 20.0, "... which is not the unit's board")
                d = rec.dwell()
                self.assertLessEqual(d["ChsLeMin"], STAIR_TICKS)
                self.assertLessEqual(d["ChsRiMin"], STAIR_TICKS)
                self.assertEqual(d["ChsLeMoveToPnt1"], TIMEOUT_TICKS)
                self.assertEqual(d["ChsRiMoveToPnt2"], 1)
                for port in SWING_PORTS:
                    self.assertEqual(snap[f"y.tblReqSpdToActCmd.{port}_Y"][1], NO_MOTION_ONSET, port)
                self.assertEqual(rec.swing_at("ChsLeMin_stb"), rec.swing_at("ChsLeMin"), "valve never opened")
                self.assertLess(abs(snap["y.tblReqSpdToActCmd.swingLe_X"][2] - PLANT_VMAX["swingLe"])
                                / PLANT_VMAX["swingLe"], 1e-3)
                ri_X = snap["y.tblReqSpdToActCmd.swingRi_X"]
                self.assertEqual(ri_X, [0.0, IDENTIFIED_X1, MIN_TBL_REQ_SPD])
                self.assertLess(ri_X[2], ri_X[1], "non-monotonic breakpoints")
                self.assertGreater(rec.swing_travel, 200.0 * DEG)
                self.assertAlmostEqual(-h.fw["y.jnts.ChsToUc.q"], plant.q["swing"], delta=1.0 * DEG)


# ==================================================================================================================
class TestAccelerometerNoiseWhenJackedUp(unittest.TestCase):
    """The procedure done right (6 deg nose up), with a noisy accelerometer. The onset detector and the mount rebuild
    react to noise very differently: one reads a single raw sample, the other 1 s means. Every number here depends on the
    noise level, which no source gives (plant-dependent)."""

    def test_1mg_noise_fires_both_staircases_on_noise_but_the_mount_survives(self):
        # FINDING (plant-dependent; the code fact behind it is source-level): the onset compares ONE raw sample's planar
        # angle with a latched one (chart_2316 l.97-98, accRaw unfiltered MdlApp.c:11102-11105), against 0.5 deg
        # (SysPar.m:131). At 6 deg the in-plane gravity is 0.10 g, so 1 mg rms (ASSUMPTION) is ~0.8 deg rms of angle
        # difference and the threshold trips within the first staircase step: both deadbands read 19.5 % against the
        # plant's 23.0 / 21.8. Probe, not pinned: 0.1 mg leaves both onsets on the valve on 3 seeds, 0.2 mg trips
        # swingRi on all 3. The mount uses 100-sample means and stays within 0.1 deg (measured 0.042 / 0.076 / 0.005);
        # the speeds are unaffected (gyro). So a real machine's identified deadband is a noise detector unless its IMU
        # delivers samples quieter than ~0.2 mg -- the part's noise and internal filtering are in no source.
        unit = unit_mount()
        for seed in (1, 2, 3):
            with self.subTest(seed=seed):
                plant = unit_plant(acc_noise=1e-3, seed=seed)
                _, rec, emu = run_calib_chs(plant)
                self.assertEqual(rec.names(), EXPECTED_SUBSTEPS)
                self.assertEqual(len(emu.saved), 1)
                d = rec.dwell()
                self.assertLessEqual(d["ChsLeMin"], STAIR_TICKS)
                self.assertLessEqual(d["ChsRiMin"], STAIR_TICKS)
                self.assertLess(max(d["ChsLeMoveToPnt1"], d["ChsRiMoveToPnt2"]), TIMEOUT_TICKS)
                snap = emu.saved[0][1]
                for port in SWING_PORTS:
                    self.assertEqual(snap[f"y.tblReqSpdToActCmd.{port}_Y"][1], NO_MOTION_ONSET, port)
                    self.assertLess(abs(snap[f"y.tblReqSpdToActCmd.{port}_X"][2] - PLANT_VMAX[port]) / PLANT_VMAX[port],
                                    1e-3, port)
                self.assertLess(rot_deg(unit, mount(snap, "y.imuMntOri_chs")), 0.1)

    def test_30mg_vibration_turns_the_identified_mount_by_degrees_and_corrupts_the_swingRi_speed(self):
        # FINDING (plant-dependent: 30 mg is GUESS): with engine-vibration-level noise the 1 s means carry ~3 mg of error
        # against chords of 0.15-0.21 g, a couple of degrees on the swing axis vz -- and vy = accRef x vz is a cross
        # product of two vectors only 6 deg apart, which multiplies a sideways error of vz by ~1/sin(6 deg) = 9.6 in yaw.
        # Measured yaw-dominated errors of 34.2 / 11.3 / 19.7 deg for seeds 1-3, all saved without complaint.
        # The larger operational damage is the speed table: the jittering planar angle drops below 80 deg early and ends
        # the Ri leg after 9 / 127 / 101 ticks (noise-free: 495) while the house is still accelerating, so swingRi X[2]
        # is 0.002 (MinTblReqSpd, breakpoints [0, 0.01, 0.002] not monotonic) / 0.356 / 0.313 against the plant's 0.36.
        # swingLe is unaffected (its leg still peaks). Onsets are 19.5 % as at 1 mg. Asserted per seed; the per-seed
        # values are deterministic regression pins.
        # FW: guard MdlApp.c:29662 (angCalib.chs < AngChsPnt2 || cnt > CntCalib_timeout); table chart_2338 l.35, 80-81.
        unit = unit_mount()
        ri_rel_err = {}
        for seed in (1, 2, 3):
            with self.subTest(seed=seed):
                plant = unit_plant(acc_noise=3e-2, seed=seed)
                _, rec, emu = run_calib_chs(plant)
                self.assertEqual(rec.names(), EXPECTED_SUBSTEPS)
                self.assertEqual(len(emu.saved), 1)
                self.assertEqual(rec.inhibit_or & CALIB_INHIBIT_MASK, 0)
                snap = emu.saved[0][1]
                M = mount(snap, "y.imuMntOri_chs")
                self.assertGreater(rot_deg(unit, M), 5.0)
                np.testing.assert_allclose(M @ M.T, np.eye(3), atol=1e-4)
                self.assertLess(rec.dwell()["ChsRiMoveToPnt2"], 300, "Ri leg not shortened by noise")
                for port in SWING_PORTS:
                    self.assertEqual(snap[f"y.tblReqSpdToActCmd.{port}_Y"][1], NO_MOTION_ONSET, port)
                le_X2 = snap["y.tblReqSpdToActCmd.swingLe_X"][2]
                self.assertLess(abs(le_X2 - PLANT_VMAX["swingLe"]) / PLANT_VMAX["swingLe"], 1e-3)
                ri_X = snap["y.tblReqSpdToActCmd.swingRi_X"]
                self.assertLessEqual(ri_X[2], PLANT_VMAX["swingRi"])
                ri_rel_err[seed] = 1.0 - ri_X[2] / PLANT_VMAX["swingRi"]
                if seed == 1:
                    self.assertEqual(ri_X, [0.0, IDENTIFIED_X1, MIN_TBL_REQ_SPD])
                    self.assertLess(ri_X[2], ri_X[1], "non-monotonic breakpoints")
        self.assertGreater(ri_rel_err.get(3, 0.0), 0.10, ri_rel_err)


if __name__ == "__main__":
    unittest.main()
