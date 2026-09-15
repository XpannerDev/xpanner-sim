"""
Calibration ENTRY and the first full calibration runs (resources/X1Exc_SIL_spec.md A7).

Charts (Stateflow XML inside X1Exc ControlModel/Models/MdlApp.slx; chart script line numbers
below are the ones the generated C quotes as '<S..>:1:N'):
    chart_2537  AutoStsMgr/Chart            -> MdlApp.c <S136>  (NoTarget / Calib<X> / _Paused / _Inhibited)
    chart_1210  MachCalib/CalibStepMgr      -> MdlApp.c <S179>  (y.calibStep sub-steps, save handshake)
    chart_2352  MachCalib/UpdateForKin      -> MdlApp.c <S185>  (what steps 28/29 write)
    chart_2143  KinematicsCalc              -> MdlApp.c <S171>  (forward forkBack / panelTop)
    chart_2291  MachCalib/CalcImuMntOri     -> MdlApp.c <S177>  (accelerometer means, chassis mount)
    chart_2338  SetTblReqSpdToActCmd        -> MdlApp.c <S187>  (valve table written by 20..26)

Execution order inside one MdlApp_step(): AutoStsMgr (MdlApp.c:39719) runs before CalibStepMgr
(:42087), which runs before UpdateForKin (:46043). CalibStepMgr reads MdlApp_Y.isCalibrating and
MdlApp_Y.autoCtrl_CurrStep directly; the main chart reads calibStep back through Delay5.

Everything here was checked against the running firmware; where the spec was wrong the assertion
follows the firmware and the comment says what the spec claimed.

Run:  cd xpanner-sim && python3 -m unittest sil.tests.test_calibration_entry -v
"""
import math
import unittest

import numpy as np

from sil import kinematics as kin
from sil.harness import Harness, SaveHandshake

CALIB_STEPS = ["CalibChs", "CalibBm1", "CalibBm2", "CalibArm", "CalibLink", "CalibTilt",
               "CalibRot", "CalibTrvl", "CalibForkRefPose", "CalibForkPntFront"]

# chart_1210 Standby [isCalibrating] && [autoCtrl_CurrStep == X] branches
# (generated in MdlApp_Standby_jfqu, MdlApp.c:32253 ff; e.g. ForkRefPose at :32353).
FIRST_SUBSTEP = {
    "CalibChs": "ChsPntRef_stb", "CalibBm1": "Bm1PntRef_stb", "CalibBm2": "Bm2PntRef_stb",
    "CalibArm": "ArmPntRef_stb", "CalibLink": "LinkPntRef_stb", "CalibTilt": "TiltPntRef_stb",
    "CalibRot": "RotPosiPntRef_stb", "CalibTrvl": "TrvlLeFwd_stb",
    "CalibForkRefPose": "ForkRefPose_stb", "CalibForkPntFront": "ForkPntFront_stb",
}

FORK_FIELDS = ("angForkUpLimit", "distUcToForkBack", "distForkBackToPanelTop")

# SysPar.m:102-107, generated as literals at MdlApp.c:30159 (800U), :29950 (100U), :30058 (100U).
# All counts are from the sub-step's ENTRY tick to the tick the next sub-step is entered.
STB_TICKS = 801             # entry tick + 800 during ticks: [cnt >= 800] is tested before cnt = cnt + 1
LOG_TICKS = 101
SAVE_FALLBACK_TICKS = 102   # [cntSave > 100]; cntSave counts during ticks only (:30094-30101)
TIMEOUT_TICKS = 1002        # _ToPnt [... || cnt > CntCalib_timeout], SysPar.m:104 (1000)
MIN_TBL_REQ_SPD = 0.002     # SysPar.m:112


# ------------------------------------------------------------------------------------
# helpers
# ------------------------------------------------------------------------------------
def booted(swing_level=True, gnss=True, plant=None):
    """Healthy machine at the harness rest pose, idle in NoTarget. swing_level=True HOLDS the
    proximity switch closed (house aligned); False only pulses it, so the boot latch is set but
    the switch is open."""
    h = Harness(plant=plant).reset().nominal_inputs()
    if swing_level:
        h.set_swing_aligned(True)
    else:
        h.pulse("u.isSwingAligned")
    if gnss:
        h.gnss_rtk_fixed()
    h.tick(3)
    return h


def fork_values(fw, prefix):
    return {k: fw[f"{prefix}.parKin.{k}"] for k in FORK_FIELDS}


def saved_fork(snapshot):
    """Fork fields of one SaveHandshake.saved snapshot -- the bytes kin_s carries to NVM
    (AppCtrlIf.c:1014-1017 -> InternalParam.c:83)."""
    return {k: snapshot[f"y.parKin.{k}"] for k in FORK_FIELDS}


def first_tick(h, column, pred, after=-1):
    """Tick of the first h.trace() row later than `after` whose `column` satisfies pred. Uses the
    shared trace, so tick numbers are the harness's (row tick = tick_count after that step)."""
    i = h.trace_paths.index(column) + 1
    for row in h.trace_rows:
        if row[0] > after and pred(row[i]):
            return row[0]
    raise AssertionError(f"{column} never satisfied the predicate after tick {after}")


def calib_is(h, name):
    v = h.fw.enum_value("CalibStep", name)
    return lambda x: x == v


def mat(fw, path):
    return np.array(fw[path], dtype=np.float64).reshape(3, 3, order="F")   # MATLAB column-major


def vec(fw, path):
    return np.array(fw[path], dtype=np.float64)


def pitch(R):
    """The angle chart_2352 extracts: -atan2(R(3,1), R(1,1)); equals th for R = Ry(th)."""
    return -math.atan2(R[2, 0], R[0, 0])


def fork_ref_pose_formula(uc_R, uc_p, cs_R, cs_p):
    """chart_2352 l.16-21 (MdlApp.c:46043-46096): R_forkBack_uc = uc.R' * cs.R * Ry(-pi/2)."""
    ang = pitch(uc_R.T @ (cs_R @ kin.Ry(-math.pi / 2)))
    v = uc_R.T @ (cs_p - uc_p)
    return ang, v          # firmware stores [v[0], 0, v[2]]


def fork_pnt_front_formula(cs_R, probe_p, origin_p):
    """chart_2352 l.24-27 (MdlApp.c:46099-46140): (R_forkBack_W' * (probe.p - origin))(1)."""
    return ((cs_R @ kin.Ry(-math.pi / 2)).T @ (probe_p - origin_p))[0]


def in_uc(fw, path):
    return mat(fw, "y.links.uc.R").T @ (vec(fw, path) - vec(fw, "y.links.uc.p"))


def cs_pitch_error_to_fork_back(fw):
    """Pitch of the contact surface's implied forkBack frame minus the firmware's forkBack pitch,
    both in the undercarriage frame."""
    uc_R = mat(fw, "y.links.uc.R")
    return (pitch(uc_R.T @ mat(fw, "y.links.contactSurface.R") @ kin.Ry(-math.pi / 2))
            - pitch(uc_R.T @ mat(fw, "y.links.forkBack.R")))


SETTLE_TICKS = 300   # joint angles pass a 3 Hz first-order LPF (chart_2143 l.188, SysPar.m:75): ~16 %/tick


def solve_pose(h, residual, q0, tol=2e-6):
    """Newton solve for (q_bm1, q_arm, q_inp) so that residual(fw) -> 0, with the FIRMWARE as the
    forward model: publish the pose through the shared IMU publisher, let the joint-angle LPF
    settle, read y.links. Local because sil.kinematics has orientations only (no link positions,
    no contact surface / probe offsets) -- using the firmware's own FK keeps the target exactly
    the geometry the firmware believes in."""
    q = np.array(q0, dtype=float)

    def ev(qq):
        h.set_pose(q_bm1=qq[0], q_arm=qq[1], q_inp=qq[2])
        h.tick(SETTLE_TICKS)
        return residual(h.fw)

    for _ in range(20):
        r = ev(q)
        if np.abs(r).max() < tol:
            return q
        J = np.zeros((3, 3))
        for j in range(3):
            dq = q.copy()
            dq[j] += 1e-4
            J[:, j] = (ev(dq) - r) / 1e-4
        step = np.linalg.solve(J, -r)
        step *= min(1.0, 0.3 / np.abs(step).max())
        q = q + step
    raise AssertionError(f"pose solve did not converge: residual {r}")


class AccelNoise:
    """Plant: white noise on the chassis accelerometer around what the shared publisher wrote
    (nominal_inputs, mirrored frame). Local: the library publishes noise-free IMUs."""

    def __init__(self, sigma, seed=7):
        self.rng = np.random.default_rng(seed)
        self.sigma = sigma
        self.base = None

    def __call__(self, h):
        if self.base is None:
            self.base = np.array(h.fw["u.chsImuAcc"], dtype=float)
        h.fw["u.chsImuAcc"] = list(self.base + self.rng.normal(0.0, self.sigma, 3))


# ------------------------------------------------------------------------------------
class TestCalibEntry(unittest.TestCase):
    """Which states accept a calibration step number, and what the first press does."""

    def test_every_calib_step_enters_paused_from_notarget(self):
        # WHY: the tablet step number alone must never move the machine; the sim uses
        # isCalibrating as the valve-ownership mux, so it has to stay false until the press.
        # FW: NoTarget guards [hasChanged(autoReq_Step) && autoReq_Step == Calib<X>] -> Calib<X>_Paused,
        # MdlApp.c:22710-22916; _Paused entry isCalibrating = false (chart_2537 SSID 49 for ForkRefPose,
        # 33 for Chs).
        for name in CALIB_STEPS:
            with self.subTest(step=name):
                h = booted()
                h.request_step(name)
                h.tick(1)
                self.assertEqual(h.curr_step(), name)
                self.assertEqual(h.main_state(), f"{name}_Paused")
                self.assertFalse(h.fw["y.isCalibrating"])
                self.assertFalse(h.is_running())
                h.tick(200)                           # nothing auto-starts while paused
                self.assertEqual(h.main_state(), f"{name}_Paused")
                self.assertEqual(h.calib_step(), "CalibStandby")
                self.assertFalse(h.fw["y.isCalibrating"])

    def test_press_starts_first_substep_in_the_same_tick(self):
        # WHY: calibStep is the sim's progress trace; the handoff main chart -> CalibStepMgr has no
        # unit delay, so the sim must not expect a tick of slack between isCalibrating and the first
        # sub-step. Observed on the tick the StartPause HIGH level is stepped (h.pulse() adds a low
        # tick afterwards, which would hide a one-tick delay).
        # FW: _Paused -> Calib<X> on [hasChanged(autoReq_StartPause) && autoReq_StartPause]
        # (MdlApp.c:19399 for ForkRefPose); CalibStepMgr Standby reads MdlApp_Y.isCalibrating /
        # autoCtrl_CurrStep (MdlApp.c:32264, :32353) and runs after AutoStsMgr (:39719 < :42087).
        for name in CALIB_STEPS:
            with self.subTest(step=name):
                h = booted()
                h.request_step(name)
                h.tick(1)
                h.fw["u.jstAutoReq_StartPause"] = 1
                h.tick(1)
                self.assertTrue(h.fw["y.isCalibrating"])
                self.assertEqual(h.main_state(), name)
                self.assertEqual(h.calib_step(), FIRST_SUBSTEP[name])
                h.fw["u.jstAutoReq_StartPause"] = 0
                h.tick(50)
                self.assertEqual(h.calib_step(), FIRST_SUBSTEP[name])   # 8 s dwell, see run tests

    def test_remote_startpause_is_the_same_button(self):
        # WHY: the radio remote is the other operator path; a sim that only drives the joystick
        # input would miss that either one starts/pauses calibration.
        # FW: LogicalOperator_lv2w = jstAutoReq_StartPause | rmtAutoReq_StartPause, MdlApp.c:39724.
        h = booted()
        h.request_step("CalibForkRefPose")
        h.tick(1)
        h.pulse("u.rmtAutoReq_StartPause")
        self.assertTrue(h.fw["y.isCalibrating"])
        self.assertEqual(h.calib_step(), "ForkRefPose_stb")
        h.pulse("u.rmtAutoReq_StartPause")
        self.assertEqual(h.main_state(), "CalibForkRefPose_Paused")

    def test_calib_step_numbers_are_ignored_in_standby_and_positioning_paused(self):
        # WHY: spec A7 "reachable ONLY from NoTarget" -- confirmed for these two states. A sim
        # scenario that sends a calibration step while a panel job is loaded tests nothing unless
        # it cancels first.
        # FW: MdlApp_Standby guards (MdlApp.c:25148-25278) and MdlApp_PositioningPaused guards
        # (MdlApp.c:24157-24315) list only cycle steps, no Calib<X> transition.
        h = booted()
        h.set_target_panel(panel_id=7)
        h.tick(3)
        h.request_step("Standby")
        h.run_until(lambda h: h.curr_step() == "Standby", 0.5, "Standby")
        for name in CALIB_STEPS:
            h.request_step(name)
            h.tick(3)
            self.assertEqual(h.main_state(), "Standby", name)
            self.assertFalse(h.fw["y.isCalibrating"])

        h.request_step("Positioning")
        h.run_until(lambda h: h.main_state() == "PositioningPaused", 0.5, "PositioningPaused")
        for name in CALIB_STEPS:
            h.request_step(name)
            h.tick(3)
            self.assertEqual(h.main_state(), "PositioningPaused", name)
            self.assertFalse(h.is_running())
            self.assertEqual(h.calib_step(), "CalibStandby")

    def test_calib_request_in_standby_then_auto_press_starts_the_panel_cycle(self):
        # WHY: mode-confusion hazard a SIL run must be able to show: the ignored calibration
        # request leaves Standby armed, so the operator's Auto press moves the machine into
        # Positioning (StartStopSts true) instead of calibrating.
        # FW: Standby [hasChanged(StartPause) && StartPause && isTarPanelValid] -> Positioning,
        # MdlApp.c:25278 (chart_2537 T281).
        h = booted()
        h.set_target_panel(panel_id=7)
        h.tick(3)
        h.request_step("Standby")
        h.run_until(lambda h: h.curr_step() == "Standby", 0.5, "Standby")
        h.jump_to_step("CalibForkRefPose")            # step number, one tick, Auto press
        self.assertEqual(h.main_state(), "Positioning")
        self.assertTrue(h.is_running())
        self.assertFalse(h.fw["y.isCalibrating"])
        self.assertEqual(h.calib_step(), "CalibStandby")

    def test_cancel_first_then_the_step_number_must_be_sent_again(self):
        # WHY: the field procedure is cancel -> send step. A tablet (or sim) that sent the step
        # while in Standby and relies on it after Cancel gets nothing, because autoReqStep never
        # CHANGES again. Re-sending goes through the harness re-edge (255, then the step).
        # FW: Standby [autoReq_Cancel == 1] -> NoTarget (MdlApp.c:25148); NoTarget guards need
        # hasChanged(autoReq_Step) (MdlApp.c:22893).
        h = booted()
        h.set_target_panel(panel_id=7)
        h.tick(3)
        h.request_step("Standby")
        h.run_until(lambda h: h.curr_step() == "Standby", 0.5, "Standby")
        h.request_step("CalibForkRefPose")
        h.tick(2)
        h.fw["u.tabletAutoReq_Cancel"] = 1
        h.tick(1)
        h.fw["u.tabletAutoReq_Cancel"] = 0
        self.assertEqual(h.main_state(), "NoTarget")
        h.tick(20)
        self.assertEqual(h.main_state(), "NoTarget")          # stale 28 on the bus: no entry
        h.request_step("CalibForkRefPose")
        h.tick(1)
        self.assertEqual(h.main_state(), "CalibForkRefPose_Paused")

    def test_steps_30_31_32_and_undefined_values_are_noops_from_notarget(self):
        # WHY: spec A7 "no step 30/31; 32 CalibSwingStopCoeff unreachable" -- confirmed. The
        # tablet's 6-bit field (CanCtrl.c:2743) can send 19..63; the sim must treat them as
        # silently ignored. 255 is the harness's own re-edge value (harness.RE_EDGE_STEP), whose
        # safety this also pins.
        # FW: MdlApp_NoTarget (MdlApp.c:22702-22938) tests exactly the ten values 20..29 and
        # Standby; CalibForkPnt2/3 states carry <comment> in chart_2537.xml and were pruned.
        for v in (30, 31, 32, 19, 33, 63, 255):
            with self.subTest(value=v):
                h = booted()
                h.request_step(v)
                h.tick(5)
                h.pulse("u.jstAutoReq_StartPause")
                h.tick(5)
                self.assertEqual(h.main_state(), "NoTarget")
                self.assertFalse(h.fw["y.isCalibrating"])
                self.assertEqual(h.calib_step(), "CalibStandby")

    def test_steps_30_31_32_do_not_disturb_a_paused_calibration(self):
        # WHY: same values arriving while a calibration is selected must neither cancel it nor
        # switch it; the selected step must still start on the next press.
        # FW: CalibChs_Paused guards list only the ten live Calib<X> targets, Cancel, inhibit and
        # StartPause (MdlApp.c:17870-18145); ForkPnt2/ForkPnt3 transitions are commented out.
        h = booted()
        h.request_step("CalibChs")
        h.tick(1)
        for v in (30, 31, 32):
            h.request_step(v)
            h.tick(3)
            self.assertEqual(h.main_state(), "CalibChs_Paused", v)
            self.assertFalse(h.fw["y.isCalibrating"])
        h.pulse("u.jstAutoReq_StartPause")
        self.assertEqual(h.calib_step(), "ChsPntRef_stb")

    def test_paused_steps_switch_directly_running_step_ignores_step_and_cancel(self):
        # WHY: the sim harness can hop between calibrations without a NoTarget round trip while
        # paused, but NOT while running; and Cancel is not an emergency stop for a running
        # calibration (spec A6 pause semantics -- confirmed): you must pause first.
        # FW: CalibChs_Paused step guards (MdlApp.c:17870-18076, chart_2537 SSID 33); running
        # CalibForkRefPose checks only Stop/StartPause, inhibit, calibStep == CalibStandby
        # (MdlApp.c:18960, :18984, :19014); _Paused [autoReq_Cancel == 1] -> NoTarget (:19671).
        h = booted()
        h.request_step("CalibChs")
        h.tick(1)
        self.assertEqual(h.main_state(), "CalibChs_Paused")
        h.request_step("CalibForkRefPose")
        h.tick(1)
        self.assertEqual(h.main_state(), "CalibForkRefPose_Paused")
        h.pulse("u.jstAutoReq_StartPause")
        self.assertEqual(h.main_state(), "CalibForkRefPose")
        self.assertTrue(h.fw["y.isCalibrating"])

        h.request_step("CalibChs")
        h.tick(3)
        self.assertEqual(h.main_state(), "CalibForkRefPose")
        self.assertEqual(h.calib_step(), "ForkRefPose_stb")

        h.fw["u.tabletAutoReq_Cancel"] = 1
        h.tick(5)
        self.assertEqual(h.main_state(), "CalibForkRefPose")
        self.assertTrue(h.fw["y.isCalibrating"])
        h.fw["u.jstAutoReq_StartPause"] = 1                 # pause, Cancel still held
        h.tick(1)
        self.assertEqual(h.main_state(), "CalibForkRefPose_Paused")
        h.fw["u.jstAutoReq_StartPause"] = 0
        h.tick(1)
        self.assertEqual(h.main_state(), "NoTarget")
        h.fw["u.tabletAutoReq_Cancel"] = 0

    def test_harness_jump_to_step_from_a_running_calibration_does_nothing(self):
        # LIBRARY BUG (documentation): Harness.jump_to_step says "Does nothing from a running state
        # -- pause first". With start=True it still pulses StartPause, and in a running state that
        # edge IS the pause: the step change is ignored but the running calibration is paused and
        # its dwell aborted. This test asserts the documented behaviour and fails.
        # FW: running CalibForkRefPose [autoReq_Stop || (hasChanged(StartPause) && StartPause)]
        # -> _Paused (MdlApp.c:18960); CalibStepMgr aborts on ~isCalibrating (:30133).
        h = booted()
        h.jump_to_step("CalibForkRefPose")
        h.tick(50)
        h.jump_to_step("CalibChs")
        self.assertEqual(h.main_state(), "CalibForkRefPose")
        self.assertTrue(h.fw["y.isCalibrating"])


# ------------------------------------------------------------------------------------
class TestCalibGates(unittest.TestCase):
    """isSwingAligned (fork), RTK fix (travel), isCalibInhibited (everything, with isMachCalib)."""

    def test_fork_steps_need_the_live_swing_switch_not_the_boot_latch(self):
        # WHY: spec A7 says fork steps are gated by ~isSwingAligned "so Step 0 still applies".
        # The formula is right but the conclusion is not: Step 0's rising edge (the latch that
        # clears BIT_SWING_NOT_INIT) is NOT enough -- the switch must be closed NOW. Conversely
        # the non-fork steps ignore swing state entirely, latch included. A sim that pulses the
        # switch at boot and then swings off-center can never run steps 28/29.
        # FW: LogicalOperator_cfu0 = MdlApp_U.isSwingAligned ^ 1 (MdlApp.c:39714, raw inport, no
        # latch) -> ForkRefPose_Paused [isCalibInhibited || isSwingBasedAutoInhibited] (MdlApp.c:19379).
        for name in ("CalibForkRefPose", "CalibForkPntFront"):
            with self.subTest(step=name):
                h = booted(swing_level=False)
                self.assertNotIn("BIT_SWING_NOT_INIT", h.inhibit_names())   # latch is set
                h.request_step(name)
                h.tick(2)
                for k in range(4):                    # presses land on both tick parities
                    h.pulse("u.jstAutoReq_StartPause")
                    h.tick(k % 2)
                    self.assertIn(h.main_state(), (f"{name}_Paused", f"{name}_Inhibited"))
                    self.assertFalse(h.fw["y.isCalibrating"])
                h.set_swing_aligned(True)
                h.tick(2)
                h.pulse("u.jstAutoReq_StartPause")
                self.assertTrue(h.fw["y.isCalibrating"])
                self.assertEqual(h.calib_step(), FIRST_SUBSTEP[name])

        h = Harness().reset().nominal_inputs()               # switch never closed at all
        h.tick(3)
        self.assertIn("BIT_SWING_NOT_INIT", h.inhibit_names())
        h.jump_to_step("CalibChs")
        self.assertTrue(h.fw["y.isCalibrating"])
        self.assertEqual(h.calib_step(), "ChsPntRef_stb")

    def test_gated_paused_and_inhibited_states_alternate_every_tick(self):
        # WHY: the "Inhibited" state is not sticky for the swing and GNSS gates. A press on the
        # very tick the gate clears is accepted on one tick parity and dropped on the other --
        # a harness that closes the switch and presses together sees apparently random refusals.
        # FW: _Paused -> _Inhibited on [isCalibInhibited || isSwingBasedAutoInhibited]
        # (MdlApp.c:19379) / [isCalibInhibited || isGnssBasedCalibInhibited] (MdlApp.c:22369),
        # but _Inhibited -> _Paused on [~isCalibInhibited] ONLY (MdlApp.c:19325, :22316),
        # so with only the swing/GNSS gate active the state flips every tick.
        def fork_blocked(fw):                               # switch open, GNSS irrelevant but good
            fw["u.methodGnss_Main"] = fw["u.methodGnss_Aux"] = 4

        def fork_unblock(fw):
            fw["u.isSwingAligned"] = 1

        def trvl_blocked(fw):                               # only the aux antenna lacks RTK fix
            fw["u.isSwingAligned"] = 1
            fw["u.methodGnss_Main"] = 4

        def trvl_unblock(fw):
            fw["u.methodGnss_Aux"] = 4

        def blocked_machine(step, set_blocked):
            h = booted(swing_level=False, gnss=False)
            set_blocked(h.fw)
            h.tick(2)
            h.request_step(step)
            return h

        def accepted(step, offset, set_blocked, unblock):
            h = blocked_machine(step, set_blocked)
            h.tick(2 + offset)
            unblock(h.fw)                                   # gate clears and Auto pressed, same tick
            h.fw["u.jstAutoReq_StartPause"] = 1
            h.tick(1)
            h.fw["u.jstAutoReq_StartPause"] = 0
            return bool(h.fw["y.isCalibrating"])

        for step, blocked, unblock in (("CalibForkRefPose", fork_blocked, fork_unblock),
                                       ("CalibTrvl", trvl_blocked, trvl_unblock)):
            with self.subTest(step=step):
                h = blocked_machine(step, blocked)
                states = []
                for _ in range(6):
                    h.tick()
                    states.append(h.main_state())
                self.assertEqual(set(states), {f"{step}_Paused", f"{step}_Inhibited"}, states)
                self.assertTrue(all(a != b for a, b in zip(states, states[1:])), states)

                r = [accepted(step, k, blocked, unblock) for k in range(4)]
                self.assertEqual(r[0], r[2], r)
                self.assertEqual(r[1], r[3], r)
                self.assertNotEqual(r[0], r[1], r)

    def test_losing_swing_alignment_mid_run_aborts_and_needs_a_new_press(self):
        # WHY: if the house is bumped off alignment during a fork calibration, the firmware drops
        # the run and does NOT resume on realignment; the 8 s dwell starts over after a new press.
        # FW: CalibForkRefPose -> _Inhibited on isSwingBasedAutoInhibited (MdlApp.c:18984);
        # CalibStepMgr abort guard [... || ~isCalibrating || ...] (chart_1210 T346, MdlApp.c:30133).
        h = booted()
        h.jump_to_step("CalibForkRefPose")
        h.tick(300)
        h.set_swing_aligned(False)
        h.tick(1)
        self.assertEqual(h.main_state(), "CalibForkRefPose_Inhibited")
        self.assertFalse(h.fw["y.isCalibrating"])
        self.assertEqual(h.calib_step(), "CalibStandby")
        h.set_swing_aligned(True)
        h.tick(100)
        self.assertEqual(h.main_state(), "CalibForkRefPose_Paused")
        self.assertEqual(h.calib_step(), "CalibStandby")
        h.trace("y.calibStep")
        h.pulse("u.jstAutoReq_StartPause")
        h.run_until(lambda h: h.calib_step() == "ForkRefPose_log", 9.0, "log")
        self.assertEqual(first_tick(h, "y.calibStep", calib_is(h, "ForkRefPose_log"))
                         - first_tick(h, "y.calibStep", calib_is(h, "ForkRefPose_stb")), STB_TICKS)

    def test_travel_needs_rtk_fixed_on_both_antennas_and_nothing_else(self):
        # WHY: spec A7 "only CalibTrvl requires methodGnss_Main == methodGnss_Aux == 4" --
        # confirmed, and it is ONLY the fix type: a bad vertical sigma with both antennas fixed
        # (low-accuracy state set, auto inhibited) does not stop travel calibration, and the other
        # nine steps need no GNSS at all.
        # FW: isGnssBasedCalibInhibited = (methodGnss_Main ~= 4 || methodGnss_Aux ~= 4),
        # MdlApp.c:39683; used only by CalibTrvl_Paused/CalibTrvl (MdlApp.c:22369, :21977).
        # Low-accuracy state = ~isRtkFixed || stdDevZ > poor threshold (ChkVerticalAccuracy <S157>,
        # MdlApp.c:39081-39106); BIT_POOR_ACCURACY is in AUTO_INHIBIT_MASK, not CALIB_INHIBIT_MASK.
        for main, aux, sigma, ok in ((4, 0, 0.008, False), (0, 4, 0.008, False),
                                     (5, 5, 0.008, False), (4, 4, 0.5, True), (4, 4, 0.008, True)):
            with self.subTest(main=main, aux=aux, sigma=sigma):
                h = booted(gnss=False)
                h.fw["u.methodGnss_Main"], h.fw["u.methodGnss_Aux"] = main, aux
                h.fw["u.gnssPosStdDevZ"] = sigma
                h.tick(30)                            # past the 20-tick accuracy on-delay
                low = sigma > 0.04 or not main == aux == 4
                self.assertEqual(bool(h.fw.internal("low_vertical_accuracy")), low)
                self.assertEqual(h.auto_inhibited(), low)
                h.jump_to_step("CalibTrvl")
                h.tick(2)
                self.assertEqual(bool(h.fw["y.isCalibrating"]), ok, h.describe())
        for name in CALIB_STEPS:
            if name == "CalibTrvl":
                continue
            with self.subTest(step=name, gnss="none"):
                h = booted(gnss=False)
                h.jump_to_step(name)
                self.assertTrue(h.fw["y.isCalibrating"])

    def test_calib_inhibit_mask_only_bites_when_isMachCalib_is_set(self):
        # WHY: spec A7 "isCalibInhibited is ANDed with isMachCalib" -- confirmed. isMachCalib is not
        # derived from the step number: it is Svc_Mod_Req.calibModEntdActr, a service-mode flag in
        # a different CAN message from the Auto_Mod_Req that carries autoReqStep (AppCtrlIf.c:141 <-
        # InpHndlr.c:1189 vs :1184). Whether the tablet raises it during steps 20-29 is not in the
        # firmware; if it does not, an IMU comm fault or remote-link error does not stop calibration.
        # FW: isCalibInhibited = (status & 13976) != 0 & (isMachCalib != 0), MdlApp.c:39679;
        # CALIB_INHIBIT_MASK 13976 includes BIT_IMU_COM_ERR and BIT_RMT_CTRL_ERR.
        for fault, bit in (("u.isChsImuFault", "IMU_COM_ERR"), ("u.isRmtOk", "RMT_CTRL_ERR")):
            for mach_calib in (0, 1):
                with self.subTest(fault=fault, isMachCalib=mach_calib):
                    h = booted()
                    h.fw[fault] = 0 if fault == "u.isRmtOk" else 1
                    h.fw["u.isMachCalib"] = mach_calib
                    h.tick(3)
                    self.assertTrue(h.inhibit_bit(bit))
                    h.jump_to_step("CalibChs")
                    h.tick(1)
                    self.assertEqual(bool(h.fw["y.isCalibrating"]), not mach_calib, h.describe())
        h = booted()                                            # clearing lands in Paused
        h.fw["u.isChsImuFault"] = 1
        h.fw["u.isMachCalib"] = 1
        h.tick(3)
        h.jump_to_step("CalibChs")
        self.assertEqual(h.main_state(), "CalibChs_Inhibited")
        h.fw["u.isChsImuFault"] = 0
        h.tick(20)
        self.assertEqual(h.main_state(), "CalibChs_Paused")
        self.assertFalse(h.fw["y.isCalibrating"])
        h.pulse("u.jstAutoReq_StartPause")
        self.assertTrue(h.fw["y.isCalibrating"])


# ------------------------------------------------------------------------------------
class TestForkRefPoseRun(unittest.TestCase):
    """Step 28 end to end: stb -> log -> save -> CalibStandby -> NoTarget (spec A7)."""

    TRACE = ("y.autoCtrl_CurrStep", "y.calibStep", "y.isCalibDataSaveReq")

    def _started(self, h):
        h.request_step("CalibForkRefPose")
        h.tick(1)
        h.trace(*self.TRACE)
        h.pulse("u.jstAutoReq_StartPause")
        return h

    @staticmethod
    def _notarget(h):
        v = h.fw.enum_value("AutoCtrlStep", "NoTarget")
        return lambda x: x == v

    def test_full_run_with_the_main_c_save_handshake(self):
        # WHY: this is the shortest end-to-end calibration a sim can pass; exact tick budget and a
        # one-tick save request are what an Isaac-side NVM emulation must reproduce.
        # FW: stb [cnt >= 800] MdlApp.c:30159; log [cnt >= 100] :29950; save during
        # isCalibDataSaveReq = true :30083; exit on hasChanged(isCalibDataSaved) && isCalibDataSaved
        # :30056-30061; main chart [calibStep == CalibStandby] -> NoTarget via Delay5 :19014;
        # handshake = main.c:394-419 (harness.SaveHandshake).
        emu = SaveHandshake()
        h = self._started(booted(plant=emu))
        h.run_until(lambda h: h.curr_step() == "NoTarget", 10.0, "NoTarget")
        h.tick(2)
        stb = first_tick(h, "y.calibStep", calib_is(h, "ForkRefPose_stb"))
        log = first_tick(h, "y.calibStep", calib_is(h, "ForkRefPose_log"))
        save = first_tick(h, "y.calibStep", calib_is(h, "ForkRefPose_save"))
        req = first_tick(h, "y.isCalibDataSaveReq", bool)
        back = first_tick(h, "y.calibStep", calib_is(h, "CalibStandby"), after=save)
        done = first_tick(h, "y.autoCtrl_CurrStep", self._notarget(h))
        self.assertEqual(log - stb, STB_TICKS)
        self.assertEqual(save - log, LOG_TICKS)
        self.assertEqual(req - save, 1)               # request is a during action, not entry
        self.assertEqual(back - req, 1)               # ack seen on the next step
        self.assertEqual(done - back, 1)              # main chart reads calibStep through Delay5
        self.assertEqual(len(emu.saved), 1)
        self.assertEqual(emu.reloads, 1)
        self.assertFalse(h.fw["u.isCalibDataSaved"])
        self.assertFalse(h.fw["y.isCalibrating"])
        self.assertEqual(saved_fork(emu.saved[0][1]), fork_values(h.fw, "y"))

    def test_without_the_handshake_save_lingers_for_the_cntSave_fallback(self):
        # WHY: spec A7 "1.0 s fallback, CntCalib_save = 100" -- confirmed to the tick: 102 ticks
        # in _save with the request held for 101. A harness with no NVM emulation still finishes.
        # FW: [cntSave > 100] MdlApp.c:30058; cntSave incremented in during only (:30094-30101).
        h = self._started(booted())
        h.run_until(lambda h: h.curr_step() == "NoTarget", 11.0, "NoTarget")
        save = first_tick(h, "y.calibStep", calib_is(h, "ForkRefPose_save"))
        req_on = first_tick(h, "y.isCalibDataSaveReq", bool)
        back = first_tick(h, "y.calibStep", calib_is(h, "CalibStandby"), after=save)
        done = first_tick(h, "y.autoCtrl_CurrStep", self._notarget(h))
        self.assertEqual(back - save, SAVE_FALLBACK_TICKS)
        self.assertEqual(back - req_on, SAVE_FALLBACK_TICKS - 1)
        self.assertEqual(sum(1 for r in h.trace_rows if r[3]), SAVE_FALLBACK_TICKS - 1)
        self.assertEqual(done - back, 1)

    def test_isCalibDataSaved_held_high_is_not_an_ack(self):
        # WHY: a lazy harness that ties isCalibDataSaved = 1 gets the slow fallback, not a fast
        # save: the chart needs an EDGE.
        # FW: hasChanged(isCalibDataSaved) && isCalibDataSaved, chart_1210 T346 (MdlApp.c:30056).
        h = booted()
        h.fw["u.isCalibDataSaved"] = 1
        h.tick(2)
        self._started(h)
        h.run_until(lambda h: h.curr_step() == "NoTarget", 11.0, "NoTarget")
        save = first_tick(h, "y.calibStep", calib_is(h, "ForkRefPose_save"))
        back = first_tick(h, "y.calibStep", calib_is(h, "CalibStandby"), after=save)
        self.assertEqual(back - save, SAVE_FALLBACK_TICKS)

    def test_stray_save_ack_aborts_but_looks_exactly_like_success(self):
        # WHY: spec A7 step 5 says "Success = calibStep returns to CalibStandby and CurrStep returns
        # to NoTarget". The firmware gives that SAME signature when a spurious isCalibDataSaved
        # edge arrives mid-dwell: the run aborts, nothing is logged or saved, and the main chart
        # still reports NoTarget. A SIL pass criterion must also require isCalibDataSaveReq to
        # have been raised.
        # FW: abort guard shared by every sub-step, chart_1210 T346 (source: connective junction
        # SSID 260), generated in ForkRefPose_stb at MdlApp.c:30133; main chart NoTarget at :19014.
        h = self._started(booted())
        h.tick(100)
        before = fork_values(h.fw, "y")
        h.fw["u.isCalibDataSaved"] = 1
        h.tick(1)
        self.assertEqual(h.calib_step(), "CalibStandby")
        h.tick(1)
        self.assertEqual(h.curr_step(), "NoTarget")
        self.assertFalse(h.fw["y.isCalibrating"])
        self.assertFalse(any(r[3] for r in h.trace_rows))
        self.assertFalse(any(r[2] == h.fw.enum_value("CalibStep", "ForkRefPose_log") for r in h.trace_rows))
        self.assertEqual(fork_values(h.fw, "y"), before)

    def test_pause_mid_dwell_restarts_the_dwell(self):
        # WHY: the dwell is not resumable. A sim that pauses (e.g. to reposition the camera or to
        # test the Stop button) must budget a fresh 8.01 s after resuming.
        # FW: running -> _Paused on autoReq_Stop (MdlApp.c:18960) sets isCalibrating false;
        # CalibStepMgr aborts to Standby on ~isCalibrating (MdlApp.c:30133); stb entry cnt = 0.
        h = booted()
        h.jump_to_step("CalibForkRefPose")
        h.tick(400)
        h.fw["u.tabletAutoReq_Stop"] = 1
        h.tick(1)
        self.assertEqual(h.main_state(), "CalibForkRefPose_Paused")
        self.assertFalse(h.fw["y.isCalibrating"])
        self.assertEqual(h.calib_step(), "CalibStandby")
        h.tick(10)
        h.fw["u.tabletAutoReq_Stop"] = 0
        h.tick(1)
        h.trace("y.calibStep")
        h.pulse("u.jstAutoReq_StartPause")
        self.assertEqual(h.calib_step(), "ForkRefPose_stb")
        h.run_until(lambda h: h.calib_step() == "ForkRefPose_log", 9.0, "log")
        self.assertEqual(first_tick(h, "y.calibStep", calib_is(h, "ForkRefPose_log"))
                         - first_tick(h, "y.calibStep", calib_is(h, "ForkRefPose_stb")), STB_TICKS)


# ------------------------------------------------------------------------------------
class TestForkCalibWrites(unittest.TestCase):
    """What steps 28/29 write, and whether it is the inverse of the firmware's own FK."""

    def test_step28_stores_an_fk_read_of_whatever_pose_the_imus_report(self):
        # WHY: step 28 has no plausibility check. At the harness rest pose (boom -40, arm 90, input
        # link -60 deg; tool nowhere near the fork) it stores the contact-surface pitch as the fork
        # up limit: -68.1 deg against the compiled -16.3 deg, distUcToForkBack [4.85, 0, -0.48] m
        # against [1.84, 0, -0.70], and silently drops a 0.150 m lateral offset. A sim whose pose is
        # wrong still "passes" step 28. The value is written during _log (inputs moving during _stb
        # do not reach it) and never flows back into parLocalTest (the *Stored inports are unread).
        # FW: chart_2352 l.16-21 R_forkBack_uc = uc.R' * cs.R * Ry(-pi/2); ang = -atan2(R(3,1), R(1,1));
        # dist = [v(1); 0; v(3)] (MdlApp.c:46043-46096); init from parKinStored = parLocalTest via
        # Switch6 (MdlApp.c:45925, :41463); y.parKin <- persistent (MdlApp.c:46150-46155).
        h = booted()
        fw = h.fw
        par0 = fork_values(fw, "par")
        self.assertEqual(fork_values(fw, "y"), par0)
        h.jump_to_step("CalibForkRefPose")

        h.tick(100)                                          # stb: inputs move, output must not
        cs_before = vec(fw, "y.links.contactSurface.p")
        kin.publish_imus(fw, {"chs": kin.Ry(math.radians(10))})  # chassis IMU alone reads 10 deg pitch
        h.tick(300)
        self.assertGreater(np.linalg.norm(vec(fw, "y.links.contactSurface.p") - cs_before), 1e-3)
        self.assertEqual(fork_values(fw, "y"), par0)
        h.set_pose(**Harness.NOMINAL_POSE)

        h.run_until(lambda h: h.calib_step() == "ForkRefPose_log", 9.0, "log")
        self.assertNotEqual(fw["y.parKin.angForkUpLimit"], par0["angForkUpLimit"])  # at log entry
        h.run_until(lambda h: h.curr_step() == "NoTarget", 2.2, "done")

        ang, v = fork_ref_pose_formula(mat(fw, "y.links.uc.R"), vec(fw, "y.links.uc.p"),
                                       mat(fw, "y.links.contactSurface.R"),
                                       vec(fw, "y.links.contactSurface.p"))
        got = fork_values(fw, "y")
        self.assertAlmostEqual(got["angForkUpLimit"], ang, delta=2e-5)
        self.assertAlmostEqual(got["distUcToForkBack"][0], v[0], delta=2e-5)
        self.assertEqual(got["distUcToForkBack"][1], 0.0)
        self.assertAlmostEqual(got["distUcToForkBack"][2], v[2], delta=2e-5)

        # Independent of the firmware's own contact-surface output: the shared library's link
        # frames predict the pitch (premise: contact surface has no angular offset from the tilt
        # frame in this build, and the house is level so uc.R = I).
        for p in ("angRotToAtt", "angAttToContactSurface"):
            self.assertEqual(fw[f"par.parKin.{p}"], 0.0)
        self.assertTrue(np.allclose(mat(fw, "y.links.uc.R"), np.eye(3), atol=1e-6))
        R_tilt = kin.link_frames(fw, None, **Harness.NOMINAL_POSE)["tilt"]
        self.assertAlmostEqual(got["angForkUpLimit"], pitch(R_tilt @ kin.Ry(-math.pi / 2)), delta=2e-5)

        # the rest-pose numbers themselves (ShortArm build, test_core pins that)
        self.assertAlmostEqual(math.degrees(got["angForkUpLimit"]), -68.10, places=1)
        self.assertAlmostEqual(got["distUcToForkBack"][0], 4.851, places=2)
        self.assertAlmostEqual(got["distUcToForkBack"][2], -0.476, places=2)
        self.assertAlmostEqual(v[1], -0.150, places=3)              # discarded lateral offset

        self.assertEqual(got["distForkBackToPanelTop"], par0["distForkBackToPanelTop"])  # step 29's
        self.assertEqual(fork_values(fw, "par"), par0)                                   # no feedback
        self.assertAlmostEqual(fw["y.jnts.UcToFork.q"], par0["angForkUpLimit"], places=6)

    def test_step28_recovers_the_compiled_fork_geometry_with_the_tool_in_the_cradle(self):
        # WHY: this is the spec's step-28 pass criterion ("the firmware recovers the fork geometry"),
        # end to end: drive the IMUs until the firmware's own contact surface sits on its own
        # forkBack frame (cs.R = forkBack.R * Ry(pi/2), same x/z in the uc frame), run step 28 with
        # the save handshake, and compare what reaches NVM with the compiled parameters. It holds to
        # float32 resolution. What Isaac must reproduce: the tool-in-cradle pose, here boom -23.2,
        # arm 161.9, input link -88.2 deg. The 0.150 m lateral offset between the contact surface
        # and the fork back is not checked by the firmware (y is zeroed).
        # FW: forward forkBack = PropagateRp(uc.R, uc.p, Ry(jntAng_Fork), distUcToForkBack),
        # p_child = R_parent * t + p_parent (chart_2143 l.218, l.440; MdlApp.c:12778), with
        # jntAng_Fork = LPF(parLocalTest angForkUpLimit) (MdlApp.c:12420); inverse chart_2352 l.16-21.
        emu = SaveHandshake()
        h = booted(plant=emu)
        fw = h.fw
        par = fork_values(fw, "par")

        h.tick(SETTLE_TICKS)                                 # forward model, as the IK target
        uc_R, uc_p = mat(fw, "y.links.uc.R"), vec(fw, "y.links.uc.p")
        self.assertTrue(np.allclose(mat(fw, "y.links.forkBack.R"), uc_R @ kin.Ry(par["angForkUpLimit"]), atol=1e-6))
        self.assertTrue(np.allclose(vec(fw, "y.links.forkBack.p"), uc_p + uc_R @ par["distUcToForkBack"], atol=1e-5))

        def cradle(fw):
            cs, fb = in_uc(fw, "y.links.contactSurface.p"), in_uc(fw, "y.links.forkBack.p")
            return np.array([cs[0] - fb[0], cs[2] - fb[2], cs_pitch_error_to_fork_back(fw)])

        q = solve_pose(h, cradle, [math.radians(a) for a in (-40, 90, -60)])
        np.testing.assert_allclose(np.degrees(q), [-23.21, 161.88, -88.24], atol=0.05)
        lateral = in_uc(fw, "y.links.contactSurface.p")[1] - in_uc(fw, "y.links.forkBack.p")[1]
        self.assertAlmostEqual(lateral, -0.150, places=3)

        h.jump_to_step("CalibForkRefPose")
        h.run_until(lambda h: h.curr_step() == "NoTarget", 10.0, "step 28 done")
        self.assertEqual(len(emu.saved), 1)
        nvm = saved_fork(emu.saved[0][1])
        self.assertAlmostEqual(nvm["angForkUpLimit"], par["angForkUpLimit"], delta=2e-6)
        np.testing.assert_allclose(nvm["distUcToForkBack"], par["distUcToForkBack"], atol=2e-6)
        self.assertEqual(nvm["distForkBackToPanelTop"], par["distForkBackToPanelTop"])

    def test_step28_keeps_the_last_log_sample_not_an_average(self):
        # WHY: spec A7 describes _log as a 1 s running mean -- true for the IMU accelerometer
        # (chart_2291 l.62-64 UpdateUnitAccAvg) but NOT for the fork: UpdateForKin overwrites its
        # persistent every _log tick, so one 10 ms glitch on the last tick IS the calibration.
        # The sim's IMU noise on that single tick sets the stored fork geometry.
        # FW: chart_2352 `if calibStep == CalibStep.ForkRefPose_log ... angForkUpLimit = ...`
        # with no accumulator (MdlApp.c:46043, :46090).
        h = booted()
        fw = h.fw
        h.jump_to_step("CalibForkRefPose")
        h.run_until(lambda h: h.calib_step() == "ForkRefPose_log", 9.0, "log")
        window = [fw["y.parKin.angForkUpLimit"]]
        for _ in range(LOG_TICKS - 2):
            h.tick()
            self.assertEqual(h.calib_step(), "ForkRefPose_log")
            window.append(fw["y.parKin.angForkUpLimit"])
        kin.publish_imus(fw, {"chs": kin.Ry(math.radians(10))})   # glitch on the last log tick only
        h.tick()
        self.assertEqual(h.calib_step(), "ForkRefPose_log")
        last = fw["y.parKin.angForkUpLimit"]
        h.set_pose(**Harness.NOMINAL_POSE)
        h.tick()
        self.assertEqual(h.calib_step(), "ForkRefPose_save")
        stored = fw["y.parKin.angForkUpLimit"]
        h.tick(200)
        self.assertEqual(fw["y.parKin.angForkUpLimit"], stored)
        self.assertEqual(stored, last)
        self.assertEqual(len(set(window)), 1)
        mean = (sum(window) + last) / (len(window) + 1)
        self.assertGreater(abs(stored - window[0]), 1e-3)
        self.assertLess(abs(mean - window[0]), abs(stored - window[0]) / 50)

    def test_step29_measures_the_probe_from_the_uc_origin_not_the_fork_back(self):
        # WHY: step 29 stores (R_forkBack_W' * (probe.p - uc.p))_x as distForkBackToPanelTop, but the
        # forward model uses that parameter as an offset FROM forkBack (panelTop.p = forkBack.R*d +
        # forkBack.p). End to end: put the firmware's own probe exactly on the firmware's own panel
        # top with the tool in the cradle orientation, run step 29, and it stores 2.589 m instead of
        # the compiled 1.0165 m -- off by (forkBack.R' * (forkBack.p - uc.p))_x = 1.572 m, i.e. the
        # round trip does not close. The commented-out ForkPntBack stage differences the same
        # uc-origin vector against this value (MdlApp.c:46144-46149), which would cancel the offset:
        # step 29 looks like the first half of a two-point procedure whose second half was removed.
        # FW: chart_2352 l.24-27 (MdlApp.c:46099-46140); forward chart_2143 l.224 + PropagateRp
        # l.440 (MdlApp.c:12788).
        emu = SaveHandshake()
        h = booted(plant=emu)
        fw = h.fw
        d = np.array(fw["par.parKin.distForkBackToPanelTop"], dtype=float)
        q_fork = fw["par.parKin.angForkUpLimit"]
        d_uc = fw["par.parKin.distUcToForkBack"]

        def probe_on_panel_top(fw):
            uc_R, uc_p = mat(fw, "y.links.uc.R"), vec(fw, "y.links.uc.p")
            top = uc_R.T @ (mat(fw, "y.links.forkBack.R") @ d + vec(fw, "y.links.forkBack.p") - uc_p)
            pr = in_uc(fw, "y.links.probe.p")
            return np.array([pr[0] - top[0], pr[2] - top[2], cs_pitch_error_to_fork_back(fw)])

        solve_pose(h, probe_on_panel_top, [math.radians(a) for a in (-40, 90, -60)])
        fb_R, fb_p, uc_p = mat(fw, "y.links.forkBack.R"), vec(fw, "y.links.forkBack.p"), vec(fw, "y.links.uc.p")
        bias = (fb_R.T @ (fb_p - uc_p))[0]
        self.assertAlmostEqual(bias, math.cos(q_fork) * d_uc[0] - math.sin(q_fork) * d_uc[2], places=5)
        self.assertAlmostEqual(bias, 1.5723, places=4)

        h.jump_to_step("CalibForkPntFront")
        h.run_until(lambda h: h.curr_step() == "NoTarget", 10.0, "step 29 done")
        self.assertEqual(len(emu.saved), 1)
        got = saved_fork(emu.saved[0][1])["distForkBackToPanelTop"]
        self.assertEqual(got, fw["y.parKin.distForkBackToPanelTop"])
        self.assertAlmostEqual(got[0], d[0] + bias, delta=2e-5)
        self.assertAlmostEqual(got[0], 2.5888, places=3)
        self.assertEqual(got[1:], [0.0, 0.0])
        self.assertGreater(abs(got[0] - d[0]), 1.5)

        cs_R, probe = mat(fw, "y.links.contactSurface.R"), vec(fw, "y.links.probe.p")
        self.assertAlmostEqual(got[0], fork_pnt_front_formula(cs_R, probe, uc_p), delta=2e-5)
        self.assertAlmostEqual(fork_pnt_front_formula(cs_R, probe, fb_p), d[0], delta=2e-5)

        par = fork_values(fw, "par")                            # step 29 leaves step 28's fields
        self.assertEqual(fw["y.parKin.angForkUpLimit"], par["angForkUpLimit"])
        self.assertEqual(fw["y.parKin.distUcToForkBack"], par["distUcToForkBack"])

    def test_aborted_step28_value_is_persisted_by_the_next_save(self):
        # WHY: pausing step 28 inside _log raises no save request, yet the half-finished fork
        # geometry stays in y.parKin; the real ECU copies y.parKin's fork fields into kin_s on every
        # isCalibrating tick (AppCtrlIf.c:802, :1014-1017) and SaveInternalParam() persists kin_s on
        # the next save of ANY calibration (main.c:405-409, InternalParam.c:72-83). A later step 29
        # therefore writes the aborted step-28 value (here the rest-pose -68 deg) to NVM.
        # FW: chart_2352 persistent is never reverted on abort (the log branch at MdlApp.c:46043 is its
        # only writer after init :45925); CalibStepMgr abort on ~isCalibrating (MdlApp.c:29983-30012).
        emu = SaveHandshake()
        h = booted(plant=emu)
        fw = h.fw
        compiled = fw["par.parKin.angForkUpLimit"]
        h.jump_to_step("CalibForkRefPose")
        h.run_until(lambda h: h.calib_step() == "ForkRefPose_log", 9.0, "log")
        h.tick(50)
        aborted = fw["y.parKin.angForkUpLimit"]
        self.assertNotAlmostEqual(aborted, compiled, places=3)
        h.fw["u.jstAutoReq_StartPause"] = 1
        h.tick(1)
        h.fw["u.jstAutoReq_StartPause"] = 0
        self.assertEqual(h.main_state(), "CalibForkRefPose_Paused")
        self.assertEqual(h.calib_step(), "CalibStandby")
        fw["u.tabletAutoReq_Cancel"] = 1
        h.tick(2)
        fw["u.tabletAutoReq_Cancel"] = 0
        self.assertEqual(h.main_state(), "NoTarget")
        self.assertEqual(emu.saved, [])
        h.tick(500)
        self.assertEqual(fw["y.parKin.angForkUpLimit"], aborted)

        h.jump_to_step("CalibForkPntFront")
        h.run_until(lambda h: h.curr_step() == "NoTarget", 10.0, "step 29 done")
        self.assertEqual(len(emu.saved), 1)
        self.assertEqual(saved_fork(emu.saved[0][1])["angForkUpLimit"], aborted)


# ------------------------------------------------------------------------------------
class TestAxisCalibWithoutPlant(unittest.TestCase):
    """Step 20 on a stationary level machine (pilot cut-off / no hydraulics in the sim)."""

    def test_noise_free_level_machine_parks_in_the_swing_deadband_staircase(self):
        # WHY: spec A7 "_Min: 20 % + 0.2 % every 2 s, saturating at 100 %, no timeout" -- confirmed
        # to the tick. The spec's level-ground signature (see the next test) needs a motion-onset
        # pulse first; a noise-free stationary machine never produces one (onset is a 0.5 deg change
        # of the PLANAR accelerometer angle), so it parks in ChsLeMin forever, valve at 100 %.
        # FW: SetPropVlvCmdCalib staircase (chart_3055 l.53-66, 76, 83; SysPar.m:105, 109-110, 164);
        # onset = |angCalib.chs - ref| > 0.5 deg (chart_2316 l.69, SysPar.m:131, MdlApp.c:44229);
        # angCalib.chs = |planar angle of accRaw vs accRef| (chart_2291 l.107-129).
        emu = SaveHandshake()
        h = booted(plant=emu)
        fw = h.fw
        h.jump_to_step("CalibChs")
        h.run_until(lambda h: h.calib_step() == "ChsLeMin", 9.2, "ChsLeMin")
        t0 = h.tick_count
        self.assertEqual(fw["y.propVlvCmd.swingLe"], 20.0)
        h.tick(198)                                   # the entry tick counts as the first of 200
        self.assertEqual(fw["y.propVlvCmd.swingLe"], 20.0)
        h.tick(1)
        self.assertAlmostEqual(fw["y.propVlvCmd.swingLe"], 20.2, places=4)
        h.tick(200)
        self.assertAlmostEqual(fw["y.propVlvCmd.swingLe"], 20.4, places=4)
        first_100 = None
        while h.tick_count - t0 < 82000:
            h.tick()
            if first_100 is None and fw["y.propVlvCmd.swingLe"] >= 100.0:
                first_100 = h.tick_count - t0
        self.assertEqual(first_100, 400 * 200 - 1)
        self.assertEqual(fw["y.propVlvCmd.swingLe"], 100.0)
        self.assertEqual(h.calib_step(), "ChsLeMin")
        self.assertTrue(fw["y.isCalibrating"])
        self.assertEqual(emu.saved, [])

    def test_stationary_level_machine_with_accel_noise_completes_and_saves(self):
        # WHY: the dangerous twin of the test above. Same stationary machine plus accelerometer
        # noise of one CAN LSB (2^-14 g, CAN1_X1Exc.dbc:46) or 1 mg rms. The published chassis
        # accelerometer carries a 3.1 mg horizontal component (the board's mount tilt, par.imuChs
        # a13/a23), so the planar angle jitters by a few degrees: every onset fires, ChsLeMoveToPnt1
        # exits on its 10 s timeout, ChsRiMoveToPnt2 exits on its first evaluation (its guard is
        # angCalib.chs < 80 deg), and imuMntOri_chs stays bit-identical -- exactly spec A7's
        # level-ground signature. What the spec does not say: the run raises the save request and
        # REWRITES the swing speed table from a machine that never moved (min 19.5 % = 20 - 0.5, peak
        # speed clamped to MinTblReqSpd 0.002 rad/s instead of 0.589), and that table's breakpoints
        # [0, 0.01, 0.002] are no longer monotonic although SysPar.m:112 sets the clamp "to have
        # monotonous change of the input array" -- the fixed 0.01 midpoint is above it. "Returned to
        # NoTarget" is not evidence the machine moved. (Seeds 1..20: 20/20 timeouts at 2^-14 g,
        # 18/20 at 1 mg -- two seeds crossed 170 deg on noise; all 40 runs saved, all 40 left the
        # mount bit-identical.)
        # FW: _ToPnt guards MdlApp.c:28887 (angCalib.chs > AngChsPnt1 || cnt > CntCalib_timeout) and
        # :29662 (angCalib.chs < AngChsPnt2 || ...); mount only rebuilt if |cross(v1, v2)| > 1e-6
        # (chart_2291 l.191-213); table chart_2338 l.34, 53, 78-79; NVM copy AppCtrlIf.c:912-914.
        for sigma in (2.0 ** -14, 1e-3):
            with self.subTest(sigma=sigma):
                emu = SaveHandshake()
                h = booted(plant=[AccelNoise(sigma, seed=7), emu])
                fw = h.fw
                mnt = [f"imuMntOri_chs.a{r}{c}" for r in (1, 2, 3) for c in (1, 2, 3)]
                compiled_mnt = [fw[f"par.imuChs.a{r}{c}"] for r in (1, 2, 3) for c in (1, 2, 3)]
                self.assertGreater(math.hypot(compiled_mnt[2], compiled_mnt[5]), 3e-3)
                stored_peak = fw["par.reqSpdToActCmd.swingLe_X"][2]
                h.trace("y.calibStep")
                h.jump_to_step("CalibChs")
                h.run_until(lambda h: h.curr_step() == "NoTarget", 60.0, "CalibChs done")
                at = lambda name: first_tick(h, "y.calibStep", calib_is(h, name))
                self.assertEqual(at("ChsPnt1_stb") - at("ChsLeMoveToPnt1"), TIMEOUT_TICKS)
                self.assertEqual(at("ChsPnt2_stb") - at("ChsRiMoveToPnt2"), 1)
                self.assertLess(at("Chs_save") - at("ChsPntRef_stb"), 5400)
                self.assertEqual(len(emu.saved), 1)
                snap = emu.saved[0][1]
                self.assertEqual([snap[f"y.{p}"] for p in mnt], compiled_mnt)
                self.assertEqual(snap["y.tblReqSpdToActCmd.swingLe_Y"][1], 19.5)
                x = snap["y.tblReqSpdToActCmd.swingLe_X"]
                self.assertAlmostEqual(x[2], MIN_TBL_REQ_SPD, places=6)
                self.assertLess(x[2], 0.01 * stored_peak)
                self.assertLess(x[2], x[1])                          # non-monotonic breakpoints

    def test_stationary_machine_with_30mg_vibration_rewrites_the_chassis_imu_mount(self):
        # WHY: raise the stationary machine's accelerometer noise to engine-vibration level (30 mg)
        # and the 1 s means v1 = accPnt1 - accRef, v2 = accPnt2 - accRef grow past the
        # |cross(v1, v2)| > 1e-6 gate, so a machine that never swung saves a new chassis IMU mount
        # built from noise -- a valid rotation (det = +1) with a random yaw, here with z flipped --
        # and nothing flags it. (Seeds 1..20: all 20 moved an element by > 0.25, 16 by > 0.8,
        # 11 flipped a33.) In this build the result cannot hurt the controller only because
        # kinematics reads parLocalTest.imuChs, not the calibrated output; the sim must still gate
        # on it, because NVM receives it.
        # FW: chart_2291 l.191-213 (v1, v2, vzNorm > 1e-6, imuMntOriCalib_chs = [vx vy vz]); copied to
        # INTP.imuMntCalib_s while isCalibrating (AppCtrlIf.c:802-813); link attitude uses
        # parLocalTest.imuChs (MdlApp.c:41898), MdlApp_U.*Stored has no reads; BIT_IMU_CALIB_ERR only
        # from SanityCheckImuCalib's mast/base/uc Euler windows (chart_2516, MdlApp.c:39570).
        emu = SaveHandshake()
        h = booted(plant=[AccelNoise(3e-2, seed=7), emu])
        fw = h.fw
        mnt = [f"y.imuMntOri_chs.a{r}{c}" for r in (1, 2, 3) for c in (1, 2, 3)]
        before = np.array([fw[p] for p in mnt])
        chs_R_before = mat(fw, "y.links.chs.R")
        self.assertAlmostEqual(before[8], 1.0, places=3)
        h.jump_to_step("CalibChs")
        h.run_until(lambda h: h.curr_step() == "NoTarget", 60.0, "CalibChs done")
        after = np.array([fw[p] for p in mnt])
        self.assertEqual(len(emu.saved), 1)
        self.assertEqual([emu.saved[0][1][p] for p in mnt], list(after))
        self.assertGreater(np.abs(after - before).max(), 0.5)
        self.assertAlmostEqual(np.linalg.det(after.reshape(3, 3)), 1.0, places=3)
        self.assertLess(after[8], -0.99)
        h.tick(300)
        self.assertNotIn("BIT_IMU_CALIB_ERR", h.inhibit_names())
        np.testing.assert_allclose(mat(fw, "y.links.chs.R"), chs_R_before, atol=1e-5)


if __name__ == "__main__":
    unittest.main()
