"""
Main AutoCtrlStep cycle: ENTRY and CONTROL semantics of Subsystem/AutoStsMgr/Chart
(chart_2537, generated as '<S136>' inside MdlApp_Subsystem, MdlApp.c:22700-25460 and
:40090-40450), the tarPanelData latch (ValidateTarPanelData, MdlApp.c:40922-40965) and the
valve arbitration that turns StartStopSts into motion ('<S196>', MdlApp.c:51737-51763).

What resources/X1Exc_SIL_spec.md claims (A6 steps 1-2, "Pause / inhibit semantics", A3.6)
is treated as a hypothesis. Where the running firmware disagrees, the assertion follows
the firmware and the comment says so.

Conventions used below
  * "Paused" and "Inhibited" both report (CurrStep=<X>, StartStopSts=False). h.main_state()
    reads the chart's active state ('PositioningPaused' vs 'PositioningInhibited'). Where the
    difference matters the tests ALSO check it by behaviour, sampling EVERY tick from a
    StartPause edge (_press_and_sample): Paused resumes on the edge tick (T181) even with an
    inhibit active and only then drops back, Inhibited never runs.
  * h.positioning_step() is MdlApp_B.positioningStep (also on y.dbg_F64_P01, MdlApp.c:52127).
  * No plant. nominal_inputs() publishes a physical rest pose: house square to the tracks
    (swing q = 0, never commanded), tilt 0 (already the Swing/Align tilt target, so Swing
    commands nothing unless a test offsets the tilt). The swing proximity switch is held
    CLOSED (h.set_swing_aligned) because the house is square. With no site calibration
    Localization parks the machine facing grid East (y.machHeading 90 deg, sil/geodesy.py).
  * THE LATCH IS NOT THE WHOLE STORY. tarPanelIdAck and the release pose (SetPosePanelRelease,
    visible as y.panelPoseErr) use the latched copy; the UC travel target (CalcUcPathTarget,
    chart_3193) reads the RAW tarPanelData bus every tick (MdlApp.c:52524-52583 copies the
    inport, :9450-9520 reads it). See test_retarget_while_running_*.
  * TRAVEL IS HELD 80 TICKS AFTER THE TRAVEL DIRECTION FLIPS. CalcFctDmdTrvl (chart_1152) starts
    its counter at CntDirChgDly = 80 ('<S124>:1:7-8', MdlApp.c:47924-47926), resets it when
    dirReversed changes and zeroes both track commands while it is below 80 ('<S124>:1:172-184',
    MdlApp.c:48285-48320, SysPar.m:412). CalcUcPathBasis forces dirReversed = false while
    ~isTarPanelValid ('<S146>:1:8-9', MdlApp.c:10106-10111) and sets it from tarPassedSts once
    valid ('<S146>:1:45-54', MdlApp.c:10203). The default target puts the UC stop point 0.157 m
    BEHIND the machine, so it flips on the latch and a start right after targeting commands no
    travel until ack + 80. Tests that need a running valve command wait (_run_until_valves) or
    dwell in Standby first.
  * With the default valid target Positioning never completes (distToWpTarLat 1.937 m >
    LatPositioningTol 0.20, SysPar.m:446). With an INVALID latch it can -- see
    test_target_cleared_while_paused_resumes_into_picking_when_facing_grid_east.

Run:  python3 -m unittest sil.tests.test_main_cycle -v     (from the repo root)
"""
import math
import unittest

from sil import kinematics as kin
from sil.harness import Harness

INVALID_TAR_PANEL_ID = 0xFFFFFF            # SysPar.m:65 InvalidTarPanelId = 2^24 - 1
TRAVEL_DIR_CHANGE_HOLD = 80                # CntDirChgDly, SysPar.m:412
SWING_REACHED_CONFIRM = 30                 # CntTarReachedConfirm, SysPar.m:471
DEFAULT_EAST, RETARGET_EAST = 4.35, -4.35  # set_target_panel default and the retarget used below

# SetCtrlMode (chart_2123): which axes are closed-loop in each Positioning sub-step.
CTRL_AXES_IN_POSITIONING = {
    "PositioningStep_Raise": {"bm1", "arm", "link"},                  # TaskSpace
    "PositioningStep_Swing": {"swing", "tilt", "rot"},                # JntSpace
    "PositioningStep_Align": {"trvlLe", "trvlRi", "tilt", "rot"},     # trvl Enabled, tilt/rot JntSpace
}
# chart_2123 Picking/PreparePick: trvl Enabled, bm1/arm/link/rotate JntSpace.
CTRL_AXES_IN_PREPARE_PICK = {"trvlLe", "trvlRi", "bm1", "arm", "link", "rot"}
_AXIS_PREFIXES = ("trvlLe", "trvlRi", "swing", "bm1", "bm2", "arm", "link", "tilt", "rot", "blade")


def _axis(port):
    """propVlvCmd port name -> axis. Kept local: the harness has no port->axis map."""
    return next(a for a in _AXIS_PREFIXES if port.startswith(a))


def _axes(valves):
    return {_axis(p) for p in valves}


def _dist_to_wp_target(h):
    """(distToWpTarLat, distToWpTarLon) in metres, from the UC travel target. Only visible on
    debug outports y.dbg_F64_P17 / P18 (MdlApp.c:51993, :51998); re-check if upstream rewires
    them. No harness equivalent."""
    return h.fw["y.dbg_F64_P17"], h.fw["y.dbg_F64_P18"]


def _release_pose_err(h):
    """y.panelPoseErr = posePanelRelease - contactSurface ('<S135>:1:4-22', MdlApp.c:50268-50275).
    posePanelRelease is built from the LATCHED tarPanelDataAck (MdlApp.c:41070, :41380), so with
    no plant (contact surface fixed) this tuple moves only when the latch takes new geometry."""
    return tuple(h.fw[f"y.panelPoseErr.{k}"] for k in ("easting", "northing", "height", "yaw"))


def _to_standby(h, panel_id=7, dwell_ticks=0, R_chs=None, **target):
    """Boot gates cleared, target latched, NoTarget -> Standby (spec A6 steps 0-1).
    R_chs re-publishes the rest pose with that chassis attitude AFTER nominal_inputs() (which
    would otherwise overwrite it). dwell_ticks >= TRAVEL_DIR_CHANGE_HOLD lets travel start on
    the Align entry tick."""
    h.nominal_inputs()
    if R_chs is not None:
        h.set_pose(R_chs=R_chs, **Harness.NOMINAL_POSE)
    h.set_swing_aligned(True)
    h.gnss_rtk_fixed()
    h.set_target_panel(panel_id=panel_id, **target)
    h.tick(3)
    h.request_step("Standby")
    h.run_until(lambda h: h.curr_step() == "Standby", 0.5, "Standby")
    h.tick(dwell_ticks)
    return h


def _to_running_positioning(h, panel_id=7, dwell_ticks=0, **kw):
    _to_standby(h, panel_id, dwell_ticks, **kw)
    h.pulse("u.jstAutoReq_StartPause")
    assert h.main_state() == "Positioning" and h.is_running(), h.describe()
    return h


def _run_until_valves(h, timeout_s=1.5):
    """Wait for a non-zero auto valve command (travel, after the direction-change hold)."""
    h.run_until(lambda h: bool(h.valves()), timeout_s, "a non-zero auto valve command")
    return h


def _press_and_sample(h, ticks):
    """jstAutoReq_StartPause high for one tick, then low; returns one sample per tick starting
    WITH the edge tick: (main_state, running, valves, positioning_step). Kept local because
    harness.pulse() does not expose the state inside its ticks, and the one-tick runs this file
    documents are only visible there. Caller must leave the switch low for at least one tick
    before (every helper above ends that way)."""
    assert h.fw["u.jstAutoReq_StartPause"] == 0
    h.fw["u.jstAutoReq_StartPause"] = 1
    samples = []
    for n in range(ticks):
        h.tick()
        if n == 0:
            h.fw["u.jstAutoReq_StartPause"] = 0
        samples.append((h.main_state(), h.is_running(), h.valves(), h.positioning_step()))
    return samples


def _running(h):
    return h.main_state() == "Positioning" and h.is_running()


def _paused(h):
    return h.main_state() == "PositioningPaused" and h.curr_step() == "Positioning" and not h.is_running()


def _inhibited(h):
    return h.main_state() == "PositioningInhibited" and h.curr_step() == "Positioning" and not h.is_running()


class TestStartPauseSwitch(unittest.TestCase):
    """Spec A6 step 2 and A3.4: the auto switch is edge-detected."""

    def setUp(self):
        self.h = Harness().reset()

    def test_joystick_edge_starts_positioning_from_standby(self):
        # WHY: the first thing a SIL scenario does after targeting is press Auto; if this
        # edge is missed the whole cycle never starts. T281 MdlApp.c:25278-25285 (guard
        # hasChanged(StartPause) && StartPause && isTarPanelValid).
        h = _to_standby(self.h)
        h.fw["u.jstAutoReq_StartPause"] = 1
        h.tick()
        self.assertEqual(h.curr_step(), "Positioning")
        self.assertEqual(h.fw["y.autoCtrl_CurrStep"], 2)
        self.assertEqual(h.main_state(), "Positioning")
        self.assertTrue(h.is_running())
        self.assertFalse(h.fw["y.isCalibrating"])

    def test_remote_edge_is_ored_in(self):
        # WHY: the radio remote is a second, independent Auto button the sim must be able
        # to drive. MdlApp.c:39724 LogicalOperator_lv2w = jstAutoReq_StartPause | rmtAutoReq_StartPause.
        # Two back-to-back pulses are two edges since harness.pulse() ticks the low level.
        h = _to_standby(self.h)
        h.pulse("u.rmtAutoReq_StartPause")
        self.assertTrue(_running(h), h.describe())
        h.pulse("u.rmtAutoReq_StartPause")
        self.assertTrue(_paused(h), h.describe())

    def test_same_edge_pauses_and_resumes_without_changing_curr_step(self):
        # WHY: pause/resume is the operator's primary control during SIL runs; CurrStep must
        # not move so the tablet (and our scenario checker) still knows where it is.
        # T193 MdlApp.c:23902-23907 (Positioning -> PositioningPaused), T181 MdlApp.c:24156-24162
        # (back); arbitration else-branch zeroes every port, MdlApp.c:51763.
        h = _run_until_valves(_to_running_positioning(self.h))
        h.pulse("u.jstAutoReq_StartPause")
        self.assertTrue(_paused(h), h.describe())
        self.assertEqual(h.valves(), {})
        h.tick(50)
        self.assertTrue(_paused(h), "a pause must hold with no input")
        h.pulse("u.jstAutoReq_StartPause")
        self.assertTrue(_running(h), h.describe())

    def test_held_switch_produces_exactly_one_toggle(self):
        # WHY: a harness that holds the switch level instead of pulsing would silently get
        # one transition and then nothing. Single previous-sample store in the chart gateway,
        # MdlApp.c:39743-39744.
        h = _to_standby(self.h)
        fw = h.fw
        fw["u.jstAutoReq_StartPause"] = 1
        h.tick()
        self.assertTrue(_running(h))
        for _ in range(100):                       # 1 s held high
            h.tick()
            self.assertTrue(_running(h), h.describe())
        fw["u.jstAutoReq_StartPause"] = 0
        h.tick(5)
        self.assertTrue(_running(h), "the falling edge must not toggle")
        fw["u.jstAutoReq_StartPause"] = 1
        h.tick(100)
        self.assertTrue(_paused(h), h.describe())

    def test_held_joystick_switch_masks_the_remote_pause(self):
        # WHY: OR-ing BEFORE edge detection means a stuck/held joystick Auto input makes the
        # remote's pause button do nothing (at the MdlApp level). The sim must model the two
        # inputs separately to reproduce this. Not stated in the spec. MdlApp.c:39724 (OR)
        # feeds the single edge detector at MdlApp.c:39743-39744.
        h = _to_standby(self.h)
        fw = h.fw
        fw["u.jstAutoReq_StartPause"] = 1
        h.tick()
        self.assertTrue(_running(h))
        h.tick(10)
        h.pulse("u.rmtAutoReq_StartPause")
        h.tick(5)
        self.assertTrue(_running(h), "remote edge is invisible while the joystick input is high")
        fw["u.jstAutoReq_StartPause"] = 0
        h.tick(5)
        self.assertTrue(_running(h), "releasing the joystick is a falling edge, not a press")
        h.pulse("u.rmtAutoReq_StartPause")       # control: the same remote press now works
        self.assertTrue(_paused(h), h.describe())


class TestStopAndCancel(unittest.TestCase):
    """Spec "Pause / inhibit semantics": Stop pauses; Cancel -> NoTarget except while running."""

    def setUp(self):
        self.h = Harness().reset()

    def test_tablet_stop_pauses_and_never_self_resumes(self):
        # WHY: tablet Stop is the operator's software stop; releasing it must not restart
        # motion. T193 guard `autoReq_Stop || (hasChanged(StartPause) && StartPause)`,
        # MdlApp.c:23902-23907; MdlApp_PositioningPaused (MdlApp.c:24152-24300) has no Stop-release exit.
        h = _run_until_valves(_to_running_positioning(self.h))
        h.fw["u.tabletAutoReq_Stop"] = 1
        h.tick()
        self.assertTrue(_paused(h), h.describe())
        self.assertEqual(h.valves(), {})
        h.fw["u.tabletAutoReq_Stop"] = 0
        h.tick(100)
        self.assertTrue(_paused(h), "Stop release must not resume")
        h.pulse("u.jstAutoReq_StartPause")
        self.assertTrue(_running(h), h.describe())

    def test_held_stop_turns_resume_into_a_one_tick_run(self):
        # WHY (actuation): Stop is a LEVEL in T193 but resume (T181) does not check it, so a
        # StartPause edge while Stop is held re-enters Positioning for exactly one 10 ms tick
        # with StartStopSts=True -- and the arbitration passes the auto valve command for that
        # tick (MdlApp.c:51737 `elseif autoCtrl_StartStopSts`). A valve model with no lag
        # will move. Spec says only "Stop -> Paused". T181 MdlApp.c:24156-24162, T193 MdlApp.c:23907.
        h = _run_until_valves(_to_running_positioning(self.h, dwell_ticks=TRAVEL_DIR_CHANGE_HOLD))
        h.pulse("u.jstAutoReq_StartPause")
        self.assertTrue(_paused(h), h.describe())
        h.fw["u.tabletAutoReq_Stop"] = 1
        h.tick(10)
        h.fw["u.jstAutoReq_StartPause"] = 1
        states, valves_on = [], []
        for _ in range(5):
            h.tick()
            states.append(h.main_state())
            valves_on.append(bool(h.valves()))
        self.assertEqual(states, ["Positioning"] + ["PositioningPaused"] * 4)
        self.assertEqual(valves_on, [True, False, False, False, False])
        self.assertEqual(h.curr_step(), "Positioning")

    def test_stop_is_not_checked_in_standby(self):
        # WHY: a held Stop does not prevent a start from Standby either -- same one-tick run,
        # carrying the Raise front-end command. MdlApp_Standby (MdlApp.c:25138-25330) never
        # reads tabletAutoReq_Stop; T193 then pauses on the next tick (MdlApp.c:23907).
        h = _to_standby(self.h)
        h.fw["u.tabletAutoReq_Stop"] = 1
        h.tick(10)
        self.assertEqual(h.main_state(), "Standby")
        h.fw["u.jstAutoReq_StartPause"] = 1
        h.tick()
        self.assertTrue(_running(h), h.describe())
        self.assertTrue(h.valves(), "the Raise command reaches the output with Stop held")
        h.tick()
        self.assertTrue(_paused(h), h.describe())
        self.assertEqual(h.valves(), {})

    def test_cancel_from_standby_relatches_the_input_and_allows_immediate_reentry(self):
        # WHY: Cancel is the tablet's End button; the sim must see NoTarget. Cancel does not
        # clear anything: the latch runs after the chart in the same step and NoTarget is in
        # its enable set, so on the Cancel tick itself ack jumps to whatever id is on the
        # input (here 8, written while Standby ignored it). Standby can then be re-entered
        # without the tablet re-sending an id. T188 in MdlApp_Standby MdlApp.c:25152; latch
        # enable MdlApp.c:40922-40927, ack MdlApp.c:40946-40957.
        h = _to_standby(self.h, panel_id=7)
        h.fw["u.tarPanelData.id"] = 8
        h.tick(5)
        self.assertEqual(h.fw["y.tarPanelIdAck"], 7, "Standby is not in the latch set")
        h.fw["u.tabletAutoReq_Cancel"] = 1
        h.tick()
        self.assertEqual(h.main_state(), "NoTarget")
        self.assertFalse(h.is_running())
        self.assertEqual(h.fw["y.tarPanelIdAck"], 8, "NoTarget re-reads the input on the Cancel tick")
        h.fw["u.tabletAutoReq_Cancel"] = 0
        h.tick(2)
        h.request_step("Standby")
        h.run_until(lambda h: h.curr_step() == "Standby", 0.5, "Standby again")

    def test_cancel_is_ignored_while_running_and_applies_once_paused(self):
        # WHY: the spec's claim that Cancel does not act on running states; a held Cancel then
        # takes effect on the tick after a pause. MdlApp_Positioning (MdlApp.c:23894-23970)
        # has no Cancel guard; MdlApp_PositioningPaused T188 at MdlApp.c:24200.
        h = _to_running_positioning(self.h)
        h.fw["u.tabletAutoReq_Cancel"] = 1
        h.tick(50)
        self.assertTrue(_running(h), h.describe())
        h.fw["u.jstAutoReq_StartPause"] = 1
        h.tick()
        self.assertTrue(_paused(h), h.describe())
        h.tick()
        self.assertEqual(h.main_state(), "NoTarget")

    def test_cancel_from_inhibited_returns_to_notarget(self):
        # WHY: an operator must be able to abandon a job that is stuck on an inhibit.
        # MdlApp_PositioningInhibited T188 checked first, MdlApp.c:23983-23986.
        h = _to_running_positioning(self.h)
        h.fw["u.isChsImuFault"] = 1
        h.tick()
        self.assertTrue(_inhibited(h), h.describe())
        h.fw["u.tabletAutoReq_Cancel"] = 1
        h.tick()
        self.assertEqual(h.main_state(), "NoTarget")


class TestInhibitWhileRunning(unittest.TestCase):
    """Spec: inhibit -> <X>Inhibited; clearing lands in <X>Paused, never back in running."""

    def setUp(self):
        self.h = Harness().reset()

    def test_gnss_degradation_inhibits_after_on_delay_and_clears_into_paused(self):
        # WHY: RTK dropping to float mid-cycle is the most common field inhibit; the sim's GNSS
        # model must reproduce the 20-tick on-delay and the no-self-resume rule.
        # chart_2496 (stdDevZ > PoorThld -> low, clears only below GoodThld with RTK fixed,
        # MdlApp.c:39088-39100), UpdateOnDelay `counter >= 20` ('<S170>:1:139-146',
        # MdlApp.c:39370-39394, CntAccuracyChkDly SysPar.m:57), no off-delay; T180 MdlApp.c:23925;
        # T208 Inhibited -> Paused MdlApp.c:24117.
        h = _run_until_valves(_to_running_positioning(self.h, dwell_ticks=TRAVEL_DIR_CHANGE_HOLD))
        h.fw["u.gnssPosStdDevZ"] = 0.10            # > verticalAccuracyPoorThld 0.04
        for n in range(1, 20):
            h.tick()
            self.assertTrue(_running(h), f"must still run inside the on-delay (tick {n})")
        h.tick()
        self.assertTrue(_inhibited(h), h.describe())
        self.assertTrue(h.auto_inhibited())
        self.assertEqual(h.valves(), {})

        # Inhibited ignores the switch on EVERY tick from the edge: a Paused state with the
        # inhibit still active would run on the edge tick (T181 before T180) and only then
        # drop back, which a single late sample could not tell apart.
        for n, (state, running, valves, _) in enumerate(_press_and_sample(h, 6)):
            self.assertEqual((state, running, valves), ("PositioningInhibited", False, {}), f"edge+{n}")

        h.fw["u.gnssPosStdDevZ"] = 0.008            # < GoodThld 0.02 -> clears, no off-delay
        h.tick()
        self.assertFalse(h.auto_inhibited())
        self.assertTrue(_paused(h), h.describe())
        h.tick(100)
        self.assertTrue(_paused(h), "a cleared inhibit must not resume")
        self.assertEqual(h.valves(), {})
        h.pulse("u.jstAutoReq_StartPause")          # now in Paused: the edge resumes
        self.assertTrue(_running(h), h.describe())

    def test_imu_fault_inhibits_on_the_same_tick(self):
        # WHY: IMU faults have no on-delay, so the valves must be zero on the very tick the
        # fault arrives -- a latency budget the sim's fault injection can check.
        # BIT_IMU_COM_ERR from raw inports ('<S170>:1:96', MdlApp.c:39508-39520);
        # isAutoCtrlInhibited is not delayed into the chart (MdlApp.c:39675, :39707);
        # T208 Inhibited -> Paused MdlApp.c:24117.
        h = _run_until_valves(_to_running_positioning(self.h))
        h.fw["u.isChsImuFault"] = 1
        h.tick()
        self.assertTrue(_inhibited(h), h.describe())
        self.assertIn("BIT_IMU_COM_ERR", h.inhibit_names())
        self.assertEqual(h.valves(), {})
        h.fw["u.isChsImuFault"] = 0
        h.tick(50)
        self.assertTrue(_paused(h), h.describe())
        h.pulse("u.jstAutoReq_StartPause")          # proves Paused, not stuck in Inhibited
        self.assertTrue(_running(h), h.describe())

    def test_remote_link_loss_does_not_inhibit_auto_at_the_mdlapp_level(self):
        # WHY: fault-injection scenarios need to know which inputs stop auto. Dropping
        # isRmtOk sets BIT_RMT_CTRL_ERR (bit 4, '<S170>:1:97', MdlApp.c:39527-39537) but
        # AUTO_INHIBIT_MASK 0xC78E excludes it (MdlApp.c:39675 `& 51086U`; spec A4.5 agrees), so
        # MdlApp keeps driving the valves. This is the application model only: when rmt_ValidSts
        # is off the ECU wrapper also forces the remote's Auto button and joystick demands to
        # zero (PrePostProc_If.c:275-285, :916, :1032; AppCtrlIf.c:229), and a receiver
        # hardware stop is not modelled here.
        h = _run_until_valves(_to_running_positioning(self.h))
        h.fw["u.isRmtOk"] = 0
        h.tick(100)
        self.assertIn("BIT_RMT_CTRL_ERR", h.inhibit_names())
        self.assertFalse(h.auto_inhibited())
        self.assertTrue(_running(h), h.describe())
        self.assertNotEqual(h.valves(), {})


class TestEntryIgnoresInhibitForOneTick(unittest.TestCase):
    """No transition INTO a running auto state checks isAutoCtrlInhibited: the inhibit is
    only seen by the running state's own guard (T180) one tick later, and the arbitration
    passes the auto command for that tick (MdlApp.c:51737). A valve model with no pilot lag
    twitches while the machine reports an inhibit. Spec A6 step 0 describes the T239->T205->T180
    path into PositioningInhibited but not this actuation."""

    def setUp(self):
        self.h = Harness().reset()

    def _assert_one_tick_run(self, h, then="PositioningInhibited"):
        samples = _press_and_sample(h, 3)
        (s0, run0, valves0, step0), (s1, run1, valves1, _), (s2, run2, valves2, _) = samples
        self.assertEqual((s0, run0), ("Positioning", True), samples)
        self.assertTrue(valves0, "the auto command reaches the output while inhibited")
        self.assertEqual((s1, run1, valves1), (then, False, {}), samples)
        self.assertEqual((s2, run2, valves2), (then, False, {}), samples)
        return valves0, step0

    def test_start_from_standby_while_inhibited_runs_one_tick(self):
        # T281 MdlApp.c:25278-25285 has no inhibit term (MdlApp_Standby takes no
        # isAutoCtrlInhibited at all); T180 MdlApp.c:23921-23925 one tick later.
        h = _to_standby(self.h)
        h.fw["u.gnssPosStdDevZ"] = 0.10
        h.tick(30)
        self.assertTrue(h.auto_inhibited())
        self.assertEqual(h.main_state(), "Standby", "Standby has no Inhibited twin")
        valves, step = self._assert_one_tick_run(h)
        self.assertEqual(step, "PositioningStep_Raise")
        self.assertTrue(_axes(valves) <= CTRL_AXES_IN_POSITIONING["PositioningStep_Raise"], valves)
        # Now truly Inhibited: a further edge never runs, on any tick.
        for n, (state, running, v, _) in enumerate(_press_and_sample(h, 4)):
            self.assertEqual((state, running, v), ("PositioningInhibited", False, {}), f"edge+{n}")

    def test_resume_on_the_tick_an_inhibit_appears_runs_one_tick(self):
        # MdlApp_PositioningPaused evaluates T181 (MdlApp.c:24156-24162) BEFORE T180
        # (MdlApp.c:24174-24180), so an edge on the same tick as a fault still resumes.
        h = _run_until_valves(_to_running_positioning(self.h, dwell_ticks=TRAVEL_DIR_CHANGE_HOLD))
        h.pulse("u.jstAutoReq_StartPause")
        self.assertTrue(_paused(h), h.describe())
        h.fw["u.isChsImuFault"] = 1                 # same tick as the edge
        self._assert_one_tick_run(h)

    def test_restart_from_complete_while_inhibited_runs_one_tick(self):
        # T256 -> T261 (MdlApp.c:40102-40114) checks edge, ack ~= asBuilt and isTarPanelValid
        # but not the inhibit. Complete is reached without a plant via the Releasing jump
        # (see TestTargetLatch.test_complete_relatches_and_restarts_only_on_a_new_id).
        h = _to_standby(self.h, panel_id=7)
        h.jump_to_step("Releasing")
        h.run_until(lambda h: h.curr_step() == "Complete", 0.1, "Complete")
        h.fw["u.tarPanelData.id"] = 8
        h.tick(3)
        h.fw["u.isChsImuFault"] = 1
        h.tick(3)
        self.assertTrue(h.auto_inhibited())
        self.assertEqual(h.main_state(), "Complete", "Complete has no Inhibited twin")
        valves, step = self._assert_one_tick_run(h)
        self.assertEqual(step, "PositioningStep_Raise")
        self.assertTrue(_axes(valves) <= CTRL_AXES_IN_POSITIONING["PositioningStep_Raise"], valves)


class TestTargetLatch(unittest.TestCase):
    """Spec A3.6: tarPanelData is latched only while idle (MdlApp.c:40922-40927):
    CurrStep in {NoTarget, Complete} or (CurrStep == Positioning && !StartStopSts).
    The latch governs tarPanelIdAck, isTarPanelValid and the release pose; it does NOT govern
    the UC travel target, which reads the raw bus (see test_retarget_while_running_*)."""

    def setUp(self):
        self.h = Harness().reset()

    def test_degenerate_ids_are_no_target(self):
        # WHY: the sim's mission bridge must encode "no panel" with the exact sentinel the
        # firmware tests. ack = (id != 16777215) ? id : 0 (MdlApp.c:40946-40957);
        # isTarPanelValid = ack != 0 (MdlApp.c:40965), delayed one tick into BIT_NO_TARGET
        # (Delay2_DSTATE_ftor, MdlApp.c:39466, :52229) and into T284 (MdlApp.c:22940-22944).
        for bad in (0, INVALID_TAR_PANEL_ID):
            with self.subTest(id=hex(bad)):
                h = _to_standby(Harness().reset(), panel_id=7)   # the same request works with id 7
                h.fw["u.tabletAutoReq_Cancel"] = 1
                h.tick()
                h.fw["u.tabletAutoReq_Cancel"] = 0
                self.assertEqual(h.main_state(), "NoTarget")
                self.assertEqual(h.fw["y.tarPanelIdAck"], 7)
                h.fw["u.tarPanelData.id"] = bad
                h.tick()
                self.assertEqual(h.fw["y.tarPanelIdAck"], 0)
                self.assertNotIn("BIT_NO_TARGET", h.inhibit_names(), "validity is delayed one tick")
                h.tick()
                self.assertIn("BIT_NO_TARGET", h.inhibit_names())
                h.request_step("Standby")          # latched value is Standby: re-edges via 255
                h.tick(20)
                self.assertEqual(h.main_state(), "NoTarget", f"id {bad:#x} must not reach Standby")

    def test_only_the_exact_24bit_sentinel_is_invalid(self):
        # WHY: a bridge that writes 0xFFFFFFFF (uint32 -1) for "none" gets a VALID target; so
        # do 2^24 and 2^24-2. The compare is `id != 16777215U` exactly (MdlApp.c:40946;
        # SysPar.m:65), validity only ack != 0 (MdlApp.c:40965).
        for panel_id in (0xFFFFFFFF, 0x1000000, 0xFFFFFE):
            with self.subTest(id=hex(panel_id)):
                h = Harness().reset().nominal_inputs()
                h.set_swing_aligned(True)
                h.gnss_rtk_fixed()
                h.set_target_panel(panel_id=panel_id)
                h.tick(3)
                self.assertEqual(h.fw["y.tarPanelIdAck"], panel_id)
                self.assertNotIn("BIT_NO_TARGET", h.inhibit_names())
                h.request_step("Standby")
                h.run_until(lambda h: h.curr_step() == "Standby", 0.5, f"Standby with id {panel_id:#x}")

    def test_ack_follows_input_in_notarget_same_tick(self):
        # WHY: pins the latch latency the harness relies on (ack same tick, valid one tick
        # later). Latch runs after the chart in the same step, MdlApp.c:40922-40957.
        h = self.h.nominal_inputs()
        h.set_swing_aligned(True)
        h.set_target_panel(panel_id=7)
        h.tick()
        self.assertEqual(h.fw["y.tarPanelIdAck"], 7)
        h.fw["u.tarPanelData.id"] = 9
        h.tick()
        self.assertEqual(h.fw["y.tarPanelIdAck"], 9)

    def test_retarget_while_running_keeps_id_and_release_pose_but_redirects_travel(self):
        # WHY (defect candidate; contradicts spec A3.6 "retargeting mid-cycle is silently
        # ignored"): running Positioning is not in the latch-enable set (MdlApp.c:40922-40927),
        # so tarPanelIdAck, isTarPanelValid and the release pose (SetPosePanelRelease reads the
        # latched copy, MdlApp.c:41070 -> posePanelRelease :41380 -> y.panelPoseErr :50271) stay
        # on panel 7. But CalcUcPathTarget (chart_3193, '<S147>:1:46-54', MdlApp.c:9450-9464)
        # reads the RAW bus that MdlApp_step copies from the inport before every step
        # (MdlApp.c:52524-52583; the ECU copies the tablet bus unconditionally,
        # AppCtrlIf.c:146-160), gated only by the latched validity. A tablet that edits the
        # geometry mid-run redirects the tracks toward the new panel while the tablet ack and
        # the placement target still say panel 7. A plant/mission bridge must hold the bus
        # constant while running.
        h = _to_running_positioning(self.h, panel_id=7)
        h.tick(10)
        dist, pose = _dist_to_wp_target(h), _release_pose_err(h)
        h.fw["u.tarPanelData.id"] = 0              # id alone: validity comes from the latched ack
        h.tick(50)
        self.assertEqual(h.fw["y.tarPanelIdAck"], 7)
        self.assertNotIn("BIT_NO_TARGET", h.inhibit_names())
        self.assertTrue(_running(h), h.describe())
        self.assertEqual((_dist_to_wp_target(h), _release_pose_err(h)), (dist, pose))

        h.set_target_panel(panel_id=8, east=RETARGET_EAST)
        h.tick()
        self.assertTrue(_running(h), h.describe())
        self.assertEqual(h.fw["y.tarPanelIdAck"], 7)
        self.assertEqual(_release_pose_err(h), pose, "release pose uses the latched geometry")
        self.assertAlmostEqual(_dist_to_wp_target(h)[1] - dist[1], DEFAULT_EAST - RETARGET_EAST, places=3,
                               msg="the UC travel target follows the raw bus on the next tick")

        # Control: the release pose is a live output -- pausing enables the latch and it moves.
        h.pulse("u.jstAutoReq_StartPause")
        self.assertTrue(_paused(h), h.describe())
        self.assertEqual(h.fw["y.tarPanelIdAck"], 8)
        self.assertAlmostEqual(_release_pose_err(h)[0] - pose[0], RETARGET_EAST - DEFAULT_EAST, places=3)

    def test_retarget_in_standby_is_ignored_and_cannot_clear_it(self):
        # WHY: Standby is NOT idle for the latch, so writing id 0 in Standby neither clears
        # the target nor reaches T253 (~isTarPanelValid -> NoTarget, MdlApp.c:25275); the
        # tablet must Cancel. Latch-enable set MdlApp.c:40922-40927 excludes Standby.
        h = _to_standby(self.h, panel_id=7)
        h.fw["u.tarPanelData.id"] = 8
        h.tick(20)
        self.assertEqual(h.fw["y.tarPanelIdAck"], 7)
        h.fw["u.tarPanelData.id"] = 0
        h.tick(20)
        self.assertEqual(h.fw["y.tarPanelIdAck"], 7)
        self.assertEqual(h.main_state(), "Standby")

    def test_retarget_while_paused_is_latched_and_resume_keeps_going(self):
        # WHY: PositioningPaused reports (Positioning, StartStopSts=False), which IS in the
        # latch set (MdlApp.c:40922-40927), so a paused machine accepts a new panel -- id and
        # release geometry -- and resumes toward it without restarting the sub-machine
        # (SetPositioningStep abort guard is only CurrStep ~= Positioning, '<S139>:951',
        # MdlApp.c:40787-40790). The travel-target distance is NOT evidence of latching (it
        # follows the raw bus in any state); the release pose is.
        h = _to_running_positioning(self.h, panel_id=7)
        h.run_until(lambda h: h.positioning_step() == "PositioningStep_Align", 0.5, "Align")
        h.pulse("u.jstAutoReq_StartPause")
        self.assertTrue(_paused(h), h.describe())
        pose = _release_pose_err(h)
        h.set_target_panel(panel_id=8, east=RETARGET_EAST)
        h.tick()
        self.assertEqual(h.fw["y.tarPanelIdAck"], 8, "latched on the same tick")
        new_pose = _release_pose_err(h)
        self.assertAlmostEqual(new_pose[0] - pose[0], RETARGET_EAST - DEFAULT_EAST, places=3,
                               msg="the new release geometry is latched while paused")
        self.assertEqual(h.positioning_step(), "PositioningStep_Align")
        (state, running, _, step), = _press_and_sample(h, 1)
        self.assertEqual((state, running), ("Positioning", True))
        self.assertEqual(step, "PositioningStep_Align", "resume must not restart at Raise")
        h.tick(5)
        self.assertEqual(h.fw["y.tarPanelIdAck"], 8)
        self.assertEqual(_release_pose_err(h), new_pose)

    def test_target_cleared_while_paused_resumes_into_picking_when_facing_grid_east(self):
        # WHY (defect candidate): clearing the id during a pause latches ack=0 and sets
        # BIT_NO_TARGET, which is outside AUTO_INHIBIT_MASK (MdlApp.c:39675) and checked by
        # neither T181 (MdlApp.c:24156-24162) nor MdlApp_Positioning, so resume runs. Worse: with
        # ~isTarPanelValid, CalcUcPathTarget collapses the travel target onto the CURRENT UC
        # position with path direction grid East ('<S147>:1:28-41', MdlApp.c:9395-9446) and
        # CalcUcPathBasis zeroes distToWpTarLat/Lon ('<S146>:1:8-13', MdlApp.c:10106-10125).
        # isUcPathAligned ('<S111>:1:13-22', MdlApp.c:47676-47708) then reduces to "UC heading
        # within UcHeadingTol 3.5 deg of grid East" (SysPar.m:448) and T474 (guard
        # MdlApp.c:23936-23942, '<S136>:474' :23945) enters Picking with no target. A machine
        # 4 deg off stays in Align; one facing North spins its tracks toward East instead.
        cases = {                                   # yaw CCW from grid East -> (state, travel ports)
            "facing grid East": (0.0, "Picking", None),
            "3 deg off East, inside UcHeadingTol": (3.0, "Picking", None),
            "4 deg off East, outside UcHeadingTol": (4.0, "Positioning", None),
            "facing grid North": (90.0, "Positioning", {"trvlLeFwd", "trvlRiRev"}),
        }
        for label, (yaw_deg, expected, travel_ports) in cases.items():
            with self.subTest(label):
                h = Harness().reset()
                R = kin.Rz(math.radians(yaw_deg))     # yaw CCW from grid East (sil/geodesy.py)
                h.gnss_site()
                h.place_chassis((0.0, 0.0, 0.0), R)
                _to_running_positioning(h, panel_id=7, R_chs=R)
                h.run_until(lambda h: h.positioning_step() == "PositioningStep_Align", 0.5, "Align")
                h.tick(40)                              # tilt reached-confirm long satisfied
                h.pulse("u.jstAutoReq_StartPause")
                h.fw["u.tarPanelData.id"] = 0
                h.tick(100)
                self.assertEqual(h.fw["y.tarPanelIdAck"], 0)
                self.assertIn("BIT_NO_TARGET", h.inhibit_names())
                self.assertFalse(h.auto_inhibited())
                self.assertTrue(_paused(h), "no ~isTarPanelValid exit from Paused")
                self.assertEqual(_dist_to_wp_target(h), (0.0, 0.0))
                (state, running, _, _), (state1, running1, valves1, _) = _press_and_sample(h, 2)
                self.assertEqual((state, running), ("Positioning", True))
                if expected == "Picking":
                    self.assertEqual((state1, running1), ("Picking", True), h.describe())
                    self.assertTrue(_axes(valves1) & {"bm1", "arm", "link"}, valves1)
                    self.assertTrue(_axes(valves1) <= CTRL_AXES_IN_PREPARE_PICK, valves1)
                else:
                    h.tick(50)
                    self.assertTrue(_running(h), h.describe())
                    self.assertEqual(h.positioning_step(), "PositioningStep_Align")
                    self.assertTrue({"trvlLe", "trvlRi"} <= _axes(h.valves()), h.valves())
                    if travel_ports is not None:
                        self.assertEqual(set(h.valves()), travel_ports,
                                         "spin toward grid East (clockwise seen from above)")

    def test_invalid_latch_reaches_notarget_via_standby_step_request(self):
        # WHY: T253 (Standby ~isTarPanelValid -> NoTarget, MdlApp.c:25275) is reachable by
        # entering Standby with an already-invalid latch, e.g. from PositioningPaused via a
        # tablet step request (T159 MdlApp.c:24226). Paused itself holds with an invalid latch
        # (see the test above); Complete + StartPause is another route (T263, tested below).
        h = _to_running_positioning(self.h, panel_id=7)
        h.pulse("u.jstAutoReq_StartPause")
        self.assertTrue(_paused(h), h.describe())
        h.fw["u.tarPanelData.id"] = 0
        h.tick(3)
        h.request_step("Standby")      # latched value is already Standby: one 255 tick first
        h.tick()
        self.assertEqual(h.main_state(), "Standby")
        h.tick()
        self.assertEqual(h.main_state(), "NoTarget")

    def test_complete_relatches_and_restarts_only_on_a_new_id(self):
        # WHY: the panel-to-panel loop (spec A6 step 10) must refuse to re-place the same id.
        # Reached without a plant by the tablet step jump Standby -> ReleasingPaused (T185
        # MdlApp.c:25240), start, and T302 isPanelDetached (vacPrs >= -0.10 on both,
        # MdlApp.c:39054-39063; MdlApp.c:40381). T256/T261/T263 MdlApp.c:40102-40130;
        # asBuilt_PanelId held from Complete entry, MdlApp.c:50125-50137.
        h = _to_standby(self.h, panel_id=7)
        h.jump_to_step("Releasing", start=False)
        self.assertEqual(h.main_state(), "ReleasingPaused")
        h.pulse("u.jstAutoReq_StartPause")
        h.run_until(lambda h: h.curr_step() == "Complete", 0.1, "Complete")
        h.tick(5)
        self.assertEqual(h.fw["y.asBuilt_PanelId"], 7)
        self.assertEqual(h.fw["y.tarPanelIdAck"], 7)
        h.pulse("u.jstAutoReq_StartPause")
        h.tick(5)
        self.assertEqual(h.main_state(), "Complete", "same id must park in Complete")
        h.fw["u.tarPanelData.id"] = 8
        h.tick(3)
        self.assertEqual(h.fw["y.tarPanelIdAck"], 8)
        h.pulse("u.jstAutoReq_StartPause")
        self.assertTrue(_running(h), h.describe())

    def test_as_built_id_is_the_raw_input_on_complete_entry(self):
        # WHY (defect candidate, data integrity): asBuilt_PanelId is NOT captured from the
        # latched tarPanelIdAck. MdlApp_step writes the RAW input into the outport first
        # (MdlApp.c:52509) and AsBuiltDataMgr keeps it only on the Complete-entry tick
        # (MdlApp.c:50125-50137). A tablet that advances the id during Releasing (latch
        # disabled, ack stays 7) makes panel 7 be recorded as 9; the next press then parks in
        # Complete (ack == asBuilt, T256 MdlApp.c:40102-40109) and id 7 can be started AGAIN.
        h = _to_standby(self.h, panel_id=7)
        h.jump_to_step("Releasing", start=False)
        h.fw["u.tarPanelData.id"] = 9
        h.tick(3)
        self.assertEqual(h.fw["y.tarPanelIdAck"], 7, "Releasing is not in the latch set")
        h.pulse("u.jstAutoReq_StartPause")
        h.run_until(lambda h: h.curr_step() == "Complete", 0.1, "Complete")
        h.tick(2)
        self.assertEqual(h.fw["y.asBuilt_PanelId"], 9, "the panel that was released was 7")
        h.pulse("u.jstAutoReq_StartPause")
        h.tick(3)
        self.assertEqual(h.main_state(), "Complete", "panel 9 cannot be started")
        h.fw["u.tarPanelData.id"] = 7
        h.tick(3)
        h.pulse("u.jstAutoReq_StartPause")
        self.assertTrue(_running(h), "panel 7 can be placed a second time")

    def test_complete_with_cleared_id_goes_to_notarget_on_start(self):
        # WHY: in Complete the id is latched again, so id 0 makes ack(0) != asBuilt(7) and the
        # start edge takes T263 (~isTarPanelValid) to NoTarget instead of Positioning.
        # MdlApp.c:40102-40130 (guard1 -> T300 NoTarget, MdlApp.c:40433-40437).
        h = _to_standby(self.h, panel_id=7)
        h.jump_to_step("Releasing")
        h.run_until(lambda h: h.curr_step() == "Complete", 0.1, "Complete")
        h.fw["u.tarPanelData.id"] = 0
        h.tick(3)
        h.pulse("u.jstAutoReq_StartPause")
        self.assertEqual(h.main_state(), "NoTarget")


class TestTabletStepRequest(unittest.TestCase):
    """autoReqStep jumps from Standby / <X>Paused / <X>Inhibited (junction chain T197..T145)."""

    def setUp(self):
        self.h = Harness().reset()

    def test_step_request_lands_in_paused_and_skips_positioning_completion(self):
        # WHY: the tablet can put the cycle into any later step without its entry guard
        # (e.g. T474 Positioning -> Picking), always as <X>Paused. SIL scenarios can use this
        # to reach later steps without a plant, and it is a path a real operator can take.
        # Standby T176 MdlApp.c:25182, PositioningPaused T177 MdlApp.c:24264; the
        # SetPositioningStep abort guard only checks CurrStep ('<S139>:951', MdlApp.c:40787-40790).
        h = _to_standby(self.h)
        h.request_step("Positioning")
        h.tick()
        self.assertTrue(_paused(h), h.describe())
        self.assertEqual(h.valves(), {})
        h.tick(2)
        self.assertEqual(h.positioning_step(), "PositioningStep_Swing",
                         "sub-machine advances while Paused (only CurrStep is checked)")
        h.request_step("Picking")
        h.tick()
        self.assertEqual(h.main_state(), "PickingPaused")
        self.assertFalse(h.is_running())
        h.tick()
        self.assertEqual(h.positioning_step(), "PositioningStep_Inactive")


class TestPositioningWithoutPlant(unittest.TestCase):
    """First actuation evidence: what Positioning puts on y.propVlvCmd when nothing moves."""

    def setUp(self):
        self.h = Harness().reset()

    def test_substeps_and_valve_ports_follow_ctrl_mode(self):
        # WHY: this is the valve interface the plant will consume. Every non-zero port must
        # belong to an axis SetCtrlMode (chart_2123) enables for the current positioningStep,
        # AND each sub-step must command the axis it exists for (a firmware that commanded
        # nothing would otherwise pass). Raise lasts one tick: the house starts square, and
        # |ChsToUc.q| <= SwingAngRngToSkipPrePose 5 deg alone makes isSwingAlignAllowed true
        # ('<S112>:1:16-18', Delay30 MdlApp.c:52287, SysPar.m:270; T1004 MdlApp.c:40847).
        # Swing -> Align (T1009, MdlApp.c:40882) waits for isTarActuatorReached.swing, which
        # needs CntTarReachedConfirm = 30 reached samples in JntSpace ('<S107>:1:51', :63, :85,
        # MdlApp.c:47224, :47375-47400, SysPar.m:471); measured end to end as 30 ticks after
        # Swing entry. The tilt is offset so Swing/Align have a JntSpace error to close; the
        # Standby dwell lets travel start on the Align entry tick. Commands are 0..100 %.
        for tilt_deg, tilt_port in ((10.0, "tiltNega"), (-10.0, "tiltPosi")):
            for east in (DEFAULT_EAST, RETARGET_EAST):
                with self.subTest(tilt_deg=tilt_deg, east=east):
                    h = Harness().reset()
                    _to_standby(h, east=east, dwell_ticks=TRAVEL_DIR_CHANGE_HOLD)
                    h.set_pose(**dict(Harness.NOMINAL_POSE, q_tilt=math.radians(tilt_deg)))
                    h.tick(2)
                    first_seen, ports_seen = {}, {}
                    for n, (_, _, valves, step) in enumerate(_press_and_sample(h, 120)):
                        new_step = step not in first_seen
                        first_seen.setdefault(step, n)
                        for p, v in valves.items():
                            self.assertGreaterEqual(v, 0.0, p)
                            self.assertLessEqual(v, 100.0, p)
                        self.assertTrue(_axes(valves) <= CTRL_AXES_IN_POSITIONING[step],
                                        f"{valves} in {step} at +{n}")
                        ports_seen.setdefault(step, set()).update(valves)
                        if new_step and step == "PositioningStep_Align":
                            self.assertTrue({"trvlLe", "trvlRi"} <= _axes(valves),
                                            f"travel on the Align entry tick after a dwell: {valves}")
                    self.assertEqual(first_seen["PositioningStep_Raise"], 0)
                    self.assertEqual(first_seen["PositioningStep_Swing"], 1)
                    self.assertEqual(first_seen["PositioningStep_Align"] - first_seen["PositioningStep_Swing"],
                                     SWING_REACHED_CONFIRM, first_seen)
                    self.assertTrue(_axes(ports_seen["PositioningStep_Raise"]) & {"bm1", "arm", "link"})
                    self.assertIn(tilt_port, ports_seen["PositioningStep_Swing"], "tilt error is closed in Swing")
                    self.assertIn(tilt_port, ports_seen["PositioningStep_Align"])
                    # Align enables travel toward a UC stop point the machine is not at (no plant).
                    self.assertTrue({"trvlLe", "trvlRi"} <= _axes(ports_seen["PositioningStep_Align"]))
                    # Swing target is ChsToUc.q = 0 (chart_1011 Positioning/Swing) and q stays 0.
                    self.assertFalse(any(p.startswith("swing") for s in ports_seen.values() for p in s))
                    h.pulse("u.jstAutoReq_StartPause")
                    self.assertEqual(h.valves(), {}, "Paused: arbitration else-branch, MdlApp.c:51763")

    def test_raise_tick_commands_only_the_front_end(self):
        # WHY: the one-tick Raise command is the only front-end actuation before Picking when
        # the house starts square; a valve model with no lag turns it into a 10 ms twitch.
        # chart_2123 Raise: bm1/arm/link TaskSpace, Swing: swing/tilt/rotate JntSpace (tilt
        # already at 0 here, so nothing); Raise -> Swing next tick (T1004, MdlApp.c:40847).
        h = _to_standby(self.h)
        (_, _, raise_ports, step0), (_, _, swing_ports, step1) = _press_and_sample(h, 2)
        self.assertEqual(step0, "PositioningStep_Raise")
        self.assertEqual(_axes(raise_ports), {"bm1", "arm", "link"}, raise_ports)
        self.assertEqual(step1, "PositioningStep_Swing")
        self.assertEqual(swing_ports, {}, "tilt at its target and swing at 0: Swing commands nothing")

    def test_travel_output_waits_80_ticks_after_the_travel_direction_flips(self):
        # WHY: a timing budget for every UC-alignment scenario and for the travel valve model.
        # The first travel command is NOT tied to Align entry or to boot: CalcFctDmdTrvl
        # (chart_1152) starts its hold counter at CntDirChgDly ('<S124>:1:7-8',
        # MdlApp.c:47924-47926) and restarts it only when dirReversed changes ('<S124>:1:172-184',
        # MdlApp.c:48285-48320, SysPar.m:412). dirReversed is false while the target is invalid
        # and becomes tarPassedSts == 1 once it is valid ('<S146>:1:9', :45-54, MdlApp.c:10111,
        # :10203). The default target's UC stop point is 0.157 m BEHIND the machine -> flip on
        # the latch -> travel at ack + 80, and reversed on both tracks ('<S124>:1:162-163').
        # A stop point AHEAD (east=10) never flips, so travel starts on Align entry.
        cases = (
            dict(boot=0, dwell=0, east=DEFAULT_EAST, flips=True),
            dict(boot=100, dwell=0, east=DEFAULT_EAST, flips=True),   # hold follows the latch, not boot
            dict(boot=0, dwell=150, east=DEFAULT_EAST, flips=True),   # dwell outlasts the hold
            dict(boot=0, dwell=0, east=10.0, flips=False),            # no flip, no hold
        )
        for c in cases:
            with self.subTest(**c):
                h = Harness().reset().nominal_inputs()
                h.set_swing_aligned(True)
                h.gnss_rtk_fixed()
                h.tick(c["boot"])
                h.set_target_panel(panel_id=7, east=c["east"])
                ack_tick = h.run_until(lambda h: h.fw["y.tarPanelIdAck"] != 0, 0.1, "ack")
                h.tick(2)
                h.request_step("Standby")
                h.run_until(lambda h: h.curr_step() == "Standby", 0.5, "Standby")
                h.tick(c["dwell"])
                h.pulse("u.jstAutoReq_StartPause")
                align_tick = h.run_until(lambda h: h.positioning_step() == "PositioningStep_Align", 0.5)
                travel_tick = h.run_until(lambda h: any(p.startswith("trvl") for p in h.valves()), 3.0)
                expected = max(align_tick, ack_tick + TRAVEL_DIR_CHANGE_HOLD) if c["flips"] else align_tick
                timing = dict(ack=ack_tick, align=align_tick, travel=travel_tick)
                self.assertEqual(travel_tick, expected, timing)
                if c["flips"]:
                    self.assertEqual({p for p in h.valves() if p.startswith("trvl")}, {"trvlLeRev", "trvlRiRev"})
                if c["boot"] == 0 and c["dwell"] == 0:
                    self.assertLess(align_tick, ack_tick + TRAVEL_DIR_CHANGE_HOLD, "case must separate the two")


if __name__ == "__main__":
    unittest.main()
