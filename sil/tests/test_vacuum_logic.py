"""
Vacuum lifter logic of X1Exc MdlApp that is observable WITHOUT a moving plant.

Purpose: pin down the pressure-model contract the Isaac Sim vacuum plant must satisfy
(resources/X1Exc_SIL_spec.md A5.1, A5.2, A8.1, A8.3, step 9). Every expectation below was
read out of the generated C and then confirmed against the running firmware.

Where the logic lives (generated C, MdlApp.c; grep -a):
  <S14>  ChkPanelAttachSts   S155 contact debounce       38995-39031  (fw.internal all_cups_in_contact)
                             S156 grip latch             39041-39077  (was_panel_attached/_detached)
                                                         init 38328-38332
  <S170> InhibitStsMgr       bits 9 / 10                 39577-39600
  <S229> StrkSnrSts          S231 1 s all-contact timer  48411-48499, isPickingReady 48510-48518
                             receiver one-shots          48526-48661, AllCatcherConfirmed 48668-48678
                             reset outside Releasing     52359-52498
  <S24>  VacLifterCtrl       manual request ORs          48687, 48693; "CurrStep <= Standby" 48711
         ForEach x2          S238 auto-release chart     48729-48833; relReq OR 48840
                             S239 per-circuit pump chart 48855-49147; PanelVacSts 49165
  outputs                    49171-49182, 50296, 52591-52606
  <S3>   AlarmCtrl           S132 Indicating_VacLifter   49909-50100
Stateflow sources (MdlApp.slx simulink/stateflow/): S155 = chart_2447 ChkSuctionCupContact,
S156 = chart_2476 ChkPanelAttachedSts, S142 = chart_2123 SetCtrlMode. S231, S238, S239 and S132
have NO chart_*.xml; their SSIDs in the generated C are the citation of record.

HOW AUTO STATES ARE REACHED WITHOUT A PLANT
  The spec's walk (A6) reaches Picking / EngagingVacuum / Releasing through the motion guards
  T474 / T475 / T295 and never mentions the TABLET STEP JUMPS of the main chart (chart_2537):
  a change of autoReqStep to Picking / EngagingVacuum / Placing / Releasing lands in the
  matching <X>Paused state (S136:177, 179, 221, 185; Standby handler MdlApp.c:25138-25265,
  PickingInhibited handler :23053-23200) and a StartPause edge then runs it (S136:191, 202,
  215). Harness.jump_to_step() is that route. Jumps exercised here start from Standby,
  PickingInhibited, PlacingPaused and PositioningPaused; the other Paused/Inhibited handlers
  carry the same guards but are not exercised in this file. A jump from a RUNNING state does
  nothing.

  The REAL entry into EngagingVacuum (T475) needs no motion either (TestStaticPickingRoute):
  a static IMU pose at the PreparePose targets (Harness.set_pose) reaches ApproachPanel, and
  static antenna fixes that put the undercarriage on the Picking waypoint
  (Harness.gnss_site + place_chassis) satisfy isUcPoseAligned.

Run:  cd ~/jude/xpanner-sim && python3 -m unittest sil.tests.test_vacuum_logic -v
"""
import itertools
import math
import unittest

import numpy as np

from sil.harness import Harness

PRESSURE_ATTACHED = -0.45     # SysPar.m:81, literal -0.45F at MdlApp.c:39045
PRESSURE_DETACHED = -0.10     # SysPar.m:82, literal -0.1F  at MdlApp.c:39056
# The float32 neighbours on the "not yet" side: the latch compares float32 inports against
# float32 literals, so these pin inclusive <= / >= to one ULP.
JUST_ABOVE_ATTACHED = float(np.nextafter(np.float32(PRESSURE_ATTACHED), np.float32(0.0)))
JUST_BELOW_DETACHED = float(np.nextafter(np.float32(PRESSURE_DETACHED), np.float32(-1.0)))
RELEASE_WINDOW_TICKS = 201    # S239:123 "cnt > 200" (MdlApp.c:48943), entry already counts 1
RECEIVER_ONESHOT_TICKS = 500  # Switch1 reload 500.0F, MdlApp.c:48556/48587/48618/48649
PICKING_READY_TICKS = 100     # S231:8 after(1, sec), MdlApp.c:48441-48443
CONTACT_CONFIRM_TICKS = 20    # S155 cnt >= CntSuctionContactConfirmDly, SysPar.m:465, MdlApp.c:39024
LED_HALF_PERIOD_TICKS = 50    # S132:119/121 after(0.5, sec), MdlApp.c:49973, :49994
LAT_TOL, LON_TOL = 0.20, 0.10  # LatPositioningTol / LonPositioningTol, SysPar.m:446-447, MdlApp.c:47703/47716
FRONT_PORTS = ("bm1Up", "bm1Down", "armIn", "armOut", "linkIn", "linkOut")

# <X>Paused state the tablet jump lands in (chart_2537 state names, fw.chart_states['main']).
PAUSED = {"Positioning": "PositioningPaused", "Picking": "PickingPaused",
          "EngagingVacuum": "VacuumPaused", "Placing": "PlacingPaused",
          "Releasing": "ReleasingPaused"}

# A static pose (firmware joint convention, degrees) whose actuator readings sit on the
# Picking PreparePose targets. Found by Newton iteration on y.dbg_F64_P11..P16
# (tarActuators_q vs actuators_q, MdlApp.c:51976-52078) with the compiled ShortArm set; lands
# within 0.02 deg of each target, far inside PreparePoseCtrlTol (bm1 2 deg, arm/link 1 deg,
# SysPar.m:451-459). If the parameter set changes the ApproachPanel wait below times out.
PREPARE_POSE_DEG = dict(q_bm1=-39.39, q_arm=105.16, q_inp=-36.48)

# Chassis origin (site metres, sil/geodesy.py bare-TM Site, chassis facing grid East) at which
# the default Harness.set_target_panel() waypoint reads distToWpTarLat = distToWpTarLon = 0
# (y.dbg_F64_P17 / P18, MdlApp.c:51993/51998; lat 1e-7, lon 3e-6 m). Found by Newton
# iteration on those two outputs with PREPARE_POSE_DEG published. Both distances are unsigned
# (S146:1:42-43, MdlApp.c:10176-10179).
UC_WAYPOINT_CHS = (0.413, 1.85, 0.0)


# -- helpers (no library equivalent: vacuum I/O is not in the harness) ---------------------
def set_vac(h, p1, p2):
    h.fw["u.vacPrs1"] = p1
    h.fw["u.vacPrs2"] = p2


def pumps(h):
    return tuple(v > 0.5 for v in h.fw["y.vacPumpCmd"])


def release_vlv(h):
    return tuple(v > 0.5 for v in h.fw["y.releaseOnOffVlvCmd"])


def suction_vlv(h):
    return tuple(v > 0.5 for v in h.fw["y.suctionOnOffVlvCmd"])


def front_valves(h):
    v = h.valves()
    return sum(v.get(p, 0.0) for p in FRONT_PORTS)


def run_lengths(seq):
    return [(k, len(list(g))) for k, g in itertools.groupby(seq)]


def vent_ticks(h, each=None, limit=1000):
    """Consecutive ticks on which both release valves are open, counting the CURRENT output
    (i.e. the tick that sampled the request) and ticking until they close. `each(h)` runs on
    every open tick."""
    n = 0
    while release_vlv(h) == (True, True):
        if each:
            each(h)
        n += 1
        if n > limit:
            raise AssertionError(f"release valves still open after {limit} ticks: {h.describe()}")
        h.tick()
    return n


def boot_to_standby(h, vac=(0.0, 0.0)):
    """Healthy machine in Standby: swing latched, RTK fixed, valid target. Vacuum inputs are
    written BEFORE the first tick so the grip latch sees them from power-up."""
    h.nominal_inputs()
    set_vac(h, *vac)
    h.set_swing_aligned(True)
    h.gnss_rtk_fixed()
    h.set_target_panel(panel_id=7)
    h.tick(3)
    h.request_step("Standby")
    h.run_until(lambda h: h.curr_step() == "Standby", 0.5, "Standby")
    return h


def jump(h, step, start=True):
    """Harness.jump_to_step() plus a check of where it landed: <X>Paused, or <X> running.
    Kept local because the library deliberately does not assert (it is also used from states
    where the jump must NOT happen)."""
    h.jump_to_step(step, start=start)
    want = step if start else PAUSED[step]
    assert h.main_state() == want and h.is_running() == start, f"wanted {want}: {h.describe()}"
    return h


def picking_with_static_plant(origin=None):
    """Running Picking with the IMUs at PREPARE_POSE_DEG. With `origin`, a bare-TM site and both
    antenna fixes for a chassis at that site position facing grid East. Site and antennas are
    written before boot (Localization never sees zero blh); the pose after boot, because
    nominal_inputs() publishes the rest pose."""
    h = Harness().reset()
    if origin is not None:
        h.gnss_site()
        h.place_chassis(origin)
    boot_to_standby(h)
    h.set_pose(**{k: math.radians(v) for k, v in PREPARE_POSE_DEG.items()})
    return jump(h, "Picking")


def approach_panel(origin=None):
    h = picking_with_static_plant(origin)
    h.run_until(lambda h: h.picking_step() == "PickingStep_ApproachPanel", 1.0, "ApproachPanel")
    return h


def start_manual_suction(h, path="u.jstSuctionReq"):
    h.pulse(path)
    h.run_until(lambda h: pumps(h) == (True, True), 0.03, "both pumps on")
    return h


# ======================================================================================
class TestGripLatch(unittest.TestCase):
    """S156 grip latch (chart_2476), observed directly (fw.internal) and through
    BIT_PANEL_ATTACHED (9) while in Positioning. Standby -> Positioning is a StartPause edge
    (S136:281, MdlApp.c:25278); GNSS is good so no other bit can mask it."""

    def setUp(self):
        self.h = boot_to_standby(Harness().reset())
        self.h.pulse("u.jstAutoReq_StartPause")
        self.h.run_until(lambda h: h.main_state() == "Positioning" and h.is_running(), 0.1, "Positioning")
        self.assertFalse(self.h.auto_inhibited(), self.h.describe())

    def test_either_sensor_declares_grip_inclusive_at_minus_0_45(self):
        # WHY: the plant's first circuit to reach -0.45 bar is what the firmware treats as
        # "panel held"; physics must attach the panel on the same condition (spec A5.1).
        # RULE: (vacPrs1 <= -0.45F) | (vacPrs2 <= -0.45F), S156:1:16 at MdlApp.c:39045; bit 9 =
        # step in {Positioning, Picking} && wasPanelAttached, MdlApp.c:39577-39582; Positioning
        # -> PositioningInhibited on S136:180 [isAutoCtrlInhibited], MdlApp.c:23921.
        h = self.h
        set_vac(h, JUST_ABOVE_ATTACHED, JUST_ABOVE_ATTACHED)   # one float32 ULP short, both
        h.tick(100)
        self.assertFalse(h.fw.internal("was_panel_attached"))
        self.assertFalse(h.inhibit_bit("PANEL_ATTACHED"))
        self.assertTrue(h.is_running())

        set_vac(h, 0.0, PRESSURE_ATTACHED)            # circuit 2 alone, exactly on the threshold
        h.tick()
        self.assertTrue(h.fw.internal("was_panel_attached"))   # latch updates in the same step
        h.run_until(lambda h: h.inhibit_bit("PANEL_ATTACHED"), 0.02, "grip on circuit 2")
        h.run_until(lambda h: h.main_state() == "PositioningInhibited", 0.02, "S136:180")

        set_vac(h, 0.0, 0.0)
        h.run_until(lambda h: not h.inhibit_bit("PANEL_ATTACHED"), 0.02, "release")
        set_vac(h, PRESSURE_ATTACHED, 0.0)            # circuit 1 alone
        h.run_until(lambda h: h.inhibit_bit("PANEL_ATTACHED"), 0.02, "grip on circuit 1")

    def test_release_needs_both_sensors_and_the_dead_band_holds(self):
        # WHY: a slow leak on one circuit, or a vent that stalls between -0.45 and -0.10, leaves
        # the firmware believing it still holds the panel; the plant's vent must reach -0.10
        # on BOTH sensors (spec A5.1, step 10).
        # RULE: detach only if (vacPrs1 >= -0.1F) & (vacPrs2 >= -0.1F), S156:1:21 at
        # MdlApp.c:39056; otherwise the persistent wasPanelAttached/Detached hold (S156:1:16-27,
        # MdlApp.c:39044-39077). A cleared inhibit goes PositioningInhibited -> PositioningPaused
        # (S136:208, MdlApp.c:24112).
        h = self.h
        set_vac(h, -0.6, -0.6)
        h.run_until(lambda h: h.inhibit_bit("PANEL_ATTACHED"), 0.02, "grip")
        for p in [(-0.2, -0.2), (-0.09, -0.2), (0.0, -0.11), (-0.2, 0.0), (-0.44, -0.44),
                  (PRESSURE_DETACHED, JUST_BELOW_DETACHED), (JUST_BELOW_DETACHED, PRESSURE_DETACHED)]:
            set_vac(h, *p)
            h.tick(300)
            self.assertTrue(h.fw.internal("was_panel_attached"), f"latch released at {p}")
            self.assertTrue(h.inhibit_bit("PANEL_ATTACHED"), f"released too early at {p}")
        set_vac(h, PRESSURE_DETACHED, PRESSURE_DETACHED)   # exactly -0.10 on both: inclusive
        h.run_until(lambda h: not h.inhibit_bit("PANEL_ATTACHED"), 0.02, "detach at -0.10/-0.10")
        self.assertTrue(h.fw.internal("was_panel_detached"))
        h.tick(20)
        self.assertEqual(h.main_state(), "PositioningPaused")   # never back to running by itself
        self.assertFalse(h.auto_inhibited())
        # The dead band also holds "detached": re-entering it from above does not grip.
        set_vac(h, -0.3, -0.3)
        h.tick(300)
        self.assertFalse(h.inhibit_bit("PANEL_ATTACHED"))
        h.pulse("u.jstAutoReq_StartPause")
        h.run_until(lambda h: h.main_state() == "Positioning" and h.is_running(), 0.05, "resume")


class TestGripLatchPowerUpAndPlacing(unittest.TestCase):
    """The same latch seen through BIT_PANEL_DETACHED (10) in Placing, reached by tablet jump."""

    def setUp(self):
        self.h = Harness().reset()

    def test_panel_detached_bit_in_placing_mirrors_the_latch(self):
        # WHY: bit 10 is the "lost the panel while carrying it" inhibit; the plant decides it
        # with the same two thresholds, so the carry phase needs the same pressure contract.
        # RULE: bit 10 = step == Placing && wasPanelDetached, MdlApp.c:39594-39600;
        # latch MdlApp.c:39044-39064. Placing reached via S136:221 (PlacingPaused, :25252).
        h = boot_to_standby(self.h)
        jump(h, "Placing", start=False)
        h.tick(2)
        self.assertTrue(h.inhibit_bit("PANEL_DETACHED"))     # ambient 0 bar = detached
        set_vac(h, -0.5, 0.0)                                 # one circuit grips
        h.run_until(lambda h: not h.inhibit_bit("PANEL_DETACHED"), 0.02, "grip clears bit 10")
        for p in [(-0.2, -0.2), (-0.09, -0.2), (-0.2, -0.05)]:
            set_vac(h, *p)
            h.tick(300)
            self.assertFalse(h.inhibit_bit("PANEL_DETACHED"), f"bit 10 set inside dead band at {p}")
        set_vac(h, PRESSURE_DETACHED, PRESSURE_DETACHED)
        h.run_until(lambda h: h.inhibit_bit("PANEL_DETACHED"), 0.02, "bit 10 at -0.10/-0.10")

    def test_latch_powers_up_neither_attached_nor_detached(self):
        # WHY: spec A3.5 says initialise both pressures to 0.0; this is why. A plant that boots
        # inside the dead band gives the firmware NO grip state at all: Placing does not see a
        # missing panel and Releasing->Complete (T302 [isPanelDetached]) cannot fire.
        # RULE: wasPanelAttached = wasPanelDetached = false at init, MdlApp.c:38328-38332
        # (chart_2476 isInit block).
        h = boot_to_standby(self.h, vac=(-0.2, -0.2))
        self.assertEqual((h.fw.internal("was_panel_attached"), h.fw.internal("was_panel_detached")), (0, 0))
        jump(h, "Placing", start=False)                       # from Standby
        h.tick(100)
        self.assertFalse(h.inhibit_bit("PANEL_DETACHED"))
        jump(h, "Positioning", start=False)                   # from PlacingPaused, S136:176
        h.tick(100)
        self.assertFalse(h.inhibit_bit("PANEL_ATTACHED"))
        set_vac(h, 0.0, 0.0)                                  # leave the band once
        h.tick(2)
        jump(h, "Placing", start=False)                       # from PositioningPaused
        h.tick(2)
        self.assertTrue(h.inhibit_bit("PANEL_DETACHED"))


class TestPanelVacSts(unittest.TestCase):
    def test_panel_vac_sts_is_a_strict_per_circuit_minus_0_35_threshold(self):
        # WHY: raw-output threshold check only (NOT an HMI claim, see TestVacuumIndicator).
        # y.PanelVacSts is a third pressure threshold next to the latch's -0.45 / -0.10: a
        # plant or a scorer that reads it as "panel gripped" is wrong in (-0.45, -0.35).
        # RULE: PanelVacSts[i] = double(vacPrs_i) < -0.35, ForEach over [vacPrs1, vacPrs2],
        # MdlApp.c:49165 (float32(-0.35) = -0.34999999 is not < -0.35).
        h = boot_to_standby(Harness().reset())
        for p, want in [((-0.35, 0.0), [False, False]), ((-0.351, 0.0), [True, False]),
                        ((0.0, -0.351), [False, True]), ((-0.44, -0.44), [True, True])]:
            set_vac(h, *p)
            h.tick()
            self.assertEqual(h.fw["y.PanelVacSts"], want, f"at {p}")
        self.assertFalse(h.fw.internal("was_panel_attached"))   # -0.44: status true, latch not


# ======================================================================================
class TestManualVacuum(unittest.TestCase):
    """Per-circuit pump chart S239 driven by jst/rmt Suction/Release requests."""

    def setUp(self):
        self.h = boot_to_standby(Harness().reset())

    def test_suction_request_starts_both_pumps_and_latches(self):
        # WHY: the plant must keep evacuating after the operator lets go of the button; the
        # pump command is a latched state, not a follow-the-switch signal.
        # RULE: Standby -(pickingReq)-> PressureChk -(vacPrs > -0.05)-> Vacuum{vacPmp=true},
        # S239:119/120, MdlApp.c:49021-49032, 48887-48931. Vacuum exits only on relReq or
        # vacPrs < -0.5. sucSol/fctSol are written true only in 'Release' (S239:44).
        h = self.h
        h.fw["u.jstSuctionReq"] = 1
        h.tick()
        self.assertEqual(pumps(h), (False, False))          # PressureChk dwells one tick
        h.fw["u.jstSuctionReq"] = 0                          # released before PressureChk exits
        h.tick()
        self.assertEqual(pumps(h), (True, True))
        for _ in range(1000):                                # 10 s, button released
            h.tick()
            self.assertEqual(pumps(h), (True, True))
            self.assertEqual(suction_vlv(h), (False, False))  # NOT opened while pumping
            self.assertEqual(release_vlv(h), (False, False))

    def test_pump_hysteresis_thresholds_are_strict_and_per_circuit(self):
        # WHY: these are the pressure levels the plant must be able to cross to see a
        # pump re-kick; circuit independence means the plant needs two separate volumes.
        # RULE: Vacuum -(vacPrs < -0.5F)-> VacuumStop, S239:121, MdlApp.c:49074;
        # VacuumStop -(vacPrs > -0.35F)-> Vacuum, S239:122, MdlApp.c:49126. ForEach over
        # [vacPrs1, vacPrs2], MdlApp.c:48699-48705.
        h = start_manual_suction(self.h)
        steps = [((-0.50, 0.0), (True, True)),     # not < -0.5
                 ((-0.51, 0.0), (False, True)),    # circuit 1 stops, circuit 2 unaffected
                 ((-0.36, 0.0), (False, True)),
                 ((-0.35, 0.0), (False, True)),    # not > -0.35
                 ((-0.34, 0.0), (True, True)),     # re-kick
                 ((-0.34, -0.6), (True, False)),
                 ((-0.34, -0.2), (True, True))]
        for p, expect in steps:
            set_vac(h, *p)
            h.tick()
            self.assertEqual(pumps(h), expect, f"at {p}")

    def test_residual_vacuum_at_request_leaves_the_pump_off_with_no_timeout(self):
        # WHY: the plant's "ambient" reading must be above -0.05 bar or a new suction never
        # starts: PressureChk has no timeout and no alarm. A sensor offset of -0.05 hangs it.
        # RULE: PressureChk -(double(vacPrs) > -0.05)-> Vacuum is its only non-release exit,
        # S239:120, MdlApp.c:48921 (float32(-0.05) < -0.05 double, so -0.05 itself fails).
        h = self.h
        set_vac(h, -0.05, -0.05)
        h.pulse("u.jstSuctionReq")
        h.tick(500)
        self.assertEqual(pumps(h), (False, False))
        self.assertEqual((h.fw["y.extLedCmd"][0], h.fw["y.extAlarmCmd"]), (False, False))
        set_vac(h, -0.049, -0.05)
        h.run_until(lambda h: pumps(h)[0], 0.02, "circuit 1 leaves PressureChk")
        h.tick(100)
        self.assertEqual(pumps(h), (True, False))
        h.pulse("u.jstReleaseReq")                            # relReq also exits PressureChk
        self.assertEqual(release_vlv(h), (True, True))

    def test_release_window_is_201_ticks_on_both_valves_with_pump_off(self):
        # WHY: this is the vent the plant gets to drop from grip to >= -0.10 bar; spec says
        # 2.0 s, the firmware gives 2.01 s. Both on/off valves open together.
        # RULE: Release entry cnt = cnt+1 (cnt was 0), stays while !(cnt > 200) i.e. 201 ticks,
        # vacPmp=false/sucSol=true/fctSol=true, MdlApp.c:48940-48985 (S239:44, S239:123).
        h = start_manual_suction(self.h)
        h.tick(50)
        h.fw["u.jstReleaseReq"] = 1                           # level write: count from tick 1
        h.tick()
        h.fw["u.jstReleaseReq"] = 0
        seq = [(release_vlv(h), suction_vlv(h), pumps(h))]
        for _ in range(300):
            h.tick()
            seq.append((release_vlv(h), suction_vlv(h), pumps(h)))
        on = ((True, True), (True, True), (False, False))
        off = ((False, False), (False, False), (False, False))
        self.assertEqual(run_lengths(seq), [(on, RELEASE_WINDOW_TICKS), (off, 301 - RELEASE_WINDOW_TICKS)])

    def test_held_release_repeats_with_a_one_tick_gap(self):
        # WHY: a stuck release switch (or a plant script that holds the request) produces a
        # continuous vent with a 10 ms blip, not a single 2 s pulse.
        # RULE: Release -> Standby after cnt > 200; Standby checks relReq first (S239:45,
        # MdlApp.c:48988-49019) and re-enters Release on the next tick.
        h = start_manual_suction(self.h)
        h.fw["u.jstReleaseReq"] = 1
        seq = []
        for _ in range(3 * (RELEASE_WINDOW_TICKS + 1)):
            h.tick()
            seq.append(release_vlv(h)[0])
        self.assertEqual(run_lengths(seq), [(True, 201), (False, 1), (True, 201), (False, 1),
                                            (True, 201), (False, 1)])

    def test_release_preempts_a_simultaneous_suction_request(self):
        # WHY: plant scripts must not assume suction wins a tie; and a suction pressed during
        # the vent is only served after the full window.
        # RULE: relReq is tested before pickingReq in Standby (S239:45 then S239:119,
        # MdlApp.c:48988-49032); 'Release' has no pickingReq exit (MdlApp.c:48940-48985).
        h = self.h
        h.fw["u.jstReleaseReq"] = 1
        h.fw["u.jstSuctionReq"] = 1
        h.tick()
        h.fw["u.jstReleaseReq"] = 0                           # suction stays held
        self.assertEqual((release_vlv(h), pumps(h)), ((True, True), (False, False)))
        n = vent_ticks(h, each=lambda h: self.assertEqual(pumps(h), (False, False), h.describe()))
        self.assertEqual(n, RELEASE_WINDOW_TICKS)
        # Release->Standby, Standby->PressureChk, PressureChk->Vacuum.
        h.run_until(lambda h: pumps(h) == (True, True), 0.03, "held suction served after the window")

    def test_manual_requests_are_only_honoured_in_notarget_and_standby(self):
        # WHY: manual suction is NOT a shortcut into the auto cycle (spec A3.4 calls it the "A6
        # shortcut"): once the machine is in any auto step the switches do nothing, and a pump
        # started by hand keeps running into Positioning where the release switch cannot stop it.
        # RULE: isPanelAttached(local) = CurrStep <= Standby (MdlApp.c:48711, rtCP_pooled5 =
        # Standby :1635) ANDs both pickingReq (49021-49028) and relReq (48840-48841); jst|rmt
        # ORs at 48687, 48693.
        h = self.h
        start_manual_suction(h, "u.rmtSuctionReq")           # remote works like the joystick
        h.pulse("u.jstAutoReq_StartPause")
        h.run_until(lambda h: h.curr_step() == "Positioning" and h.is_running(), 0.1, "Positioning")
        for path in ("u.jstReleaseReq", "u.rmtReleaseReq"):
            h.fw[path] = 1
            h.tick(50)
            h.fw[path] = 0
            self.assertEqual(pumps(h), (True, True), f"{path} stopped the pump in Positioning")
            self.assertEqual(release_vlv(h), (False, False))

        h2 = boot_to_standby(Harness().reset())
        h2.pulse("u.jstAutoReq_StartPause")
        h2.run_until(lambda h: h.curr_step() == "Positioning", 0.1, "Positioning")
        for path in ("u.jstSuctionReq", "u.rmtSuctionReq"):
            h2.fw[path] = 1
            h2.tick(200)
            h2.fw[path] = 0
            self.assertEqual(pumps(h2), (False, False), f"{path} honoured in Positioning")

        h3 = Harness().reset().nominal_inputs()               # NoTarget, no target at all
        h3.tick(2)
        self.assertEqual(h3.curr_step(), "NoTarget")
        start_manual_suction(h3)
        h3.pulse("u.rmtReleaseReq")
        self.assertEqual(release_vlv(h3), (True, True))

    def test_teo_todo_inports_are_not_read(self):
        # WHY: the glue writes isVacLifterInstalled = TRUE, tarVacPrs = -0.5, cntRelSolRun = 200
        # and cntCatcherSigRst = 5000 "To-do requested by Teo" (AppCtrlIf.c:655-658) but the
        # model uses literals; the sim must not expect to tune the vacuum through these inports.
        # A future firmware that wires them breaks this test.
        # RULE: zero reads of MdlApp_U.{tarVacPrs,cntRelSolRun,cntCatcherSigRst,
        # isVacLifterInstalled} (grep -a -c = 0 each); literals -0.5F (MdlApp.c:49074), 200U
        # (48943) and the receiver one-shot reload 500.0F (48556). Spec A3.7. Setting
        # isVacLifterInstalled = 0 is only shown not to disable the pump/release/receiver paths.
        def teo_inports(h):
            h.fw["u.tarVacPrs"] = -0.9
            h.fw["u.cntRelSolRun"] = 20
            h.fw["u.cntCatcherSigRst"] = 10
            h.fw["u.isVacLifterInstalled"] = 0

        h = self.h
        teo_inports(h)
        start_manual_suction(h)
        set_vac(h, -0.51, -0.51)
        h.tick()
        self.assertEqual(pumps(h), (False, False))            # -0.5 literal, not -0.9
        h.fw["u.jstReleaseReq"] = 1
        h.tick()
        h.fw["u.jstReleaseReq"] = 0
        self.assertEqual(vent_ticks(h), RELEASE_WINDOW_TICKS)  # not 21

        h = boot_to_standby(Harness().reset(), vac=(-0.6, -0.6))   # reset() zeroed the inports
        teo_inports(h)
        jump(h, "Releasing", start=False)
        h.fw["u.isSuctionCupContact"] = [1, 1, 1, 1]
        seq = []
        for _ in range(RECEIVER_ONESHOT_TICKS + 20):
            h.tick()
            seq.append(tuple(h.fw["y.isPanelReceiverConfirmed"]))
        self.assertEqual(run_lengths(seq), [((True,) * 4, RECEIVER_ONESHOT_TICKS), ((False,) * 4, 20)])  # not 10


# ======================================================================================
class TestContactDebounce(unittest.TestCase):
    """S155 areAllSuctionCupsInContact (chart_2447): 20-tick debounce, consumed only in
    ApproachPanel."""

    def test_debounce_is_20_ticks_resets_on_a_drop_and_is_inert_in_prepare_pick(self):
        # WHY: boundary documentation for the plant: four cups seated during PreparePick
        # change NOTHING the firmware outputs (step, sub-step, every valve), even though the
        # debounced flag itself is set after 20 ticks. A plant cannot use contacts to hurry
        # the pick; the arm has to reach the prepare pose first. A contact that chatters for one
        # tick restarts the 20 ticks.
        # RULE: debounce cnt >= 20U (S155:1:20, MdlApp.c:39024, SysPar.m:465), cnt = 0 on any
        # raw drop (chart_2447 'else cnt = uint32(0)'); the flag's only consumers are inside
        # pickingStep == ApproachPanel -- the front-end freeze (S142:1:78, MdlApp.c:7863-7873)
        # and T475 (MdlApp.c:23018-23024).
        def run(contacts):
            h = boot_to_standby(Harness().reset())            # nominal pose: not the prepare pose
            jump(h, "Picking")
            h.fw["u.isSuctionCupContact"] = contacts
            rows, flag = [], []
            for _ in range(500):
                h.tick()
                rows.append((h.curr_step(), h.is_running(), h.picking_step(), sorted(h.valves().items())))
                flag.append(h.fw.internal("all_cups_in_contact"))
            return rows, flag

        rows0, flag0 = run([0, 0, 0, 0])
        rows4, flag4 = run([1, 1, 1, 1])
        self.assertEqual(run_lengths(flag0), [(0, 500)])
        self.assertEqual(run_lengths(flag4), [(0, CONTACT_CONFIRM_TICKS - 1), (1, 500 - CONTACT_CONFIRM_TICKS + 1)])
        self.assertEqual(rows0, rows4)
        self.assertEqual(rows4[-1][:3], ("Picking", True, "PickingStep_PreparePick"))
        self.assertGreater(sum(v for p, v in rows4[-1][3] if p in FRONT_PORTS), 0.0)

        h = boot_to_standby(Harness().reset())
        flag = []
        for c in [[1, 1, 1, 1]] * (CONTACT_CONFIRM_TICKS - 1) + [[1, 0, 1, 1]] + [[1, 1, 1, 1]] * 30:
            h.fw["u.isSuctionCupContact"] = c
            h.tick()
            flag.append(h.fw.internal("all_cups_in_contact"))
        self.assertEqual(run_lengths(flag), [(0, CONTACT_CONFIRM_TICKS + CONTACT_CONFIRM_TICKS - 1),
                                             (1, 30 - CONTACT_CONFIRM_TICKS + 1)])

    def test_static_prepare_pose_reaches_approach_panel_and_four_cups_freeze_the_front_end(self):
        # WHY: the freeze is the firmware's "cups are down, stop pushing" rule; the plant's cup
        # contact model has to report all four for 20 ticks before bm1/arm/link let go. And it
        # needs NO motion to test: a static IMU pose at the PreparePose targets reaches
        # ApproachPanel.
        # RULE: PreparePick -> ApproachPanel on isTarActuatorReached rotate&bm1&arm&link
        # (S137:37, MdlApp.c:40535-40548); in ApproachPanel, areAllSuctionCupsInContact sets
        # ctrlMode bm1/arm/link = Disabled (S142:1:78-81, MdlApp.c:7863-7873), which zeroes
        # each axis controller output (e.g. arm S32/Switch, MdlApp.c:46324-46334). The decay
        # of the valve command after that is downstream shaping and is only bounded here.
        h = approach_panel()                                  # control: no contacts, no freeze
        for _ in range(300):
            h.tick()
            self.assertGreater(front_valves(h), 0.0, h.describe())
        self.assertEqual(h.picking_step(), "PickingStep_ApproachPanel")

        h = approach_panel()
        h.fw["u.isSuctionCupContact"] = [1, 1, 1, 1]
        seq = []
        for _ in range(300):
            h.tick()
            seq.append((h.fw.internal("all_cups_in_contact"), front_valves(h)))
        flip = [f for f, _ in seq].index(1) + 1
        self.assertEqual(flip, CONTACT_CONFIRM_TICKS)
        self.assertTrue(all(v > 0.0 for _, v in seq[:flip - 1]))
        after = [v for _, v in seq[flip - 1:]]
        self.assertTrue(all(b <= a for a, b in zip(after, after[1:])), "front valves rose after the freeze")
        self.assertEqual(after[-150:], [0.0] * 150)               # at rest within 1.5 s of the flip
        # No GNSS here, so T475's isUcPoseAligned (Delay11, MdlApp.c:23023-23024) is false:
        # TestStaticPickingRoute supplies it.
        self.assertEqual((h.curr_step(), h.picking_step()), ("Picking", "PickingStep_ApproachPanel"))
        h.fw["u.isSuctionCupContact"] = [1, 1, 1, 0]            # one cup lifts: control resumes
        h.tick()
        self.assertFalse(h.fw.internal("all_cups_in_contact"))
        self.assertGreater(front_valves(h), 0.0)


# ======================================================================================
class TestStaticPickingRoute(unittest.TestCase):
    """Picking -> EngagingVacuum through the REAL guard T475 with a static plant: IMUs at
    PREPARE_POSE_DEG and both antennas fixed so the undercarriage sits on the Picking waypoint
    (UC_WAYPOINT_CHS). T475 = areAllSuctionCupsInContact && Delay1(pickingStep) ==
    ApproachPanel && Delay11(isUcPoseAligned), S136:475 at MdlApp.c:23018-23027."""

    def test_t475_fires_on_the_debounce_tick_and_the_pump_starts_1_02_s_after_the_fourth_contact(self):
        # WHY: the plant's contact-timing budget for the real pick. T475 reads the 20-tick flag
        # computed earlier in the SAME step, so EngagingVacuum starts on contact tick 20. The
        # 1 s pump gate (S231) is not restarted by T475 or by the step: it counts raw contacts
        # in any step, so the pump starts 1.02 s after the fourth cup seated -- NOT 0.2 s + 1.02 s
        # -- and cups seated earlier (during PreparePick) shorten the wait after T475.
        # RULE: MdlApp_Subsystem runs S155 (MdlApp.c:39024) before the main chart's Picking
        # handler (called at :40237), which reads MdlApp_B.areAllSuctionCupsInContact (:23023);
        # S231 input = contacts[0..2] & contacts[3] with no step term (MdlApp.c:38996, :48411),
        # Standby -> Count resets the timer (:48491-48495), after(1, sec) at :48442;
        # pickingReq = CurrStep == EngagingVacuum && isPickingReady (MdlApp.c:49023-49026).
        h = approach_panel(UC_WAYPOINT_CHS)                   # control: aligned, no contacts
        self.assertLess(max(abs(h.fw["y.dbg_F64_P17"]), abs(h.fw["y.dbg_F64_P18"])), 1e-3)
        h.tick(300)
        self.assertEqual((h.main_state(), h.picking_step(), pumps(h)),
                         ("Picking", "PickingStep_ApproachPanel", (False, False)))

        h = approach_panel(UC_WAYPOINT_CHS)
        h.fw["u.isSuctionCupContact"] = [1, 1, 1, 1]
        rows = []
        for _ in range(PICKING_READY_TICKS + 10):
            h.tick()
            rows.append((h.main_state(), h.fw.internal("all_cups_in_contact"), pumps(h)))
        self.assertEqual(run_lengths(r[0] for r in rows),
                         [("Picking", CONTACT_CONFIRM_TICKS - 1), ("EngagingVacuum", len(rows) - CONTACT_CONFIRM_TICKS + 1)])
        self.assertEqual(run_lengths(r[1] for r in rows)[0], (0, CONTACT_CONFIRM_TICKS - 1))
        self.assertEqual(run_lengths(r[2] for r in rows),
                         [((False, False), PICKING_READY_TICKS + 1), ((True, True), len(rows) - PICKING_READY_TICKS - 1)])
        self.assertTrue(h.is_running())
        set_vac(h, 0.0, PRESSURE_ATTACHED)
        h.run_until(lambda h: h.main_state() == "ReadyToPlace", 0.02, "T257")

        # Cups seated from the moment Picking starts (still in PreparePick): T475 waits for
        # ApproachPanel (Delay1, so one tick later), but the pump still starts on contact tick 102.
        h = picking_with_static_plant(UC_WAYPOINT_CHS)
        h.fw["u.isSuctionCupContact"] = [1, 1, 1, 1]
        approach = t475 = pump = None
        for i in range(1, 400):
            h.tick()
            if approach is None and h.picking_step() == "PickingStep_ApproachPanel":
                approach = i
            if t475 is None and h.main_state() == "EngagingVacuum":
                t475 = i
            if any(pumps(h)):
                pump = i
                break
        self.assertIsNotNone(approach, h.describe())
        self.assertGreater(approach, CONTACT_CONFIRM_TICKS)      # flag already set on arrival
        self.assertEqual(t475, approach + 1)
        self.assertEqual(pump, PICKING_READY_TICKS + 2)
        self.assertLess(pump - t475, PICKING_READY_TICKS)

    def test_undercarriage_lat_lon_tolerances_gate_t475(self):
        # WHY: how precisely the plant's GNSS/track model must park the undercarriage for the
        # pick: outside 0.20 m lateral or 0.10 m longitudinal the four cups can be down forever
        # and the firmware stays in Picking with no inhibit.
        # RULE: isLatAligned = |distToWpTar_lat| < LatPositioningTol, isLonAligned =
        # |distToWpTar_lon| < LonPositioningTol (S111:1:14-15, MdlApp.c:47703, :47716; SysPar.m:
        # 446-447), held CntSettleTimeReq = 30 ticks (S111:1:27-35, :47726-47749, SysPar.m:431).
        cases = [("lon +0.095", (0.095, 0.0), True), ("lon +0.105", (0.105, 0.0), False),
                 ("lon -0.105", (-0.105, 0.0), False), ("lat +0.195", (0.0, 0.195), True),
                 ("lat +0.205", (0.0, 0.205), False)]
        for label, (de, dn), enters in cases:
            with self.subTest(label):
                h = approach_panel((UC_WAYPOINT_CHS[0] + de, UC_WAYPOINT_CHS[1] + dn, 0.0))
                # Grid East is the path direction here: lon follows E, lat follows N (unsigned).
                self.assertAlmostEqual(h.fw["y.dbg_F64_P18"], abs(de), delta=1e-3)
                self.assertAlmostEqual(h.fw["y.dbg_F64_P17"], abs(dn), delta=1e-3)
                h.fw["u.isSuctionCupContact"] = [1, 1, 1, 1]
                h.tick(100)
                self.assertEqual(h.curr_step(), "EngagingVacuum" if enters else "Picking", h.describe())
                self.assertFalse(h.auto_inhibited())


# ======================================================================================
class TestAutoVacuum(unittest.TestCase):
    """Auto-cycle vacuum logic in Picking / EngagingVacuum / Releasing, reached by tablet jump."""

    def setUp(self):
        self.h = Harness().reset()

    def test_vacuum_during_picking_latches_picking_inhibited(self):
        # WHY: spec A8.1 -- a plant whose pressure crosses -0.45 before EngagingVacuum (e.g. cups
        # sealing on contact) parks the machine with no operator-visible reason except bit 9.
        # RULE: T234 [isAutoCtrlInhibited || isPanelAttached] (MdlApp.c:23002-23015); the only
        # exit S136:220 [~isAutoCtrlInhibited] (MdlApp.c:23195) and bit 9 stays set while
        # CurrStep == Picking (MdlApp.c:39577-39582).
        h = boot_to_standby(self.h)
        jump(h, "Picking")
        set_vac(h, -0.5, 0.0)
        h.run_until(lambda h: h.main_state() == "PickingInhibited", 0.02, "PickingInhibited")
        self.assertTrue(h.inhibit_bit("PANEL_ATTACHED"))
        h.tick(3000)                                           # 30 s
        self.assertEqual((h.main_state(), h.curr_step(), h.is_running()), ("PickingInhibited", "Picking", False))
        h.pulse("u.jstAutoReq_StartPause")                     # operator presses auto: nothing
        h.tick(10)
        self.assertEqual(h.main_state(), "PickingInhibited")
        set_vac(h, -0.2, -0.2)                                 # partial vent: still latched
        h.tick(500)
        self.assertTrue(h.inhibit_bit("PANEL_ATTACHED"))
        self.assertEqual(h.main_state(), "PickingInhibited")
        set_vac(h, -0.1, -0.1)                                 # full vent on both circuits
        h.run_until(lambda h: not h.inhibit_bit("PANEL_ATTACHED"), 0.02, "bit 9 clears")
        h.tick(10)
        self.assertEqual((h.main_state(), h.curr_step(), h.is_running()), ("PickingPaused", "Picking", False))
        h.pulse("u.jstAutoReq_StartPause")
        h.run_until(lambda h: h.main_state() == "Picking" and h.is_running(), 0.05, "Picking resumes")

    def test_picking_inhibited_escapes(self):
        # WHY: spec A8.1 lists "a step change" as an escape. Bit 9 is step-gated, so a request
        # for Positioning parks the machine again (PositioningInhibited), while Cancel, Standby
        # and EngagingVacuum escape. The EngagingVacuum request skips straight to ReadyToPlace
        # with the pumps never started (a "held" panel with no pump running). Other target
        # steps (Placing, Releasing) are not exercised here.
        # RULE: PickingInhibited handler MdlApp.c:23053-23200: Cancel S136:188 (:23066),
        # S136:159 Standby, S136:176 Positioning (:23108), S136:179 EngagingVacuum (:23139);
        # bit 9 condition MdlApp.c:39577-39582 is step-gated, the latch itself (39044-39064) is
        # not; PositioningPaused -> S136:180 [isAutoCtrlInhibited] (:24176); EngagingVacuum
        # -> ReadyToPlace S136:257 [isPanelAttached] (:40199).
        def latched():
            h = boot_to_standby(Harness().reset())
            jump(h, "Picking")
            set_vac(h, -0.5, 0.0)
            h.run_until(lambda h: h.inhibit_bit("PANEL_ATTACHED") and h.main_state() == "PickingInhibited",
                        0.05, "latched")
            h.tick(10)
            return h

        h = latched()
        h.fw["u.tabletAutoReq_Cancel"] = 1
        h.run_until(lambda h: h.curr_step() == "NoTarget", 0.05, "Cancel escapes")
        h.fw["u.tabletAutoReq_Cancel"] = 0
        h.tick(2)
        self.assertFalse(h.inhibit_bit("PANEL_ATTACHED"))
        self.assertFalse(h.auto_inhibited())
        self.assertTrue(h.fw.internal("was_panel_attached"))   # bit cleared, grip still latched

        h = latched()
        h.request_step("Standby")
        h.run_until(lambda h: h.curr_step() == "Standby", 0.05, "Standby escapes")
        h.tick(2)
        self.assertFalse(h.inhibit_bit("PANEL_ATTACHED"))

        h = latched()
        h.request_step("Positioning")
        h.run_until(lambda h: h.curr_step() == "Positioning", 0.05, "Positioning")
        h.tick(50)
        self.assertTrue(h.inhibit_bit("PANEL_ATTACHED"))
        self.assertEqual(h.main_state(), "PositioningInhibited")

        h = latched()
        jump(h, "EngagingVacuum", start=False)
        h.tick(2)
        self.assertFalse(h.inhibit_bit("PANEL_ATTACHED"))
        self.assertFalse(h.auto_inhibited())
        h.pulse("u.jstAutoReq_StartPause")
        h.run_until(lambda h: h.main_state() == "ReadyToPlace", 0.02, "skips to ReadyToPlace")
        self.assertEqual(pumps(h), (False, False))

    def test_engaging_vacuum_pump_waits_for_one_second_of_all_four_contacts(self):
        # WHY: the auto pump start is gated by a SECOND contact debounce the spec does not
        # mention: all four raw contacts for 1.0 s (not the 20-tick one), then PressureChk.
        # The plant's cups must stay seated >= 1.02 s before any pressure change is expected.
        # It runs while VacuumPaused too: pickingReq has no StartStopSts term.
        # RULE: S231 Standby -(all4)-> Count -after(1,sec)-> PickingReady, any drop -> Standby
        # (MdlApp.c:48436-48499); pickingReq = EngagingVacuum && isPickingReady
        # (MdlApp.c:48510-48518, 49021-49032). after() is tested before the drop guard, so a
        # drop on exactly the expiring tick does not cancel it (S231:8 at 48441 vs S231:11 at 48451).
        h = boot_to_standby(self.h)
        jump(h, "EngagingVacuum")
        h.fw["u.isSuctionCupContact"] = [1, 1, 1, 0]
        h.tick(300)
        self.assertEqual(pumps(h), (False, False))
        h.fw["u.isSuctionCupContact"] = [1, 1, 1, 1]
        h.tick(80)                                             # > 20-tick debounce, < 1 s
        self.assertEqual(pumps(h), (False, False))
        h.fw["u.isSuctionCupContact"] = [0, 1, 1, 1]           # one-tick drop resets the timer
        h.tick()
        h.fw["u.isSuctionCupContact"] = [1, 1, 1, 1]
        n = 0
        while not any(pumps(h)):
            h.tick()
            n += 1
            self.assertLess(n, 200, h.describe())
        self.assertEqual(n, PICKING_READY_TICKS + 2)
        self.assertEqual(pumps(h), (True, True))
        self.assertEqual((h.main_state(), h.is_running()), ("EngagingVacuum", True))

        # Both sides of the expiry boundary: contacts on ticks 1..k, a drop on tick k+1.
        for k, want in ((PICKING_READY_TICKS - 1, (False, False)), (PICKING_READY_TICKS, (True, True))):
            h = boot_to_standby(Harness().reset())
            jump(h, "EngagingVacuum")
            h.fw["u.isSuctionCupContact"] = [1, 1, 1, 1]
            h.tick(k)
            h.fw["u.isSuctionCupContact"] = [1, 1, 0, 1]
            h.tick(300)
            self.assertEqual(pumps(h), want, f"drop on tick {k + 1}")

        h = boot_to_standby(Harness().reset())                 # paused: same timer, same start
        jump(h, "EngagingVacuum", start=False)
        h.fw["u.isSuctionCupContact"] = [1, 1, 1, 1]
        h.tick(PICKING_READY_TICKS + 1)
        self.assertEqual(pumps(h), (False, False))
        h.tick()
        self.assertEqual(pumps(h), (True, True))
        self.assertEqual((h.main_state(), h.is_running()), ("VacuumPaused", False))

    def test_engaging_vacuum_has_no_timeout_and_ignores_contact_loss(self):
        # WHY: spec A8.3 -- a vacuum sensor in diag fault reads 0.0 bar (PrePostProc_If.c:
        # 739-756) and a cup lifting after the pump starts does not stop it; the harness, not
        # the firmware, must own the timeout. After T257 the pump chart keeps regulating on its
        # own hysteresis in ReadyToPlace: the step change does not reset it.
        # RULE: EngagingVacuum guards are pause, inhibit, [isPanelAttached] only
        # (S136:192/189/257, MdlApp.c:40160-40203); pump chart Vacuum exits only on relReq /
        # vacPrs < -0.5F (49074), VacuumStop re-kicks on > -0.35F (49126).
        h = boot_to_standby(self.h)
        jump(h, "EngagingVacuum")
        h.fw["u.isSuctionCupContact"] = [1, 1, 1, 1]
        h.run_until(lambda h: pumps(h) == (True, True), 1.1, "pump start")
        h.fw["u.isSuctionCupContact"] = [0, 0, 0, 0]
        for _ in range(6000):                                  # 60 s at 0.0 bar
            h.tick()
            self.assertEqual(pumps(h), (True, True))
        self.assertEqual((h.main_state(), h.is_running()), ("EngagingVacuum", True))
        set_vac(h, 0.0, PRESSURE_ATTACHED)
        h.run_until(lambda h: h.main_state() == "ReadyToPlace", 0.02, "T257 on one circuit")
        self.assertFalse(h.is_running())
        set_vac(h, -0.6, -0.6)
        h.tick()
        self.assertEqual(pumps(h), (False, False))             # VacuumStop in ReadyToPlace
        h.tick(300)
        self.assertEqual(pumps(h), (False, False))
        set_vac(h, -0.3, -0.3)
        h.tick()
        self.assertEqual(pumps(h), (True, True))               # re-kick while carrying
        self.assertEqual(h.main_state(), "ReadyToPlace")

    def test_pause_and_cancel_do_not_stop_the_pump(self):
        # WHY: pausing or ending auto during EngagingVacuum leaves both pumps running
        # indefinitely; only a manual release (now allowed, step <= Standby) stops them. The
        # plant must keep honouring vacPumpCmd after the auto cycle is gone. Open question
        # for David whether that is intended (keeps a held panel) -- it also happens with none.
        # RULE: S239 has no guard on StartStopSts or CurrStep in Vacuum/VacuumStop
        # (MdlApp.c:49041-49146); Cancel from VacuumPaused S136:188 (MdlApp.c:15322) -> NoTarget.
        h = boot_to_standby(self.h)
        jump(h, "EngagingVacuum")
        h.fw["u.isSuctionCupContact"] = [1, 1, 1, 1]
        h.run_until(lambda h: pumps(h) == (True, True), 1.1, "pump start")
        h.pulse("u.jstAutoReq_StartPause")
        h.run_until(lambda h: h.main_state() == "VacuumPaused", 0.05, "VacuumPaused")
        h.tick(500)
        self.assertEqual((h.main_state(), pumps(h)), ("VacuumPaused", (True, True)))
        h.fw["u.tabletAutoReq_Cancel"] = 1
        h.run_until(lambda h: h.curr_step() == "NoTarget", 0.05, "Cancel")
        h.fw["u.tabletAutoReq_Cancel"] = 0
        h.tick(1000)
        self.assertEqual(pumps(h), (True, True))
        h.pulse("u.jstReleaseReq")
        self.assertEqual((release_vlv(h), pumps(h)), ((True, True), (False, False)))

    def test_auto_release_waits_for_four_receiver_edges_then_repeats_until_both_vented(self):
        # WHY: spec step 9 -- the blow-off needs all four panel-receiver confirmations, and a
        # vent that stalls above -0.10 on one circuit keeps the machine in Releasing. The
        # firmware does not stop venting there: the 201-tick pulse repeats every 202 ticks.
        # RULE: S238 StandBy -(Releasing && StartStopSts)-> waitCatcherConfirm
        # -(allCatcherConfirmed)-> relReq, exit only on hasChanged(CurrStep) (S238:8/16/19,
        # MdlApp.c:48760-48830); allCatcherConfirmed = AND of the four one-shots (:48674-48678),
        # each armed on a rising edge of contact && step == Releasing (MdlApp.c:48526-48661);
        # T302 [isPanelDetached] (MdlApp.c:40377).
        h = boot_to_standby(self.h, vac=(-0.6, -0.6))
        jump(h, "Releasing")
        h.tick(200)
        self.assertEqual(release_vlv(h), (False, False))
        contacts = [0, 0, 0, 0]
        for i in range(4):
            self.assertEqual(release_vlv(h), (False, False), f"vented with only {i} cups")
            contacts[i] = 1
            h.fw["u.isSuctionCupContact"] = contacts
            h.tick(50)
        self.assertEqual(h.fw["y.isPanelReceiverConfirmed"], [True, True, True, True])
        self.assertEqual(release_vlv(h), (True, True))
        self.assertEqual(suction_vlv(h), (True, True))
        set_vac(h, -0.15, -0.05)                               # circuit 1 stalls in the dead band
        seq = []
        for _ in range(700):
            h.tick()
            seq.append((h.curr_step(), release_vlv(h)[0]))
        self.assertEqual({s for s, _ in seq}, {"Releasing"})
        lengths = run_lengths(v for _, v in seq)
        self.assertIn((False, 1), lengths)
        self.assertIn((True, RELEASE_WINDOW_TICKS), lengths)
        self.assertNotIn(False, [v for v, n in lengths if n > 1])
        set_vac(h, PRESSURE_DETACHED, -0.05)
        h.run_until(lambda h: h.curr_step() == "Complete", 0.02, "T302")

    def test_receiver_confirmation_is_a_five_second_one_shot_and_pause_rearms(self):
        # WHY: spec step 9 says a pause in waitCatcherConfirm "silently cancels the blow-off".
        # Firmware: the pause aborts the wait but resume re-arms it, and contacts made while
        # paused count (ReleasingPaused still reports CurrStep = Releasing). The real trap is
        # resuming > 5 s after the cups seated: confirmations have expired, the cups are still
        # down, no new edge arrives, and the machine waits forever. Re-seating ONE cup does not
        # help: all four must re-seat within 5 s of each other.
        # RULE: S238:11 hasChanged(StartStopSts) at 48800 before S238:16 at 48815 (MdlApp.c);
        # one-shot reload 500 on rising edge, else previous - 1 (MdlApp.c:48553-48661);
        # AllCatcherConfirmed = AND of all four (MdlApp.c:48674-48678).
        h = boot_to_standby(self.h, vac=(-0.6, -0.6))
        jump(h, "Releasing", start=False)
        h.fw["u.isSuctionCupContact"] = [1, 1, 1, 1]           # seated while paused
        seq = []
        for _ in range(RECEIVER_ONESHOT_TICKS + 20):
            h.tick()
            seq.append(tuple(h.fw["y.isPanelReceiverConfirmed"]))
        self.assertEqual(run_lengths(seq), [((True,) * 4, RECEIVER_ONESHOT_TICKS), ((False,) * 4, 20)])
        h.pulse("u.jstAutoReq_StartPause")
        h.tick(600)
        self.assertEqual(h.main_state(), "Releasing")
        self.assertEqual(release_vlv(h), (False, False))       # expired: stuck in waitCatcherConfirm
        h.fw["u.isSuctionCupContact"] = [0, 1, 1, 1]           # lift and re-seat cup 0 only
        h.tick()
        h.fw["u.isSuctionCupContact"] = [1, 1, 1, 1]
        h.tick(100)
        self.assertEqual(h.fw["y.isPanelReceiverConfirmed"], [True, False, False, False])
        self.assertEqual(release_vlv(h), (False, False))
        h.fw["u.isSuctionCupContact"] = [1, 0, 0, 0]           # the other three, within 5 s
        h.tick()
        h.fw["u.isSuctionCupContact"] = [1, 1, 1, 1]
        h.run_until(lambda h: release_vlv(h) == (True, True), 0.03, "blow-off after re-seat")

        h = boot_to_standby(Harness().reset(), vac=(-0.6, -0.6))   # pause/resume inside 5 s
        jump(h, "Releasing")
        h.fw["u.isSuctionCupContact"] = [1, 1, 1, 0]
        h.tick(10)
        h.pulse("u.jstAutoReq_StartPause")                     # pause: S238 -> StandBy
        h.run_until(lambda h: h.main_state() == "ReleasingPaused", 0.05, "ReleasingPaused")
        h.fw["u.isSuctionCupContact"] = [1, 1, 1, 1]           # 4th cup seats while paused
        h.tick(100)
        self.assertEqual(release_vlv(h), (False, False))
        h.pulse("u.jstAutoReq_StartPause")
        h.run_until(lambda h: release_vlv(h) == (True, True), 0.03, "resume re-arms and vents")

    def test_contacts_already_seated_before_releasing_count_as_edges(self):
        # WHY: a plant whose cups are resting on the receiver when the step becomes Releasing
        # does not need to lift them: the step change itself is the edge. Nothing carries over
        # from before Releasing either -- the one-shot memory is held at 0 outside the step.
        # RULE: edge of (contact & CurrStep == Releasing), MdlApp.c:48526-48556; memory forced
        # to 0 while CurrStep != Releasing and saturated >= 0, MdlApp.c:52359-52498.
        h = boot_to_standby(self.h, vac=(-0.6, -0.6))
        h.fw["u.isSuctionCupContact"] = [1, 1, 1, 1]
        h.tick(50)
        self.assertEqual(h.fw["y.isPanelReceiverConfirmed"], [False] * 4)   # Standby: no count
        jump(h, "Releasing")                                   # 1 tick paused + pulse (high, low)
        self.assertEqual(h.fw["y.isPanelReceiverConfirmed"], [True] * 4)
        self.assertEqual(release_vlv(h), (True, True))

    def test_pause_during_blow_off_keeps_venting(self):
        # WHY: once the blow-off has begun, pausing auto does not close the valves; the plant
        # keeps venting (and re-venting) while StartStopSts is false.
        # RULE: S238 'relReq' exits only on hasChanged(autoCtrl_CurrStep) (MdlApp.c:48774-48793),
        # and pause keeps CurrStep = Releasing (ReleasingPaused entry S136:27, MdlApp.c:40367).
        h = boot_to_standby(self.h, vac=(-0.6, -0.6))
        jump(h, "Releasing")
        h.fw["u.isSuctionCupContact"] = [1, 1, 1, 1]
        h.run_until(lambda h: release_vlv(h) == (True, True), 0.03, "blow-off")
        h.pulse("u.jstAutoReq_StartPause")
        h.run_until(lambda h: h.main_state() == "ReleasingPaused", 0.05, "ReleasingPaused")
        seq = []
        for _ in range(500):
            h.tick()
            seq.append(release_vlv(h)[0])
        self.assertEqual((h.main_state(), h.curr_step(), h.is_running()), ("ReleasingPaused", "Releasing", False))
        lengths = run_lengths(seq)
        self.assertTrue(all(v for v, n in lengths if n > 1), lengths)   # only 1-tick gaps
        self.assertIn((False, 1), lengths)                               # the window did re-arm
        self.assertGreaterEqual(sum(seq), 490)


# ======================================================================================
class TestVacuumIndicator(unittest.TestCase):
    """<S132> Indicating_VacLifter (AlarmCtrl <S3>) -> extLedCmd[0] / extAlarmCmd
    (AppCtrlIf.c:663-667), MdlApp.c:49909-50100.

    States: off -(PanelVacSts any, S132:111)-> PanelVacAchived{led steady, alarm off};
    PanelVacAchived -(~PanelVacSts, S132:112)-> off; off/PanelVacAchived -(pump running,
    S132:126/141)-> NormalBeeping{led+alarm 0.5 s off / 0.5 s on} -(no pump, S132:128)-> off.
    NormalBeeping has NO PanelVacSts exit, so while a pump runs the LED says nothing about
    pressure. y.extAlarmCmd = S132 alarm | Indicating_MachineMotion <S131> | Indicating_
    MachineCalib <S130> (MdlApp.c:50099-50100); the other two are quiet in every scenario
    below (the tests assert alarm == led)."""

    def _led_alarm(self, h, n):
        led, alarm = [], []
        for _ in range(n):
            h.tick()
            led.append(h.fw["y.extLedCmd"][0])
            alarm.append(h.fw["y.extAlarmCmd"])
        return led, alarm

    def test_led_blinks_while_any_pump_runs_and_is_steady_once_pumps_stop_below_minus_0_35(self):
        # WHY: this is what the operator sees; the plant has to let the pumps stop (reach
        # < -0.5 bar) for the beeping to end, and the LED is only steady if the pressure then
        # also reads below -0.35 (PanelVacSts).
        # RULE: NormalBeeping S132:119/121 after(0.5,sec) (MdlApp.c:49973, :49994), exit
        # S132:128 [vacPumpCmd==false] (:49946); off -> PanelVacAchived on PanelVacSts (S132:111,
        # :50058-50067), PanelVacAchived during {extLed1 = true, alarm false} (:50010-50012);
        # extLedCmd[0] = extLed1 (MdlApp.c:52591).
        h = Harness().reset().nominal_inputs()
        h.tick(2)
        start_manual_suction(h)
        led, alarm = self._led_alarm(h, 400)
        self.assertEqual({n for _, n in run_lengths(led)[1:-1]}, {LED_HALF_PERIOD_TICKS})
        self.assertEqual(alarm, led)
        set_vac(h, -0.6, -0.6)                                    # both circuits reach VacuumStop
        h.tick(3)
        self.assertEqual(pumps(h), (False, False))
        led, alarm = self._led_alarm(h, 300)
        self.assertEqual((set(led), set(alarm)), ({True}, {False}))

    def test_led_ignores_pressure_while_a_pump_runs_in_the_auto_cycle(self):
        # WHY: in EngagingVacuum between -0.35 and -0.45 the LED is NOT "vacuum achieved": it
        # blinks exactly as at 0 bar because a pump runs. After T257 it goes steady only once
        # both pumps stop, and a re-kick while carrying makes it blink again.
        # RULE: as the class docstring; pump chart thresholds MdlApp.c:49074, :49126.
        h = boot_to_standby(Harness().reset())
        jump(h, "EngagingVacuum")
        h.fw["u.isSuctionCupContact"] = [1, 1, 1, 1]
        h.run_until(lambda h: pumps(h) == (True, True), 1.1, "pump start")
        set_vac(h, -0.40, -0.40)
        led, alarm = self._led_alarm(h, 400)
        self.assertEqual(h.fw["y.PanelVacSts"], [True, True])
        self.assertEqual({n for _, n in run_lengths(led)[1:-1]}, {LED_HALF_PERIOD_TICKS})
        self.assertEqual(alarm, led)
        self.assertEqual(h.main_state(), "EngagingVacuum")

        set_vac(h, -0.6, -0.6)                                    # grip, both pumps stop
        h.run_until(lambda h: h.main_state() == "ReadyToPlace", 0.02, "T257")
        h.tick(2)
        led, alarm = self._led_alarm(h, 300)
        self.assertEqual((set(led), set(alarm), pumps(h)), ({True}, {False}, (False, False)))

        set_vac(h, -0.3, -0.3)                                    # leak past -0.35: re-kick
        led, alarm = self._led_alarm(h, 300)
        self.assertEqual(pumps(h), (True, True))
        self.assertEqual({n for _, n in run_lengths(led)[1:-1]}, {LED_HALF_PERIOD_TICKS})
        self.assertEqual(alarm, led)

    def test_led_is_off_while_the_latch_still_holds_after_a_stalled_vent(self):
        # WHY: the one real HMI/latch mismatch. In Releasing the pump is off; a vent that
        # stalls in (-0.35, -0.10) shows LED off while the firmware still considers the panel
        # attached (T302 cannot fire). A scorer must take grip ground truth from the latch
        # thresholds, never from the LED.
        # RULE: off lights only on PanelVacSts (vacPrs < -0.35, MdlApp.c:49165; S132:111 at
        # :50058); PanelVacAchived -> off on ~PanelVacSts (S132:112, :50015); latch detaches
        # only at >= -0.1F on both (MdlApp.c:39056).
        h = boot_to_standby(Harness().reset(), vac=(-0.6, -0.6))
        jump(h, "Releasing")
        h.fw["u.isSuctionCupContact"] = [1, 1, 1, 1]
        h.run_until(lambda h: release_vlv(h) == (True, True), 0.03, "blow-off")
        set_vac(h, -0.2, -0.2)
        h.tick(2)
        led, alarm = self._led_alarm(h, 600)
        self.assertEqual((set(led), set(alarm), pumps(h)), ({False}, {False}, (False, False)))
        self.assertTrue(h.fw.internal("was_panel_attached"))
        self.assertEqual(h.curr_step(), "Releasing")
        set_vac(h, -0.4, -0.4)                                    # same latch, below -0.35: lit
        h.tick(2)
        led, _ = self._led_alarm(h, 100)
        self.assertEqual(set(led), {True})


if __name__ == "__main__":
    unittest.main()
