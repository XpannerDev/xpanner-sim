"""
Core harness checks: the build is the one we think it is, the harness talks to it
correctly, and the boot-time gates behave as resources/X1Exc_SIL_spec.md says.

Run:  python3 -m unittest discover -s sil/tests -t .   (from the repo root)
"""
import unittest

from sil.harness import Harness, firmware


class TestBuild(unittest.TestCase):
    def test_manifest_matches_shipped_code(self):
        m = firmware().manifest
        self.assertEqual(m["inhibit_masks"]["AUTO_INHIBIT_MASK"]["value"], 0xC78E)
        self.assertEqual(firmware().enums["AutoCtrlStep"]["Complete"], 9)
        self.assertNotIn("CalibForkPnt2", firmware().enums["AutoCtrlStep"])  # commented out upstream

    def test_compiled_parameter_set_is_shortarm(self):
        # SysPar.m:4 sources ECR88D_ShortArm. If this ever flips to 2.1 the LongArm
        # schema problem in the spec (A0.2) may be fixed upstream: re-read before trusting.
        fw = firmware()
        fw.reset()
        self.assertAlmostEqual(fw["par.parKin.lenArm"], 1.7, places=5)
        self.assertAlmostEqual(fw["par.parKin.angForkUpLimit"], -0.284918, places=5)


class TestHarness(unittest.TestCase):
    def setUp(self):
        self.h = Harness().reset()

    def test_reset_restores_patched_parameters(self):
        fw = self.h.fw
        fw["par.parKin.lenArm"] = 2.1
        self.h.reset()
        self.assertAlmostEqual(fw["par.parKin.lenArm"], 1.7, places=5)

    def test_same_inputs_same_trajectory_after_reset(self):
        def run():
            h = Harness().reset().nominal_inputs()
            h.trace("y.autoCtrl_CurrStep", "y.autoCtrl_InhibitSts", "y.jnts.BmMntToBm1.q")
            h.pulse("u.isSwingAligned")
            h.set_target_panel()
            h.tick(3)
            h.request_step("Standby")
            h.tick(50)
            return h.trace_rows
        self.assertEqual(run(), run())

    def test_unknown_signal_names_fail_loudly(self):
        with self.assertRaises(KeyError):
            self.h.fw["u.isSwingAlinged"]


class TestBootGates(unittest.TestCase):
    """Spec A6 step 0 and A3.4."""

    def setUp(self):
        self.h = Harness().reset()

    def test_first_tick_state(self):
        self.h.tick()
        self.assertEqual(self.h.curr_step(), "NoTarget")
        self.assertFalse(self.h.fw["y.autoCtrl_StartStopSts"])
        self.assertFalse(self.h.fw["y.isCalibrating"])
        self.assertIn("BIT_SWING_NOT_INIT", self.h.inhibit_names())
        self.assertIn("BIT_RMT_CTRL_ERR", self.h.inhibit_names())
        self.assertTrue(self.h.auto_inhibited())

    def test_isRmtOk_clears_only_the_remote_bit(self):
        self.h.tick()
        before = self.h.inhibit_status()
        self.h.fw["u.isRmtOk"] = 1
        self.h.tick()
        self.assertEqual(before & ~self.h.inhibit_status(), 1 << 4)

    def test_swing_not_init_holds_until_alignment_is_seen(self):
        h = self.h.nominal_inputs()
        h.tick(300)
        self.assertIn("BIT_SWING_NOT_INIT", h.inhibit_names())
        h.pulse("u.isSwingAligned")
        h.tick(2)
        self.assertNotIn("BIT_SWING_NOT_INIT", h.inhibit_names())

    def test_aligned_at_power_up_counts_as_the_edge(self):
        # Written first as "a level from tick 0 is not an edge" -- and the firmware
        # disagreed: the edge detector's previous value is zeroed by initialize(), so a
        # machine that powers up already aligned latches immediately. Physically right.
        h = self.h.nominal_inputs()
        h.fw["u.isSwingAligned"] = 1
        h.tick(2)
        self.assertNotIn("BIT_SWING_NOT_INIT", h.inhibit_names())

    def test_no_gnss_inhibits_auto_after_the_on_delay(self):
        h = self.h.nominal_inputs()
        h.pulse("u.isSwingAligned")
        h.set_target_panel()
        h.tick(15)
        self.assertFalse(h.auto_inhibited())
        h.tick(10)                             # past the 20-tick on-delay (SysPar.m:57)
        self.assertTrue(h.auto_inhibited())
        h.gnss_rtk_fixed()
        h.tick(5)
        self.assertFalse(h.auto_inhibited())

    def test_accuracy_thresholds_are_inports_not_constants(self):
        # Without nominal_inputs() the thresholds are 0: good GNSS can never clear it.
        h = self.h
        h.fw["u.isRmtOk"] = 1
        h.pulse("u.isSwingAligned")
        h.set_target_panel()
        h.gnss_rtk_fixed()
        h.tick(100)
        self.assertTrue(h.auto_inhibited())
        h.fw["u.verticalAccuracyGoodThld"] = 0.02
        h.fw["u.verticalAccuracyPoorThld"] = 0.04
        h.tick(5)
        self.assertFalse(h.auto_inhibited())

    def test_notarget_to_standby(self):
        h = self.h.nominal_inputs()
        h.pulse("u.isSwingAligned")
        h.set_target_panel(panel_id=7)
        h.tick(3)                              # tarPanelIdAck, then isTarPanelValid, one tick each
        self.assertEqual(h.fw["y.tarPanelIdAck"], 7)
        h.request_step("Standby")
        h.run_until(lambda h: h.curr_step() == "Standby", 0.5, "Standby")
        self.assertFalse(h.fw["y.autoCtrl_StartStopSts"])

    def test_repeating_the_same_step_value_is_a_noop(self):
        h = self.h.nominal_inputs()
        h.pulse("u.isSwingAligned")
        h.fw["u.autoReqStep"] = h.fw.enum_value("AutoCtrlStep", "Standby")   # written BEFORE a target exists
        h.tick(2)
        h.set_target_panel()
        h.tick(20)                             # target now valid, but autoReqStep never changed again
        self.assertEqual(h.curr_step(), "NoTarget")


if __name__ == "__main__":
    unittest.main()
