"""
The actuation plant (sil/valves.py, sil/plant.py) against the compiled X1Exc firmware.

Three layers, each proven before the next one relies on it:
  1. OPEN LOOP   the table inversion, the pilot-pressure curve, the valve lag and the cylinder
                 Jacobian, as numbers (hand calculation, spec A4.3's independently computed table,
                 finite differences) and against y.cyls.*.strk.
  2. SENSORS     a plant that moves (without the firmware's command) is read back correctly by the
                 firmware: joint angles, gyro rates, cylinder stroke speed, and the swing angle, which
                 the firmware takes from the antenna baseline only while u.ehPiPrs says "operated".
  3. CLOSED LOOP the firmware drives the plant: Positioning Raise/Swing/Align (task space front end,
                 joint-space swing/tilt/rotator), Picking PreparePick (joint-space boom/arm/link, and the
                 min-speed hold from rest), a backwards axis breaks its loop, and calibration steps
                 20/23/24/25/26 identify the plant's own valve (onset command, speed at the reference
                 command), rebuild the chassis / arm / input-link / tilt IMU mounts of the PLANT's
                 hardware -- correcting a wrong stored mount -- which pins the accelerometer sign (-1 g,
                 test_accelerometer_sign_is_minus_one_g). Every calibration runs with strict joint stops.
  The plant's physical parameters are frozen from the compiled set and never follow par.* (layer 2,
  TestHardwareIsNotTheParameterSet), so every "firmware reads X" above is a firmware property, not the
  plant echoing the firmware's own parameters back.

Charts cited as chart_NNNN are the Stateflow XML of MdlApp.slx (script line numbers); MdlApp.c:N is
X1Exc/Asw/GeneratedCode/MdlApp_ert_rtw/MdlApp.c. Every plant constant that is not from the firmware is
tagged GUESS/ASSUMPTION in sil/valves.py or sil/plant.py.

Run:  cd xpanner-sim && python3 -m unittest sil.tests.test_valve_plant -v
"""
import math
import unittest

import numpy as np

from sil import kinematics as kin
from sil import valves as vlv
from sil.harness import Harness, SaveHandshake
from sil.plant import Hardware, JointStopError, KinematicPlant

DEG = math.pi / 180.0

# PropVlvRefCmd, the fixed command of every _ToPnt leg and Y[2] of every identified table (SysPar.m:136-156).
PROP_VLV_REF_CMD = dict(swingLe=60.0, swingRi=60.0, bm1Up=70.0, bm1Down=70.0, armIn=70.0, armOut=70.0,
                        linkIn=70.0, linkOut=70.0, tiltPosi=70.0, tiltNega=70.0, rotPosi=100.0, rotNega=100.0)
IDENTIFIED_X1 = float(np.float32(0.01))     # chart_2338 l.78-111: the knee a calibration writes
CNT_CALIB_TIMEOUT = 1000                    # SysPar.m:104; a leg ends on angle OR cnt > 1000 (MdlApp.c:36320)


def booted(plant, extra=(), setup=None):
    """Healthy machine with `plant` publishing every sensor (it owns isSwingAligned: the house starts at
    swing 0, so the switch is closed on the first tick and the swing latch is set). NoTarget.
    setup(fw) runs right after the reset, before the first tick (patch par.* there)."""
    h = Harness(plant=[plant, *extra]).reset()
    if setup is not None:
        setup(h.fw)
    h.nominal_inputs()
    h.gnss_rtk_fixed()
    h.tick(3)
    return h


def tool_pitch(fw, q_bm1, q_arm, q_inp):
    """Elevation of the tilt axis (tilt-mount x) above horizontal on a level house, radians."""
    off = fw["par.parKin.angOutpLinkToTiltMnt"] + fw["par.parKin.angTiltMntToTilt"]
    return -(q_bm1 + q_arm + kin.fourbar_output(fw, q_inp) + off)


def tilt_travel_for_leg(leg, pitch):
    """Joint travel that turns the gravity vector by `leg` about an axis `pitch` above horizontal."""
    s = math.sin(leg / 2) / math.cos(pitch)
    return math.inf if s > 1 else 2 * math.asin(s)


def swing_by_remote_lever(h, lever_pct, ticks, settle=100):
    """The operator's remote lever (u.rmtLvrDmd.swing < 0 -> propVlvCmd.swingLe, chart_2327 l.8-9;
    isRmtOperated outranks everything but RCV, chart_2258 l.26-27). A real firmware path, so the plant
    and the firmware's swing estimate move exactly as they would under an operator."""
    h.fw["u.rmtLvrDmd.swing"] = lever_pct
    h.tick(ticks)
    h.fw["u.rmtLvrDmd.swing"] = 0.0
    h.tick(settle)


def to_standby(h, dwell=90):
    h.set_target_panel(panel_id=7)
    h.tick(3)
    h.request_step("Standby")
    h.run_until(lambda h: h.curr_step() == "Standby", 0.5, "Standby")
    h.tick(dwell)
    return h


def chs_frame(fw, path):
    """A y.links.* position in the chassis frame (y.links.*.R is column-major)."""
    R = np.array(fw["y.links.chs.R"], dtype=float).reshape(3, 3, order="F")
    return R.T @ (np.array(fw[path], dtype=float) - np.array(fw["y.links.chs.p"], dtype=float))


def mount(src, prefix):
    return np.array([[src[f"{prefix}.a{r}{c}"] for c in (1, 2, 3)] for r in (1, 2, 3)], dtype=float)


# =====================================================================================================
class TestValveTables(unittest.TestCase):
    """valves.port_speed and friends, with numbers written out."""

    @classmethod
    def setUpClass(cls):
        cls.h = Harness().reset()
        cls.tables = vlv.read_tables(cls.h.fw)

    def test_inverse_reproduces_the_firmware_forward_map_on_every_port(self):
        # WHY: the plant is "the machine the firmware's table describes"; any speed the firmware asks
        # for through its own interp1 must come back out of the plant unchanged.
        # FW: chart_2463 l.93-112 interp1(X, Y, speed) after clamping to [X(1), X(end)] (l.72-91);
        # table source parLocalTest.reqSpdToActCmd (MdlApp.c:50334ff).
        for port, (X, Y) in self.tables.items():
            with self.subTest(port=port):
                self.assertEqual(len(X), 3)
                self.assertEqual((X[0], Y[0]), (0.0, 0.0))
                for s in np.linspace(X[1], X[2], 9):
                    cmd = vlv.table_speed_to_cmd(s, X, Y)
                    self.assertAlmostEqual(vlv.port_speed(cmd, X, Y), s, delta=1e-9 * max(1.0, X[2]))

    def test_hand_calculated_points(self):
        # swingLe: X = [0, 0.001, 0.5889427], Y = [0, 32, 60] (ECR88D_ShortArm.m, compiled)
        X, Y = self.tables["swingLe"]
        self.assertAlmostEqual(X[2], 0.5889427, places=6)
        self.assertEqual(list(Y), [0.0, 32.0, 60.0])
        self.assertEqual(vlv.port_speed(0.0, X, Y), 0.0)
        self.assertEqual(vlv.port_speed(31.99, X, Y), 0.0)                                  # below deadband
        self.assertAlmostEqual(vlv.port_speed(32.0, X, Y), 0.001, places=9)                 # the table's own point
        self.assertAlmostEqual(vlv.port_speed(46.0, X, Y), 0.001 + 14 * (X[2] - 0.001) / 28, places=9)
        self.assertAlmostEqual(vlv.port_speed(46.0, X, Y), 0.2949714, places=6)
        self.assertAlmostEqual(vlv.port_speed(60.0, X, Y), X[2], places=12)
        self.assertAlmostEqual(vlv.port_speed(100.0, X, Y), X[2], places=12)               # ASSUMPTION: clamped
        # bm1Up: stroke m/s, X = [0, 0.001, 0.41], Y = [0, 32, 80]; 56 % -> 0.001 + 24 * 0.409 / 48
        X, Y = self.tables["bm1Up"]
        self.assertAlmostEqual(vlv.port_speed(56.0, X, Y), 0.2055, places=6)

    def test_deadband_and_vmax_overrides(self):
        # WHY: calibration identifies the plant's deadband, so a test must be able to build a plant whose
        # valve differs from the stored table. Shape: (deadband, X[1]) -> (Y[2], vmax), clamped.
        X, Y = self.tables["rotPosi"]                        # Y = [0, 17, 80]
        self.assertEqual(list(Y), [0.0, 17.0, 80.0])
        ps = lambda c: vlv.port_speed(c, X, Y, deadband=25.0, vmax=0.3)
        self.assertEqual(ps(24.9), 0.0)
        self.assertAlmostEqual(ps(25.0), 0.001, places=9)
        self.assertAlmostEqual(ps(52.5), 0.001 + 27.5 * 0.299 / 55, places=9)
        self.assertAlmostEqual(ps(80.0), 0.3, places=12)
        self.assertAlmostEqual(ps(100.0), 0.3, places=12)
        self.assertAlmostEqual(vlv.port_speed(20.0, X, Y), X[1] + 3 * (X[2] - X[1]) / 63, places=9)  # stored: moves

    def test_axis_sense_follows_chart_2463_including_the_inverted_blade(self):
        # FW: chart_2463 l.28-47: the first port of each pair takes max(actuatorVelReq, 0); blade is the
        # exception (bladeDown = max(req, 0), l.46-47). The remote lever is the mirror image for swing /
        # boom / arm / link (chart_2327 l.8-17) -- irrelevant here, the plant only sees ports.
        t = self.tables
        for axis, (pos, neg) in vlv.AXIS_PORTS.items():
            with self.subTest(axis=axis):
                self.assertGreater(vlv.axis_actuator_speed({pos: 60.0}, t, axis), 0.0)
                self.assertLess(vlv.axis_actuator_speed({neg: 60.0}, t, axis), 0.0)
        self.assertEqual(vlv.AXIS_PORTS["blade"], ("bladeDown", "bladeUp"))
        self.assertEqual(vlv.AXIS_PORTS["swing"], ("swingLe", "swingRi"))
        # The case the firmware never produces (module docstring). EH axes, ASSUMPTION: net of both pilots.
        both = vlv.axis_actuator_speed({"armIn": 50.0, "armOut": 20.0}, t, "arm")
        self.assertAlmostEqual(both, vlv.port_speed(30.0, *t["armIn"]), places=12)
        # tilt / rotator, OEM glue AppCtrlIf.c:692-699: one flow command posi + nega, Posi sets the direction.
        glued = vlv.ecu_glue({"tiltPosi": 30.0, "tiltNega": 20.0, "rotNega": 12.0, "armIn": 5.0, "armOut": 7.0})
        self.assertEqual((glued["tiltPosi"], glued["tiltNega"]), (50.0, 0.0))
        self.assertEqual((glued["rotPosi"], glued["rotNega"]), (0.0, 12.0))
        self.assertEqual((glued["armIn"], glued["armOut"]), (5.0, 7.0))
        self.assertEqual(vlv.ecu_glue({"tiltNega": 9.0}), {"tiltNega": 9.0, "tiltPosi": 0.0, "rotPosi": 0.0, "rotNega": 0.0})

    def test_pilot_pressure_is_the_ecu_expected_valve_output_pressure(self):
        # WHY: u.ehPiPrs.swing* > 5 bar keeps the swing angle integrating after the command drops
        # (chart_1167). The curve is the ECU's own: 150 + 6.5 mA per %, then hystUp current->pressure.
        # FW: PrePostProc_If.c:41-48, 153-158, 1247-1268; LookUpTblFloat ApplCmnFcts.c:1089 (clamped).
        self.assertEqual(vlv.pilot_pressure_bar(0.0), 0.0)
        self.assertAlmostEqual(vlv.pilot_pressure_bar(20.0), 1.5 + (280.0 - 243.7) * 3.5 / 90.0, places=9)   # 2.9117
        self.assertAlmostEqual(vlv.pilot_pressure_bar(50.0), 5.0 + (475.0 - 333.7) * 9.5 / 182.1, places=9)  # 12.3715
        self.assertAlmostEqual(vlv.pilot_pressure_bar(100.0), 28.5 + 32.0 * 2.5 / 61.2, places=9)           # 29.807
        self.assertAlmostEqual(vlv.pilot_pressure_bar((333.7 - 150.0) / 6.5), 5.0, places=9)    # on at 28.26 %
        self.assertAlmostEqual(vlv.pilot_pressure_bar((243.7 + 1.5 * 90.0 / 3.5 - 150.0) / 6.5), 3.0, places=9)
        self.assertEqual(vlv.pilot_pressure_bar(150.0), vlv.pilot_pressure_bar(100.0))           # current clamp
        # every stored swing deadband is above the 5 bar point: a moving house always reads "operated"
        for p in ("swingLe", "swingRi"):
            self.assertGreater(vlv.pilot_pressure_bar(self.tables[p][1][1]), 5.0)

    def test_valve_lag_is_an_exact_first_order_step(self):
        s = {p: 0.0 for p in vlv.PORTS}
        for _ in range(10):
            s = vlv.lag_step(s, {"armIn": 50.0}, 0.01, 0.1)
        self.assertAlmostEqual(s["armIn"], 50.0 * (1.0 - math.exp(-1.0)), places=9)
        self.assertEqual(vlv.lag_step(s, {"armIn": 12.0}, 0.01, 0.0)["armIn"], 12.0)
        self.assertEqual(vlv.lag_alpha(0.01, 0.0), 1.0)


# =====================================================================================================
class TestCylinderGeometry(unittest.TestCase):
    """valves.stroke / stroke_jacobian = the firmware's CalStrkAndSpd (chart_1076 l.106-125)."""

    @classmethod
    def setUpClass(cls):
        cls.h = Harness().reset()
        cls.cyls = vlv.cylinders(cls.h.fw)

    def test_parameters_are_the_calstrkandspd_arguments(self):
        c = self.cyls
        self.assertEqual(c["boom"].top_mount, False)
        self.assertEqual(c["arm"].top_mount, True)
        self.assertEqual(c["input_link"].top_mount, True)
        self.assertEqual(c["input_link"].ang_sml, 0.0)          # single(0) literal, chart_1076 l.38-42, 67
        self.assertAlmostEqual(c["boom"].side1, 0.476, places=6)
        self.assertAlmostEqual(c["boom"].side2, 1.81, places=6)
        self.assertAlmostEqual(c["arm"].side1, 1.855, places=6)
        self.assertAlmostEqual(c["input_link"].side2, 0.42, places=6)

    def test_jacobian_against_hand_formula_finite_difference_and_spec_a43(self):
        # WHY: the plant turns stroke m/s into joint rad/s with this; a wrong sign or factor makes every
        # cylinder loop in the firmware see a different machine than the one its tables describe.
        # Spec A4.3 computed |dL/dq| and L independently (3 decimals); the formula is written out here too.
        c = self.cyls["boom"]
        q = -40 * DEG
        inc = -(q + c.ang_sml - c.ang_lrg)
        L = math.sqrt(c.side1 ** 2 + c.side2 ** 2 - 2 * c.side1 * c.side2 * math.cos(inc))
        self.assertAlmostEqual(vlv.stroke(c, q), L, places=12)
        self.assertAlmostEqual(vlv.stroke_jacobian(c, q), -c.side1 * c.side2 * math.sin(inc) / L, places=12)
        spec = [("boom", -31, 0.419, 1.986), ("boom", -69, 0.235, 2.208), ("arm", 31, 0.302, 1.395),
                ("arm", 155, 0.215, 2.327), ("input_link", 0, 0.081, 1.935)]
        for axis, qd, jac, length in spec:
            with self.subTest(axis=axis, q=qd):
                cyl, qr = self.cyls[axis], qd * DEG
                self.assertAlmostEqual(abs(vlv.stroke_jacobian(cyl, qr)), jac, delta=6e-4)
                self.assertAlmostEqual(vlv.stroke(cyl, qr), length, delta=6e-4)
                fd = (vlv.stroke(cyl, qr + 1e-6) - vlv.stroke(cyl, qr - 1e-6)) / 2e-6
                self.assertAlmostEqual(vlv.stroke_jacobian(cyl, qr), fd, delta=1e-6)

    def test_positive_stroke_speed_moves_each_joint_the_documented_way(self):
        # bm1Up -> boom q decreasing (raise), armIn -> arm q increasing, linkIn -> input link q increasing
        # below its dead centre and DEcreasing above it (why the plant's limit is +10 deg).
        c, v = self.cyls, 0.1
        self.assertLess(vlv.joint_rate_from_stroke_speed(c["boom"], -40 * DEG, v), 0.0)
        self.assertGreater(vlv.joint_rate_from_stroke_speed(c["arm"], 90 * DEG, v), 0.0)
        self.assertGreater(vlv.joint_rate_from_stroke_speed(c["input_link"], -60 * DEG, v), 0.0)
        self.assertLess(vlv.joint_rate_from_stroke_speed(c["input_link"], 20 * DEG, v), 0.0)
        self.assertAlmostEqual(vlv.joint_rate_from_stroke_speed(c["arm"], 90 * DEG, v),
                               v / vlv.stroke_jacobian(c["arm"], 90 * DEG), places=12)

    def test_dead_centres_are_the_spec_over_centre_limits(self):
        # spec A4.3 "Derived hard limits"; arm +5.6 and link +14.2 were also located from y.cyls in
        # test_imu_kinematics.test_cylinder_strokes_confirm_the_signs_and_the_dead_centres.
        want = {"boom": (-106.1, 73.9), "arm": (-174.4, 5.6), "input_link": (-165.8, 14.2)}
        for axis, (lo, hi) in want.items():
            got = [math.degrees(a) for a in vlv.dead_centres(self.cyls[axis])]
            self.assertAlmostEqual(got[0], lo, delta=0.05, msg=axis)
            self.assertAlmostEqual(got[1], hi, delta=0.05, msg=axis)
            for a in vlv.dead_centres(self.cyls[axis]):
                self.assertLess(abs(vlv.stroke_jacobian(self.cyls[axis], a)), 1e-9)

    def test_dead_centre_guard_keeps_the_rate_finite(self):
        # GUESS guard (MIN_JACOBIAN): at the exact dead centre the rate is v / MIN_JACOBIAN, not inf.
        c = self.cyls["input_link"]
        q = vlv.dead_centres(c)[1]
        r = vlv.joint_rate_from_stroke_speed(c, q, 0.1)
        self.assertTrue(math.isfinite(r))
        self.assertAlmostEqual(abs(r), 0.1 / vlv.MIN_JACOBIAN, places=9)

    def test_stroke_equals_the_firmware_cylinder_output(self):
        # WHY: the plant's geometry must be the firmware's, not a copy that drifts. y.cyls.*.strk is
        # CalStrkAndSpd of the firmware's own joint estimate (MdlApp.c:13453-13483); the joint-angle LPF
        # seeds itself from its first sample, so one tick after reset is exact.
        poses = [dict(boom=-35, arm=40, input_link=-120), dict(boom=-60, arm=150, input_link=0),
                 dict(boom=-45, arm=90, input_link=-60)]
        for pose in poses:
            with self.subTest(**pose):
                plant = KinematicPlant(q0=pose, degrees=True)
                h = Harness(plant=plant).reset().nominal_inputs()
                h.tick()
                s = plant.strokes()
                for axis, name in vlv.FW_CYLINDER.items():
                    self.assertAlmostEqual(h.fw[f"y.cyls.{name}.strk"], s[axis], delta=2e-5, msg=axis)


# =====================================================================================================
class TestPlantSensors(unittest.TestCase):
    """The plant moves on its own (plant.manual, the remote lever); the firmware must read it back."""

    def test_firmware_reads_the_plant_pose_on_a_tilted_machine(self):
        # WHY: every closed-loop result below assumes plant ground truth == firmware estimate at rest.
        # FW: joint angles chart_2143 l.29-33, 155-166; rotator MdlApp.c:42032; position/heading from the
        # antennas chart_2143 l.37-59, y.machHeading = pi/2 - yaw (MdlApp.c:13422).
        plant = KinematicPlant(q0=dict(boom=-55, arm=120, input_link=-30, tilt=12, rotator=33), degrees=True,
                               chs_origin=(3.0, -2.0, 0.5))
        plant.set_ground(roll=2 * DEG, pitch=-5 * DEG, yaw=30 * DEG)
        h = Harness(plant=plant).reset().nominal_inputs()
        h.gnss_rtk_fixed()
        h.tick(2)                                  # the inhibit word reads the swing latch one tick late
        fw = h.fw
        for name, want in plant.firmware_joints(fw).items():
            with self.subTest(joint=name):
                self.assertAlmostEqual(fw[f"y.jnts.{name}.q"], want, delta=0.002 * DEG)
        np.testing.assert_allclose(fw["y.links.chs.p"], plant.chs_origin, atol=1e-4)
        R = np.array(fw["y.links.chs.R"], dtype=float).reshape(3, 3, order="F")
        np.testing.assert_allclose(R, plant.R_chs, atol=2e-5)
        self.assertEqual(h.inhibit_names(), ["BIT_NO_TARGET"])
        self.assertTrue(fw.internal("swing_init"), "switch closed at swing 0 on the first tick")

    def test_firmware_rates_and_cylinder_speeds_equal_the_plant_while_axes_move(self):
        # WHY: the firmware's velocity loops close on gyro-difference rates and on CalStrkAndSpd stroke
        # speed. Driving the axes (plant.manual, outside the firmware) and reading y.jnts.qDot and
        # y.cyls.*.spd back closes the Jacobian round trip: valve stroke speed -> plant joint rate -> gyro
        # -> firmware joint rate -> firmware stroke speed == the valve's stroke speed.
        # FW: rates chart_2143 l.268-278 (3 Hz LPF l.293); cylinder speed MdlApp.c:13453-13483; the swing
        # angle integrates while u.ehPiPrs.swing* > 5 bar (chart_1167), which plant.manual produces.
        plant = KinematicPlant(q0=dict(boom=-45, arm=80, input_link=-70, tilt=-10, rotator=0), degrees=True)
        h = booted(plant)
        fw = h.fw

        # phase 1: boom, arm, input link, rotator
        plant.manual = {"bm1Up": 36.0, "armIn": 38.0, "linkIn": 34.0, "rotPosi": 40.0}
        h.tick(150)                                        # ~ 28 time constants of the 3 Hz LPF
        self.assertFalse(any(plant.at_limit.values()), plant.q)
        self.assertEqual(h.valves(), {}, "the firmware commands nothing: this is the plant alone")
        # Cylinder joints change rate as their Jacobian turns: 1 % covers the LPF phase lag.
        for axis, name in (("boom", "BmMntToBm1"), ("arm", "Bm2ToArm"), ("input_link", "ArmToInpLink")):
            with self.subTest(rate=axis):
                self.assertNotEqual(plant.qdot[axis], 0.0)
                self.assertAlmostEqual(fw[f"y.jnts.{name}.qDot"], plant.qdot[axis], delta=0.01 * abs(plant.qdot[axis]))
        for axis, name in vlv.FW_CYLINDER.items():
            with self.subTest(stroke_speed=axis):
                self.assertAlmostEqual(fw[f"y.cyls.{name}.spd"], plant.speeds[axis], delta=0.01 * abs(plant.speeds[axis]))
        # the rotator rate is a 10-sample regression on the absolute angle (chart_1179): exact at constant speed
        self.assertNotEqual(plant.qdot["rotator"], 0.0)
        self.assertAlmostEqual(fw["y.jnts.TiltToRot.qDot"], plant.qdot["rotator"], delta=1e-5)
        # velocity_targets() at the angles the plant evaluated its Jacobians at == what it integrated
        tgt = vlv.velocity_targets(fw, q=plant.q_used, cmds=plant.effective, hardware=plant.hardware)
        for axis in ("boom", "arm", "input_link", "tilt", "rotator", "swing"):
            self.assertAlmostEqual(tgt[axis], plant.targets[axis], delta=1e-12, msg=axis)

        # phase 2: swing and tilt, front end still
        plant.manual = {"swingLe": 40.0, "tiltPosi": 40.0}
        h.tick(150)
        self.assertEqual(plant.qdot["input_link"], 0.0)
        self.assertAlmostEqual(fw["y.jnts.ChsToUc.qDot"], -plant.qdot["swing"], delta=1e-5)
        self.assertAlmostEqual(fw["y.jnts.TiltMntToTilt.qDot"], plant.qdot["tilt"], delta=1e-5)

        # phase 3, a FIRMWARE property the plant exposes: the tilt rate subtracts the arm rate rotated
        # through Ry(-(q_outp + off)) with q_outp from the 3 Hz filtered angles (chart_2143 l.188, 211, 275)
        # while the gyros are current. Moving the output link while the house yaws (w_arm then has a
        # component the rotation mixes) leaks the filter lag into the tilt rate. Boom/arm motion cancels
        # in that product (phase 1 had none). Measured 0.0018 rad/s at this operating point.
        plant.manual = {"swingLe": 40.0, "tiltPosi": 40.0, "linkIn": 34.0}
        h.tick(60)
        leak = fw["y.jnts.TiltMntToTilt.qDot"] - plant.qdot["tilt"]
        self.assertGreater(abs(leak), 1e-3)
        self.assertLess(abs(leak), 3e-3)

        # stop and let the LPFs settle: every angle, including swing from the GNSS baseline, agrees
        plant.manual = {}
        h.tick(150)
        for name, want in plant.firmware_joints(fw).items():
            self.assertAlmostEqual(fw[f"y.jnts.{name}.q"], want, delta=0.01 * DEG, msg=name)
        self.assertGreater(plant.q["swing"], 5 * DEG, "precondition: the house really swung")

    def test_pilot_pressure_is_what_keeps_a_coasting_house_observable(self):
        # WHY: the firmware has no swing encoder. It advances its swing angle only on ticks after the swing
        # was "operated": |propVlvCmd.swing*| > 1e-6 OR u.ehPiPrs.swing* > 5 bar, released below 3 bar.
        # A valve with lag keeps the house moving for a moment after the command drops; the pilot
        # pressure is the only input that covers that moment. A plant without it leaves a permanent
        # swing-angle error, silently (no inhibit).
        # FW: chart_1167 l.15-55 (generated MdlApp.c:51862-51936, read through Delay16 MdlApp.c:11947);
        # chart_2143 l.85-107 (RefHold/SwingHold re-latch while invalid).
        results = {}
        for pilot in (True, False):
            plant = KinematicPlant(tau_valve=0.3, publish_pilot=pilot)
            h = booted(plant)
            h.fw["u.rmtLvrDmd.swing"] = -60.0
            h.tick(200)
            h.fw["u.rmtLvrDmd.swing"] = 0.0
            released_at = plant.q["swing"]
            h.tick(200)
            coast = plant.q["swing"] - released_at
            self.assertGreater(coast, 0.3 * DEG, "precondition: the house coasts after release")
            self.assertEqual(h.inhibit_names(), ["BIT_NO_TARGET"])
            results[pilot] = (-h.fw["y.jnts.ChsToUc.q"], plant.q["swing"], coast)
        fw_q, true_q, _ = results[True]
        self.assertAlmostEqual(fw_q, true_q, delta=0.01 * DEG)
        fw_q, true_q, coast = results[False]
        # the remote lever's filtered command still counts as 'operated' for a few ticks after release
        # (isSwingCmdOn), so part of the coast is seen even without pilot pressure -- most is not
        self.assertGreater(true_q - fw_q, 0.5 * coast, "without ehPiPrs the coast is lost")


# =====================================================================================================
class TestClosedLoopSigns(unittest.TestCase):
    """The firmware closes its loops through the plant. If one sign were wrong the loop would diverge."""

    SWING_OFFSET_TICKS = 300          # remote lever -30 %: ~13 deg of house CCW (swingLe)

    def positioning_run(self, plant, budget_ticks):
        """Boot, swing the house off square with the remote lever, Standby, start Positioning. Returns
        (h, first tick of each positioning step, contact-surface Z in chassis at Swing entry)."""
        h = booted(plant)
        swing_by_remote_lever(h, -30.0, self.SWING_OFFSET_TICKS)
        self.offset = plant.q["swing"]
        to_standby(h)
        h.pulse("u.jstAutoReq_StartPause")
        first, cs_z = {}, None
        for _ in range(budget_ticks):
            h.tick()
            step = h.positioning_step()
            if step not in first:
                first[step] = h.tick_count
                if step == "PositioningStep_Swing":
                    cs_z = chs_frame(h.fw, "y.links.contactSurface.p")[2]
        return h, first, cs_z

    def test_positioning_raise_swing_and_align_converge(self):
        # WHY: the first end-to-end actuation check. Positioning commands, in order: Raise (boom/arm/link in
        # TASK space until the tool is high enough to swing), Swing (swing, tilt, rotator in JOINT space to
        # 0), Align (travel; tilt and rotator still in joint space). Every one of those loops converges on
        # the plant, so the sign of swing, tilt and rotator, and the task-space front end, are right.
        # FW: SetCtrlMode chart_2123 l.20-36; targets chart_1011 l.100-135 (ChsToUc, tilt, rot = 0);
        # Raise -> Swing on isSwingAlignAllowed = |ChsToUc| <= 5 deg OR tool Z in chassis >= 0.8 m (Delay30,
        # MdlApp.c:40847, SysPar.m:265, 270); Swing -> Align on isTarActuatorReached.swing (MdlApp.c:40882,
        # PlacingCtrlTol.swing 1 deg, CntTarReachedConfirm 30, SysPar.m:437, 471). The swing re-latch when
        # the house crosses the switch (+-1 deg) is part of the run.
        plant = KinematicPlant(q0=dict(tilt=6.0, rotator=-10.0), degrees=True)
        h, first, cs_z = self.positioning_run(plant, 3000)
        fw = h.fw
        self.assertGreater(self.offset, 10 * DEG, "precondition: house off square by more than the 5 deg skip")
        self.assertIn("PositioningStep_Swing", first, h.describe())
        self.assertGreaterEqual(cs_z, 0.8, "Raise lifted the tool to the swing height")
        self.assertLess(cs_z, 0.9)
        self.assertIn("PositioningStep_Align", first, h.describe())
        self.assertLess(first["PositioningStep_Align"] - first["PositioningStep_Swing"], 400)
        self.assertLessEqual(abs(plant.q["swing"]), 1.0 * DEG)
        self.assertAlmostEqual(-fw["y.jnts.ChsToUc.q"], plant.q["swing"], delta=0.05 * DEG)
        self.assertLessEqual(abs(plant.q["tilt"]), 0.55 * DEG)
        self.assertLessEqual(abs(plant.q["rotator"]), 0.55 * DEG)
        # Only the travel valves may stay open: Align drives the undercarriage onto the target lane, and this
        # plant does not move the undercarriage (plant docstring, Travel), so Align can NEVER complete here.
        # The test deliberately ends at Align entry; Align itself is out of this plant's scope.
        self.assertEqual({p for p in h.valves() if not p.startswith("trvl")}, set(), h.valves())
        self.assertEqual(h.main_state(), "Positioning")
        self.assertEqual(h.inhibit_status(), 0, h.describe())

    def test_prepare_pick_reaches_the_joint_space_targets_of_boom_arm_and_link(self):
        # WHY: Raise is task space and redundant -- one backwards cylinder is compensated by the other two
        # (see the mutation test). PreparePick drives bm1/arm/link/rotate in JOINT space to the pose above
        # the panel stack, and only advances to ApproachPanel when all four are within PreparePoseCtrlTol
        # (bm1 2 deg, arm/link 1 deg, rot 0.5 deg) for 30 ticks. The rotator starts at its target here:
        # its sign is covered by Positioning, and its final approach is slow by design (the error->demand
        # table asks for less than X[1] = 0.001 rad/s near the target and the minimum-speed hold commands
        # exactly that, chart_2463 l.66-67 -- ~0.06 deg/s, so ~10 s per degree inside the last degree).
        # FW: chart_2123 l.47-64; chart_1011 l.145-170 (target panelTopWithMargin); PreparePick ->
        # ApproachPanel needs isTarActuatorReached.rotate && bm1 && arm && link (MdlApp.c:40540-40548,
        # SysPar.m:451-461). Cycle steps are reachable from Standby by a step jump (chart_2537).
        plant = KinematicPlant(q0=dict(boom=-50.0, arm=70.0, input_link=-90.0), degrees=True)
        h = booted(plant)
        to_standby(h)
        h.jump_to_step("Picking")
        self.assertEqual(h.picking_step(), "PickingStep_PreparePick", h.describe())
        start = dict(plant.q)
        h.run_until(lambda h: h.picking_step() == "PickingStep_ApproachPanel", 10.0, "ApproachPanel")
        fw = h.fw
        moved = {a: abs(plant.q[a] - start[a]) for a in ("boom", "arm", "input_link")}
        for a in ("boom", "arm", "input_link"):
            self.assertGreater(moved[a], 5 * DEG, f"precondition: {a} had to move ({moved})")
        # firmware target strokes on the debug outports (MdlApp.c:51976, 52063, 52073): the plant is there
        tol = {"boom": 2.0 * DEG, "arm": 1.0 * DEG, "input_link": 1.0 * DEG}
        for axis, p in (("boom", "P11"), ("arm", "P13"), ("input_link", "P15")):
            with self.subTest(axis=axis):
                target_strk = fw[f"y.dbg_F64_{p}"]
                err = abs(plant.strokes()[axis] - target_strk) / abs(vlv.stroke_jacobian(plant.cyls[axis], plant.q[axis]))
                self.assertLessEqual(err, tol[axis] * 1.2, f"{axis} {math.degrees(err):.3f} deg from target")
        self.assertLessEqual(abs(plant.q["rotator"]), 0.55 * DEG)
        self.assertEqual(h.main_state(), "Picking")

    def test_min_speed_hold_from_rest_needs_the_effective_command_rule(self):
        # WHY: valves.effective_commands is an ASSUMPTION; this is the scenario that needs it. The rotator starts
        # 0.8 deg off its PreparePick target: outside PreparePoseCtrlTol.rot (0.5 deg), but so close that the
        # error->demand table asks for less than X[1] = 0.001 rad/s, so the minimum-speed hold commands EXACTLY
        # the stored deadband, 17.0 %, from rest (chart_2463 l.66-67 then interp1 l.110). A first-order lag
        # started at 0 only tends to 17.0 and never reaches it: without the rule the rotator stays at 0.8 deg
        # and PreparePick never advances (measured: no ApproachPanel in 30 s). With it the rotator creeps at
        # X[1] and PreparePick completes. Mutant M07 (rule removed) must fail here.
        # FW: PreparePick -> ApproachPanel needs isTarActuatorReached.rotate (MdlApp.c:40540-40548).
        dbs = vlv.deadbands(vlv.read_tables(Harness().reset().fw))
        eff = vlv.effective_commands({"rotNega": dbs["rotNega"]}, {p: 0.0 for p in vlv.PORTS}, dbs)
        self.assertEqual(eff["rotNega"], dbs["rotNega"])                   # the rule, as a number
        self.assertEqual(vlv.effective_commands({"rotNega": dbs["rotNega"] - 0.1}, {p: 0.0 for p in vlv.PORTS},
                                                dbs)["rotNega"], 0.0)       # below the deadband: closed

        plant = KinematicPlant(q0=dict(boom=-50.0, arm=70.0, input_link=-90.0, rotator=0.8), degrees=True)
        seen = set()
        probe = lambda h: seen.add(h.fw["y.propVlvCmd.rotNega"]) if h.fw["y.propVlvCmd.rotNega"] else None
        h = booted(plant, extra=[probe])
        to_standby(h)
        self.assertEqual(plant.valve["rotNega"], 0.0, "precondition: rotator valve at rest")
        h.jump_to_step("Picking")
        h.run_until(lambda h: h.picking_step() == "PickingStep_ApproachPanel", 12.0, "ApproachPanel")
        self.assertEqual(seen, {float(np.float32(dbs["rotNega"]))}, "the firmware only ever asked for the deadband")
        self.assertLessEqual(abs(plant.q["rotator"]), 0.5 * DEG)
        self.assertGreater(0.8 * DEG - plant.q["rotator"], 0.3 * DEG)

    def test_a_backwards_axis_breaks_its_loop(self):
        # WHY: the two convergence tests above could pass with a plant that ignores the command's sign on
        # an axis nobody needed to move. Here each axis is plumbed backwards in turn (plant option
        # axis_sign, kept in the plant so this regression is permanent) inside the SAME scenario and time
        # budget the correct plant converges in, and the loop must fail: the step never advances, or the
        # axis runs to its stop / away from its target.
        positioning = {"swing": "PositioningStep_Align", "tilt": None, "rotator": None}
        for axis, blocked_step in positioning.items():
            with self.subTest(axis=axis, scenario="Positioning"):
                plant = KinematicPlant(q0=dict(tilt=6.0, rotator=-10.0), degrees=True, axis_sign={axis: -1})
                h, first, _ = self.positioning_run(plant, 1000)
                if blocked_step:
                    self.assertNotIn(blocked_step, first, h.describe())
                    self.assertGreater(abs(plant.q["swing"]), 45 * DEG, "house runs away")
                elif axis == "tilt":
                    self.assertIn("PositioningStep_Align", first, "swing still converges")
                    self.assertGreater(abs(plant.q["tilt"]), 20 * DEG, "tilt runs away from 0")
                else:
                    self.assertIn("PositioningStep_Align", first, "swing still converges")
                    self.assertGreater(abs(plant.q["rotator"]), 30 * DEG, "rotator runs away from 0")
        for axis in ("boom", "arm", "input_link"):
            with self.subTest(axis=axis, scenario="PreparePick"):
                plant = KinematicPlant(q0=dict(boom=-50.0, arm=70.0, input_link=-90.0), degrees=True,
                                       axis_sign={axis: -1})
                h = booted(plant)
                to_standby(h)
                h.jump_to_step("Picking")
                h.tick(600)                        # the correct plant reaches ApproachPanel in ~200 ticks
                self.assertEqual(h.picking_step(), "PickingStep_PreparePick", h.describe())
                self.assertTrue(plant.at_limit[axis], f"{axis} at its stop: {math.degrees(plant.q[axis]):.1f} deg")




# =====================================================================================================
class TestHardwareIsNotTheParameterSet(unittest.TestCase):
    """The plant is a machine; par.* is what the firmware BELIEVES about it. They must be able to disagree."""

    def test_the_compiled_snapshot_is_what_reset_restores(self):
        h = Harness().reset()
        compiled, live = Hardware.compiled(h.fw), Hardware.live(h.fw)
        self.assertEqual(len(h.fw.paths("par.")), len(compiled.paths()))
        for p in h.fw.paths("par."):
            self.assertEqual(compiled[p], live[p], p)
        h.fw["par.jntAngRotZeroOffs"] = 0.25
        self.assertEqual(Hardware.compiled(h.fw)["par.jntAngRotZeroOffs"], 0.0, "compiled ignores a live patch")
        self.assertAlmostEqual(Hardware.live(h.fw)["par.jntAngRotZeroOffs"], 0.25, places=6)
        with self.assertRaises(KeyError):
            compiled.replace({"par.noSuchThing": 1.0})
        with self.assertRaises(ValueError):
            compiled.replace({"imuArm": np.eye(2)})

    def test_patched_mount_offset_table_and_antenna_are_seen_by_the_firmware_not_adopted_by_the_plant(self):
        # WHY (verifier must-fix 1): the plant used to rebuild its "hardware" from par.* every reset and every
        # publish, so a wrong stored mount / zero offset / table could never be seen and loading calibration
        # results silently changed the plant. Here the firmware's parameters are patched AFTER the reset and
        # BEFORE the first tick (the moment the old plant re-read them), and a unit whose rotator encoder zero
        # is 0.3 rad is simulated:
        #   imuArm      stored mount turned 10 deg about the board's y  -> firmware arm angle off by ~10 deg
        #   rot zero    stored 0.3 rad = the unit's hardware offset     -> firmware reads the true rotator
        #               (raw = rot - hardware offset; MdlApp.c:42032 adds the stored one back)
        #   armIn_Y[1]  stored deadband 45 %                            -> plant valve still opens at 32.5 %
        #   distAntMainToChs z +71 mm (the open ECR88 question)        -> firmware chassis origin 71 mm off
        OFF = 0.3
        plant = KinematicPlant(q0=dict(arm=100.0, rotator=20.0), degrees=True, hardware={"par.jntAngRotZeroOffs": OFF})
        true_arm_mount = None

        def patch(fw):
            nonlocal true_arm_mount
            true_arm_mount = kin.mounts_from_fw(fw)["imuArm"]
            kin.write_mounts(fw, {"imuArm": true_arm_mount @ kin.Ry(10 * DEG)})
            fw["par.jntAngRotZeroOffs"] = OFF
            Y = fw["par.reqSpdToActCmd.armIn_Y"]
            fw["par.reqSpdToActCmd.armIn_Y"] = [Y[0], 45.0, Y[2]]
            d = fw["par.parKin.distAntMainToChs"]
            fw["par.parKin.distAntMainToChs"] = [d[0], d[1], d[2] + 0.071]

        h = booted(plant, setup=patch)
        fw = h.fw
        np.testing.assert_array_equal(plant.hardware.mounts()["imuArm"], true_arm_mount)
        self.assertAlmostEqual(fw["y.jnts.TiltToRot.q"], plant.q["rotator"], delta=1e-5)       # offsets agree
        self.assertAlmostEqual(fw["u.jntAngRaw_Rot"], plant.q["rotator"] - OFF, delta=1e-6)
        arm_err = fw["y.jnts.Bm2ToArm.q"] - plant.q["arm"]
        self.assertGreater(abs(arm_err), 9.0 * DEG, "the firmware misreads the arm through its wrong mount")
        self.assertLess(abs(arm_err), 10.5 * DEG)
        self.assertAlmostEqual(fw["y.jnts.BmMntToBm1.q"], plant.q["boom"], delta=0.002 * DEG)  # others untouched
        self.assertAlmostEqual(fw["y.links.chs.p"][2] - plant.chs_origin[2], 0.071, delta=2e-4)
        # the firmware's stored deadband is 45 %; the plant's valve is the compiled one and opens at 40 %
        self.assertEqual(plant.tables["armIn"][1][1], 32.5)
        plant.manual = {"armIn": 40.0}
        q = plant.q["arm"]
        h.tick(60)
        self.assertGreater(plant.q["arm"] - q, 1.0 * DEG)
        # a harness reset (par restored) and a second patch change nothing in the plant either
        plant.manual = {}
        hw = plant.hardware
        h = booted(plant, setup=lambda fw: fw.__setitem__("par.jntAngRotZeroOffs", -1.0))
        self.assertIs(plant.hardware, hw)
        self.assertAlmostEqual(h.fw["u.jntAngRaw_Rot"], plant.q["rotator"] - OFF, delta=1e-6)
        self.assertAlmostEqual(h.fw["y.jnts.TiltToRot.q"], plant.q["rotator"] - OFF - 1.0, delta=1e-5)

    def test_stop_contact_is_recorded_and_strict_limits_raise(self):
        # The joint limits are partly GUESS/ESTIMATE (plant docstring); contact with them must never be silent.
        with self.assertRaises(ValueError):
            KinematicPlant(q0=dict(tilt=50.0), degrees=True)
        plant = KinematicPlant(q0=dict(tilt=44.0), degrees=True)
        h = booted(plant)
        self.assertEqual(plant.limit_hits, [])
        plant.manual = {"tiltPosi": 60.0}
        h.tick(100)
        self.assertTrue(plant.at_limit["tilt"])
        self.assertEqual(plant.qdot["tilt"], 0.0)
        self.assertAlmostEqual(plant.q["tilt"], 45.0 * DEG, places=12)
        self.assertEqual({k for _, k, _ in plant.limit_hits}, {"tilt"})
        plant.strict_limits = True
        with self.assertRaises(JointStopError) as cm:
            h.tick()
        self.assertIn("tilt at 45.00 deg", str(cm.exception))


# =====================================================================================================
class TestCalibrationIdentifiesThePlant(unittest.TestCase):
    """Spec A7: 'run 20-26 and compare the identified table against the sim' -- the calibration of the
    simulation itself. The plant's valve is deliberately NOT the stored table, so a pass cannot be the
    stored numbers echoed back.

    WHAT "IDENTIFIED" MEANS HERE (chart_2338 GenCorrTblSetReqSpdToActCmd l.68-116). The table a calibration
    writes is X = [0, 0.01, peak], Y = [0, onset command - 0.5, PropVlvRefCmd]. That is NOT the plant's line
    (deadband, 0.001) -> (Y[2], vmax): the knee is at 0.01 and Y[2] is the fixed reference command of the
    _ToPnt leg, not a saturation point. So these tests check two numbers per port -- the onset command and the
    speed AT PropVlvRefCmd -- and neither validates the shape between them (the GUESSed linear knee). The
    onset lands within a few 0.2 % steps of the plant's deadband because the plant is quasi-static: its
    accelerometer is pure gravity (no centripetal or tangential terms, plant docstring) and its valve is the
    GUESSed line, so a real machine's identified onset will scatter more.

    Every run is STRICT about joint stops (plant.strict_limits): the firmware ends a leg on angle OR after
    CntCalib_timeout = 10 s without any alarm (MdlApp.c:36320), and a leg that ran into a stop would still
    'succeed'. Start poses are therefore chosen with the post-leg coast in mind: _ToPnt commands go through a
    1 s cubic ramp-down after the leg threshold (SmoothPropVlvCmd, chart_3055 l.194-243, CntCalib_ramp 100),
    so every axis travels about 0.5 s x its reference speed PAST the leg angle."""

    @staticmethod
    def calibrate(plant, step, extra=(), setup=None):
        """Run one calibration to completion with strict joint stops. Returns (h, emu, dwell) where dwell is
        {calibStep name: ticks spent in it} (a leg that dwells > CNT_CALIB_TIMEOUT ended on the timeout)."""
        plant.strict_limits = True
        emu = SaveHandshake()
        dwell, cur = {}, [None, 0]

        def clock(h):
            s = h.calib_step()
            if s != cur[0]:
                if cur[0] is not None:
                    dwell[cur[0]] = h.tick_count - cur[1]
                cur[0], cur[1] = s, h.tick_count

        h = booted(plant, extra=list(extra) + [clock, emu], setup=setup)
        h.jump_to_step(step)
        h.run_until(lambda h: h.curr_step() == "NoTarget", 400.0, f"{step} done")
        return h, emu, dwell

    def assertIdentified(self, snap, port, deadband, speed_at_ref, dwell=None):
        X, Y = snap[f"y.tblReqSpdToActCmd.{port}_X"], snap[f"y.tblReqSpdToActCmd.{port}_Y"]
        # the calibration's own table shape (class docstring)
        self.assertEqual((X[0], X[1], Y[0]), (0.0, IDENTIFIED_X1, 0.0), port)
        self.assertEqual(Y[2], PROP_VLV_REF_CMD[port], port)
        # _Min: staircase 20 % + 0.2 % per 2 s until a 0.5 deg onset, stores cmd - 0.5 (chart_3055 l.53-99,
        # chart_2316, chart_2338 l.49-66). The onset needs 0.5 deg of travel at a speed that starts at
        # X[1] = 0.001, so it lands a few 0.2 % steps above the deadband (observed 2-3 steps: identified
        # = deadband - 0.1 or + 0.1). Allowed: 0..5 steps -- still far from every stored deadband (17-33 %).
        self.assertGreaterEqual(Y[1], deadband - 0.5 - 1e-4, port)
        self.assertLessEqual(Y[1], deadband - 0.5 + 1.01, port)
        # _ToPnt: peak joint (or stroke) speed at PropVlvRefCmd (chart_2383 l.75ff). Cylinder axes get 1 %:
        # the firmware's stroke speed is J(filtered q) * filtered qDot, and a constant stroke speed is a
        # changing joint rate (observed 0.01 % for the arm from 70 deg, 0.55 % from 130 deg).
        rel = 0.01 if port.startswith(("bm1", "arm", "link")) else 0.005
        self.assertAlmostEqual(X[2], speed_at_ref, delta=rel * speed_at_ref, msg=port)

    def assertLegsEndedOnAngle(self, dwell, legs):
        for leg in legs:
            self.assertIn(leg, dwell)
            self.assertLessEqual(dwell[leg], CNT_CALIB_TIMEOUT, f"{leg} ended on the 10 s timeout, not on angle")

    def test_calib_rot_identifies_the_valve_and_its_zero_offset_never_reaches_the_save(self):
        # FW: CalibRot 11 states, angCalib.rot = wrap(u.jntAngRaw_Rot - angRefRot) (MdlApp.c:43544), legs to
        # |angCalib.rot| > 60 / 120 deg (SysPar.m:126-127), PropVlvRefCmd.rot = 100 % (SysPar.m:153-154).
        # ZERO OFFSET, verifier must-fix 3. The unit's encoder zero is 0.3 rad and the firmware's stored offset
        # agrees, so the firmware reads the true rotator (25 deg) before the run; the calibration starts there.
        # CalcAngDistCalib averages the RAW angle over RotPosiPntRef_log (chart_2291 l.80-81) and outputs
        # y.jntAngRotZeroOffs = -angRefRot only while calibStep_prev == RotPosiPntRef_log (l.101-105); on every
        # other tick it outputs the STORED offset. AppCtrlIf.c:802-882 copies that outport into INTP on every
        # calibrating tick, so INTP holds -raw_ref for 1 s and is overwritten with the stored value long before
        # Rot_save. FINDING: the zero offset CalibRot measures is never what gets saved.
        OFF = 0.3
        plant = KinematicPlant(q0=dict(rotator=25.0), degrees=True, hardware={"par.jntAngRotZeroOffs": OFF},
                               deadband={"rotPosi": 25.0, "rotNega": 22.0}, vmax={"rotPosi": 0.30, "rotNega": 0.24})
        zero = {}
        probe = lambda h: zero.setdefault(h.calib_step(), []).append(h.fw["y.jntAngRotZeroOffs"])
        h, emu, dwell = self.calibrate(plant, "CalibRot", extra=[probe],
                                       setup=lambda fw: fw.__setitem__("par.jntAngRotZeroOffs", OFF))
        stored = float(np.float32(OFF))
        raw_ref = 25.0 * DEG - OFF
        self.assertAlmostEqual(zero["CalibStandby"][-1], stored, places=7)          # before the jump
        self.assertAlmostEqual(zero["RotPosiPntRef_log"][-1], -raw_ref, delta=2e-6)
        self.assertEqual(set(zero["RotPosiMin"][1:]), {stored})
        self.assertEqual(set(zero["Rot_save"]), {stored})
        self.assertEqual(len(emu.saved), 1)
        snap = emu.saved[0][1]
        self.assertEqual(snap["y.jntAngRotZeroOffs"], stored)
        self.assertIdentified(snap, "rotPosi", 25.0, 0.30)     # 100 % >= Y[2] 80: clamped at vmax (ASSUMPTION)
        self.assertIdentified(snap, "rotNega", 22.0, 0.24)
        self.assertLegsEndedOnAngle(dwell, ("RotPosi", "RotNega"))

    def test_calib_rot_cannot_identify_a_deadband_below_the_staircase_start(self):
        # WHY/FINDING: the stored rotator deadband is 17 % (par.reqSpdToActCmd.rotPosi_Y), below
        # PropVlvCmdInitOffs = 20 % (SysPar.m:159-178). On a machine that really has it, motion starts on
        # the staircase's first step and the calibration overwrites 17 with 19.5 -- 2.5 % of extra
        # command on every small rotator move afterwards. Not a plant artefact: it follows from the
        # staircase start.
        plant = KinematicPlant()
        h, emu, _ = self.calibrate(plant, "CalibRot")
        self.assertEqual(len(emu.saved), 1)
        snap = emu.saved[0][1]
        stored = h.fw["par.reqSpdToActCmd.rotPosi_Y"]
        self.assertEqual(stored[1], 17.0)
        self.assertAlmostEqual(snap["y.tblReqSpdToActCmd.rotPosi_Y"][1], 19.5, places=4)
        self.assertAlmostEqual(snap["y.tblReqSpdToActCmd.rotNega_Y"][1], 19.5, places=4)

    ARM_PLANT = dict(q0=dict(boom=-31.0, arm=121.0), degrees=True,
                     deadband={"armIn": 24.0, "armOut": 23.0}, vmax={"armIn": 0.12, "armOut": 0.10})

    def test_calib_arm_identifies_stroke_speed_and_the_compiled_mount_with_the_arm_hanging_vertically(self):
        # WHY: the arm table's X unit is cylinder stroke m/s. The plant converts the valve's stroke speed to
        # a joint rate with valves.joint_rate_from_stroke_speed; the firmware measures the joint rate from
        # gyros and converts back with its own CalStrkAndSpd. Identified peak == plant stroke speed closes
        # that loop to 1 %. armOut's reference command 70 % is below its Y[2] = 80, so its identified speed is
        # the plant's line at 70 %, not vmax.
        # The same run rebuilds the arm IMU mount from gravity. It equals the COMPILED par.imuArm exactly when
        # the reference pose has the arm chord hanging vertically (boom + arm = 90 deg) AND the accelerometer
        # reads -1 g up; see test_accelerometer_sign_is_minus_one_g for the other sign.
        # POSE: boom -31 (its stop, not moved here), arm 121: the 20 deg leg ends at 141 and the post-leg coast
        # stays below the 155 deg stop at vmax 0.12 m/s. From arm 130 the coast reaches 155; with the STORED arm
        # speed (0.42 m/s) it reaches 155 from every arm-vertical start (arm >= 121) -- measured.
        # FW: CalibArm legs AngArmPnt1/2 = 20/40 deg (SysPar.m:120-121); PropVlvRefCmd.arm = 70 (SysPar.m:147);
        # mount rebuild chart_2291 l.275-301; NVM copy AppCtrlIf.c:801-1018.
        plant = KinematicPlant(**self.ARM_PLANT)
        h, emu, dwell = self.calibrate(plant, "CalibArm")
        self.assertEqual(len(emu.saved), 1)
        snap = emu.saved[0][1]
        self.assertIdentified(snap, "armIn", 24.0, 0.12)
        X, Y = plant.tables["armOut"]
        self.assertEqual(Y[2], 80.0)
        self.assertIdentified(snap, "armOut", 23.0, vlv.port_speed(70.0, X, Y, deadband=23.0, vmax=0.10))
        self.assertLegsEndedOnAngle(dwell, ("ArmInToPnt1", "ArmOutToPnt2"))
        self.assertLess(np.abs(mount(snap, "y.imuMntOri_arm") - mount(h.fw, "par.imuArm")).max(), 1e-4)

    def test_calib_arm_recovers_the_units_mount_from_a_wrong_stored_one_but_not_its_speed_table(self):
        # WHY (verifier: the calibration tests only showed self-consistency, compiled mount in = compiled mount
        # out): here the STORED par.imuArm is turned 10 deg away from the unit's board, so the firmware misreads
        # the arm by ~10 deg. The mount rebuild uses raw accelerometer vectors only (chart_2291 l.275-301), so it
        # recovers the UNIT's mount (plant.hardware), not the stored one, and loaded into par the firmware reads
        # the true arm.
        # FINDING: the same run identifies a WRONG arm speed table. The _ToPnt peak is the firmware's cylinder
        # stroke speed, CalStrkAndSpd(q_fw, qDot) = J(q_fw) * qDot (chart_1076 l.106-125), evaluated at the
        # MISREAD angle; J falls with q here, so the identified armIn speed is J(q + ~10 deg)/J(q) of the true
        # one -- between 0.79 and 0.87 over the leg (observed 0.83). The mount is fixed on the next boot, the
        # table stays wrong until CalibArm is run again.
        plant = KinematicPlant(**self.ARM_PLANT)

        def wrong_mount(fw):
            kin.write_mounts(fw, {"imuArm": kin.mounts_from_fw(fw)["imuArm"] @ kin.Ry(10 * DEG)})

        h, emu, dwell = self.calibrate(plant, "CalibArm", setup=wrong_mount)
        fw = h.fw
        self.assertEqual(len(emu.saved), 1)
        snap = emu.saved[0][1]
        self.assertLegsEndedOnAngle(dwell, ("ArmInToPnt1", "ArmOutToPnt2"))
        identified = mount(snap, "y.imuMntOri_arm")
        self.assertLess(np.abs(identified - plant.hardware.mounts()["imuArm"]).max(), 1e-4)
        self.assertGreater(np.abs(identified - mount(fw, "par.imuArm")).max(), 0.15, "not the stored mount echoed")
        err = fw["y.jnts.Bm2ToArm.q"] - plant.q["arm"]
        self.assertTrue(9.0 * DEG < err < 10.5 * DEG, f"precondition: the arm reads {math.degrees(err):.2f} deg high")
        cyl = plant.cyls["arm"]
        ratio = lambda q, d: vlv.stroke_jacobian(cyl, (q + d) * DEG) / vlv.stroke_jacobian(cyl, q * DEG)
        got = snap["y.tblReqSpdToActCmd.armIn_X"][2] / 0.12
        self.assertGreater(got, ratio(141.0, 10.5))                   # leg 121 -> 141 deg, misread 9..10.5 deg
        self.assertLess(got, ratio(121.0, 9.0))
        kin.write_mounts(fw, {"imuArm": identified})                  # what the NVM reload would do
        h.tick(100)
        self.assertAlmostEqual(fw["y.jnts.Bm2ToArm.q"], plant.q["arm"], delta=0.01 * DEG)

    def test_calib_link_identifies_the_valve_and_reproduces_the_mount_with_the_input_link_horizontal(self):
        # Same as the arm, input-link board (bktImu, par.imuLink): the compiled mount comes back when the
        # reference pose has the input link chord horizontal (boom + arm + input link = 0) with -1 g.
        # POSE: input link -70 (arm 110): the 40 deg leg ends at -30 and the coast accelerates toward the
        # +14.2 deg dead centre (dL/dq -> 0, constant stroke speed); from -50 it reaches the +10 GUESS stop even
        # at vmax 0.18 m/s, and with the stored 0.59 m/s from -70 too (measured).
        # FW: CalibLink legs AngLinkPnt1/2 = 40/80 deg (SysPar.m:122-123).
        plant = KinematicPlant(q0=dict(boom=-40.0, arm=110.0, input_link=-70.0), degrees=True,
                               deadband={"linkIn": 24.0, "linkOut": 23.0}, vmax={"linkIn": 0.18, "linkOut": 0.18})
        h, emu, dwell = self.calibrate(plant, "CalibLink")
        self.assertEqual(len(emu.saved), 1)
        snap = emu.saved[0][1]
        for port, db in (("linkIn", 24.0), ("linkOut", 23.0)):
            X, Y = plant.tables[port]
            self.assertIdentified(snap, port, db, vlv.port_speed(70.0, X, Y, deadband=db, vmax=0.18))
        self.assertLegsEndedOnAngle(dwell, ("LinkInToPnt1", "LinkOutToPnt2"))
        self.assertLess(np.abs(mount(snap, "y.imuMntOri_link") - mount(h.fw, "par.imuLink")).max(), 1e-4)

    def test_calib_tilt_identifies_the_valve_and_reproduces_the_compiled_mount(self):
        # WHY (verifier: CalibTilt -1 g was only a docstring): at the default pose the tilt axis is 21.9 deg
        # below horizontal, the tool hangs forward, and the tilt board's rebuilt mount (vx = rotation axis from
        # two gravity chords, vy = vx x accRef, chart_2291 l.331-357) equals the compiled par.imuTilt with -1 g
        # (with +1 g it comes out as M @ diag(1,-1,-1), max error 2.0 -- measured).
        # GEOMETRY vs the +-45 deg ESTIMATE stop: legs are 35 and 70 deg of GRAVITY angle (CalcAccVecAngle,
        # SysPar.m:124-125), which about an axis 21.9 deg off horizontal is 37.8 and 76.4 deg of joint travel.
        # Plus the post-leg coast: at 0.18 rad/s at 70 % (vmax 0.22 here) the tilt peaks at +42.7 / -38.6 deg;
        # at 0.41 rad/s (vmax 0.5) it reaches the stop. At the STORED tilt speed (0.135 rad/s at 70 %) the 70 deg leg
        # cannot finish inside the 10 s timeout at this pose (measured: TiltNegaToPnt2 ended on the timeout, no
        # alarm) and needs 9.8 s of it with the tilt axis horizontal.
        plant = KinematicPlant(deadband={"tiltPosi": 24.0, "tiltNega": 23.0}, vmax={"tiltPosi": 0.22, "tiltNega": 0.22})
        h, emu, dwell = self.calibrate(plant, "CalibTilt")
        fw = h.fw
        pitch = tool_pitch(fw, plant.q["boom"], plant.q["arm"], plant.q["input_link"])
        self.assertAlmostEqual(math.degrees(pitch), -21.9, delta=0.1)
        self.assertAlmostEqual(math.degrees(tilt_travel_for_leg(70 * DEG, pitch)), 76.4, delta=0.1)
        self.assertEqual(len(emu.saved), 1)
        snap = emu.saved[0][1]
        for port, db in (("tiltPosi", 24.0), ("tiltNega", 23.0)):
            X, Y = plant.tables[port]
            self.assertIdentified(snap, port, db, vlv.port_speed(70.0, X, Y, deadband=db, vmax=0.22))
        self.assertLegsEndedOnAngle(dwell, ("TiltPosiToPnt1", "TiltNegaToPnt2"))
        compiled = mount(fw, "par.imuTilt")
        self.assertGreater(np.abs(compiled - np.eye(3)).max(), 0.5, "precondition: a non-trivial mount")
        self.assertLess(np.abs(mount(snap, "y.imuMntOri_tilt") - compiled).max(), 1e-4)

    def test_a_tilt_leg_that_needs_more_than_the_stop_hits_it_and_only_the_plant_notices(self):
        # WHY (verifier must-fix 5): with the tool pitched 47 deg the 35 deg leg needs 52 deg of joint travel,
        # more than the +-45 deg ESTIMATE. The firmware would press on, time the leg out after 10 s and save
        # without any alarm (MdlApp.c:36320: angle OR timeout). The strict plant stops the scenario at the
        # first tick of stop contact, inside TiltPosiToPnt1, with the firmware still calibrating.
        plant = KinematicPlant(q0=dict(boom=-40.0, arm=115.0, input_link=-60.0), degrees=True, strict_limits=True)
        h = booted(plant, extra=[SaveHandshake()])
        pitch = tool_pitch(h.fw, plant.q["boom"], plant.q["arm"], plant.q["input_link"])
        self.assertGreater(math.degrees(tilt_travel_for_leg(35 * DEG, pitch)), 45.0)
        h.jump_to_step("CalibTilt")
        with self.assertRaises(JointStopError) as cm:
            h.run_until(lambda h: h.curr_step() == "NoTarget", 400.0, "CalibTilt done")
        self.assertIn("calibStep=TiltPosiToPnt1", str(cm.exception))
        self.assertTrue(h.fw["y.isCalibrating"])
        self.assertAlmostEqual(plant.q["tilt"], 45.0 * DEG, places=9)

    def test_calib_chs_jacked_up_nose_up_identifies_swing_valves_and_the_compiled_mount(self):
        # WHY: step 20 is the one calibration that exercises swing (~180 deg left, back ~100 deg right), the
        # GNSS swing angle, the pilot-pressure validity and the accelerometer publisher together. With the
        # undercarriage pitched 6 deg nose UP (URDF pitch -6 deg; the deck's dozer jack-up) the house's
        # gravity vector turns with swing, CalcImuMntOri rebuilds the chassis mount from three logged vectors,
        # and it equals the COMPILED par.imuChs (observed 3e-6) with -1 g. The chassis alone cannot fix the
        # sign: nose DOWN with +1 g gives the same mount (measured), nose down with -1 g turns it 180 deg
        # about Z (asserted below), and a ROLL jack-up gives neither (90 deg yaw error, measured) -- so the
        # procedure's "pitch > 5 deg" is load-bearing. On a machine that does not move the mount stays
        # untouched or is rebuilt from noise (test_calibration_entry). NOT represented: the centripetal
        # acceleration a real chassis IMU sees while swinging (plant docstring), which moves angCalib.chs.
        # FW: CalibChs chart_1210 (ChsLeMoveToPnt1 [angCalib.chs > 170 deg], ChsRiMoveToPnt2 [< 80 deg],
        # SysPar.m:114-115); mount rebuild chart_2291 l.191-217; tables chart_2338 l.78-81.
        for pitch_deg in (-6.0, 6.0):
            with self.subTest(pitch_deg=pitch_deg):
                plant = KinematicPlant(deadband={"swingLe": 24.0, "swingRi": 23.0},
                                       vmax={"swingLe": 0.50, "swingRi": 0.45})
                plant.set_ground(pitch=pitch_deg * DEG)
                h, emu, dwell = self.calibrate(plant, "CalibChs")   # one firmware per process: check it now
                self.assertEqual(len(emu.saved), 1)
                snap = emu.saved[0][1]
                compiled = mount(h.fw, "par.imuChs")
                identified = mount(snap, "y.imuMntOri_chs")
                if pitch_deg < 0:
                    self.assertIdentified(snap, "swingLe", 24.0, 0.50)       # 60 % = Y[2]: vmax
                    self.assertIdentified(snap, "swingRi", 23.0, 0.45)
                    self.assertGreater(np.abs(compiled - np.eye(3)).max(), 0.5, "precondition: a non-trivial mount")
                    self.assertLess(np.abs(identified - compiled).max(), 1e-4)
                    # the firmware's swing estimate equals the plant after ~280 deg of swinging
                    h.tick(200)
                    self.assertAlmostEqual(-h.fw["y.jnts.ChsToUc.q"], plant.q["swing"], delta=0.01 * DEG)
                else:
                    self.assertLess(np.abs(identified - compiled @ np.diag([-1.0, -1.0, 1.0])).max(), 1e-4)

    def test_accelerometer_sign_is_minus_one_g(self):
        # FINDING: kinematics.publish_imus writes +1 g up (specific force) and calls the real sign "NOT
        # established". The compiled mounts establish it: re-run the arm calibration of the test above with
        # +1 g and the rebuilt mount is turned 180 deg about the arm pin axis (x and z columns negated, the
        # pin-axis y column kept). To get the compiled mount with +1 g the arm chord would have to point
        # straight UP at the reference pose (boom + arm = -90 deg), and boom + arm cannot go below
        # -69 + 31 = -38 deg. Input link (horizontal), tilt (default pose) and chassis (nose-up jack-up) all
        # agree with -1 g at their natural poses (pinned above). KinematicPlant therefore defaults to -1 g; the
        # shared publisher (Harness.set_pose / nominal_inputs) still writes +1 g and is owned elsewhere.
        self.assertEqual(KinematicPlant().acc_sign, -1.0)
        lo_b, _ = KinematicPlant().limits["boom"]
        lo_a, _ = KinematicPlant().limits["arm"]
        self.assertGreater(lo_b + lo_a, -90.0 * DEG, "the +1 g reference pose (arm straight up) is unreachable")
        plant = KinematicPlant(acc_sign=1.0, **self.ARM_PLANT)
        h, emu, _ = self.calibrate(plant, "CalibArm")
        compiled = mount(h.fw, "par.imuArm")
        identified = mount(emu.saved[0][1], "y.imuMntOri_arm")
        self.assertGreater(np.abs(identified - compiled).max(), 1.9)
        self.assertLess(np.abs(identified - compiled @ np.diag([-1.0, 1.0, -1.0])).max(), 1e-4)


class TestPublishCache(unittest.TestCase):

    def test_a_plant_editing_the_imu_inports_after_this_one_is_rebased_every_tick(self):
        # WHY: KinematicPlant skips re-writing unchanged sensors (~0.3 ms a tick) -- only while every inport
        # it owns still holds what it wrote. A later plant that edits them in place (noise, a fault) must
        # act on fresh values each tick, not on its own output of the previous tick.
        written = []

        class Negate:
            def __call__(self, h):
                written.append(h.fw["u.chsImuAcc"])
                h.fw["u.chsImuAcc"] = [-v for v in h.fw["u.chsImuAcc"]]

        plant = KinematicPlant()
        h = booted(plant, extra=[Negate()])
        first = h.fw["u.chsImuAcc"]
        self.assertGreater(abs(first[2]), 0.99)
        for _ in range(5):
            h.tick()
            self.assertEqual(written[-1], [-v for v in first], "the plant re-wrote its own value")
            self.assertEqual(h.fw["u.chsImuAcc"], first)


if __name__ == "__main__":
    unittest.main()
