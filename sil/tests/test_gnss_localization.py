"""
GNSS -> machine pose. Checks resources/X1Exc_SIL_spec.md A3.2 / A3.3 against the running firmware.

Firmware path under test (generated C: X1Exc/Asw/GeneratedCode/MdlApp_ert_rtw/MdlApp.c):
  Localization     chart_2222  MdlApp.c:14794-15215, called at :41996 with the u.* geodetic inports
                   blh -> ECEF(par.parEllSrc) -> datum Helmert -> geodetic(parEllTar) -> TM
                   -> horizontal Helmert -> height - vertical plane -> single(x - siteOrigin)
                   Output MdlApp_B.enh_LocalMain/Aux, read here via fw.internal('enh_local_*').
  KinematicsCalc   chart_2143  MdlApp.c:11706-11882 (yaw from antenna baseline, links.chs.R/p),
                   :11887-12185 (GNSS swing angle), :12410-12437 (joint-angle LPF, sign flip),
                   :13420-13448 (machHeading); called at :42052, i.e. the SAME tick as Localization
  ValidateSwingAngCalc chart_1167  MdlApp.c:51831-51936, read back through unit delays Delay16-18
                   (read :42043-42049 before KinematicsCalc, written :52313-52325 at the end of the step)
  ChkVerticalAccuracy  chart_2496  MdlApp.c:39081-39112, on-delay in InhibitStsMgr :39365-39397

HOW THE TESTS DRIVE IT: sil.geodesy (the publisher the Isaac plant uses) with the spec A3.3
"bare TM" site: identity datum/horizontal Helmert (sf=1), zero vertical plane, no geoid, TM
(type 0) with k0=1, no false origin, projection origin = scene reference, siteOrigin = (0, 0, h0).
Then site E/N/H == world X/Y/Z. World -> (lat, lon, h) uses sil.geodesy.KruegerTM, an
independent Krueger n^6 series (Karney 2011), NOT a copy of the firmware's Redfearn series -- so
a round trip that closes checks the firmware's projection, not a transcription of it. IMUs come
from Harness.set_pose (sil.kinematics) so the chassis tilt the firmware reads is the one the
antennas were placed with.

ACHIEVED TOLERANCE, AND WHY IT IS NOT BETTER
  The limit is float32: enh_LocalMain is cast to single after subtracting siteOrigin
  (MdlApp.c:15203-15213) and links.chs.p/R are single. So position agrees to a few float32 ULPs
  of the coordinate magnitude and yaw to ULP/|baseline|. f32_tol() below states that bound per
  assertion. (The compiled-site test runs 200 km off the central meridian, where third-order TM
  terms are ~40 m, at f32_tol(0) = 2e-6 m -- that is the check that the series itself is right.)

Conventions (all asserted below):
  world/site: X = grid East, Y = grid North, Z = up.   y.links.*.R is COLUMN-major, R[r + 3c].
  chassis yaw psi: rotation about +Z, CCW from grid East (Rz(psi) in links.chs.R).
  y.machHeading = wrap(pi/2 - psi): clockwise-from-grid-North azimuth of chassis +X.
  y.jnts.ChsToUc.q = -(house rotation about +Z since the swing-alignment edge, counted only on
                     ticks after u.ehPiPrs.swing* was above 5 bar), through a 3 Hz first-order LPF.
"""
import math
import unittest

import numpy as np

from sil import kinematics as kin
from sil.geodesy import WGS84_A, WGS84_ESQ, KruegerTM, Site, place_antennas
from sil.harness import Harness

DEG = math.pi / 180.0
Rx, Ry, Rz = kin.Rx, kin.Ry, kin.Rz


# ------------------------------------------------------------------------------------------
# Local helpers the library does not have (observers of firmware outputs, not publishers)
# ------------------------------------------------------------------------------------------
def col_major(flat):
    """y.links.*.R -> 3x3. MdlApp.c:11874 writes R[i_0 + 3*i_2] (row i_0, col i_2)."""
    return np.array([[flat[r + 3 * c] for c in range(3)] for r in range(3)])


def yaw_312(R):
    """Z of the 312 sequence R = Rz(z) Rx(x) Ry(y), same as the firmware (MdlApp.c:11066-11076)."""
    return math.atan2(-R[0][1], R[1][1])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def f32_tol(*coords):
    """Agreement bound for a single-precision output whose inputs are this large: 4 ULPs of the
    largest magnitude (the cast at MdlApp.c:15203 plus float32 R*d arithmetic), floor 1e-6 m."""
    big = max(abs(c) for c in coords) + 2.0
    return 4.0 * float(np.spacing(np.float32(big))) + 1e-6


# Firmware joint-angle filter, KinematicsCalc LPF1st_JntAngs (chart_2143 script line 188,
# MdlApp.c:11286-11390, called at :12425 with the literals 0.01F and 3.0F = SampleTime and
# FcJntAng, SysPar.m:1, :75): y = y + alpha*wrap(u - y), alpha = Ts / (1/(2*pi*fc) + Ts) (:11329).
# Kept local: sil.kinematics publishes IMUs, it has no model of the firmware's OUTPUT filters.
LPF_ALPHA = 0.01 / (1.0 / (2 * math.pi * 3.0) + 0.01)


class GnssTestBase(unittest.TestCase):
    TILT = Rx(3 * DEG) @ Ry(-6 * DEG)          # a non-trivial chassis roll/pitch (jack-up-like)

    def setUp(self):
        self.h = Harness().reset().nominal_inputs()
        self.fw = fw = self.h.fw
        self.d_aux = np.array(fw["par.parKin.distAntMainToAntAux"])
        self.d_chs = np.array(fw["par.parKin.distAntMainToChs"])
        self.L = math.hypot(self.d_aux[0], self.d_aux[1])

    # -- publishers ---------------------------------------------------------------------
    def site(self, *args, **kw):
        """Write a bare-TM site through the harness (Harness.gnss_site -> sil.geodesy.Site)."""
        return self.h.gnss_site(Site(*args, **kw))

    def attitude(self, R):
        """All five IMUs for a chassis at world attitude R, front at the harness rest pose."""
        self.h.set_pose(R_chs=R, **Harness.NOMINAL_POSE)

    def place(self, main, R, site=None, d_aux=None, swap=False):
        """Antennas for a chassis at attitude R with the main antenna at world `main`.
        The normal path IS sil.geodesy.place_antennas. `d_aux` (a wrong baseline) and `swap`
        (Main/Aux cabled backwards) build deliberately broken publishers the library has no
        reason to offer, so only those stay local."""
        site = site or self.h.site
        main = np.asarray(main, dtype=float)
        if d_aux is None and not swap:
            return place_antennas(self.fw, site, main, R)
        aux = main + np.asarray(R) @ (self.d_aux if d_aux is None else np.asarray(d_aux))
        m, a = (aux, main) if swap else (main, aux)
        self.fw["u.blh_Main"] = site.blh(m)
        self.fw["u.blh_Aux"] = site.blh(a)
        return aux

    def ready(self):
        """Every non-position input healthy (RTK fixed, swing aligned, target set) so the inhibit
        word isolates what GNSS does to it."""
        self.h.gnss_rtk_fixed()
        self.h.pulse("u.isSwingAligned")
        self.h.set_target_panel()

    # -- observers ----------------------------------------------------------------------
    def chs_R(self):
        return col_major(self.fw["y.links.chs.R"])

    def chs_p(self):
        return np.array(self.fw["y.links.chs.p"])

    def enh(self, which="main"):
        """Localization output alone (MdlApp_B.enh_LocalMain/Aux), before KinematicsCalc."""
        return np.array([self.fw.internal(f"enh_local_{which}_{k}") for k in "enh"])

    def assertVecClose(self, got, exp, tol, msg=""):
        got, exp = np.asarray(got, dtype=float), np.asarray(exp, dtype=float)
        err = float(np.abs(got - exp).max())
        self.assertLessEqual(err, tol, f"{msg} got={got} exp={exp} err={err:.3g} tol={tol:.3g}")

    def assertAngleClose(self, got, exp, tol, msg=""):
        err = abs(wrap(got - exp))
        self.assertLessEqual(err, tol, f"{msg} got={got} exp={exp} err={err:.3g} tol={tol:.3g}")


# ==========================================================================================
class TestTransverseMercatorHelper(unittest.TestCase):
    """sil.geodesy.KruegerTM is the tests' ground truth AND the Isaac publisher's projection;
    prove it before trusting any firmware comparison."""

    def test_inverse_undoes_forward_to_nanometres(self):
        # WHY: if the helper is wrong, every firmware round-trip failure below is uninterpretable.
        # No firmware involved: this pins sil/geodesy.py:31-96, the reference the firmware's own
        # MdlApp_TmProj (MdlApp.c:14547-14755, a different series) is compared against below.
        for lat0, lon0 in ((32.9, -96.8), (38.0, 127.0), (-33.9, 151.2), (0.5, 10.0)):
            tm = KruegerTM(lat0 * DEG, lon0 * DEG)
            for east in (-2000.0, -3.3, 0.0, 250.0, 2000.0):
                for north in (-2000.0, -7.1, 0.0, 40.0, 2000.0):
                    e2, n2 = tm.forward(*tm.inverse(east, north))
                    self.assertLess(max(abs(e2 - east), abs(n2 - north)), 1e-8)

    def test_series_matches_independent_ellipsoid_geometry(self):
        # WHY: forward/inverse self-consistency cannot catch a wrong coefficient shared by both.
        # Checked against things that do not use the series: the published WGS84 quarter meridian
        # (10 001 965.729 m); the meridian arc integrated numerically from the radius of curvature
        # M(phi) (on the central meridian northing = A*(xi' + sum alpha_j sin 2j xi'), so this
        # pins every alpha); and conformality off the meridian (equal scale in both directions,
        # orthogonal grid lines), which a wrong series structure breaks. No firmware rule here
        # either; the firmware side of the projection is test_compiled_site_calibration_round_trips.
        tm = KruegerTM(0.0, 0.0)
        self.assertAlmostEqual(tm.A * math.pi / 2, 10001965.729, delta=1e-3)

        def M(phi):
            return WGS84_A * (1 - WGS84_ESQ) / (1 - WGS84_ESQ * math.sin(phi) ** 2) ** 1.5

        def meridian_arc(phi, n=2000):                       # Simpson's rule
            h = phi / n
            s = M(0.0) + M(phi) + sum((4 if i % 2 else 2) * M(i * h) for i in range(1, n))
            return s * h / 3

        for lat_deg in (15.0, 30.0, 45.0, 60.0, 75.0):
            with self.subTest(arc=lat_deg):
                self.assertAlmostEqual(tm.forward(lat_deg * DEG, 0.0)[1],
                                       meridian_arc(lat_deg * DEG), delta=1e-6)

        tm = KruegerTM(38.0 * DEG, 127.0 * DEG)
        d = 1e-7
        for lat_deg in (-60.0, 5.0, 35.3, 60.0):
            for dlon_deg in (0.5, 2.2, 3.5):
                with self.subTest(conformal=(lat_deg, dlon_deg)):
                    phi, lam = lat_deg * DEG, (127.0 + dlon_deg) * DEG
                    fp, fm = tm.forward(phi + d, lam), tm.forward(phi - d, lam)
                    gp, gm = tm.forward(phi, lam + d), tm.forward(phi, lam - d)
                    dphi = [(fp[i] - fm[i]) / (2 * d) for i in range(2)]
                    dlam = [(gp[i] - gm[i]) / (2 * d) for i in range(2)]
                    N = WGS84_A / math.sqrt(1 - WGS84_ESQ * math.sin(phi) ** 2)
                    k_n = math.hypot(*dphi) / M(phi)
                    k_e = math.hypot(*dlam) / (N * math.cos(phi))
                    self.assertAlmostEqual(k_n, k_e, delta=1e-7)
                    cos_angle = (dphi[0] * dlam[0] + dphi[1] * dlam[1]) / (
                        math.hypot(*dphi) * math.hypot(*dlam))
                    self.assertAlmostEqual(cos_angle, 0.0, delta=1e-7)


# ==========================================================================================
class TestPoseRoundTrip(GnssTestBase):
    """Spec A3.2: links.chs.p = enh_LocalMain + R_chs * distAntMainToChs; A3.3 identity chain."""

    SITES = ((32.9, -96.8, 180.0), (38.0, 127.0, 102.2), (-33.9, 151.2, 40.0))
    ORIGINS = ((0.0, 0.0, 0.0), (12.3, -45.6, 2.5), (-480.0, 350.0, -3.0), (1500.0, -1500.0, 10.0))
    YAWS = (0.0, 0.5, -2.0, 3.1)

    def test_chassis_position_is_main_antenna_plus_rotated_offset(self):
        # WHY: this one offset carries every GNSS fix into the whole kinematic chain; if the
        # Isaac->blh publisher and the firmware disagree, every tool-position check is biased.
        # Driven through Harness.place_chassis (main = origin - R*d_chs, sil/geodesy.py:150) so
        # the Isaac plant's inverse is exercised too: a sign error there doubles the offset.
        # Firmware: Localization enh_LocalMain MdlApp.c:15203-15213 (chart_2222 line 48);
        # links.chs.R = Rz(yaw)*Rx(roll)*Ry(pitch) :11815, links.chs.p = enh_LocalMain + R*d :11882
        # (chart_2143 lines 58-59). The tilt is non-trivial (3/-6 deg) and R is compared at an
        # ANGULAR tolerance, so a swapped tilt order (Ry*Rx, ~5e-3 off) cannot hide in it.
        for lat0, lon0, h0 in self.SITES:
            self.site(lat0, lon0, h0)
            for origin in self.ORIGINS:
                for yaw in self.YAWS:
                    with self.subTest(site=(lat0, lon0), origin=origin, yaw=yaw):
                        R = Rz(yaw) @ self.TILT
                        self.attitude(R)
                        main = self.h.place_chassis(origin, R)
                        aux = main + R @ self.d_aux
                        self.h.tick()
                        tol = f32_tol(*main, *aux)
                        self.assertVecClose(self.enh("main"), main, tol, "enh_LocalMain")
                        self.assertVecClose(self.enh("aux"), aux, tol, "enh_LocalAux")
                        self.assertVecClose(self.chs_p(), origin, tol, "links.chs.p")
                        self.assertVecClose(self.chs_R(), R, 2e-6 + tol / self.L, "links.chs.R")

    def test_compiled_site_calibration_round_trips(self):
        # WHY: the shipped parameter set carries a real Korean TM site (false E/N 200 km/600 km,
        # site 200 km off the central meridian, ShortArm.m:131-164); the firmware's truncated
        # series must still close there if a lab copies that calibration into the u.* inports.
        # Firmware: MdlApp_TmProj MdlApp.c:14547-14755; siteOrigin subtracted in double :15203.
        fw = self.fw
        site = self.site(math.degrees(fw["par.parProj.latOrg"]), math.degrees(fw["par.parProj.lonOrg"]),
                         false_e=fw["par.parProj.false_E"], false_n=fw["par.parProj.false_N"],
                         site_origin=fw["par.siteOrigin"])
        lat, lon, _ = site.blh([0.0, 0.0, 0.0])
        self.assertAlmostEqual(lat / DEG, 35.321, delta=1e-3)      # the site really is off-meridian
        self.assertAlmostEqual(lon / DEG, 129.216, delta=1e-3)
        R = Rz(0.3) @ self.TILT
        self.attitude(R)
        for origin in ((0.0, 0.0, 0.0), (30.0, -20.0, 1.0), (-300.0, 400.0, 0.0)):
            with self.subTest(origin=origin):
                main = self.h.place_chassis(origin, R)
                self.h.tick()
                self.assertVecClose(self.enh("main"), main, f32_tol(*main), "enh_LocalMain")
                self.assertVecClose(self.chs_p(), origin, f32_tol(*main), "links.chs.p")
                self.assertAngleClose(self.fw["y.machHeading"], math.pi / 2 - 0.3, 1e-5)

    def test_pose_is_same_tick_and_a_fix_jump_teleports_the_machine(self):
        # WHY: spec A3.2 "no odometry, no INS fusion": a publisher glitch or a late GNSS frame is
        # applied in full on the very next step, so the SIL clock must line GNSS up with physics.
        # Firmware: Localization runs at MdlApp.c:41996, KinematicsCalc at :42052 in the same
        # MdlApp_step; chs.p is written straight from enh_LocalMain (:11882), no filter.
        self.site(32.9, -96.8, 180.0)
        R = Rz(0.2)
        self.attitude(R)
        self.place([0.0, 0.0, 0.0], R)
        self.h.tick(5)
        before_p, before_enh = self.chs_p(), self.enh("main")
        self.place([5.0, 0.0, 0.0], R)
        self.h.tick(1)
        self.assertVecClose(self.enh("main"), before_enh + [5.0, 0.0, 0.0], 1e-5, "Localization, 1 tick")
        self.assertVecClose(self.chs_p(), before_p + [5.0, 0.0, 0.0], 1e-5, "chs.p, same tick")

    def test_chassis_tilt_comes_from_the_imu_and_must_match_the_antennas(self):
        # WHY: CalibChs runs in the dozer jack-up pose (pitch > 5 deg). The firmware tilts the
        # antenna reference by IMU roll/pitch before taking yaw, and rotates distAntMainToChs
        # (z = -1.345 m) by that tilt: IMU and GNSS must be published from the same pose.
        # Firmware: R_ChsTilt = Rx(roll)*Ry(pitch) MdlApp.c:11706-11773 (chart_2143 lines 34-49),
        # links.chs.R = Rz(yaw)*R_ChsTilt :11815-11874, 312 Euler of the IMU :11062-11076.
        self.site(32.9, -96.8, 180.0)
        main = np.array([3.0, 4.0, 0.5])
        for yaw, roll, pitch in ((0.3, 0.0, 8 * DEG), (-2.5, 3 * DEG, -6 * DEG)):
            with self.subTest(yaw=yaw, roll=roll, pitch=pitch):
                R = Rz(yaw) @ Rx(roll) @ Ry(pitch)
                self.attitude(R)
                self.place(main, R)
                self.h.tick()
                self.assertVecClose(self.chs_R(), R, 1e-5, "links.chs.R")
                self.assertVecClose(self.chs_p(), main + R @ self.d_chs, f32_tol(*main))
                self.assertAngleClose(self.fw["y.machHeading"], math.pi / 2 - yaw, 1e-5)

        # A plant that pitches the antennas 8 deg but publishes a LEVEL IMU. The firmware's pose
        # follows from its own rule: reference = I*d_aux (untilted), yaw = atan2(ref x base,
        # ref . base) over x/y (:11771), R = Rz(yaw) (:11815). Asserting that model (not just
        # "is wrong") catches a firmware that ignored the IMU for yaw but not for the offset.
        # Size of the damage vs truth: ~0.19 m horizontal, -0.26 deg heading.
        R_true = Rz(0.3) @ Ry(8 * DEG)
        self.attitude(np.eye(3))
        self.place(main, R_true)
        self.h.tick()
        base, ref = R_true @ self.d_aux, self.d_aux
        yaw_fw = math.atan2(ref[0] * base[1] - base[0] * ref[1], ref[0] * base[0] + ref[1] * base[1])
        self.assertVecClose(self.chs_R(), Rz(yaw_fw), 1e-5, "modelled wrong R")
        self.assertVecClose(self.chs_p(), main + Rz(yaw_fw) @ self.d_chs, f32_tol(*main), "modelled wrong p")
        self.assertAngleClose(self.fw["y.machHeading"], math.pi / 2 - yaw_fw, 1e-5, "modelled wrong heading")
        err = self.chs_p() - (main + R_true @ self.d_chs)
        self.assertGreater(math.hypot(err[0], err[1]), 0.15)
        self.assertGreater(abs(yaw_fw - 0.3), 0.2 * DEG)


# ==========================================================================================
class TestHeading(GnssTestBase):
    """Spec A3.2 "There is no heading message": yaw = angle of the antenna baseline."""

    def test_compiled_antenna_geometry(self):
        # DATA PIN (parameter values, not firmware logic): the Isaac antenna frames must reproduce
        # the Main->Aux baseline DIRECTION and the Main->chassis offset (the baseline LENGTH is not
        # used, see test_only_the_baseline_direction_matters). Source: ShortArm.m:165-166,
        # MdlApp.c:455-459. The firmware facts here: KinematicsCalc reads them from parLocalTest
        # every step (copied at MdlApp.c:41404-41579, used at :11747-11749 and :11878) and echoes
        # them to y.parKin (:45956-45985); the u.parKinStored inport copy is dead (spec A2), so a
        # plant that wants a different antenna geometry must patch par.*, not u.*.
        fw = self.fw
        self.assertVecClose(fw["par.parKin.distAntMainToAntAux"], (0.682, 0.959, 0.0), 1e-6)
        self.assertVecClose(fw["par.parKin.distAntMainToChs"], (0.57, -0.087, -1.345), 1e-6)
        self.h.tick()
        self.assertEqual(fw["y.parKin.distAntMainToAntAux"], fw["par.parKin.distAntMainToAntAux"])
        self.assertEqual(fw["y.parKin.distAntMainToChs"], fw["par.parKin.distAntMainToChs"])

        self.site(32.9, -96.8, 180.0)
        R, main = Rz(0.4), np.array([2.0, 1.0, 0.0])
        self.place(main, R)
        fw["u.parKinStored.distAntMainToChs"] = [9.0, 9.0, 9.0]
        fw["u.parKinStored.distAntMainToAntAux"] = [-9.0, 9.0, 0.0]
        self.h.tick()
        self.assertVecClose(self.chs_p(), main + R @ self.d_chs, f32_tol(*main), "u.parKinStored is dead")
        self.assertAngleClose(fw["y.machHeading"], math.pi / 2 - 0.4, 1e-5, "u.parKinStored is dead")
        d_chs = self.d_chs + [0.1, 0.0, 0.0]
        fw["par.parKin.distAntMainToChs"] = list(d_chs)
        self.h.tick()                                          # live on the same step
        self.assertVecClose(self.chs_p(), main + R @ d_chs, f32_tol(*main), "par.parKin is live")
        self.assertEqual(fw["y.parKin.distAntMainToChs"], fw["par.parKin.distAntMainToChs"])

    def test_machHeading_is_clockwise_from_grid_north_of_chassis_x(self):
        # WHY: a sign or 90-degree offset error here would drive Positioning the wrong way and
        # still "converge"; the convention must be pinned before any travel scenario.
        # Firmware: yaw = atan2(cross, dot) of tilted reference vs baseline MdlApp.c:11771
        # (chart_2143 line 49); machHeading = -yaw + pi/2, wrapped MdlApp.c:13423-13445 (line 341).
        self.site(38.0, 127.0, 50.0)
        main = [-480.0, 350.0, 0.0]
        # enh_LocalMain/Aux are each rounded to float32 once (<= ULP/2 per component, :15203), so
        # each baseline component is off by <= 1 ULP and its direction by <= sqrt(2)*ULP/L; a
        # difference of two headings by twice that. f32_tol (4 ULP) / L bounds both: 1.05e-4 rad
        # here, against a measured worst case of 1.6e-5 rad over a 1-deg sweep of yaw.
        tol = f32_tol(*main) / self.L
        seen = {}
        for yaw_deg in (-170, -120, -90, -30, 0, 30, 45, 90, 135, 179.5):
            with self.subTest(yaw_deg=yaw_deg):
                yaw = yaw_deg * DEG
                self.place(main, Rz(yaw))                  # IMU level: nominal_inputs()
                self.h.tick()
                self.assertAngleClose(yaw_312(self.chs_R()), yaw, tol, "links.chs.R yaw")
                self.assertAngleClose(self.fw["y.machHeading"], math.pi / 2 - yaw, tol, "machHeading")
                self.assertLessEqual(abs(self.fw["y.machHeading"]), math.pi + 1e-6)
                seen[yaw_deg] = self.fw["y.machHeading"]
        # Rotating the baseline by +45 deg (CCW seen from above) lowers machHeading by 45 deg.
        self.assertAngleClose(seen[45] - seen[0], -45 * DEG, tol)
        self.assertAngleClose(seen[0], math.pi / 2, tol)       # chassis +X east -> heading 90 deg
        self.assertAngleClose(seen[90], 0.0, tol)              # chassis +X north -> heading 0

    def test_chassis_imu_yaw_is_ignored(self):
        # WHY: the house IMU has a gyro-integrated yaw that drifts; the firmware must not use it,
        # so an Isaac IMU publisher may emit any yaw without changing the pose.
        # Firmware: links.chs.R = Rz(euAng_ChsEstm_z)*Rx(euAng(1))*Ry(euAng(2)) MdlApp.c:11815 --
        # on the chs.R/p path only euAng[0] (:11709) and euAng[1] (:11717) are read. euAng(3) IS read
        # on the tilt-joint path (:12191-12194, chart_2143 line 156), but lines 158-161 cancel its
        # drift against the tilt IMU (angDriftUfToTilt), so no joint angle depends on it either.
        # Only the CHASSIS IMU is yawed here (the other four stay at the true pose), which is the
        # case that distinguishes "ignored" from "consistently yawed IMUs cancel".
        self.site(32.9, -96.8, 180.0)
        R = Rz(0.3) @ Rx(2 * DEG) @ Ry(6 * DEG)
        pose = dict(Harness.NOMINAL_POSE, q_tilt=0.3)            # non-zero tilt: the drift term matters
        joints = ("BmMntToBm1", "Bm2ToArm", "ArmToInpLink", "ArmToOutpLink", "TiltMntToTilt", "ChsToUc")
        results = []
        for imu_yaw in (0.0, 1.2, -2.0, 3.0):
            frames = kin.link_frames(self.fw, R, **pose)
            frames["chs"] = Rz(imu_yaw) @ frames["chs"]
            kin.publish_imus(self.fw, frames)
            self.place([3.0, 4.0, 0.5], R)
            self.h.tick(300)                                    # joint LPF settles (3 Hz)
            with self.subTest(imu_yaw=imu_yaw):                 # the yaw really reached the firmware
                self.assertAngleClose(self.fw["y.chs.euAng"][2], 0.3 + imu_yaw, 1e-4)
            results.append((self.chs_p(), self.fw["y.links.chs.R"], self.fw["y.machHeading"],
                            [self.fw[f"y.jnts.{j}.q"] for j in joints]))
        for r in results[1:]:
            self.assertVecClose(r[0], results[0][0], 1e-6)
            self.assertVecClose(r[1], results[0][1], 1e-6)
            self.assertAlmostEqual(r[2], results[0][2], delta=1e-6)
            self.assertVecClose(r[3], results[0][3], 1e-6, "joint angles")
        self.assertVecClose(col_major(results[0][1]), R, 1e-5)
        self.assertAlmostEqual(results[0][3][4], 0.3, delta=1e-5)

    def test_only_the_baseline_direction_matters(self):
        # WHY: tells the asset builder which antenna errors matter. Spec A3.2 says the antennas
        # "must sit at exactly [0.682, 0.959, 0] m apart" and "1 cm of relative error ... is ~0.5
        # deg". The firmware takes atan2(cross, dot) (MdlApp.c:11771), so LENGTH errors are
        # invisible; only the component perpendicular to the baseline turns heading. There is no
        # plausibility check anywhere: distAntMainToAntAux is read only at :11747-11749 (the
        # SetActuat bus copy at :8192-8253 has no reader), enh_LocalAux only at :11730-11736.
        self.site(32.9, -96.8, 180.0)
        R = Rz(0.3)
        u = self.d_aux[:2] / self.L
        left = np.array([-u[1], u[0], 0.0])                   # CCW normal in the house frame

        self.place([0.0, 0.0, 0.0], R)
        self.ready()
        self.h.tick(30)
        self.assertEqual(self.h.inhibit_status(), 0, self.h.describe())
        ref_heading, ref_p = self.fw["y.machHeading"], self.chs_p()
        for scale in (0.5, 2.0):
            with self.subTest(scale=scale):
                self.place([0.0, 0.0, 0.0], R, d_aux=self.d_aux * scale)
                self.h.tick(500)                               # a slow check would show by 5 s
                self.assertAlmostEqual(self.fw["y.machHeading"], ref_heading, delta=2e-6)
                self.assertVecClose(self.chs_p(), ref_p, 1e-6)
                self.assertEqual(self.h.inhibit_status(), 0, "no baseline plausibility check")

        lateral = 0.010                                        # 1 cm sideways = atan(0.01/1.177) = 0.487 deg
        self.place([0.0, 0.0, 0.0], R, d_aux=self.d_aux + lateral * left)
        self.h.tick()
        shift = wrap(self.fw["y.machHeading"] - ref_heading)
        self.assertAlmostEqual(shift, -math.atan2(lateral, self.L), delta=5e-6)

    def test_swapped_antennas_flip_heading_and_shift_the_machine(self):
        # WHY: Main/Aux mapping is a one-line publisher mistake with no firmware error flag; the
        # SIL must recognise its signature (180 deg heading, chassis moved by the baseline).
        # Firmware: baseline = Aux - Main MdlApp.c:11730-11736; anchor = Main MdlApp.c:11882.
        self.site(32.9, -96.8, 180.0)
        yaw = 0.7
        R = Rz(yaw)
        main = np.array([10.0, -5.0, 0.0])
        self.place(main, R)
        self.ready()
        self.h.tick(30)
        self.assertEqual(self.h.inhibit_status(), 0, self.h.describe())
        aux = self.place(main, R, swap=True)
        self.h.tick()
        R_flip = Rz(yaw + math.pi)
        self.assertAngleClose(self.fw["y.machHeading"], math.pi / 2 - yaw - math.pi, 1e-5)
        self.assertVecClose(self.chs_p(), aux + R_flip @ self.d_chs, f32_tol(*aux))
        self.h.tick(500)
        self.assertEqual(self.h.inhibit_status(), 0, "no inhibit bit reacts to a swap")


# ==========================================================================================
class TestGnssSwingAngle(GnssTestBase):
    """The swing joint has no encoder: jnts.ChsToUc.q is the rotation of the antenna baseline
    about the chassis Z axis since the isSwingAligned edge, integrated ONLY while the swing is
    'operated'. chart_2143 lines 62-151 (MdlApp.c:11887-12185), validity from chart_1167
    ValidateSwingAngCalc (MdlApp.c:51831-51936) through a one-tick unit delay."""

    ORIGIN = (10.0, -5.0, 0.0)
    YAW0 = 0.7

    def setUp(self):
        super().setUp()
        self.site(32.9, -96.8, 180.0)
        self.yaw = self.YAW0
        self.put(self.YAW0)
        self.ready()                                  # the alignment edge latches baselineZero here
        self.h.tick(10)
        self.assertTrue(self.fw.internal("swing_init"))
        self.assertEqual(self.h.inhibit_status(), 0, self.h.describe())
        self.raw = 0.0                                # model of jntAng_SwingRaw (chart_2143 line 107)
        self.y = 0.0                                  # model of the filtered UcToChs (line 198)
        self.operated = False                         # model of isSwingOperated (chart_1167 line 47)

    def put(self, yaw):
        """A plant that swings the house about the chassis origin (Harness.place_chassis) and
        publishes the matching IMUs."""
        self.yaw = yaw
        R = Rz(yaw)
        self.attitude(R)
        self.h.place_chassis(self.ORIGIN, R)

    def q(self):
        return self.fw["y.jnts.ChsToUc.q"]

    def swing(self, prs, dyaw, ticks, tol=5e-6):
        """Hold u.ehPiPrs.swingLe = prs and turn the house by dyaw per tick. Checks q against the
        firmware's rule every tick:
          valid(k)  = operated(k-1)          (Delay16 read at :42043 before KinematicsCalc,
                                              written at :52313 after ValidateSwingAngCalc)
          raw      += dyaw while valid       (else RefHold/SwingHold re-latch, :11947-11956)
          y        += alpha*(raw - y)        (LPF1st_JntAngs, fc 3 Hz, :12425)
          q         = -y                     (jntAng_ChsToUc = -jntAng_UcToChs, chart_2143 line
                                              201 at :12437; written as y.jnts.ChsToUc.q =
                                              -jntAngFilt[8], line 265, at :13134)
          operated  = prs > 5 ? 1 : prs < 3 ? 0 : hold   (strict '>' 5.0F at MdlApp.c:51912, strict
                                              '<' 3.0F at :51918, hold :51928; = RngPiPrsOperOn/Off,
                                              SysPar.m:47-48; chart_1167 lines 47-52)
        The other OR-term of 'operated', isSwingCmdOn = |propVlvCmd.swingLe/Ri| > 1e-6 (:51861), is
        false throughout: auto never starts in these tests, so every valve port stays 0."""
        for _ in range(ticks):
            self.fw["u.ehPiPrs.swingLe"] = prs
            self.put(self.yaw + dyaw)
            self.h.tick()
            if self.operated:
                self.raw += dyaw
            self.y += LPF_ALPHA * (self.raw - self.y)
            self.operated = True if prs > 5 else (False if prs < 3 else self.operated)
            self.assertAlmostEqual(self.q(), -self.y, delta=tol,
                                   msg=f"prs={prs} yaw-yaw0={self.yaw - self.YAW0:.3f} {self.h.describe()}")
        self.assertEqual(self.h.valves(), {}, "the model assumes isSwingCmdOn == false")

    def test_swing_angle_tracks_minus_the_baseline_rotation_through_a_3hz_filter(self):
        # WHY: this is the only swing-angle sensor the firmware has; Positioning holds
        # tarJnts.ChsToUc.q = 0 (chart_1011:135) and Placing swings to jnts.ChsToUc.q - tarSwingAng
        # (chart_1011:276). A plant must know its sign (house CCW = q negative), its lag (3 Hz
        # first order) and that the first tick of pressure is not yet "valid" -- or a closed-loop
        # swing will look like a firmware overshoot.
        # Firmware: swingAngRef = atan2(axis.(ref x curr), ref.curr) :11927-11938 (chart_2143
        # line 78), positive for CCW about +Z; jntAng_ChsToUc = -jntAng_UcToChs :12437 (line 201),
        # output y.jnts.ChsToUc.q = -jntAngFilt[8] :13134 (line 265); LPF :12425;
        # Delay16 :42043 / :52313.
        self.swing(prs=10.0, dyaw=0.01, ticks=20)     # pressure and motion start together
        self.assertAlmostEqual(self.raw, 0.19, delta=1e-9)   # first increment fell in the delay
        self.swing(prs=10.0, dyaw=0.0, ticks=100)
        self.assertAlmostEqual(self.q(), -0.19, delta=1e-5)

    def test_swing_angle_needs_eh_pilot_pressure_not_rcv_pilot_pressure(self):
        # WHY: spec A3.4 says u.ehPiPrs "only matters below MdlApp" (leave 0) and B2.9 says
        # rcvPiPrs gates swing-angle validity. Both backwards: ValidateSwingAngCalc reads
        # MdlApp_U.ehPiPrs.swingLe/Ri (MdlApp.c:51912, :51918; chart_1167 port named "rcvPiPrs"
        # but wired to inport ehPiPrs, :51836). u.rcvPiPrs feeds only manual-operation detection
        # (<S154>, :39175-39190 are its only readers). A plant that swings the house with
        # ehPiPrs = 0 gets a swing angle frozen at 0 while GNSS heading moves -- with no inhibit.
        self.swing(prs=0.0, dyaw=0.01, ticks=20)
        self.swing(prs=0.0, dyaw=0.0, ticks=50)
        self.assertEqual(self.q(), 0.0, "house turned 0.2 rad, no pressure: frozen")
        self.assertAngleClose(self.fw["y.machHeading"], math.pi / 2 - (self.YAW0 + 0.2), 1e-5)
        self.assertEqual(self.h.inhibit_status(), 0, self.h.describe())

        self.fw["u.rcvPiPrs.swingLe"] = 10.0
        self.swing(prs=0.0, dyaw=0.01, ticks=20)
        self.swing(prs=0.0, dyaw=0.0, ticks=50)
        self.assertEqual(self.q(), 0.0, "rcvPiPrs does not validate the swing angle")
        self.fw["u.rcvPiPrs.swingLe"] = 0.0

        self.swing(prs=10.0, dyaw=0.0, ticks=1)
        self.swing(prs=10.0, dyaw=0.01, ticks=10)
        self.swing(prs=10.0, dyaw=0.0, ticks=100)
        self.assertAlmostEqual(self.q(), -0.1, delta=1e-5, msg="only the pressurised 0.1 rad counts")

    def test_swing_angle_holds_between_3_and_5_bar_and_freezes_below_3(self):
        # WHY: a pilot-pressure noise model must respect the hysteresis; a pressure that decays
        # below 3 bar while the house still coasts leaves a permanent swing-angle offset (the
        # angle resumes INCREMENTALLY, it never re-reads the absolute baseline).
        # Firmware: operated on swingLe > 5.0F (RngPiPrsOperOn), off < 3.0F (RngPiPrsOperOff), else
        # hold (MdlApp.c:51910-51928, chart_1167 lines 47-52); frozen raw via RefHold/SwingHold
        # re-latch :11947-11956; raw = wrap(wrap(ref - RefHold) + SwingHold) :11994-12037 (line 107).
        # BOTH comparisons are strict, and each is pinned on both sides of its literal: 5.0 is not
        # on, 5.01 is; 3.0 still holds (on AND off), 2.99 is off. swing() checks every tick, so a
        # model or firmware on-threshold anywhere outside (5.0, 5.01] or off-threshold outside
        # (2.99, 3.0] diverges here. The inport is real32 (slprj/ert/_sharedutils/PiPrs_t.h:25) and
        # 5.0/3.0 are exact in float32, so the strictness is the firmware's, not a rounding artefact.
        self.swing(prs=5.0, dyaw=0.01, ticks=10)      # exactly 5 bar from off: not operated
        self.swing(prs=5.0, dyaw=0.0, ticks=50)
        self.assertEqual(self.q(), 0.0, "house turned 0.1 rad at exactly 5 bar: frozen")
        self.swing(prs=5.01, dyaw=0.0, ticks=2)       # just above 5: operated (valid one tick later)
        self.swing(prs=5.01, dyaw=0.01, ticks=20)
        self.swing(prs=4.0, dyaw=0.01, ticks=10)      # mid band: still operated
        self.swing(prs=3.0, dyaw=0.01, ticks=10)      # exactly 3 bar: not off, still operated
        self.swing(prs=3.0, dyaw=0.0, ticks=100)
        self.assertAlmostEqual(self.q(), -0.40, delta=1e-5)
        self.swing(prs=2.99, dyaw=0.01, ticks=10)     # just below 3: off, only the delayed first tick counts
        self.swing(prs=3.0, dyaw=0.01, ticks=10)      # band after off holds OFF, at both edges
        self.swing(prs=4.0, dyaw=0.01, ticks=10)
        self.swing(prs=5.0, dyaw=0.01, ticks=10)
        self.swing(prs=5.0, dyaw=0.0, ticks=100)
        self.assertAlmostEqual(self.q(), -0.41, delta=1e-5)
        self.swing(prs=10.0, dyaw=-0.01, ticks=10)    # back: increments only (first one delayed)
        self.swing(prs=10.0, dyaw=0.0, ticks=100)
        self.assertAlmostEqual(self.q(), -0.32, delta=1e-5)
        self.assertAlmostEqual(self.yaw - self.YAW0, 0.80, delta=1e-9)   # truth: house at +0.80 rad

    def test_swing_with_travel_freezes_the_angle_and_drops_the_alignment_latch(self):
        # WHY: spec A6.0 "swinging and travelling together for 100 consecutive ticks invalidates
        # the latch". Pinned to the tick, and the angle is frozen during it: a plant that drives
        # tracks and swing together loses auto (BIT_SWING_NOT_INIT) until the house is re-aligned.
        # Firmware: isTrvlOperated from u.ehPiPrs.trvl* > 5 (:51884-51907); isJntAngSwingValid =
        # operated && ~trvlOperated (chart_1167 line 55, Delay16 :52313); counter on Delay18 &
        # Delay17 :11958-11984, CntSwingZeroInvalidChkDly 100 (SysPar.m:269, literal 100U at :11973),
        # == 100 -> hasSwingAlignedAfterKeyOn = false :11988-11991 (chart_2143 line 104); re-latched
        # by the next isSwingAligned rising edge :12067-12143 (lines 113, 139), which reads the
        # inport directly, so on the edge tick itself; the inhibit word reads it through Delay19
        # (:39645, updated :52237), one tick later.
        self.fw["u.ehPiPrs.trvlLeFwd"] = 10.0
        self.fw["u.ehPiPrs.swingLe"] = 10.0
        for k in range(1, 103):
            self.put(self.yaw + 0.001)
            self.h.tick()
            self.assertEqual(self.q(), 0.0, f"k={k}: travel invalidates the swing angle")
            # one tick for Delay17/18, then 100 counts; the inhibit word lags one more tick
            self.assertEqual(bool(self.fw.internal("swing_init")), k <= 100, f"k={k}")
            self.assertEqual(self.h.inhibit_bit("SWING_NOT_INIT"), k >= 102, f"k={k}")
        self.fw["u.ehPiPrs.trvlLeFwd"] = 0.0
        self.fw["u.ehPiPrs.swingLe"] = 0.0
        self.h.tick(200)
        self.assertFalse(self.fw.internal("swing_init"), "stopping does not restore it")
        self.assertTrue(self.h.inhibit_bit("SWING_NOT_INIT"), self.h.describe())
        self.fw["u.isSwingAligned"] = 1
        self.h.tick()
        self.assertTrue(self.fw.internal("swing_init"), "a new alignment edge does, on the edge tick")
        self.assertTrue(self.h.inhibit_bit("SWING_NOT_INIT"), "the inhibit word lags one tick")
        self.h.tick()
        self.assertEqual(self.h.inhibit_status(), 0, self.h.describe())
        self.fw["u.isSwingAligned"] = 0
        self.h.tick()
        self.assertTrue(self.fw.internal("swing_init"), "the latch needs the edge, not the level")


# ==========================================================================================
class TestSiteCalibrationInports(GnssTestBase):
    """Spec A3.3: which geodetic parameters are live, and what their zero defaults do."""

    def test_geodetic_inports_left_at_zero_freeze_the_pose_silently(self):
        # WHY: the likely first-run trap -- initialize() zeroes MdlApp_U, so a harness that
        # publishes blh but never writes the site calibration gets a machine parked at the site
        # origin facing grid East, with NO inhibit, whatever the antennas do. Not only a harness
        # trap: the ECU's NVM default site calibration is all zeros too (Asw/NvmCfg.c:55-110).
        # Firmware: sf=0 makes both Helmert matrices zero (MdlApp.c:14898-14908, 15088-15098);
        # CartesianToGeodetic returns 0 for ell_a <= 1 (:13977-13979); TmProj returns 0 for a <= 0
        # (:14584); atan2(0,0) = 0 (_sharedutils/rt_atan2f_snf.c:48-60) -> yaw 0 -> machHeading pi/2
        # (:13423). The geodetic u.* are never written here: nominal_inputs() leaves them at 0.
        self.ready()
        R_tilt = Rx(2 * DEG) @ Ry(5 * DEG)            # the IMU tilt still flows into the offset
        live = Site(32.9, -96.8, 180.0)               # used ONLY to generate plausible blh
        for main, yaw in (([0.0, 0.0, 0.0], 1.0), ([50.0, 20.0, 3.0], -2.0)):
            with self.subTest(main=main, yaw=yaw):
                R = Rz(yaw) @ R_tilt
                self.attitude(R)
                self.place(main, R, site=live)
                self.h.tick(30)
                self.assertVecClose(self.enh("main"), [0.0, 0.0, 0.0], 0.0, "Localization output")
                self.assertVecClose(self.enh("aux"), [0.0, 0.0, 0.0], 0.0, "Localization output")
                # Same parked pose for both fixes. Not bit-identical: the IMU roll/pitch the firmware
                # decodes from a differently-yawed quaternion differs by a float32 ULP (~4e-9 m here).
                self.assertVecClose(self.chs_p(), R_tilt @ self.d_chs, 1e-6, "parked at site origin")
                self.assertAlmostEqual(self.fw["y.machHeading"], math.pi / 2, delta=1e-6)
                self.assertEqual(self.h.inhibit_status(), 0, self.h.describe())

    def test_blh_left_at_zero_teleports_the_machine_silently(self):
        # WHY: the other half of the same trap -- a valid site but a GNSS publisher that has not
        # produced a fix yet (blh = 0 rad, 0 rad, 0 m). The quality gate never looks at position
        # (chart_2496 reads only methodGnss/stdDevZ, MdlApp.c:39081-39110), so auto is not
        # inhibited while the machine is placed thousands of km away.
        # Firmware: blh (0,0,0) is a valid ECEF point (a,0,0) (GeodeticToCartesian :13901); TmProj
        # evaluates it 97 deg off-meridian without a range guard (:14568-14586 guards only
        # non-finite, ellipsoid validity and |lat| >= pi/2; lonDiff unwrapped at :14704).
        self.site(32.9, -96.8, 180.0)
        self.ready()
        self.fw["u.blh_Main"] = [0.0, 0.0, 0.0]
        self.fw["u.blh_Aux"] = [0.0, 0.0, 0.0]
        self.h.tick(30)
        p = self.enh("main")
        self.assertTrue(np.all(np.isfinite(p)), p)
        self.assertGreater(math.hypot(p[0], p[1]), 1.0e6)
        self.assertTrue(np.all(np.isfinite(self.chs_p())))
        # Heading is pi/2 here ONLY because Main == Aux gives a zero baseline and atan2(0,0) = 0,
        # not because of the far-away projection:
        self.assertAlmostEqual(self.fw["y.machHeading"], math.pi / 2, delta=1e-6)
        self.assertEqual(self.h.inhibit_status(), 0, self.h.describe())
        # With a non-degenerate zero-ish pair the heading is finite and arbitrary. The aux offset
        # must survive float32 at 2e7 m (ULP 2 m), hence 1e-5 rad rather than a real 1 m baseline.
        self.fw["u.blh_Aux"] = [1.0e-5, 0.0, 0.0]
        self.h.tick(30)
        self.assertGreater(np.abs(self.enh("aux") - self.enh("main")).max(), 1.0)
        self.assertTrue(math.isfinite(self.fw["y.machHeading"]))
        self.assertGreater(abs(wrap(self.fw["y.machHeading"] - math.pi / 2)), 0.1)
        self.assertEqual(self.h.inhibit_status(), 0, self.h.describe())

    GEODETIC_PAR = ("par.parProj.", "par.parHorAdj.", "par.parVerAdj.", "par.parDatumTrans.",
                    "par.parEllTar.", "par.parGeoid.", "par.siteOrigin", "par.parIsGeoidCorrectionEnabled")

    def test_only_parEllSrc_a_b_are_read_from_parLocalTest(self):
        # WHY: confirms spec A3.3 ("8 live inports"; parEllSrc is the one taken from
        # parLocalTest) and pins its corollary: the compiled Korean site calibration in
        # parLocalTest (TM 38N/127E, siteOrigin 401531/304964) never reaches Localization, so a
        # harness that copies a site into par.* changes nothing. (Spec A2's "configure through
        # parLocalTest" is about the dead *Stored inports, not the geodetic set.)
        # Firmware call site MdlApp.c:41996-42020: parLocalTest.parEllSrc.a/.b (:42001-42002),
        # everything else MdlApp_U.*; u.parEllTar.e is not passed although the ECU writes it
        # (Asw/ModelInterface/AppCtrlIf.c:265).
        fw = self.fw
        self.site(32.9, -96.8, 180.0)
        R = Rz(0.4)
        self.place([5.0, 6.0, 1.0], R)
        self.h.tick()

        def outputs():
            return (list(self.chs_p()), fw["y.links.chs.R"], fw["y.machHeading"],
                    list(self.enh("main")), list(self.enh("aux")))
        ref = outputs()

        def garbage(v):
            if isinstance(v, list):
                return [garbage(x) for x in v]
            if isinstance(v, bool):
                return not v
            if isinstance(v, int):
                return v + 1
            return 3.0 * v + 12345.678

        leaves = [p for p in fw.paths("par.") if p.startswith(self.GEODETIC_PAR)]
        leaves += ["par.parEllSrc.e", "par.parEllSrc.eSq", "par.parEllSrc.fInv"]
        self.assertEqual(len(leaves), 52, leaves)            # every leaf of the 8 buses + parEllSrc rest
        for p in leaves:
            fw[p] = garbage(fw[p])
        for p in fw.paths("u.parEllSrc."):
            fw[p] = 123.0
        fw["u.parEllTar.e"] = 0.5
        self.h.tick()
        self.assertEqual(outputs(), ref)

        # live: par.parEllSrc.a and .b (the receiver's ellipsoid). 100 m on either moves the fix.
        for k in ("a", "b"):
            with self.subTest(live=k):
                nominal = fw[f"par.parEllSrc.{k}"]
                fw[f"par.parEllSrc.{k}"] = nominal + 100.0
                self.h.tick()
                self.assertGreater(np.abs(self.enh("main") - ref[3]).max(), 10.0)
                fw[f"par.parEllSrc.{k}"] = nominal
                self.h.tick()
                self.assertEqual(outputs(), ref)

    def test_horizontal_helmert_never_touches_height(self):
        # WHY: spec A3.3 asymmetry -- the horizontal Helmert's Z is computed and discarded, height
        # comes from the ellipsoid height minus the vertical plane. A site calibration with a bad
        # horizontal part still gives correct heights, which can hide it in a lift-only scenario.
        # Firmware: enh_Main = [HorAdj(1); HorAdj(2); blh_TarMain(3) - PolyInterp(parVerAdj,
        # enh_ProjMain)] MdlApp.c:15167-15182 (chart_2222 line 44). Asserted on enh_LocalMain,
        # i.e. Localization alone.
        self.site(32.9, -96.8, 180.0)
        self.fw["u.parHorAdj.sf"] = 0.0
        for main in ([0.0, 0.0, 0.0], [50.0, 20.0, 3.0]):
            with self.subTest(main=main):
                self.place(main, np.eye(3))
                self.h.tick()
                self.assertVecClose(self.enh("main")[:2], [0.0, 0.0], 1e-6, "E/N collapsed to 0")
                self.assertAlmostEqual(self.enh("main")[2], main[2], delta=1e-5)

        # The vertical plane IS applied, and is evaluated at the PRE-Helmert projected E/N.
        self.fw["u.parHorAdj.sf"] = 1.0
        self.fw["u.parHorAdj.dx"] = 1000.0            # post-Helmert E moves 1 km ...
        self.fw["u.parVerAdj.a10"] = 0.01             # ... but the plane sees E = 50 m
        self.place([50.0, 20.0, 3.0], np.eye(3))
        self.h.tick()
        self.assertVecClose(self.enh("main"), [1050.0, 20.0, 3.0 - 0.01 * 50.0], f32_tol(1050.0))
        self.assertVecClose(self.chs_p(), [1050.0, 20.0, 2.5] + self.d_chs, f32_tol(1050.0))

    def test_false_origin_not_absorbed_by_siteOrigin_quantises_the_pose(self):
        # WHY: spec A3.3 recommends false_E = false_N = 0 "so float precision stays near zero".
        # Quantified here: with the compiled Korean false northing (600 km) and siteOrigin E/N left
        # at 0, local northing lands in float32's 1/16 m grid and heading jumps ~1.7 deg between
        # values -- a sub-tolerance motion (UcHeadingTol 3.5 deg) the controller will chase.
        # Firmware: single(enh_Main - siteOrigin) MdlApp.c:15203-15213 (chart_2222 line 48).
        fw = self.fw
        fe, fn, h0 = fw["par.parProj.false_E"], fw["par.parProj.false_N"], 100.0
        self.assertEqual(float(np.spacing(np.float32(fn))), 0.0625)
        tm = KruegerTM(38.0 * DEG, 127.0 * DEG)

        def sweep():
            # blh straight from the projection: the PHYSICAL point is TM northing `north`, which
            # Site.blh (world metres relative to siteOrigin) cannot express when siteOrigin = 0.
            out = []
            for k in range(30):
                north = 0.005 * k
                fw["u.blh_Main"] = list(tm.inverse(0.0, north)) + [h0]
                fw["u.blh_Aux"] = list(tm.inverse(self.d_aux[0], north + self.d_aux[1])) + [h0]
                self.h.tick()
                out.append((north, float(self.enh("main")[1]), fw["y.machHeading"]))
            return out

        self.site(38.0, 127.0, false_e=fe, false_n=fn, site_origin=[0.0, 0.0, h0])
        coarse = sweep()
        self.assertTrue(all((n_out / 0.0625).is_integer() for _, n_out, _ in coarse))
        self.assertLessEqual(len({n_out for _, n_out, _ in coarse}), 4)       # 145 mm of travel
        headings = [hd for _, _, hd in coarse]
        self.assertGreater((max(headings) - min(headings)) / DEG, 1.0)

        self.site(38.0, 127.0, false_e=fe, false_n=fn, site_origin=[fe, fn, h0])
        for north, n_out, hd in sweep():
            self.assertAlmostEqual(n_out, north, delta=1e-5)
            self.assertAlmostEqual(hd, math.pi / 2, delta=1e-5)


# ==========================================================================================
class TestFixQuality(GnssTestBase):
    """chart_2496 ChkVerticalAccuracy and its independence from Localization. Each assertion
    reads both the raw state fw.internal('low_vertical_accuracy') and the delayed inhibit. The
    whole inhibit word is compared (0 or 0x2), so another bit cannot pass for poor accuracy."""

    def _boot(self, std, m_main=4, m_aux=4, ticks=50):
        # A valid site and a real fix, so the gate is not tested on top of the frozen pose of
        # test_geodetic_inports_left_at_zero_freeze_the_pose_silently.
        self.site(32.9, -96.8, 180.0)
        self.place([0.0, 0.0, 0.0], Rz(0.2))
        self.h.pulse("u.isSwingAligned")
        self.h.set_target_panel()
        self.h.gnss_rtk_fixed(std)                           # the Isaac publisher's healthy fix ...
        self.fw["u.methodGnss_Main"] = m_main                # ... then degrade one antenna if asked
        self.fw["u.methodGnss_Aux"] = m_aux
        self.h.tick(ticks)
        return self.h.auto_inhibited()

    def assertGate(self, low, inhibited, msg=""):
        self.assertEqual(bool(self.fw.internal("low_vertical_accuracy")), low, f"state {msg} {self.h.describe()}")
        self.assertEqual(self.h.inhibit_status(), 0x2 if inhibited else 0, f"inhibit {msg} {self.h.describe()}")

    def test_degraded_fix_does_not_stop_the_pose_updating(self):
        # WHY: spec A3.3 "Localization ignores fix quality" -- a float/single fix still moves the
        # believed machine every tick; only auto is inhibited, 20 ticks later, in another chart.
        # Firmware: methodGnss_* are declared in the chart_2222 signature but not passed to
        # MdlApp_Localization (call MdlApp.c:41996-42020); on-delay counter >= 20 :39370-39385,
        # CntAccuracyChkDly = 20 (SysPar.m:57).
        self.assertFalse(self._boot(0.008), self.h.describe())
        self.assertGate(low=False, inhibited=False)

        # !isRtkFixed alone (sigma still good): state flips at once, inhibit on the 20th tick.
        self.fw["u.methodGnss_Main"] = 5
        for k in range(1, 26):
            self.h.tick()
            self.assertGate(low=True, inhibited=k >= 20, msg=f"RTK float k={k}")
        self.fw["u.methodGnss_Main"] = 4
        self.h.tick()
        self.assertGate(low=False, inhibited=False, msg="recovered")

        # Everything degraded while the machine moves: the pose keeps tracking.
        R_tilt = Rx(1 * DEG) @ Ry(4 * DEG)
        self.fw["u.methodGnss_Main"] = 1                     # single point
        self.fw["u.methodGnss_Aux"] = 0                      # no fix
        self.fw["u.gnssPosStdDevZ"] = 3.0
        for k in range(1, 31):
            main = np.array([0.1 * k, 0.05 * k, 0.0])
            yaw = 0.2 + 0.01 * k
            R = Rz(yaw) @ R_tilt
            self.attitude(R)
            self.place(main, R)
            self.h.tick()
            self.assertVecClose(self.enh("main"), main, f32_tol(*main), f"k={k}")
            self.assertVecClose(self.chs_p(), main + R @ self.d_chs, f32_tol(*main), f"k={k}")
            self.assertAngleClose(self.fw["y.machHeading"], math.pi / 2 - yaw, 1e-5)
            self.assertGate(low=True, inhibited=k >= 20, msg=f"k={k}")

    def test_accuracy_band_between_thresholds_holds_the_previous_state(self):
        # WHY: SIL noise models must know the gate has hysteresis and starts "poor": a sigma
        # between 0.02 and 0.04 m never arms auto from boot but never drops a running one.
        # Firmware: init lowVerticalAccuracyState = true MdlApp.c:38340; good if std < good &&
        # RTK, poor if std > poor || !RTK, else hold :39092-39110 (chart_2496).
        self.assertTrue(self._boot(0.03), "boot in band: stays poor")
        self.assertGate(low=True, inhibited=True)
        self.h.reset().nominal_inputs()
        self.assertFalse(self._boot(0.008), "boot good")
        self.fw["u.gnssPosStdDevZ"] = 0.03
        self.h.tick(500)
        self.assertGate(low=False, inhibited=False, msg="good -> band: stays good")
        self.fw["u.gnssPosStdDevZ"] = 0.04                   # == poor threshold: '>' is strict
        self.h.tick(100)
        self.assertGate(low=False, inhibited=False, msg="== poor threshold is not poor")
        self.fw["u.gnssPosStdDevZ"] = 0.041
        self.h.tick(19)
        self.assertGate(low=True, inhibited=False, msg="poor, inside the on-delay")
        self.h.tick(1)
        self.assertGate(low=True, inhibited=True, msg="poor after the 20-tick on-delay")
        self.fw["u.gnssPosStdDevZ"] = 0.03
        self.h.tick(100)
        self.assertGate(low=True, inhibited=True, msg="poor -> band: stays poor")
        self.fw["u.gnssPosStdDevZ"] = 0.008
        self.h.tick(1)
        self.assertGate(low=False, inhibited=False, msg="recovery has no off-delay")

    def test_good_threshold_is_strict_and_rtk_float_or_nonfinite_sigma_is_poor(self):
        # WHY: pins the exact predicates a GNSS noise model will sit on. sigma == good threshold
        # is not good (strict '<'); either antenna not RTK fixed (4) is poor; NaN/Inf sigma is poor.
        # Firmware: MdlApp.c:39082-39106 (chart_2496 lines 12-19). The thresholds are real32 INPORTS
        # (ECU copies them from INTP, AppCtrlIf.c:651-652; nominal_inputs() writes the model defaults
        # SysPar.m:59-60), so 0.02 == 0.02 exactly and the strictness test is not a float artefact.
        cases = (("sigma == good threshold", dict(std=0.02), True),
                 ("sigma just below good", dict(std=0.019), False),
                 ("main RTK float (5)", dict(std=0.008, m_main=5), True),
                 ("aux RTK float (5)", dict(std=0.008, m_aux=5), True),
                 ("NaN sigma", dict(std=float("nan")), True),
                 ("Inf sigma", dict(std=float("inf")), True))
        for name, kw, poor in cases:
            with self.subTest(name):
                self.h.reset().nominal_inputs()
                self.assertEqual(self._boot(**kw), poor, self.h.describe())
                self.assertGate(low=poor, inhibited=poor)


if __name__ == "__main__":
    unittest.main()
