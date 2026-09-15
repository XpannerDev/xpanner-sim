"""
IMU quaternion -> joint angle chain (X1Exc ImuToLink + KinematicsCalc), adjudicated by the
running firmware instead of by reading the source.

Citations "MdlApp.c:N" mean X1Exc/Asw/GeneratedCode/MdlApp_ert_rtw/MdlApp.c line N;
"chart_NNNN" is the Stateflow XML of the same model.

WHAT THE FIRMWARE DOES (and what each test pins)
  ImuToLink (MdlApp.c:10878-11106), once per IMU. In THIS local-test build the mount matrix is
  parLocalTest.imu* ('<S1>/Switch7..12' folded onto '<S1>/Constant6' = parLocalTest,
  MdlApp.c:836-837, :41866-41975); the ECU's u.*Imu_MntOriStored (filled from NVM,
  AppCtrlIf.c:565ff) is not read by this build.
      q_fw   = (w, -x, y, -z)                                          :10910-10915
      imuOri = R(q_fw), scalar-first, body->world, NEVER normalised    :10917-10926
      linkOri = imuOri * mntOri   => mntOri = R_imu<-link              :10940
      link.angVel = mntOri' * (-wx, wy, -wz)                           :10966
      link.accRaw = (-ax, ay, -az), IMU axes, calibration only         :11102-11105
      euAng = Euler 312: R = Rz(e3) Rx(e1) Ry(e2)                      :11066-11074
  KinematicsCalc (MdlApp.c:11543-13700):
      jnt q  = difference of e2 (the Ry term) of two link frames       :11670-11684
      four-bar output: float32 Freudenstein half-angle form            :11145-11190, :11777-11800
      tilt   = atan2 of R1'*Rz(drift)'*R2                              :12187-12376
      q passes a seeded, angle-aware 3 Hz first-order LPF              :12425 (call), :11302, :11327
      link FK: R_child = R_parent*Ry(q), p_child = p_parent + R_parent*t   :12464-12508
      joint rate = axis' * (R_p' R_c w_child - w_parent)  (gyro only)  :13141-13152

MODEL USED HERE
  sil.kinematics IS the model: link_frames() composes the link world orientations (house ->
  boom about Y -> arm about Y -> input link about Y; tilt link through the four-bar),
  publish_imus() writes R_world<-imu = R_world<-link @ mntOri.T as a pre-mirrored unit
  quaternion plus gyro and accelerometer. These are the functions nominal_inputs(), every
  scenario and the Isaac plant share, so each test here also tests that one publisher.
  Boom/arm/input-link attitudes are plain Ry products, independent of any firmware code path.
  The tilt-link attitude uses kin.fourbar_output(), a port of the firmware formula; TestFourBar
  checks it against the firmware and against an independent circle-intersection closure.
  Where a test needs a frame the library deliberately cannot publish (untransposed mount, a
  board on the wrong link, a reversed axis, a URDF literal) it writes the quaternion itself
  through publish_raw() and says so.

WHAT ZERO AND + MEAN (TestPhysicalSignConvention; X forward, Y left, Z up, ENH world)
  boom  BmMntToBm1   0 = boom pin -> arm pin chord parallel to house X;   - = arm pin UP
  arm   Bm2ToArm     0 = arm chord collinear with boom chord;             + = arm IN (under)
  link  ArmToInpLink 0 = input link parallel to arm chord;                + = curl IN
  outp  ArmToOutpLink same zero/sense, solved from the input link
  These are exactly the URDF boom/arm/input_link/bucket joint definitions (axis 0 1 0, origin
  rpy 0, distal pin on +X), so URDF angle == firmware angle, no offset, no sign flip.
  Physical bounds the firmware's own cylinder geometry imposes in this ShortArm build: arm
  q > +5.6 deg, input link q < +14.2 deg (dead centres, located from y.cyls.*.strk); the arm-end
  IK never commands boom q > 0 (chart_1011), but the measurement is not clamped.

SETTLING
  LPF1st_JntAngs is 3 Hz (tau 53 ms, alpha 0.1586 per 10 ms tick) and seeds itself from its first
  sample, so the first tick after reset() is exact. Scenarios that change a pose mid-run use
  SETTLE_S = 2 s (~37 tau); single-pose scans use reset() + one tick.

Run:  cd xpanner-sim && python3 -m unittest sil.tests.test_imu_kinematics -v
"""
import math
import unittest
from pathlib import Path

import numpy as np

from sil import kinematics as kin
from sil.firmware import REPO
from sil.harness import Harness

DEG = math.pi / 180.0
SETTLE_S = 2.0
ANG_TOL = 0.002 * DEG          # float32 Euler round trip; observed <= 1e-4 deg
URDF_XACRO = REPO / "assets" / "ecr88" / "urdf" / "ecr88.urdf.xacro"
UP = np.array([0.0, 0.0, 1.0])

Y_JOINTS = ("BmMntToBm1", "Bm2ToArm", "ArmToInpLink")
URDF_IMU_PORT = {"imu_chs": "chs", "imu_boom": "bm1", "imu_arm": "arm", "imu_link": "bkt", "imu_tilt": "tilt"}


# ----------------------------------------------------------------------------------------
# local helpers -- only what sil.kinematics does not (and should not) provide
# ----------------------------------------------------------------------------------------
def x1exc_data(fw):
    """The same parameter directory Harness.load_imu_mounts() resolves names against."""
    return Path(fw.manifest["x1exc_dir"]) / "ControlModel" / "Data"


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def quat_to_rot(q):
    """Inverse of kin.rot_to_quat. The publisher never needs it; used only to state the mirror
    precondition."""
    w, x, y, z = q
    return np.array([[w * w + x * x - y * y - z * z, 2 * (x * y - w * z), 2 * (w * y + x * z)],
                     [2 * (x * y + w * z), w * w - x * x + y * y - z * z, 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (w * x + y * z), w * w - x * x - y * y + z * z]])


def euler312(R):
    """(e1 about X, e2 about Y, e3 about Z) with R = Rz(e3) Rx(e1) Ry(e2). Ground-truth comparator."""
    return (math.asin(R[2, 1]), math.atan2(-R[2, 0], R[2, 2]), math.atan2(-R[0, 1], R[1, 1]))


def euler321(R):
    """URDF/ROS convention R = Rz(e3) Ry(e2) Rx(e1); what a naive comparator would use."""
    return (math.atan2(R[2, 1], R[2, 2]), math.asin(-R[2, 0]), math.atan2(R[1, 0], R[0, 0]))


def fwmat(v9):
    """Firmware 3x3 outputs (y.links.*.R) are column-major (MdlApp.c:10940 loop indices)."""
    return np.array(v9, dtype=float).reshape(3, 3, order="F")


_URDF_IMU_CACHE = {}


def urdf_imu_mounts(variant):
    """imu_mount name -> (parent link, [r, p, y]) from the asset EXPANDED for one machine_variant.
    Since f2a3923 the rpy are per-variant xacro properties (ecr88_params.xacro branches), so the
    xacro source no longer carries literals; expanding is the only faithful read. No default:
    which unit's frames a test uses is the point of the test, so every caller names it."""
    if variant not in _URDF_IMU_CACHE:
        import subprocess
        import xml.etree.ElementTree as ET
        xml = subprocess.run(["xacro", str(URDF_XACRO), f"machine_variant:={variant}"],
                             capture_output=True, text=True, check=True).stdout
        out = {}
        for j in ET.fromstring(xml).findall("joint"):
            child = j.find("child").get("link")
            if child in URDF_IMU_PORT:
                out[child] = (j.find("parent").get("link"),
                              [float(x) for x in j.find("origin").get("rpy").split()])
        _URDF_IMU_CACHE[variant] = out
    return _URDF_IMU_CACHE[variant]


def firmware_readout(fw, seen):
    """What the firmware reports for the link attitudes it SEES (linkOri per port), written from
    the source formulas: e2 differences (MdlApp.c:11670-11684), four-bar (:11777-11800, via
    kin.fourbar_output), tilt drift removal (:12187-12376). Kept here, not in sil.kinematics:
    a correct plant never needs it; it exists to PREDICT the error of a wrong publisher.
    `seen` must be orthonormal: the firmware rebuilds R1/R2 from Euler angles (:12188, :12257),
    so a non-unit quaternion enters only through its Euler angles (see euler312_seen).
    The drift division by g mirrors :12321-12322 line for line; it scales both atan2 arguments
    and cancels, so it changes nothing unless g is exactly 0."""
    E = {k: euler312(R) for k, R in seen.items()}
    qb = wrap(E["bm1"][1] - E["chs"][1])
    qa = wrap(E["arm"][1] - E["bm1"][1])
    qi = wrap(E["bkt"][1] - E["arm"][1])
    qo = kin.fourbar_output(fw, qi)
    off = fw["par.parKin.angOutpLinkToTiltMnt"] + fw["par.parKin.angTiltMntToTilt"]
    c, t = E["chs"], E["tilt"]
    R1 = kin.Rz(c[2]) @ kin.Rx(c[0]) @ kin.Ry(c[1] + qb + qa + qo + off)
    R2 = kin.Rz(t[2]) @ kin.Rx(t[0]) @ kin.Ry(t[1])
    g = R1[0, 0] ** 2 + R1[1, 0] ** 2
    drift = math.atan2((R1[0, 0] * R2[1, 0] - R1[1, 0] * R2[0, 0]) / g,
                       (R1[0, 0] * R2[0, 0] + R1[1, 0] * R2[1, 0]) / g)
    Rt = R1.T @ kin.Rz(drift).T @ R2
    return {"BmMntToBm1": qb, "Bm2ToArm": qa, "ArmToInpLink": qi, "ArmToOutpLink": qo,
            "TiltMntToTilt": math.atan2(Rt[2, 1], Rt[1, 1])}


def fourbar_terms(fw, q_inp):
    """(kA, kB, kC, det) of the firmware's Freudenstein form, same algebra as kin.fourbar_output
    (MdlApp.c:11145-11190). Only used to state the singularity preconditions."""
    k = lambda n: fw[f"par.parKin.{n}"]
    a, b, c, d = k("lenInpLink"), k("lenConnRod"), k("lenOutpLink"), k("lenGndLink")
    ang = q_inp - k("angArmToGndLink")
    kA = -2 * a * c * math.sin(ang)
    kB = 2 * c * (d - a * math.cos(ang))
    kC = a * a - b * b + c * c + d * d - 2 * a * d * math.cos(ang)
    return kA, kB, kC, kA * kA + kB * kB - kC * kC


def fourbar_singular_q_inp(fw, root=-1):
    """Input-link angle where kC = kB: cos(angInp) = (2cd - a^2 + b^2 - c^2 - d^2) / (2a(c - d)).
    There det = kA^2, so the half-angle numerator -kA + sqrt(det) = -kA + |kA|. On the root
    returned by default (root=-1, angInp < 0) kA = -2ac sin(angInp) > 0, the numerator is 0 too
    and the firmware's branch is 0/0. On the other root (root=+1) kA < 0, the numerator is 2|kA|
    and atan2 is well defined."""
    k = lambda n: fw[f"par.parKin.{n}"]
    a, b, c, d = k("lenInpLink"), k("lenConnRod"), k("lenOutpLink"), k("lenGndLink")
    cos_inp = (2 * c * d - a * a + b * b - c * c - d * d) / (2 * a * (c - d))
    return root * math.acos(cos_inp) + k("angArmToGndLink")


def euler312_seen(R_link, norm_sq):
    """The orthonormal attitude the firmware's tilt solve works with when the IMU quaternion has
    squared norm norm_sq: R = |q|^2 R_link, e1 = asin(R32) (MdlApp.c:10917-10925, :11066) shrinks,
    e2 and e3 are atan2 of ratios (:11069, :11074) and survive, and R1/R2 are rebuilt from these
    three angles (:12188, :12257)."""
    _, e2, e3 = euler312(R_link)
    return kin.Rz(e3) @ kin.Rx(math.asin(norm_sq * R_link[2, 1])) @ kin.Ry(e2)


def discrete_outputs(fw):
    """Every non-float y.* output (flags, enums, status words)."""
    out = {}
    for p in fw.paths("y."):
        v = fw[p]
        if isinstance(v, float) or (isinstance(v, list) and v and isinstance(v[0], float)):
            continue
        out[p] = v
    return out


# ----------------------------------------------------------------------------------------
class ImuCase(unittest.TestCase):
    def setUp(self):
        self.h = Harness().reset().nominal_inputs()
        self.fw = self.h.fw

    def pose(self, R_chs=None, q_bm1=0.0, q_arm=0.0, q_inp=0.0, q_tilt=0.0, rates=None, settle=True):
        """Shared publisher (Harness.set_pose -> kin.link_frames + kin.publish_imus)."""
        frames = self.h.set_pose(R_chs, q_bm1, q_arm, q_inp, q_tilt, rates=rates)
        if settle:
            self.settle()
        return frames

    def publish_raw(self, port, R_imu_world):
        """Write one IMU attitude the library would never produce (mutations, URDF literals)."""
        self.fw[f"u.{port}ImuQuat"] = kin.mirror(kin.rot_to_quat(R_imu_world))

    def settle(self):
        self.h.run_seconds(SETTLE_S)

    def q(self, joint):
        return self.fw[f"y.jnts.{joint}.q"]

    def qdot(self, joint):
        return self.fw[f"y.jnts.{joint}.qDot"]

    def assertAngle(self, got, want, tol=ANG_TOL, what=""):
        # got must be reported in (-pi, pi]: the error below is 2*pi-blind, so a missing WrapToPi
        # upstream would otherwise pass. float32 pi is 8.7e-8 above math.pi.
        self.assertLessEqual(abs(got), math.pi + 1e-6, f"{what}: {got} rad is outside +-pi")
        err = wrap(got - want)
        self.assertLessEqual(abs(err), tol, f"{what}: firmware {math.degrees(got):.5f} deg, "
                                            f"model {math.degrees(want):.5f} deg")


# ========================================================================================
class TestMountMatrices(ImuCase):

    def test_compiled_mounts_are_rotations_that_can_discriminate_a_transpose(self):
        # WHY: the mntOri vs mntOri' check (TestJointAnglesFromImus) is only meaningful if
        # M*M != I; an involutive mount would pass either way and decide nothing about the URDF
        # rpy. Source: mounts read from parLocalTest.imu* (MdlApp.c:41866-41975).
        for name, Mm in kin.mounts_from_fw(self.fw).items():
            self.assertAlmostEqual(np.linalg.det(Mm), 1.0, places=5, msg=name)
            self.assertLess(np.abs(Mm.T @ Mm - np.eye(3)).max(), 1e-6, name)
            self.assertGreater(np.abs(Mm @ Mm - np.eye(3)).max(), 1.0, f"{name} is ~an involution")
        # imuBm2 is not in kin.MOUNT_NAMES because the sensor does not exist (lenBm2 = 0): the
        # slot is the identity.
        bm2 = np.array([[self.fw[f"par.imuBm2.a{i}{j}"] for j in (1, 2, 3)] for i in (1, 2, 3)])
        self.assertTrue(np.allclose(bm2, np.eye(3)))

    def test_mounts_come_from_parLocalTest_in_this_local_test_build(self):
        # WHY: a SIL scenario that "applies a calibration" must patch par.imu*; writing the
        # NVM-style inports u.*Imu_MntOriStored does nothing in THIS build, so a scenario driven
        # through them would silently test nothing. This is a build configuration, not a
        # property of the production ECU: every ImuToLink call passes parLocalTest.imu*.aij
        # because '<S1>/Switch7..12' were folded onto '<S1>/Constant6' = parLocalTest
        # (MdlApp.c:836-837, :41866-41975), whereas the ECU fills the Stored inports from NVM
        # (AppCtrlIf.c:565ff).
        self.pose(kin.Rz(0.3), -40 * DEG, 90 * DEG, -30 * DEG)
        before = [self.q(j) for j in Y_JOINTS]
        stored = [p for p in self.fw.paths("u.") if "_MntOriStored.a" in p]
        self.assertGreaterEqual(len(stored), 45, "precondition: the Stored inports exist")
        for p in stored:
            self.fw[p] = 0.37
        self.settle()
        self.assertEqual(before, [self.q(j) for j in Y_JOINTS])

        # Positive control: the same size of change through par.imuBm1 is seen exactly.
        # linkOri = R_imu @ (M @ Ry(1 deg)) = R_link @ Ry(1 deg)  =>  boom q + 1 deg.
        M = kin.mounts_from_fw(self.fw)["imuBm1"]
        kin.write_mounts(self.fw, {"imuBm1": M @ kin.Ry(1 * DEG)})
        self.settle()
        self.assertAngle(self.q("BmMntToBm1"), before[0] + 1 * DEG, what="par.imuBm1 patched")

    def test_mirror_is_a_180deg_rotation_about_y_so_the_publisher_premirrors(self):
        # WHY: tells the Isaac publisher exactly what to emit. The source comment says
        # "Left handed to right handed" (MdlApp.c:10907) but (w,-x,y,-z) is conjugation by
        # Ry(180 deg): a PROPER rotation of world and body frames, no handedness change. So raw
        # IMU frames are the firmware frames turned 180 deg about Y (a Z-down world).
        # PRECONDITION (pure Python, no firmware): kin.mirror really is that conjugation.
        rng = np.random.default_rng(7)
        for _ in range(5):
            R = kin.rpy_to_R(*rng.uniform(-math.pi, math.pi, 3))
            conj = kin.Ry(math.pi) @ R @ kin.Ry(math.pi).T
            self.assertTrue(np.allclose(quat_to_rot(kin.mirror(kin.rot_to_quat(R))), conj, atol=1e-12),
                            "precondition")
            v = rng.normal(size=3)
            self.assertTrue(np.allclose(kin.mirror(list(v)), kin.Ry(math.pi) @ v, atol=1e-12), "precondition")

        # FIRMWARE: the mirror is applied on input (MdlApp.c:10910-10915), so a publisher that
        # skips the pre-mirror misreads every Y joint by degrees.
        want = (-45 * DEG, 100 * DEG, -50 * DEG)
        frames = self.pose(kin.rpy_to_R(4 * DEG, -3 * DEG, 1.1), *want)
        for j, w in zip(Y_JOINTS, want):
            self.assertAngle(self.q(j), w, what=f"{j} pre-mirrored")
        M = kin.mounts_from_fw(self.fw)
        for port in ("chs", "bm1", "arm", "bkt"):
            self.fw[f"u.{port}ImuQuat"] = list(kin.rot_to_quat(frames[port] @ M[kin.PORT_MOUNT[port]].T))
        self.settle()
        worst = max(abs(wrap(self.q(j) - w)) for j, w in zip(Y_JOINTS, want))
        self.assertGreater(worst, 5 * DEG, "firmware accepted an un-mirrored quaternion")

    def test_quaternion_sign_is_irrelevant_but_its_norm_moves_chassis_roll_and_tilt(self):
        # WHY: decides whether the Isaac publisher must renormalise (CAN quantisation leaves
        # |q| != 1). The firmware never normalises: r11 = qw^2+qx^2-qy^2-qz^2 (MdlApp.c:10917),
        # so R scales by |q|^2. The Y joints survive (e2 = atan2 of a ratio, :11069) but
        # e1 = asin(|q|^2 R32) (:11066) does not. Tilt moves ONLY through that e1: the tilt solve
        # rebuilds R1 from the chassis Euler angles and R2 from the tilt link's (:12188, :12257),
        # both orthonormal, so a short chassis quaternion shifts R1 and a short tilt quaternion
        # shifts R2. (The division in the drift estimate, :12321-12322, scales both atan2
        # arguments and cancels; it is not a norm path.) Asserted by predicting the firmware's
        # tilt from the shrunken e1 alone (euler312_seen + firmware_readout).
        # A sign flip leaves every product of two components unchanged.
        R_chs = kin.rpy_to_R(8 * DEG, 6 * DEG, 40 * DEG)
        frames = self.pose(R_chs, -40 * DEG, 100 * DEG, -30 * DEG, 12 * DEG)
        self.assertAlmostEqual(np.linalg.norm(self.fw["u.chsImuQuat"]), 1.0, places=6,
                               msg="precondition: kin.publish_imus writes unit quaternions")

        def snapshot():
            return ([self.q(j) for j in Y_JOINTS], self.fw["y.chs.euAng"], self.q("TiltMntToTilt"),
                    self.fw["y.links.chs.R"])
        ref = snapshot()
        published = {p: self.fw[f"u.{p}ImuQuat"] for p in kin.PORT_MOUNT}

        for p, qq in published.items():
            self.fw[f"u.{p}ImuQuat"] = [-c for c in qq]
        self.settle()
        self.assertEqual(snapshot(), ref, "sign flip must be bit-identical everywhere")

        self.fw["u.chsImuQuat"] = [0.98 * c for c in published["chs"]]
        self.settle()
        for j, r in zip(Y_JOINTS, ref[0]):
            self.assertAngle(self.q(j), r, what=f"{j} with a 2 % short chassis quaternion")
        roll = self.fw["y.chs.euAng"][0]
        self.assertAlmostEqual(roll, math.asin(0.98 ** 2 * math.sin(ref[1][0])), delta=2e-6,
                               msg="chassis roll = asin(|q|^2 sin(roll))")
        self.assertGreater(ref[1][0] - roll, 0.25 * DEG)                       # observed 0.317 deg
        self.assertAngle(self.fw["y.chs.euAng"][1], ref[1][1], what="chassis pitch")
        self.assertGreater(abs(self.q("TiltMntToTilt") - ref[2]), 0.3 * DEG)   # observed 0.509 deg

        def norm_sq(port):          # as the firmware holds it (float32 inport)
            return sum(c * c for c in self.fw[f"u.{port}ImuQuat"])

        def predicted_tilt(short_port):
            seen = dict(frames)
            seen[short_port] = euler312_seen(frames[short_port], norm_sq(short_port))
            return firmware_readout(self.fw, seen)["TiltMntToTilt"]
        # observed 7e-5 deg; an unchanged-e1 prediction would be off by the whole 0.509 deg
        self.assertAngle(self.q("TiltMntToTilt"), predicted_tilt("chs"), tol=0.001 * DEG,
                         what="tilt with a short chassis quaternion, predicted from chassis e1 alone")

        self.fw["u.chsImuQuat"] = published["chs"]
        self.fw["u.tiltImuQuat"] = [0.98 * c for c in published["tilt"]]
        self.settle()
        self.assertGreater(abs(self.q("TiltMntToTilt") - ref[2]), 0.01 * DEG)  # observed 0.022 deg
        self.assertAngle(self.q("TiltMntToTilt"), predicted_tilt("tilt"), tol=0.001 * DEG,
                         what="tilt with a short tilt quaternion, predicted from tilt-link e1 alone")


# ========================================================================================
class TestJointAnglesFromImus(ImuCase):

    def test_nominal_inputs_publish_the_documented_rest_pose(self):
        # WHY: every other area's scenario starts from Harness.nominal_inputs(); its IMUs must be
        # the physical rest pose the harness documents ("level house, boom -40, arm 90, input
        # link -60, tilt 0 deg", harness.py module docstring), not the identity quaternions it
        # used to write. The firmware readings are compared with those LITERAL degrees, so a
        # change to Harness.NOMINAL_POSE that the docs do not follow fails here too.
        # Seeded LPF (MdlApp.c:11302): one tick is exact.
        documented_deg = {"q_bm1": -40.0, "q_arm": 90.0, "q_inp": -60.0, "q_tilt": 0.0}
        self.assertEqual({k: round(math.degrees(v), 9) for k, v in Harness.NOMINAL_POSE.items()}, documented_deg)
        self.h.tick()
        for j, key in zip(Y_JOINTS, ("q_bm1", "q_arm", "q_inp")):
            self.assertAngle(self.q(j), documented_deg[key] * DEG, what=j)
            self.assertEqual(self.qdot(j), 0.0, j)
        self.assertAngle(self.q("TiltMntToTilt"), documented_deg["q_tilt"] * DEG, tol=0.005 * DEG, what="tilt")
        self.assertEqual(self.q("Bm1ToBm2"), 0.0)
        roll, pitch, _ = self.fw["y.chs.euAng"]
        self.assertAngle(roll, 0.0, what="chassis roll")
        self.assertAngle(pitch, 0.0, what="chassis pitch")

        # Contrast: identity quaternions through the compiled mounts are a contorted pose
        # (observed boom +71.0, tilt -107.0 deg), which is why nominal_inputs() changed.
        self.h.reset().nominal_inputs()
        for p in kin.PORT_MOUNT:
            self.fw[f"u.{p}ImuQuat"] = [1.0, 0.0, 0.0, 0.0]
        self.h.tick()
        self.assertGreater(self.q("BmMntToBm1"), 60 * DEG)
        self.assertLess(self.q("TiltMntToTilt"), -90 * DEG)

    def test_transposed_mount_reproduces_boom_arm_input_link_angles(self):
        # WHY: the URDF B2.3 correction adjudicated by the firmware: an IMU frame at rpy(mntOri')
        # on its link, published as-is, gives back the URDF joint angle. Source: linkOri =
        # imuOri*mntOri (MdlApp.c:10940); q = e2 differences (MdlApp.c:11670-11684).
        for q in ((-35, 95, -40), (-69, 155, -120), (-31, 31, 10), (0, 60, -80)):
            with self.subTest(pose_deg=q):
                self.h.reset().nominal_inputs()
                self.pose(kin.Rz(0.3), *(a * DEG for a in q))
                for j, want in zip(Y_JOINTS, q):
                    self.assertAngle(self.q(j), want * DEG, what=j)
                self.assertAngle(self.q("TiltMntToTilt"), 0.0, tol=0.005 * DEG, what="tilt")
                self.assertEqual(self.q("Bm1ToBm2"), 0.0)

    def test_untransposed_mount_does_not(self):
        # WHY: proves the check above discriminates. Publishing R_link @ mntOri (the pre-
        # 2026-09-14 URDF rpy(M)) gives the firmware R_link @ M @ M, and every Y joint is off
        # by tens of degrees. Local mutation of kin.publish_imus. Source: MdlApp.c:10940.
        want = (-35 * DEG, 95 * DEG, -40 * DEG)
        frames = self.pose(kin.Rz(0.3), *want, settle=False)
        M = kin.mounts_from_fw(self.fw)
        for port in ("chs", "bm1", "arm", "bkt"):
            self.publish_raw(port, frames[port] @ M[kin.PORT_MOUNT[port]])
        self.settle()
        for j, w in zip(Y_JOINTS, want):
            self.assertGreater(abs(math.degrees(wrap(self.q(j) - w))), 5.0,
                               f"{j} still matches under the untransposed mount")

    def test_urdf_imu_frames_follow_each_variants_unit_and_kijang_matches_this_binary(self):
        # WHY: contract between the Isaac asset and the firmware. Mount matrices are per-unit
        # calibration results, so since f2a3923 each machine_variant's IMU rpy are rpy(M') of ITS
        # unit's parameter file: the 2.1 m variants -> ECR88D_LongArm.m, ECR88_KIJANG (1.7 m) ->
        # ECR88D_ShortArm.m, which is what this binary is compiled with (SysPar.m:4). So a 1.7 m
        # SIL run needs NO mount patching; a 2.1 m run needs load_imu_mounts('ECR88D_LongArm.m').
        # Also B2.2: the bktImu frame hangs from input_link, because MdlApp.c:41945-41948 feeds
        # bktImu into imuLink, whose angle is the Freudenstein INPUT (MdlApp.c:11777).
        fw = self.fw
        data = x1exc_data(fw)
        param_sets = {f.name: kin.mounts_from_param_file(f) for f in sorted(data.glob("ECR88D_*.m"))}

        def matched_sets(mounts):
            return [name for name, mset in param_sets.items()
                    if all(np.abs(kin.rpy_to_R(*mounts[u][1]) - mset[kin.PORT_MOUNT[p]].T).max() < 1e-6
                           for u, p in URDF_IMU_PORT.items())]

        expect_parent = {"imu_chs": "house_link", "imu_boom": "boom_link", "imu_arm": "arm_link",
                         "imu_link": "input_link", "imu_tilt": "tilt_link"}
        for variant, unit in (("ECR88_US1_2P1M", "ECR88D_LongArm.m"),
                              ("ECR88_US1_2P1M_NEWSUCTION", "ECR88D_LongArm.m"),
                              ("ECR88_KIJANG", "ECR88D_ShortArm.m")):
            with self.subTest(machine_variant=variant):
                self.assertEqual(matched_sets(urdf_imu_mounts(variant)), [unit])
                self.assertEqual({k: v[0] for k, v in urdf_imu_mounts(variant).items()}, expect_parent)
        self.assertEqual(kin.identify_mount_source(fw, data), "ECR88D_ShortArm.m",
                         "compiled par.imu* source changed")

        # The 1.7 m asset against this binary, compiled mounts untouched.
        mounts = urdf_imu_mounts("ECR88_KIJANG")
        for u, p in URDF_IMU_PORT.items():     # and not the old untransposed rpy(M) either
            self.assertGreater(np.abs(kin.rpy_to_R(*mounts[u][1])
                                      - param_sets["ECR88D_ShortArm.m"][kin.PORT_MOUNT[p]]).max(), 0.5)

        qb, qa, qi, qt = -50 * DEG, 110 * DEG, -60 * DEG, 8 * DEG
        link = kin.link_frames(fw, kin.rpy_to_R(3 * DEG, -4 * DEG, 2.0), qb, qa, qi, qt)
        for name, (_, rpy) in mounts.items():
            self.publish_raw(URDF_IMU_PORT[name], link[URDF_IMU_PORT[name]] @ kin.rpy_to_R(*rpy))
        self.settle()
        for j, want in zip(Y_JOINTS + ("TiltMntToTilt",), (qb, qa, qi, qt)):
            self.assertAngle(self.q(j), want, what=f"{j} [ECR88_KIJANG frames, compiled ShortArm mounts]")
        inhibit = self.h.inhibit_status()

        # Negative control for B2.2: the same board bolted to the OUTPUT link. The firmware takes
        # it for the input link and re-solves the four-bar, so the tool attitude it believes is
        # off by tens of degrees (observed 50.6), and no inhibit bit changes.
        q_outp = kin.fourbar_output(fw, qi)
        self.publish_raw("bkt", link["arm"] @ kin.Ry(q_outp) @ kin.rpy_to_R(*mounts["imu_link"][1]))
        self.settle()
        self.assertGreater(abs(wrap(self.q("ArmToOutpLink") - q_outp)), 10 * DEG)
        self.assertEqual(self.h.inhibit_status(), inhibit)

    def test_foreign_mount_set_is_predictable_and_costs_degrees(self):
        # WHY: what running the 2.1 m asset (machine_variant ECR88_US1_2P1M, LongArm IMU frames)
        # against this ShortArm binary WITHOUT load_imu_mounts does -- the deliberate cross-unit
        # case. The firmware sees linkOri = R_link @ M_urdf' @ M_compiled (MdlApp.c:10940);
        # firmware_readout() predicts every reported joint from that, and the firmware must match
        # the prediction -- so the error is explained, not just "large".
        # Observed worst over the sweep: boom 0.17, arm 3.8, input link 4.7, output link 13.3
        # (four-bar gain near q_inp 0), tilt 15.8 deg (arm 100 deg, tilt axis near vertical).
        # No fault is raised. The Isaac IMU frames must come from the same parameter file as the
        # mounts the firmware runs with: positive control at the end, same frames after
        # h.load_imu_mounts('ECR88D_LongArm.m').
        fw = self.fw
        mounts = urdf_imu_mounts("ECR88_US1_2P1M")
        M = kin.mounts_from_fw(fw)
        self.assertGreater(max(np.abs(kin.rpy_to_R(*mounts[u][1]) - M[kin.PORT_MOUNT[p]].T).max()
                               for u, p in URDF_IMU_PORT.items()), 1e-3,
                           "precondition: the US frames are not the compiled unit's")
        worst = dict.fromkeys(Y_JOINTS + ("ArmToOutpLink", "TiltMntToTilt"), 0.0)
        for pose in ((-69, 31, -120, 0), (-50, 90, -60, 12), (-31, 155, 0, -20), (-40, 100, -30, 30)):
            with self.subTest(pose_deg=pose):
                self.h.reset().nominal_inputs()
                qb, qa, qi, qt = (a * DEG for a in pose)
                link = kin.link_frames(fw, kin.rpy_to_R(3 * DEG, -4 * DEG, 2.0), qb, qa, qi, qt)
                seen = {}
                for name, (_, rpy) in mounts.items():
                    port = URDF_IMU_PORT[name]
                    R_imu = link[port] @ kin.rpy_to_R(*rpy)
                    self.publish_raw(port, R_imu)
                    seen[port] = R_imu @ M[kin.PORT_MOUNT[port]]
                self.settle()
                predicted = firmware_readout(fw, seen)
                truth = {"BmMntToBm1": qb, "Bm2ToArm": qa, "ArmToInpLink": qi,
                         "ArmToOutpLink": kin.fourbar_output(fw, qi), "TiltMntToTilt": qt}
                for j in worst:
                    self.assertAngle(self.q(j), predicted[j], tol=0.001 * DEG, what=f"{j} predicted")
                    worst[j] = max(worst[j], abs(math.degrees(wrap(self.q(j) - truth[j]))))
        self.assertGreater(worst["BmMntToBm1"], 0.1)
        self.assertGreater(worst["Bm2ToArm"], 3.0)
        self.assertGreater(worst["ArmToInpLink"], 4.0)
        self.assertGreater(worst["ArmToOutpLink"], 10.0)
        self.assertGreater(worst["TiltMntToTilt"], 10.0)

        # Positive control: the same US frames with the matching unit's mounts loaded.
        qb, qa, qi, qt = (a * DEG for a in (-40, 100, -30, 30))
        self.h.reset().nominal_inputs().load_imu_mounts("ECR88D_LongArm.m")
        link = kin.link_frames(fw, kin.rpy_to_R(3 * DEG, -4 * DEG, 2.0), qb, qa, qi, qt)
        for name, (_, rpy) in mounts.items():
            self.publish_raw(URDF_IMU_PORT[name], link[URDF_IMU_PORT[name]] @ kin.rpy_to_R(*rpy))
        self.settle()
        for j, want in zip(Y_JOINTS + ("TiltMntToTilt",), (qb, qa, qi, qt)):
            self.assertAngle(self.q(j), want, what=f"{j} [ECR88_US1_2P1M frames, LongArm mounts loaded]")

    def test_bm2_zero_quaternion_and_garbage_are_harmless(self):
        # WHY: the real ECU sends zeros on the bm2 slot (PrePostProc_If.c:824-833); an all-zero
        # quaternion makes a zero "rotation" matrix, which must not leak into any joint.
        # Source: if ~hasBm2, imus.bm2 = imus.bm1 (MdlApp.c:11654-11659), par hasBm2 = false.
        self.assertFalse(self.fw["par.parKin.hasBm2"])
        qb = -40 * DEG
        rates = {"chs": [0, 0, 0.1], "bm1": list(kin.Ry(-qb) @ [0, 0, 0.1] + [0, 0.2, 0])}
        self.pose(kin.rpy_to_R(5 * DEG, 7 * DEG, -0.4), qb, 100 * DEG, -30 * DEG, rates=rates, settle=False)
        self.fw["u.bm2ImuQuat"] = [0.0, 0.0, 0.0, 0.0]
        self.fw["u.bm2ImuAngRate"] = [0.0, 0.0, 0.0]
        self.settle()
        names = ("BmMntToBm1", "Bm1ToBm2", "Bm2ToArm", "ArmToInpLink", "ArmToOutpLink")
        zeros = [(self.q(j), self.qdot(j)) for j in names]
        self.assertEqual(self.q("Bm1ToBm2"), 0.0)
        self.assertAlmostEqual(self.qdot("Bm1ToBm2"), 0.0, delta=1e-6)   # float32 R'R ~ I
        self.assertTrue(all(math.isfinite(a) and math.isfinite(b) for a, b in zeros))

        self.fw["u.bm2ImuQuat"] = [0.3, 0.5, -0.2, 0.7]
        self.fw["u.bm2ImuAngRate"] = [1.0, -2.0, 3.0]
        self.settle()
        self.assertEqual(zeros, [(self.q(j), self.qdot(j)) for j in names])

        # Positive control: with hasBm2 set, the same garbage reaches the joints.
        self.fw["par.parKin.hasBm2"] = 1
        self.settle()
        self.assertGreater(abs(self.q("Bm1ToBm2")), 10 * DEG)

    def test_chassis_roll_pitch_uses_euler_312_not_321(self):
        # WHY: CalibChs runs on the dozer jack-up pose (pitch > 5 deg) and SIL comparators need
        # the roll/pitch the firmware believes. 312 puts Ry innermost, so R_chs*Ry(q) keeps e2
        # additive (MdlApp.c:1787 rtCP_pooled60 = 312, :11066-11074).
        R_chs = kin.rpy_to_R(8 * DEG, 6 * DEG, 40 * DEG)          # URDF rpy, i.e. 321
        want = (-40 * DEG, 100 * DEG, -30 * DEG)
        frames = self.pose(R_chs, *want)

        # PRECONDITIONS (pure Python): this pose separates the two sequences.
        e321 = {k: euler321(R)[1] for k, R in frames.items()}
        naive = (e321["bm1"] - e321["chs"], e321["arm"] - e321["bm1"], e321["bkt"] - e321["arm"])
        self.assertGreater(max(abs(n - w) for n, w in zip(naive, want)), 1.0 * DEG, "precondition")
        e1, e2, e3 = euler312(R_chs)
        self.assertGreater(abs(e1 - euler321(R_chs)[0]), 0.02 * DEG, "precondition")

        # FIRMWARE: joints exact, and the chassis Euler output IS the 312 pair.
        for j, w in zip(Y_JOINTS, want):
            self.assertAngle(self.q(j), w, what=j)
        roll, pitch, _ = self.fw["y.chs.euAng"]
        self.assertAngle(roll, e1, tol=0.0005 * DEG, what="y.chs.euAng roll (312)")
        self.assertAngle(pitch, e2, tol=0.0005 * DEG, what="y.chs.euAng pitch (312)")
        # links.chs.R = Rz(GNSS heading) * Rx(e1) * Ry(e2) (MdlApp.c:11815). With no GNSS the
        # heading term is atan2(0,0) = 0, so the output is the true attitude with its 312 yaw removed.
        self.assertEqual(self.fw["y.euAng_ChsEstm_z"], 0.0, "precondition: no GNSS heading")
        self.assertLess(np.abs(fwmat(self.fw["y.links.chs.R"]) - kin.Rz(-e3) @ R_chs).max(), 1e-5)
        self.assertLess(np.abs(fwmat(self.fw["y.links.bm1.R"]) - kin.Rz(-e3) @ frames["bm1"]).max(), 1e-5)

    def test_each_imu_heading_is_ignored(self):
        # WHY: the five IMUs are independent AHRS units with no shared heading reference; Isaac
        # need not (and the real sensors cannot) agree on world yaw. Rz(d)*Rz(e3)Rx Ry leaves e2
        # untouched, so an arbitrary per-IMU heading must not move any joint -- including tilt,
        # whose world-Z drift is removed explicitly (MdlApp.c:12321-12328).
        want = (-40 * DEG, 100 * DEG, -30 * DEG)
        frames = self.pose(kin.rpy_to_R(8 * DEG, 6 * DEG, 40 * DEG), *want, 12 * DEG)
        joints = Y_JOINTS + ("TiltMntToTilt",)
        ref = [self.q(j) for j in joints]
        drift = {"chs": 0.7, "bm1": -1.1, "arm": 2.0, "bkt": -2.5, "tilt": 0.4}
        kin.publish_imus(self.fw, {k: kin.Rz(drift[k]) @ R for k, R in frames.items()})
        self.settle()
        for j, r in zip(joints, ref):
            self.assertAngle(self.q(j), r, tol=0.005 * DEG, what=j)
        self.assertAngle(self.q("TiltMntToTilt"), 12 * DEG, tol=0.005 * DEG, what="tilt")

    def test_joint_angle_lpf_is_3hz_and_seeded_by_first_sample(self):
        # WHY: a closed-loop SIL sees the joint angle through this lag (53 ms). Seeding from the
        # first sample means no start-up transient from 0. Source: MdlApp.c:12425
        # MdlApp_LPF1st_JntAngs(jntAngFilt, 0.01F, 3.0F, false) (MATLAB comment :12410),
        # :11302 (yPrev = u on first call), :11327-11328 (alpha = Ts/(tau+Ts)).
        # The one-tick expectations rely on Harness.tick() writing ExtU BEFORE MdlApp_step() and
        # on the IMU inports having no unit delay: a pose written now is sampled by this step.
        self.pose(None, -40 * DEG, 50 * DEG, 20 * DEG, settle=False)
        self.h.tick()
        self.assertAngle(self.q("BmMntToBm1"), -40 * DEG, what="first tick after reset()")
        self.h.tick(99)
        self.pose(None, -30 * DEG, 60 * DEG, 30 * DEG, settle=False)
        self.h.tick()
        alpha = 0.01 / (1.0 / (2 * math.pi * 3.0) + 0.01)
        # 0.01 deg separates Ts/(tau+Ts) = 0.1586 from 1-exp(-Ts/tau) = 0.1717 (0.13 deg here).
        self.assertAngle(self.q("BmMntToBm1"), (-40 + alpha * 10) * DEG, tol=0.01 * DEG,
                         what="one tick after a 10 deg step")
        self.h.run_seconds(1.0)
        self.assertAngle(self.q("BmMntToBm1"), -30 * DEG, what="after 1 s")


# ========================================================================================
class TestFourBar(ImuCase):
    """The input link is the only sensed four-bar member; the tool attitude the firmware
    controls comes from this solve (MdlApp.c:11777-11800, CalcAngLinkOutp :11145-11190)."""

    def read_outp(self, q_inp, R_chs=None, q_bm1=-40 * DEG, q_arm=90 * DEG):
        """reset() + one tick: the seeded LPF makes the first sample exact."""
        self.h.reset().nominal_inputs()
        self.pose(R_chs, q_bm1, q_arm, q_inp, settle=False)
        self.h.tick()
        return self.q("ArmToOutpLink"), self.q("ArmToInpLink")

    def test_library_fourbar_output_matches_the_firmware_away_from_its_singularity(self):
        # WHY: kin.fourbar_output builds the tilt-link attitude for every published pose
        # (Harness.set_pose, the Isaac plant's reference); it must be the firmware's branch and
        # formula. Compared at the input angle the firmware itself read, so only the four-bar is
        # judged. The 3 deg band around the half-angle singularity is excluded -- see
        # test_firmware_four_bar_solve_breaks_down_near_its_half_angle_singularity.
        q_sing = fourbar_singular_q_inp(self.fw)
        worst = 0.0
        for qi_deg in range(-180, 180, 5):
            if abs(wrap(qi_deg * DEG - q_sing)) < 3 * DEG:
                continue
            got, q_in_read = self.read_outp(qi_deg * DEG)
            worst = max(worst, abs(wrap(got - kin.fourbar_output(self.fw, q_in_read))))
        self.assertLess(math.degrees(worst), 0.002)          # observed 0.00015 deg

    def test_four_bar_output_matches_independent_loop_closure(self):
        # WHY: Isaac closes the loop from PIN coordinates, so it and the firmware (Freudenstein,
        # branch 2*atan2(-kA + sqrt(det), kC - kB) at MdlApp.c:11186) must agree on the branch
        # and to within the parameter inconsistency: the firmware uses lenGndLink 0.2597 /
        # angArmToGndLink 4.1000 deg, while the pins distArmToInpLink=(1.441,0,0.0185),
        # lenArm=1.7 give 0.25966 / 4.0856 deg. No firmware code in this model.
        fw = self.fw
        A = np.array(fw["par.parKin.distArmToInpLink"], dtype=float)
        D = np.array([fw["par.parKin.lenArm"], 0.0, 0.0])
        a, b, c = fw["par.parKin.lenInpLink"], fw["par.parKin.lenConnRod"], fw["par.parKin.lenOutpLink"]

        def closures(q_inp):
            # planar coords (x, -z): an Ry-sense angle is a CCW angle there.
            B = np.array([A[0] + a * math.cos(q_inp), -A[2] + a * math.sin(q_inp)])
            d2 = np.array([D[0], -D[2]])
            r = np.linalg.norm(B - d2)
            along = (c * c - b * b + r * r) / (2 * r)
            h = math.sqrt(max(0.0, c * c - along * along))
            e = (B - d2) / r
            n = np.array([-e[1], e[0]])
            return [math.atan2(*(along * e + s * h * n)[::-1]) for s in (1.0, -1.0)]

        prev = None
        for qi_deg in (-120, -90, -60, -40, -20, 0, 10):
            with self.subTest(q_inp_deg=qi_deg):
                got, _ = self.read_outp(qi_deg * DEG, R_chs=kin.Rz(-0.2))
                near, far = closures(qi_deg * DEG)
                self.assertAngle(got, near, tol=0.05 * DEG, what="branch 1")
                self.assertGreater(abs(wrap(got - far)), 10 * DEG)
                if prev is not None:
                    self.assertGreater(got, prev, "output link must turn the same way as input")
                prev = got

    def test_firmware_reports_the_ground_link_angle_when_the_loop_cannot_close(self):
        # WHY: pins what the firmware outputs for an impossible four-bar, which is easy to get
        # wrong (the kin.fourbar_output docstring once said "the firmware would output 0"; it now
        # states the behaviour pinned here). CalcAngLinkOutp does set angOutpLink = 0 when det < 0
        # (MdlApp.c:11177-11182), but KinematicsCalc then ADDS angArmToGndLink
        # (MdlApp.c:11789-11791), so y.jnts.ArmToOutpLink reads 4.1 deg with no fault. None (and
        # link_frames raising) is the right library behaviour for a plant.
        fw = self.fw
        # PRECONDITION: with the compiled lengths the loop closes at every input angle (Grashof
        # double crank: the ground link is shortest), so this branch needs a bad parameter set.
        self.assertTrue(all(kin.fourbar_output(fw, a * DEG) is not None for a in range(-180, 181)))
        for qi_deg in (-90, 0, 90):
            with self.subTest(q_inp_deg=qi_deg):
                self.h.reset().nominal_inputs()
                fw["par.parKin.lenConnRod"] = 0.05
                self.assertIsNone(kin.fourbar_output(fw, qi_deg * DEG))
                with self.assertRaises(ValueError):
                    kin.link_frames(fw, None, -40 * DEG, 90 * DEG, qi_deg * DEG)
                R_arm = kin.Ry(-40 * DEG) @ kin.Ry(90 * DEG)
                kin.publish_imus(fw, {"chs": np.eye(3), "bm1": kin.Ry(-40 * DEG), "arm": R_arm,
                                      "bkt": R_arm @ kin.Ry(qi_deg * DEG)})
                self.h.tick()
                self.assertAlmostEqual(self.q("ArmToOutpLink"), fw["par.parKin.angArmToGndLink"], places=7)
                self.assertNotEqual(self.q("ArmToOutpLink"), 0.0)

    def test_firmware_four_bar_solve_breaks_down_near_its_half_angle_singularity(self):
        # FIRMWARE DEFECT (numerical). The half-angle form 2*atan2(-kA + sqrt(det), kC - kB)
        # (MdlApp.c:11186) is 0/0 on the firmware's own branch where kC = kB, i.e. input link
        # q = -107.087 deg for these lengths -- inside the URDF input_link_joint range
        # (-126..+43 deg). The true output is continuous there, but float32 cancellation in both
        # arguments makes y.jnts.ArmToOutpLink wrong by >1 deg at every sampled input within
        # +-0.0002 deg (observed 2.8 .. 92.8 deg), with no guard or flag, and the LPF passes it
        # through when the machine dwells there. Further out the error is float32 rounding
        # scatter, not a clean band. A 120-sample firmware scan per shell gave maxima of 4.8 deg at
        # 0.0002-0.001, 1.5 at 0.001-0.003, 0.57 at 0.003-0.005, 0.38 at 0.005-0.01, 0.19 at
        # 0.01-0.03, 0.07 at 0.03-0.1 and 0.0015 deg at 1-3 deg from the singular angle. The
        # ">0.1 deg" part is pinned below on the 0.003-0.01 deg shell. The tool pose jumps with
        # it: links.contactSurface.p moves 2.1 m at the centre. A conditioned form (e.g. atan2 of
        # the two circle-intersection coordinates) would not have this.
        fw = self.fw
        q_sing = fourbar_singular_q_inp(fw)
        self.assertAlmostEqual(math.degrees(q_sing), -107.087, delta=0.001, msg="precondition")
        # PRECONDITIONS (pure Python, the fourbar_singular_q_inp docstring): kC = kB, kA > 0, so the
        # numerator -kA + sqrt(det) vanishes too; the other kC = kB root is not singular.
        kA, kB, kC, det = fourbar_terms(fw, q_sing)
        self.assertLess(abs(kC - kB), 1e-12, "precondition")
        self.assertGreater(kA, 0.1, "precondition: kA > 0 at the singular root")   # 0.2585
        self.assertLess(abs(-kA + math.sqrt(det)), 1e-12, "precondition")
        kA2, kB2, kC2, det2 = fourbar_terms(fw, fourbar_singular_q_inp(fw, root=+1))
        self.assertLess(abs(kC2 - kB2), 1e-12, "precondition")
        self.assertGreater(-kA2 + math.sqrt(det2), 0.1, "precondition: other root not singular")

        def err_at(dq_deg):
            got, q_in_read = self.read_outp(q_sing + dq_deg * DEG)
            ref = kin.fourbar_output(fw, q_in_read)       # float64: well conditioned here
            return abs(math.degrees(wrap(got - ref))), ref, np.array(fw["y.links.contactSurface.p"])

        near = [err_at(dq) for dq in np.linspace(-2e-4, 2e-4, 41)]
        refs = [r for _, r, _ in near]
        self.assertLess(math.degrees(max(refs) - min(refs)), 0.01, "precondition: true output is continuous")
        self.assertGreater(min(e for e, _, _ in near), 1.0)
        self.assertGreater(max(e for e, _, _ in near), 45.0)
        shell = [err_at(s * dq)[0] for dq in np.linspace(0.003, 0.01, 15) for s in (-1.0, 1.0)]
        self.assertGreater(max(shell), 0.1)                   # observed 0.35 deg
        far = [err_at(dq)[0] for dq in (-3.0, -2.0, 2.0, 3.0)]
        self.assertLess(max(far), 0.002)

        # Consequence for the tool: contact surface position at the centre vs 0.05 deg either side.
        centre = err_at(0.0)[2]
        sides = (err_at(-0.05)[2] + err_at(0.05)[2]) / 2
        self.assertGreater(np.linalg.norm(centre - sides), 0.5)       # metres; observed 2.1


# ========================================================================================
class TestTilt(ImuCase):

    def test_tilt_angle_through_transposed_mount(self):
        # WHY: imuTilt is a full axis permutation (link X ~ sensor -Z), the mount most exposed
        # to a transpose mistake. Tilt = atan2 of R1'*Rz(drift)'*R2 (MdlApp.c:12187-12376). The
        # tilt-link attitude comes from kin.link_frames (four-bar checked in TestFourBar).
        for rpy, qt in (((0, 0, 0.5), 12 * DEG), ((8 * DEG, 6 * DEG, 40 * DEG), -20 * DEG)):
            with self.subTest(chassis_rpy=rpy, tilt_deg=math.degrees(qt)):
                self.h.reset().nominal_inputs()
                frames = self.pose(kin.rpy_to_R(*rpy), -40 * DEG, 40 * DEG, -30 * DEG, qt)
                self.assertAngle(self.q("TiltMntToTilt"), qt, tol=0.005 * DEG, what="tilt")
                M = kin.mounts_from_fw(self.fw)["imuTilt"]
                self.publish_raw("tilt", frames["tilt"] @ M)         # local mutation: no transpose
                self.settle()
                self.assertGreater(abs(self.q("TiltMntToTilt") - qt), 5 * DEG)

    def test_tilt_error_is_amplified_when_the_tilt_axis_nears_vertical(self):
        # WHY: SIL comparators must not treat tilt as uniformly accurate. The firmware removes a
        # world-Z "drift" = the angle between the horizontal projections of the tilt-mount (R1)
        # and tilt-link (R2) X columns, i.e. of the tilt axis (MdlApp.c:12321-12323), then takes
        # tilt from R1' Rz(drift)' R2 (:12328, :12376). Mechanism: an attitude error of delta with
        # a component `perp` out of the vertical plane through the tilt axis swings the axis'
        # horizontal projection, of length cos(elevation), by delta*perp/cos(elev) in azimuth; all
        # of that is removed as drift, and a world-Z rotation projects sin(elev) of itself onto
        # the tilt axis. Tilt error ~ delta * perp * tan(elev), asserted below. It is an
        # observability limit (rotation about a vertical axis looks like heading drift), not a
        # numerical one: the source's division by R1(1,1)^2 + R1(2,1)^2 scales both atan2
        # arguments and cancels, so its missing guard would matter only at exactly 0 (0/0).
        # What the firmware lacks is a flag. On a level chassis the injected error stays in the
        # vertical plane (perp = 0) and cancels; with roll it does not. The injected error is a
        # 0.1 deg rotation about the tilt-mount Y axis applied to the published tilt-LINK attitude
        # (equivalent to a board/mount error of that rotation).
        delta_deg = 0.1
        bad = kin.Ry(delta_deg * DEG)

        def tilt_error(rpy, q_arm, with_error=True):
            self.h.reset().nominal_inputs()
            frames = self.pose(kin.rpy_to_R(*rpy), -40 * DEG, q_arm, -30 * DEG, 0.0, settle=False)
            R_mnt = frames["tilt"]                               # tilt link at q_tilt = 0 = tilt mount
            frames["tilt"] = R_mnt @ (bad if with_error else np.eye(3)) @ kin.Rx(12 * DEG)
            kin.publish_imus(self.fw, frames)
            self.settle()
            err = abs(math.degrees(self.q("TiltMntToTilt")) - 12.0)
            axis, moved = R_mnt[:, 0], R_mnt[:, 2]               # Ry(delta) moves X toward -Z
            elev = math.asin(abs(axis[2]))
            n = np.cross(UP, axis)
            perp = abs(moved @ n) / np.linalg.norm(n)
            law = delta_deg * perp * math.tan(elev)
            return err, math.degrees(elev), law, discrete_outputs(self.fw)

        err_flat_tool, elev_lo, law_lo, flags_lo = tilt_error((8 * DEG, 6 * DEG, 40 * DEG), 40 * DEG)
        err_vert_tool, elev_hi, law_hi, flags_hi = tilt_error((8 * DEG, 6 * DEG, 40 * DEG), 100 * DEG)
        err_clean, _, _, flags_clean = tilt_error((8 * DEG, 6 * DEG, 40 * DEG), 100 * DEG, with_error=False)
        err_level, _, law_level, _ = tilt_error((0, 0, 40 * DEG), 100 * DEG)
        self.assertLess(elev_lo, 35.0)             # observed 27.9 deg
        self.assertGreater(elev_hi, 75.0)          # observed 81.9 deg
        self.assertLess(err_clean, 0.001)          # the pose itself is fine without the error
        self.assertLess(err_flat_tool, 0.02)       # observed 0.008 deg
        self.assertGreater(err_vert_tool, 0.5)     # observed 0.68 deg: ~7x the injected error
        self.assertLess(err_level, 0.002)
        # The tan law explains each case (observed 0.0083/0.0083, 0.6833/0.6816, 0/0 deg).
        self.assertAlmostEqual(err_flat_tool, law_lo, delta=0.0005)
        self.assertAlmostEqual(err_vert_tool, law_hi, delta=0.01 * law_hi)
        self.assertLess(law_level, 1e-6, "precondition: level chassis keeps the error in plane")
        # No flag, enum or status output distinguishes the ill-conditioned case (idle machine).
        # The snapshot must not be empty, or the equalities below would hold vacuously.
        self.assertGreater(len(flags_hi), 30)      # 42 today
        self.assertEqual(flags_hi, flags_clean)
        self.assertEqual(flags_hi, flags_lo)


# ========================================================================================
class TestPhysicalSignConvention(ImuCase):
    """What zero and + mean. Every Y joint uses R_child = R_parent*Ry(q) with the distal pin at
    +len along child X (MdlApp.c:12464-12508), and the world is ENH, Z = height."""

    def _pose(self, qb, qa, qi=-30 * DEG):
        self.h.reset().nominal_inputs()
        self.pose(None, qb, qa, qi, settle=False)
        self.h.tick()                                          # seeded LPF: exact

    def test_negative_boom_angle_raises_the_arm_pin(self):
        # WHY: decides the URDF boom axis sign. Olivia's "-31 boom down .. -69 max" has the same
        # sense. PREMISE, not firmware behaviour: the firmware world is Z-up. The support for it
        # is physical -- the main GNSS antenna is on the cab roof and par distAntMainToChs puts
        # the chassis origin 1.345 m below it; links.chs.p (MdlApp.c:11851) merely applies that,
        # so the first two assertions only confirm the parameter reaches the output.
        # FIRMWARE: Ry(q)*[L,0,0] has z = -L sin q (PropagateRp, MdlApp.c:12470).
        # The anchor that does NOT rest on Z-up is the boom-cylinder stroke law in
        # test_cylinder_strokes_confirm_the_signs_and_the_dead_centres (the boom cylinder
        # lengthens as q decreases, and a boom cylinder extends to raise the boom).
        L = self.fw["par.parKin.lenBm1"]
        self._pose(0.0, 90 * DEG)
        self.assertAlmostEqual(self.fw["y.links.chs.p"][2], self.fw["par.parKin.distAntMainToChs"][2], places=5)
        self.assertLess(self.fw["y.links.chs.p"][2], 0.0, "premise: chassis origin below the antenna")
        for qb in (-69 * DEG, -31 * DEG, 0.0, 11.53 * DEG):
            with self.subTest(boom_deg=math.degrees(qb)):
                self._pose(qb, 90 * DEG)
                rise = self.fw["y.links.bm2.p"][2] - self.fw["y.links.bm1.p"][2]
                self.assertAlmostEqual(rise, -L * math.sin(qb), places=4)
        self._pose(-31 * DEG, 90 * DEG)
        self.assertGreater(self.fw["y.links.bm2.p"][2] - self.fw["y.links.bm1.p"][2], 1.8)

    def test_joint_sign_is_set_by_the_link_frames_not_by_the_world_frame(self):
        # WHY: tells Isaac which convention actually decides "+". e2 differences are invariant to
        # any pre-rotation of the world (Rx(pi) world: e2 -> e2 + pi, cancelled in the difference
        # and re-wrapped by the angle-aware LPF, MdlApp.c:11332-11338 WrapToPiVec -- assertAngle
        # also checks the raw value is inside +-pi), so an upside-down world still yields the
        # right q -- but links.chs.R and every FK height flip, which is what the controller
        # consumes (MdlApp.c:11815-11851). Reversing the joint axis of the BODY frames (every
        # frame * Rx(pi): Y right, Z down) negates every q. So: the world must be Z-up for FK,
        # and the sign is fixed by the link frames: X toward the distal pin, Y = joint axis
        # pointing to the machine's LEFT, Z = X x Y (ROS style).
        want = (-40 * DEG, 100 * DEG, -30 * DEG)
        base = kin.link_frames(self.fw, kin.rpy_to_R(0.0, -5 * DEG, 0.4), *want)

        kin.publish_imus(self.fw, {k: kin.Rx(math.pi) @ R for k, R in base.items()})   # Z-down world
        self.settle()
        for j, w in zip(Y_JOINTS, want):
            self.assertAngle(self.q(j), w, what=f"{j} in a Z-down world")
        self.assertLess(fwmat(self.fw["y.links.chs.R"])[2, 2], -0.9)
        self.assertLess(self.fw["y.links.bm2.p"][2] - self.fw["y.links.bm1.p"][2], -2.0)

        self.h.reset().nominal_inputs()
        kin.publish_imus(self.fw, {k: R @ kin.Rx(math.pi) for k, R in base.items()})   # axis reversed
        self.settle()
        for j, w in zip(Y_JOINTS, want):
            self.assertAngle(self.q(j), -w, what=f"{j} with link Y reversed")

    def test_positive_arm_angle_folds_the_arm_under_the_boom(self):
        # WHY: decides the URDF arm axis sign and zero. q = 0 is arm collinear with the boom
        # chord (boom pin -> bucket pin along boom X), + rotates the bucket pin to the boom's -Z
        # side, i.e. arm IN. Olivia's "31 arm-out .. 155 arm-in" grows the same way.
        # Source: links.outpLink = arm*Ry(q) with t = [lenArm,0,0] (MdlApp.c:12481-12496).
        L = self.fw["par.parKin.lenArm"]
        for qa in (31 * DEG, 90 * DEG, 155 * DEG):
            with self.subTest(arm_deg=math.degrees(qa)):
                self._pose(-40 * DEG, qa)
                R_bm1 = fwmat(self.fw["y.links.bm1.R"])
                d = R_bm1.T @ (np.array(self.fw["y.links.outpLink.p"]) - np.array(self.fw["y.links.arm.p"]))
                self.assertAlmostEqual(d[0], L * math.cos(qa), places=4)
                self.assertAlmostEqual(d[2], -L * math.sin(qa), places=4)
                self.assertLess(d[2], 0.0)

    def test_measured_boom_angle_is_not_clamped_to_nonpositive(self):
        # SPEC CONTRADICTED (B3.6 "Firmware: BmMntToBm1.q <= 0 always, Bm2ToArm.q >= 0 always").
        # The clamps exist only inside the arm-end IK target solver (chart_1011 lines 439-440
        # "qBm1Max = 0 % Boom angle is always negative", applied to q_Tar at 468-473 and qCand at
        # 512-518). The measurement path (MdlApp.c:11670-11684, LPF :12425) is unclamped.
        # WHY: Isaac must not clip reported angles, and any URDF boom range above 0 is space the
        # firmware will measure but never command during arm-end position control.
        self._pose(11.53 * DEG, -5 * DEG)
        self.assertAngle(self.q("BmMntToBm1"), 11.53 * DEG, what="boom below chord")
        self.assertAngle(self.q("Bm2ToArm"), -5 * DEG, what="arm past collinear")

    def test_cylinder_strokes_confirm_the_signs_and_the_dead_centres(self):
        # WHY: independent physical cross-check using Leica-measured pin geometry, not IMUs: the
        # boom cylinder (isCylBm1TopMnt = false) must EXTEND as the boom rises (q decreasing);
        # arm and bucket cylinders (top-mounted) extend as their joint curls in (q increasing).
        # The stroke extremum is a dead centre the hydraulics cannot drive through, so it bounds
        # the reachable range. Located here from the firmware's y.cyls.*.strk, then compared to
        # the closed form. Source: CalStrkAndSpd angIncluded = +/-(q + angCylSml - angCylLrg),
        # L^2 = a^2 + b^2 - 2ab cos(angIncluded) (MdlApp.c:11464-11490); callers :13453-13483
        # (bucket passes angCylSml = 0).
        fw = self.fw

        def strk(cyl, qb, qa, qi):
            self._pose(qb * DEG, qa * DEG, qi * DEG)
            return fw[f"y.cyls.{cyl}.strk"]

        boom = [strk("bm1", qb, 90, -30) for qb in (11.53, 0, -31, -69)]
        self.assertEqual(boom, sorted(boom), "boom cylinder must lengthen as boom q decreases")

        grid = [round(3.0 + 0.1 * i, 1) for i in range(51)]              # 3.0 .. 8.0 deg
        arm = {qa: strk("arm", -40, qa, -30) for qa in grid}
        dead_arm = min(arm, key=arm.get)
        self.assertLessEqual(abs(dead_arm - math.degrees(fw["par.parKin.angCylLrgArm"]
                                                         - fw["par.parKin.angCylSmlArm"])), 0.1)
        self.assertAlmostEqual(dead_arm, 5.6, delta=0.05, msg="the figure quoted in the module docstring")
        self.assertAlmostEqual(arm[dead_arm], fw["par.parKin.lenToCylLrgArm"] - fw["par.parKin.lenToCylSmlArm"],
                               places=5)
        a0, a31, a90, a155 = (strk("arm", -40, qa, -30) for qa in (0, 31, 90, 155))
        self.assertGreater(a0, arm[dead_arm])          # q = 0 is already past dead centre
        self.assertLess(arm[8.0], a31)
        self.assertLess(a31, a90)
        self.assertLess(a90, a155)

        grid = [round(11.5 + 0.1 * i, 1) for i in range(56)]             # 11.5 .. 17.0 deg
        bkt = {qi: strk("bkt", -40, 90, qi) for qi in grid}
        dead_link = max(bkt, key=bkt.get)
        self.assertLessEqual(abs(dead_link - math.degrees(math.pi + fw["par.parKin.angCylLrgBkt"])), 0.1)
        self.assertAlmostEqual(dead_link, 14.2, delta=0.05, msg="the figure quoted in the module docstring")
        self.assertAlmostEqual(bkt[dead_link], fw["par.parKin.lenToCylLrgBkt"] + fw["par.parKin.lenToCylSmlBkt"],
                               places=5)
        b150, b100, b60, b0, b20 = (strk("bkt", -40, 90, qi) for qi in (-150, -100, -60, 0, 20))
        self.assertLess(b150, b100)
        self.assertLess(b100, b60)
        self.assertLess(b60, b0)
        self.assertLess(b0, bkt[dead_link])
        self.assertGreater(bkt[dead_link], b20)        # past dead centre it shortens again


# ========================================================================================
class TestJointRatesFromGyros(ImuCase):

    POSE = (-40 * DEG, 100 * DEG, -30 * DEG)

    def test_joint_rates_are_gyro_differences(self):
        # WHY: spec A3.1 says rates come from gyros, not differentiated angles; the velocity loops
        # and cylinder speed (y.cyls.*.spd) depend on it. The chassis itself turns about all three
        # axes so the subtraction of the parent rate is exercised. kin.publish_imus writes
        # gyro = mirror(M @ w). Source: MdlApp.c:13141-13149, LPF1st_JntAngVels 3 Hz (:13336),
        # angVel = M'*gyro (:10966), output link rate = input rate * angVelRatio (:11814, :13148).
        fw = self.fw
        R_chs = kin.rpy_to_R(8 * DEG, 6 * DEG, 40 * DEG)
        qb, qa, qi = self.POSE
        qt = 12 * DEG
        q_outp = kin.fourbar_output(fw, qi)
        off = fw["par.parKin.angOutpLinkToTiltMnt"] + fw["par.parKin.angTiltMntToTilt"]
        w_chs = np.array([0.05, -0.1, 0.4])
        qbd, qad, qid, qtd = 0.2, -0.3, 0.5, -0.4
        eps = 1e-6
        ratio = (kin.fourbar_output(fw, qi + eps) - kin.fourbar_output(fw, qi - eps)) / (2 * eps)
        w_bm1 = kin.Ry(-qb) @ w_chs + [0, qbd, 0]
        w_arm = kin.Ry(-qa) @ w_bm1 + [0, qad, 0]
        w_inp = kin.Ry(-qi) @ w_arm + [0, qid, 0]
        # The output-link term is about the tilt-mount Y axis, orthogonal to the tilt axis X, so it
        # cannot change tilt qDot; it is there only to keep the published tilt gyro physical.
        w_tilt = kin.Rx(-qt) @ (kin.Ry(-(q_outp + off)) @ w_arm + [0, ratio * qid, 0]) + [qtd, 0, 0]
        self.pose(R_chs, qb, qa, qi, qt, rates={"chs": w_chs, "bm1": w_bm1, "arm": w_arm, "bkt": w_inp,
                                                 "tilt": w_tilt})
        for j, want in zip(Y_JOINTS, (qbd, qad, qid)):
            self.assertAlmostEqual(self.qdot(j), want, delta=1e-5, msg=j)        # observed 1e-8
        self.assertAlmostEqual(self.qdot("ArmToOutpLink"), ratio * qid, delta=1e-5)
        self.assertAlmostEqual(self.qdot("TiltMntToTilt"), qtd, delta=1e-5)
        # boom lowering (qDot > 0) retracts the boom cylinder; arm going out retracts the arm
        # cylinder; input link curling in extends the bucket cylinder.
        self.assertLess(fw["y.cyls.bm1.spd"], 0.0)
        self.assertLess(fw["y.cyls.arm.spd"], 0.0)
        self.assertGreater(fw["y.cyls.bkt.spd"], 0.0)

    def test_pure_house_rotation_gives_zero_front_joint_rates(self):
        # WHY: a swinging house carries the whole front; the firmware must not see boom/arm/link/
        # tilt motion from it. Also pins the swing-rate sign: ChsToUc.qDot = -(chassis body yaw
        # rate), MdlApp.c:13151-13152 (links.uc = chs*Rz(ChsToUc), :12754). So positive UcToChs
        # (house w.r.t. undercarriage) is counter-clockwise seen from above; positive ChsToUc is
        # clockwise.
        R_chs = kin.rpy_to_R(3 * DEG, -2 * DEG, 1.0)
        frames = kin.link_frames(self.fw, R_chs, *self.POSE, 10 * DEG)
        w_world = R_chs @ np.array([0.0, 0.0, 0.4])
        self.pose(R_chs, *self.POSE, 10 * DEG, rates={k: R.T @ w_world for k, R in frames.items()})
        for j in Y_JOINTS + ("ArmToOutpLink", "TiltMntToTilt"):
            self.assertAlmostEqual(self.qdot(j), 0.0, delta=1e-5, msg=j)         # observed 6e-8
        self.assertAlmostEqual(self.qdot("ChsToUc"), -0.4, delta=1e-5)

    def test_moving_attitude_with_zero_gyro_gives_zero_qdot(self):
        # WHY: spec A3.1 "zero gyros give perfect-looking angles and dead velocity loops". An
        # Isaac IMU publisher that forgets the gyro channel would pass every angle check.
        # Source: no q differentiation anywhere on the qDot path (MdlApp.c:13139-13165).
        qb0, qa, qi = self.POSE
        for k in range(200):
            self.pose(None, qb0 + 0.2 * k * 0.01, qa, qi, settle=False)     # rates=None -> gyro 0
            self.h.tick()
        self.assertGreater(math.degrees(self.q("BmMntToBm1") - qb0), 20.0)
        for j in Y_JOINTS:
            self.assertEqual(self.qdot(j), 0.0, j)

    def test_accelerometer_reaches_only_accRaw(self):
        # WHY: tells the publisher which channel is safety-relevant outside calibration. The
        # mirrored acc is stored as link.accRaw (MdlApp.c:11102-11105) and read only by
        # CalcImuMntOri during calibration steps (chart_2291). The acc channel is live (accRaw
        # follows the input), yet no angle, rate or chassis attitude output moves.
        fw = self.fw
        R_chs = kin.rpy_to_R(8 * DEG, 6 * DEG, 40 * DEG)
        self.pose(R_chs, *self.POSE, 12 * DEG, rates={"bm1": [0, 0.2, 0], "chs": [0, 0, 0.3]})

        # kin.publish_imus' accelerometer is consistent with its quaternion: the reading the firmware
        # stores, rotated out of the IMU axes, is ACC_SIGN * 1 g along up in chassis axes (ACC_SIGN = -1
        # since 2026-09-15: the sign under which calibration rebuilds the compiled mounts).
        M = kin.mounts_from_fw(fw)["imuChs"]
        self.assertLess(np.abs(M.T @ np.array(fw["y.chs.accRaw"]) - R_chs.T @ (kin.ACC_SIGN * UP)).max(), 1e-6)

        joints = Y_JOINTS + ("ArmToOutpLink", "TiltMntToTilt", "ChsToUc")

        def snapshot():
            return ([(self.q(j), self.qdot(j)) for j in joints], fw["y.links.chs.R"], fw["y.chs.euAng"])
        ref = snapshot()
        for port in kin.PORT_MOUNT:
            fw[f"u.{port}ImuAcc"] = [0.7, -1.9, 0.3]
        self.settle()
        self.assertEqual(fw["y.chs.accRaw"], [float(np.float32(v)) for v in kin.mirror([0.7, -1.9, 0.3])])
        self.assertEqual(snapshot(), ref)


if __name__ == "__main__":
    unittest.main()
