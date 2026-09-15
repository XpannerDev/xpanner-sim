"""
kinematics.py -- IMU publishing that the X1Exc firmware accepts, and the firmware's own
link geometry, in plain numpy.

Everything here was first proven against the running firmware in
sil/tests/test_imu_kinematics.py (joint angles reproduced to 0.002 deg) and is lifted out
so the harness defaults, every scenario, and the Isaac Sim plant share ONE publisher.

CONVENTIONS (all established by that test file, citations there)
    World        Z-up. The firmware's FK heights invert in a Z-down world even though joint
                 angles still come out right.
    Link frames  X toward the distal pin, Y = pin axis to the machine's left, Z up.
                 Firmware joint angle == URDF joint angle (same zero, same sign) for boom,
                 arm and four-bar input link. Negative boom = arm pin UP; positive arm = arm IN.
    Mounts       par.imu<X> = M = R_imu<-link  (linkOri = imuOri * mntOri, MdlApp.c:10940).
                 So R_world<-imu = R_world<-link @ M.T.
    Quaternion   scalar-first body->world. The firmware mirrors inputs (w,x,y,z)->(w,-x,y,-z),
                 vectors (x,y,z)->(-x,y,-z) (MdlApp.c:10910-10915) -- a rotation by 180 deg
                 about Y, an involution -- so the publisher applies the same mirror first.
                 Renormalise: a 2 % short quaternion moves chassis roll by 0.3 deg and tilt by
                 0.5 deg (the firmware never normalises).
    Gyro         link body rate w (link coords): gyro = mirror(M @ w). Joint RATES come only
                 from gyro differences; zero gyro gives qDot == 0 while angles move.
    Accel        specific force in g, at rest +1 g along world up: acc = mirror(R_imu.T @ up).
                 Only calibration (CalcImuMntOri) reads it. The sign of the real sensor's
                 specific force is NOT established; vy of the identified mount is invariant
                 to it (verifier note on chart_2291), vx/vz are not.
    Euler        firmware uses 312 for chassis roll/pitch; a 321 comparator is off by >1 deg
                 at roll 8 / pitch 6.
"""
import math
import re
from pathlib import Path

import numpy as np

PORT_MOUNT = {"chs": "imuChs", "bm1": "imuBm1", "arm": "imuArm", "bkt": "imuLink", "tilt": "imuTilt"}
MOUNT_NAMES = ("imuChs", "imuBm1", "imuArm", "imuLink", "imuTilt")
UP = np.array([0.0, 0.0, 1.0])


def Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def rpy_to_R(r, p, y):
    """URDF fixed-axis XYZ: R = Rz(y) Ry(p) Rx(r)."""
    return Rz(y) @ Ry(p) @ Rx(r)


def rot_to_quat(R):
    """Scalar-first unit quaternion, Shepperd pivot (a naive sqrt-of-diagonal loses ~2e-4 rad,
    which shows up as 0.08-0.15 deg of spurious tilt)."""
    m = R
    t = [1 + m[0, 0] + m[1, 1] + m[2, 2], 1 + m[0, 0] - m[1, 1] - m[2, 2],
         1 - m[0, 0] + m[1, 1] - m[2, 2], 1 - m[0, 0] - m[1, 1] + m[2, 2]]
    k = int(np.argmax(t))
    r = math.sqrt(t[k])
    s = 0.5 / r
    if k == 0:
        q = [0.5 * r, (m[2, 1] - m[1, 2]) * s, (m[0, 2] - m[2, 0]) * s, (m[1, 0] - m[0, 1]) * s]
    elif k == 1:
        q = [(m[2, 1] - m[1, 2]) * s, 0.5 * r, (m[0, 1] + m[1, 0]) * s, (m[0, 2] + m[2, 0]) * s]
    elif k == 2:
        q = [(m[0, 2] - m[2, 0]) * s, (m[0, 1] + m[1, 0]) * s, 0.5 * r, (m[1, 2] + m[2, 1]) * s]
    else:
        q = [(m[1, 0] - m[0, 1]) * s, (m[0, 2] + m[2, 0]) * s, (m[1, 2] + m[2, 1]) * s, 0.5 * r]
    q = np.array(q)
    return q / np.linalg.norm(q)


def mirror(v):
    """The firmware's input mirror. 4-vector (w,x,y,z)->(w,-x,y,-z); 3-vector (x,y,z)->(-x,y,-z)."""
    v = list(v)
    if len(v) == 4:
        return [v[0], -v[1], v[2], -v[3]]
    return [-v[0], v[1], -v[2]]


# -- mount matrices --------------------------------------------------------------------------
def mounts_from_fw(fw):
    return {n: np.array([[fw[f"par.{n}.a{i}{j}"] for j in (1, 2, 3)] for i in (1, 2, 3)], dtype=float)
            for n in MOUNT_NAMES}


def mounts_from_param_file(path):
    """imu* 3x3 mounts from an X1Exc ControlModel/Data/ECR88D_*.m file."""
    src = Path(path).read_text(encoding="latin-1")
    out = {}
    for name in MOUNT_NAMES:
        i = src.index(f"'{name}',")
        pairs = re.findall(r"'a(\d\d)',\s+single\(([^)]+)\)", src[i:i + 2000])[:9]
        if [k for k, _ in pairs] != ["11", "12", "13", "21", "22", "23", "31", "32", "33"]:
            raise ValueError(f"{path}: cannot read {name}")
        out[name] = np.array([float(v) for _, v in pairs]).reshape(3, 3)
    return out


def write_mounts(fw, mounts):
    """Patch par.imu* (reset() restores the compiled set)."""
    for name, M in mounts.items():
        for i in range(3):
            for j in range(3):
                fw[f"par.{name}.a{i + 1}{j + 1}"] = float(M[i, j])


def identify_mount_source(fw, x1exc_data_dir, tol=1e-6):
    """Which parameter file the CURRENTLY LOADED par.imu* came from ('ECR88D_ShortArm.m', ...)
    or None. Mount matrices are per-unit calibration results: mixing a URDF built from one
    unit with a binary carrying another's mounts misreads arm/link by ~4 deg with no fault."""
    cur = mounts_from_fw(fw)
    for f in sorted(Path(x1exc_data_dir).glob("ECR88D_*.m")):
        try:
            ref = mounts_from_param_file(f)
        except (ValueError, OSError):
            continue
        if all(np.abs(cur[n] - ref[n]).max() < tol for n in MOUNT_NAMES):
            return f.name
    return None


# -- firmware link geometry -------------------------------------------------------------------
def fourbar_output(fw, q_inp):
    """ArmToOutpLink from ArmToInpLink, the firmware's own Freudenstein branch
    (MdlApp.c:11140-11200, :11777-11800). Returns None where the firmware would output 0
    because the loop cannot close (det < 0)."""
    k = lambda n: fw[f"par.parKin.{n}"]
    a, b, c, d = k("lenInpLink"), k("lenConnRod"), k("lenOutpLink"), k("lenGndLink")
    ang_inp = q_inp - k("angArmToGndLink")
    kA = -2 * a * c * math.sin(ang_inp)
    kB = 2 * c * (d - a * math.cos(ang_inp))
    kC = a * a - b * b + c * c + d * d - 2 * a * d * math.cos(ang_inp)
    det = kA * kA + kB * kB - kC * kC
    if det < 0:
        return None
    q = 2 * math.atan2(-kA + math.sqrt(det), kC - kB) + k("angArmToGndLink")
    return math.atan2(math.sin(q), math.cos(q))


def link_frames(fw, R_chs=None, q_bm1=0.0, q_arm=0.0, q_inp=0.0, q_tilt=0.0):
    """World orientation of each IMU-carrying link for a machine pose, in firmware geometry.
    Boom swing is pinned at 0 by the firmware (MdlApp.c:12411); hasBm2 = false."""
    R_chs = np.eye(3) if R_chs is None else np.asarray(R_chs, dtype=float)
    R_bm1 = R_chs @ Ry(q_bm1)
    R_arm = R_bm1 @ Ry(q_arm)
    q_outp = fourbar_output(fw, q_inp)
    if q_outp is None:
        raise ValueError(f"four-bar does not close at input link {math.degrees(q_inp):.2f} deg")
    off = fw["par.parKin.angOutpLinkToTiltMnt"] + fw["par.parKin.angTiltMntToTilt"]
    R_tilt = R_arm @ Ry(q_outp) @ Ry(off) @ Rx(q_tilt)       # MdlApp.c:12508, :12659
    return {"chs": R_chs, "bm1": R_bm1, "arm": R_arm, "bkt": R_arm @ Ry(q_inp), "tilt": R_tilt}


def publish_imus(fw, frames, rates=None, accel=True):
    """Write u.<port>ImuQuat (and AngRate / Acc) for world link orientations `frames`
    (port -> R_world<-link). rates: port -> link body angular velocity in link coordinates."""
    M = mounts_from_fw(fw)
    for port, R_link in frames.items():
        Mm = M[PORT_MOUNT[port]]
        R_imu = np.asarray(R_link) @ Mm.T
        fw[f"u.{port}ImuQuat"] = mirror(rot_to_quat(R_imu))
        w = np.zeros(3) if rates is None or port not in rates else np.asarray(rates[port], dtype=float)
        fw[f"u.{port}ImuAngRate"] = mirror(Mm @ w)
        if accel:
            fw[f"u.{port}ImuAcc"] = mirror(R_imu.T @ UP)
