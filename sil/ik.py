"""
ik.py -- place the contact surface at a target position with its Z axis along a target
direction, using swing / boom / arm / bucket inside the URDF limits.

Boom, arm and bucket all turn about the house Y axis, so the tool's pitch is
c + boom + arm + bucket: fixing the pitch fixes the bucket, and the position is a 2-D
search over boom and arm. Swing is set so the tool lands on the target's bearing (the
boom is offset sideways from the swing centre). Tilt and rotator stay at 0, so any
out-of-plane component of the target direction remains as error, and is reported.

This is how assets/ecr88/cycle.json was solved; it lives here so scenarios (e.g. the
step-28 pose) are planned with the same code.
"""
import math

import numpy as np

JOINTS = ("swing_joint", "boom_joint", "arm_joint", "bucket_joint")


def _pitch(v):
    return math.degrees(math.atan2(v[0], v[2]))


def q_of(sw, bm, ar, bk):
    return dict(zip(JOINTS, map(math.radians, (sw, bm, ar, bk))))


def solve(model, target_p, target_z, tcp="contact_surface_link", step_deg=1.0, swing_deg=None):
    """Returns (dict swing/boom/arm/bucket in deg, position error m, direction error deg).
    swing_deg: hold swing fixed (e.g. 0 when the scenario also claims isSwingAligned) instead of
    turning the tool onto the target's bearing; the sideways miss then shows in the error."""
    target_p = np.asarray(target_p, float)
    target_z = np.asarray(target_z, float) / np.linalg.norm(target_z)
    B, A, K = (model.limits_deg(j) for j in ("boom_joint", "arm_joint", "bucket_joint"))
    c = _pitch(model.fk(tcp, q_of(0, 0, 0, 0))[:3, 2])
    y0 = model.fk(tcp, q_of(0, 0, 0, 0))[1, 3]            # sideways offset of the tool at swing 0
    r = math.hypot(target_p[0], target_p[1])
    sw = (math.degrees(math.atan2(target_p[1], target_p[0]) - math.asin(max(-1.0, min(1.0, y0 / max(r, 1e-9)))))
          if swing_deg is None else float(swing_deg))
    Rz = np.array([[math.cos(math.radians(sw)), -math.sin(math.radians(sw)), 0],
                   [math.sin(math.radians(sw)), math.cos(math.radians(sw)), 0], [0, 0, 1]])
    want = _pitch(Rz.T @ target_z)

    def err(bm, ar):
        bk = want - c - bm - ar
        if not K[0] <= bk <= K[1]:
            return None
        return np.linalg.norm(model.fk(tcp, q_of(sw, bm, ar, bk))[:3, 3] - target_p)

    best = (1e9, None)
    for bm in np.arange(B[0], B[1] + 1e-9, step_deg):
        for ar in np.arange(A[0], A[1] + 1e-9, step_deg):
            e = err(bm, ar)
            if e is not None and e < best[0]:
                best = (e, (bm, ar))
    if best[1] is None:
        return None
    bm, ar = best[1]
    step = step_deg / 2
    while step > 1e-3:
        moved = False
        for dbm, dar in ((step, 0), (-step, 0), (0, step), (0, -step)):
            b2, a2 = min(max(bm + dbm, B[0]), B[1]), min(max(ar + dar, A[0]), A[1])
            e = err(b2, a2)
            if e is not None and e < best[0] - 1e-12:
                best, bm, ar, moved = (e, (b2, a2)), b2, a2, True
        if not moved:
            step /= 2
    bk = want - c - bm - ar
    T = model.fk(tcp, q_of(sw, bm, ar, bk))
    derr = math.degrees(math.acos(max(-1.0, min(1.0, float(T[:3, 2] @ target_z)))))
    return dict(swing=sw, boom=bm, arm=ar, bucket=bk), best[0], derr
