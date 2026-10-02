"""
loading_planner.py -- geometry and decisions for the dig-and-load demo (stockpile -> dump truck), with
no Isaac imports so every number here can be unit-tested on the host.

The machine is the ECR88 URDF in this repo (default 2.1 m variant, bucket four-bar), with a BUCKET hung
on the output link in place of the PanelLift tool. There is no drawing of the machine's own bucket:
BUCKET below is CLASS-TYPICAL (a 0.25 m^3 general-purpose bucket for an 8-9 t excavator).

Frames
------
* `base` = the URDF base_link frame. Ground is z = GRADE_BASE (-1.445). The machine faces +x at swing 0,
  swing is +z (positive = house turns left, toward +y). World = base + (0, 0, -GRADE_BASE).
* `output_link` = the firmware's output link (bucket pin at its origin, x along the link).
* `B` (bucket frame) = output_link shifted by BUCKET['bracket'] along x and tilted by BUCKET['tilt_deg']
  about y. The bucket is an open box: interior x in [0, H], y in [-W/2, W/2], z in [0, D]; the open face
  is z = 0 and the MOUTH NORMAL is -z_B. The tooth edge is the x = H edge of that face.

Why the tilt (2026-10-02, second pass): a bucket must HANG BELOW its pin through the working range, or the
pin -- and the arm with it -- ends up at bucket height when pouring and hits the truck rail (run 4). With the
arm hanging (boom -50, arm 120) the output link points 28.5 deg below forward at the joint's mid-range
(-41.5 deg); tilting the box 61.5 deg puts the body straight down there with the mouth facing the cab, so
full curl (+43) is mouth-up (+95 deg) with the body behind the pin and full dump (-126) is mouth-down
(-95 deg) with the body in front of the pin. body direction = mouth direction + 90 deg.

Mouth direction convention: an angle in the house x-z plane, 0 = forward (+x), +90 = up, -90 = down,
+-180 = back toward the cab.
"""
import math

import numpy as np

GRADE_BASE = -1.445
JOINTS = ("swing_joint", "boom_joint", "arm_joint", "bucket_joint")

BUCKET = dict(bracket=0.15,     # pin to the bucket's back, CLASS-TYPICAL (a bucket wraps close around its pin;
                                # the firmware's 0.33 m output link is the PanelLift tool mount, not a bucket)
              tilt_deg=61.5,    # see module docstring (DERIVED: body straight down at mid-range of the joint)
              width=0.75, depth=0.55, height=0.60,   # CLASS-TYPICAL, ~0.25 m^3
              wall=0.05)        # thick enough that 3 cm particles cannot tunnel through (GUESS)

# Joint speed caps for the scripted cycle, deg/s. CLASS-TYPICAL for an 8-9 t machine; the firmware's own
# swing in Isaac peaked at 15.9 deg/s at the 70 % auto limit (scripts/sil_isaac_positioning.py).
VMAX_DEG_S = dict(swing=22.0, boom=22.0, arm=30.0, bucket=55.0)


# -- small linear algebra --------------------------------------------------------------------------
def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def q_dict(swing, boom, arm, bucket):
    """Degrees -> {urdf joint: rad} for sil.urdf_fk.UrdfModel.fk."""
    return dict(zip(JOINTS, map(math.radians, (swing, boom, arm, bucket))))


# -- bucket geometry ---------------------------------------------------------------------------------
def bucket_T_output(b=BUCKET):
    """4x4 transform of the bucket frame B in output_link."""
    T = np.eye(4)
    T[:3, :3] = Ry(math.radians(b["tilt_deg"]))
    T[:3, 3] = (b["bracket"], 0.0, 0.0)
    return T


def bucket_walls(b=BUCKET):
    """Five plates (centre in B, full size) that make the open box. The mouth (z = 0 face) is open."""
    W, D, H, t = b["width"], b["depth"], b["height"], b["wall"]
    return dict(
        shell=((H / 2, 0.0, D + t / 2), (H + 2 * t, W + 2 * t, t)),          # back of the bucket
        side_l=((H / 2, W / 2 + t / 2, D / 2), (H + 2 * t, t, D)),
        side_r=((H / 2, -W / 2 - t / 2, D / 2), (H + 2 * t, t, D)),
        heel=((-t / 2, 0.0, D / 2), (t, W + 2 * t, D)),                      # the end at the bracket
        toe=((H + t / 2, 0.0, D / 2), (t, W + 2 * t, D)),                    # the end carrying the edge
    )


def bucket_points(b=BUCKET):
    """Named points in B: the interior centre, the mouth centre, the tooth edge midpoint, the heel."""
    W, D, H = b["width"], b["depth"], b["height"]
    return dict(centre=(H / 2, 0.0, D / 2), mouth=(H / 2, 0.0, 0.0), tip=(H, 0.0, 0.0), heel=(0.0, 0.0, 0.0))


def point_in_output(name, b=BUCKET):
    p = np.array([*bucket_points(b)[name], 1.0])
    return (bucket_T_output(b) @ p)[:3]


def mouth_normal_in_output(b=BUCKET):
    return bucket_T_output(b)[:3, :3] @ np.array([0.0, 0.0, -1.0])


# -- forward kinematics of the bucket ------------------------------------------------------------------
def output_T(model, swing, boom, arm, bucket):
    return model.fk("output_link", q_dict(swing, boom, arm, bucket))


def pin_base(model, swing, boom, arm):
    """Bucket pin (ground_link origin) in base, independent of the bucket angle."""
    return model.fk("ground_link", q_dict(swing, boom, arm, 0.0))[:3, 3]


def bucket_point_base(model, name, swing, boom, arm, bucket, b=BUCKET):
    T = output_T(model, swing, boom, arm, bucket)
    return (T @ np.array([*point_in_output(name, b), 1.0]))[:3]


def bucket_corners_base(model, swing, boom, arm, bucket, b=BUCKET):
    """The 8 outer corners of the bucket box (walls included) in base."""
    W, D, H, t = b["width"], b["depth"], b["height"], b["wall"]
    T = output_T(model, swing, boom, arm, bucket) @ bucket_T_output(b)
    out = []
    for x in (-t, H + t):
        for y in (-W / 2 - t, W / 2 + t):
            for z in (0.0, D + t):
                out.append((T @ np.array([x, y, z, 1.0]))[:3])
    return np.array(out)


def mouth_dir_deg(model, swing, boom, arm, bucket, b=BUCKET):
    """Mouth direction in the HOUSE x-z plane (swing removed): 0 fwd, +90 up, -90 down, +-180 back."""
    T = output_T(model, swing, boom, arm, bucket)
    n = Rz(-math.radians(swing)) @ (T[:3, :3] @ mouth_normal_in_output(b))
    return math.degrees(math.atan2(n[2], n[0]))


def mouth_offset_c(model, b=BUCKET):
    """mouth_dir = c - (boom + arm + bucket): all three turn about the house y axis, and a positive turn
    about +y tips +x toward -z, i.e. lowers the direction angle. Returns c (deg)."""
    return mouth_dir_deg(model, 0.0, 0.0, 0.0, 0.0, b)


def bucket_for_mouth_dir(model, boom, arm, want_dir_deg, b=BUCKET):
    """Bucket angle (deg, clamped to the URDF limits) giving the wanted mouth direction at (boom, arm).
    Returns (bucket_deg, achieved_dir_deg)."""
    lo, hi = model.limits_deg("bucket_joint")
    lo, hi = lo + LIMIT_MARGIN_DEG, hi - LIMIT_MARGIN_DEG
    c = mouth_offset_c(model, b)
    bk = c - boom - arm - want_dir_deg
    bk = (bk + 180.0) % 360.0 - 180.0
    bk = min(max(bk, lo), hi)
    return bk, mouth_dir_deg(model, 0.0, boom, arm, bk, b)


# -- inverse kinematics ---------------------------------------------------------------------------------
def solve_pin(model, target_base, swing, step_deg=2.0):
    """Boom/arm (deg) putting the bucket pin at target_base (3-vector, base frame) with the given swing.
    Grid over the URDF limits then coordinate descent, like sil/ik.py. Returns (boom, arm, err_m)."""
    target = np.asarray(target_base, float)
    B, A = (tuple(l + m for l, m in zip(model.limits_deg(j), (LIMIT_MARGIN_DEG, -LIMIT_MARGIN_DEG)))
            for j in ("boom_joint", "arm_joint"))

    def err(bm, ar):
        return float(np.linalg.norm(pin_base(model, swing, bm, ar) - target))

    best = (1e9, None)
    for bm in np.arange(B[0], B[1] + 1e-9, step_deg):
        for ar in np.arange(A[0], A[1] + 1e-9, step_deg):
            e = err(bm, ar)
            if e < best[0]:
                best = (e, (bm, ar))
    bm, ar = best[1]
    step = step_deg / 2
    while step > 1e-3:
        moved = False
        for dbm, dar in ((step, 0), (-step, 0), (0, step), (0, -step)):
            b2 = min(max(bm + dbm, B[0]), B[1])
            a2 = min(max(ar + dar, A[0]), A[1])
            e = err(b2, a2)
            if e < best[0] - 1e-12:
                best, bm, ar, moved = (e, (b2, a2)), b2, a2, True
        if not moved:
            step /= 2
    return float(bm), float(ar), float(best[0])


def solve_bucket_point(model, name, target_base, swing, mouth_dir_deg_want, b=BUCKET):
    """Pose (swing, boom, arm, bucket in deg) that puts bucket point `name` at target_base with the mouth
    pointing mouth_dir_deg_want. Because the mouth direction fixes the output link's absolute pitch, the
    pin-to-point offset is a constant vector for that direction, so each pass is one solve_pin call.
    swing=None turns the house onto the target's bearing, correcting for the boom's sideways offset
    (distChsToBmMntY = -0.15 m: at swing 0 the pin is 15 cm right of the x axis).
    Returns (pose dict, point error m, mouth direction error deg)."""
    target = np.asarray(target_base, float)
    c = mouth_offset_c(model, b)
    want = mouth_dir_deg_want
    auto = swing is None
    if auto:
        swing = math.degrees(math.atan2(target[1], target[0]))
    for _ in range(4):
        # output link pitch for that mouth direction, as a bucket angle at boom = arm = 0
        T0 = output_T(model, 0.0, 0.0, 0.0, c - want)
        off_house = T0[:3, :3] @ point_in_output(name, b)      # pin -> point, house frame, at that pitch
        off = Rz(math.radians(swing)) @ off_house
        bm, ar, e_pin = solve_pin(model, target - off, swing)
        bk, got = bucket_for_mouth_dir(model, bm, ar, want, b)
        p = bucket_point_base(model, name, swing, bm, ar, bk, b)
        dir_ok = abs(((got - want) + 180.0) % 360.0 - 180.0) < 0.5
        bearing_err = math.degrees(math.atan2(target[1], target[0]) - math.atan2(p[1], p[0])) if auto else 0.0
        if dir_ok and abs(bearing_err) < 0.05:
            break
        want = got                 # the bucket limit clamps the direction: re-aim the point for what we can do
        swing += bearing_err
    pose = dict(swing=float(swing), boom=bm, arm=ar, bucket=bk)
    derr = abs(((got - mouth_dir_deg_want) + 180.0) % 360.0 - 180.0)
    return pose, float(np.linalg.norm(p - target)), float(derr)


# -- joint-space trajectory -------------------------------------------------------------------------------
def _smooth(s):
    s = min(max(s, 0.0), 1.0)
    return s * s * (3.0 - 2.0 * s)


class JointTrajectory:
    """Piecewise point-to-point moves in joint space (degrees), each a smoothstep whose duration is set by
    the slowest joint under VMAX_DEG_S, synchronised so every joint arrives together. `hold` dwells."""

    def __init__(self, q0, vmax=VMAX_DEG_S):
        self.q0 = dict(q0)
        self.vmax = dict(vmax)
        self.segments = []            # (t_start, duration, q_from, q_to, label)
        self.t_end = 0.0
        self._last = dict(q0)

    def move(self, q_to, label="", min_time=0.6, speed_scale=1.0):
        q_to = dict(self._last, **q_to)
        dur = min_time
        for k, v in q_to.items():
            dq = abs(v - self._last[k])
            dur = max(dur, dq / (self.vmax[k] * speed_scale))
        self.segments.append((self.t_end, dur, dict(self._last), q_to, label))
        self.t_end += dur
        self._last = q_to
        return self

    def hold(self, seconds, label="hold"):
        self.segments.append((self.t_end, seconds, dict(self._last), dict(self._last), label))
        self.t_end += seconds
        return self

    @property
    def duration(self):
        return self.t_end

    def sample(self, t):
        """(joint targets deg, label of the active segment)."""
        if t >= self.t_end:
            return dict(self._last), "done"
        for t0, dur, qa, qb, label in self.segments:
            if t < t0 + dur:
                s = _smooth((t - t0) / dur) if dur > 0 else 1.0
                return {k: qa[k] + (qb[k] - qa[k]) * s for k in qa}, label
        return dict(self._last), "done"


# -- perception-side grids ---------------------------------------------------------------------------------
class BedGrid:
    """A dump-truck bed as cells. x along the bed length, y across, heights are above the bed floor."""

    def __init__(self, centre_xy, yaw_deg, length, width, floor_z, n_long=3, n_wide=2, max_h=2.5):
        self.c = np.asarray(centre_xy, float)
        self.yaw = math.radians(yaw_deg)
        self.L, self.W, self.floor_z = float(length), float(width), float(floor_z)
        self.nl, self.nw = int(n_long), int(n_wide)
        self.max_h = max_h

    def to_bed(self, pts):
        pts = np.asarray(pts, float).reshape(-1, 3)
        d = pts[:, :2] - self.c
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        x = c * d[:, 0] - s * d[:, 1]
        y = s * d[:, 0] + c * d[:, 1]
        return np.column_stack([x, y, pts[:, 2] - self.floor_z])

    WALL_MARGIN = 0.12      # returns this close to a wall are the wall's top face, not the load (run 9: 0.5 m "load")

    def inside(self, pb):
        return ((np.abs(pb[:, 0]) <= self.L / 2 - self.WALL_MARGIN) & (np.abs(pb[:, 1]) <= self.W / 2 - self.WALL_MARGIN)
                & (pb[:, 2] > -0.05) & (pb[:, 2] < self.max_h))

    def cell_of(self, pb):
        i = np.clip(((pb[:, 0] + self.L / 2) / self.L * self.nl).astype(int), 0, self.nl - 1)
        j = np.clip(((pb[:, 1] + self.W / 2) / self.W * self.nw).astype(int), 0, self.nw - 1)
        return i, j

    def heights(self, pts_world, pct=90.0, min_pts=3):
        """(nl, nw) surface height above the floor per cell: the pct-percentile of the points in the cell
        (robust to stray returns), 0 where fewer than min_pts points landed."""
        pb = self.to_bed(pts_world)
        pb = pb[self.inside(pb)]
        h = np.zeros((self.nl, self.nw))
        if len(pb) == 0:
            return h
        i, j = self.cell_of(pb)
        for a in range(self.nl):
            for b in range(self.nw):
                z = pb[(i == a) & (j == b), 2]
                if len(z) >= min_pts:
                    h[a, b] = max(0.0, float(np.percentile(z, pct)))
        return h

    def counts(self, pts_world):
        pb = self.to_bed(pts_world)
        pb = pb[self.inside(pb)]
        n = np.zeros((self.nl, self.nw), int)
        if len(pb):
            i, j = self.cell_of(pb)
            np.add.at(n, (i, j), 1)
        return n

    def cell_centre_world(self, i, j, h=0.0):
        x = -self.L / 2 + (i + 0.5) * self.L / self.nl
        y = -self.W / 2 + (j + 0.5) * self.W / self.nw
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return np.array([self.c[0] + c * x - s * y, self.c[1] + s * x + c * y, self.floor_z + h])

    @staticmethod
    def lowest_cell(h):
        return tuple(int(v) for v in np.unravel_index(int(np.argmin(h)), h.shape))

    @staticmethod
    def evenness(h):
        return dict(min=float(h.min()), max=float(h.max()), range=float(h.max() - h.min()), std=float(h.std()))

    @property
    def cell_area(self):
        return (self.L / self.nl) * (self.W / self.nw)


class TerrainGrid:
    """Square x-y grid of surface heights (above z0) for the stockpile; dig progress = volume below a
    reference surface."""

    def __init__(self, centre_xy, half, cell, z0, max_h=3.0):
        self.c = np.asarray(centre_xy, float)
        self.half, self.cell, self.z0 = float(half), float(cell), float(z0)
        self.n = int(round(2 * half / cell))
        self.max_h = float(max_h)            # nothing on the pile is taller than this: drop stray high returns
        self.last_counts = np.zeros((self.n, self.n), int)

    def heights(self, pts_world, pct=90.0, min_pts=2):
        pts = np.asarray(pts_world, float).reshape(-1, 3)
        d = pts[:, :2] - self.c + self.half
        ok = ((d[:, 0] >= 0) & (d[:, 0] < 2 * self.half) & (d[:, 1] >= 0) & (d[:, 1] < 2 * self.half)
              & (pts[:, 2] - self.z0 < self.max_h) & (pts[:, 2] - self.z0 > -0.3))
        h = np.zeros((self.n, self.n))
        self.last_counts = np.zeros((self.n, self.n), int)
        if not ok.any():
            return h
        i = (d[ok, 0] / self.cell).astype(int)
        j = (d[ok, 1] / self.cell).astype(int)
        z = pts[ok, 2] - self.z0
        order = np.lexsort((j, i))
        i, j, z = i[order], j[order], z[order]
        key = i * self.n + j
        starts = np.flatnonzero(np.r_[True, key[1:] != key[:-1]])
        ends = np.r_[starts[1:], len(key)]
        for s, e in zip(starts, ends):
            self.last_counts[i[s], j[s]] = e - s
            if e - s >= min_pts:
                h[i[s], j[s]] = max(0.0, float(np.percentile(z[s:e], pct)))
        return h

    def sampler(self, h, fallback=None):
        """height(x, y) from a heights array. A cell the survey did not see (fewer than 2 returns -- the far
        slope is in the LiDAR's shadow) returns `fallback` when given: the dig entry must not assume the
        unseen far side is flat (run 11 drove the bucket into the pile from above that way)."""
        counts = self.last_counts.copy()

        def f(x, y):
            i = int((x - self.c[0] + self.half) / self.cell)
            j = int((y - self.c[1] + self.half) / self.cell)
            if not (0 <= i < self.n and 0 <= j < self.n):
                return 0.0 if fallback is None else float(fallback)
            if fallback is not None and counts[i, j] < 2:
                return float(fallback)
            return float(h[i, j])
        return f

    def volume(self, h):
        return float(np.sum(h) * self.cell * self.cell)

    def removed_volume(self, h_ref, h, seen=None):
        """Volume below the reference surface; with `seen` (bool array) only cells surveyed in BOTH maps count,
        so a shadowed cell is not reported as dug out."""
        diff = np.clip(h_ref - h, 0.0, None)
        if seen is not None:
            diff = np.where(seen, diff, 0.0)
        return float(np.sum(diff) * self.cell * self.cell)


class SafetyMonitor:
    """Stop when a person is inside stop_r, resume only after they are beyond resume_r (hysteresis), and
    keep the event log the report prints."""

    def __init__(self, stop_r=6.0, resume_r=7.5, lost_s=2.0):
        self.stop_r, self.resume_r, self.lost_s = float(stop_r), float(resume_r), float(lost_s)
        self.paused = False
        self.events = []
        self.last_seen = None

    def update(self, t, dist, source="gt"):
        if dist is None:
            # detection lost (person left the field of view): a stop must not latch forever
            if self.paused and self.last_seen is not None and t - self.last_seen >= self.lost_s:
                self.paused = False
                self.events.append(dict(t=float(t), event="resume_lost", dist=None, source=source))
            return self.paused
        self.last_seen = float(t)
        if not self.paused and dist <= self.stop_r:
            self.paused = True
            self.events.append(dict(t=float(t), event="stop", dist=float(dist), source=source))
        elif self.paused and dist >= self.resume_r:
            self.paused = False
            self.events.append(dict(t=float(t), event="resume", dist=float(dist), source=source))
        return self.paused


# -- the site -------------------------------------------------------------------------------------------------
SITE = dict(
    # stockpile: a cone in front of the machine. The bucket pin cannot go below ~1.2 m above grade with the
    # OEM-TEAM joint limits (boom -69..-31, arm 31..155), so flat ground cannot be dug: the demo digs a pile.
    mound_centre=(4.7, 0.0), mound_radius=2.2, mound_height=1.9,   # tall: PBD creep takes ~0.5 m off in the first minute
    bench_h=0.6,                 # the stockpile sits on a 0.6 m bench: with the pin floored at 1.18 m the edge only
                                 # reaches 0.45 m above grade, so a pile on grade can be skimmed, not dug (run 8)
    bench_half=3.2,
    # dump truck parked on the machine's left, body along x, bed centre at 90 deg swing.
    truck_swing_deg=90.0, bed_centre=(0.0, 4.4), bed_yaw_deg=180.0,   # cab toward -x: at yaw 0 the cab's near corner
                                                                     # (bearing 41 deg, radius 3.9, top 2.65 m) sat in the
                                                                     # loaded swing path and stopped the house at 47 deg (run 8)
    # 10 t class tipper (CLASS-TYPICAL). The rail top (1.70 m) must stay under the carried bucket's bottom corner
    # (1.84 m at CARRY_H 2.3); a 15-25 t truck's 2.3 m rail needs the higher carry that is blocked by open item 13.
    bed_length=4.4, bed_width=2.2, bed_floor_above_grade=1.20, bed_rail_h=0.50,
    bed_cells=(3, 2),
    # person: walks past between the truck and the pile
    # the person walks in from the right, STANDS 3.5 m in front of the machine for 10 s, and leaves again: the
    # machine must stop while they are inside stop_radius and go back to work once they are past resume_radius
    person_path=((9.0, -5.0), (3.3, -1.2), (3.3, -1.2), (9.0, -6.5)), person_dwell_s=10.0,
    person_speed=1.2, person_start_t=35.0,
    stop_radius=6.0, resume_radius=7.5,
)


# -- one dig-and-dump cycle -----------------------------------------------------------------------------------
MOUTH_DIG_ENTER = -175.0     # bucket hanging, teeth down, mouth toward the cab: the tooth edge is 0.7 m below the
                             # pin, which is the only way it reaches the pile with the pin's 1.18 m floor (run 8:
                             # at -110 the body pointed forward and the edge stopped 1.0 m above grade)
MOUTH_CURL = 92.0            # carrying: mouth straight up (full curl reaches +95.5 with the arm hanging)
MOUTH_DUMP = -85.0           # pouring: mouth almost straight down, body in front of the pin
PIN_ABOVE_RAIL = 0.15        # the bucket pin (arm tip) stays this far above the truck rail while over the bed
CARRY_H = 2.3                # bucket centre above grade while swinging loaded (bottom corner 1.84 m: clears the 1.65 m
                             # rail and, once the PBD cone has slumped to ~1.7 m at its apex, the benched pile's slope).
                             # KNOWN-GOOD for swing tracking (runs 9/10: 92 deg target, 92 deg actual). With 2.6 and 3.0
                             # (boom -63 / -68) the swing crawled at ~5 deg/s instead of 22 in runs 11-15 -- unexplained,
                             # see docs/LOADING_DEMO.md 3-1 row 13; do not raise this without re-checking the seg logs.
LIMIT_MARGIN_DEG = 3.0       # the planner never asks for a joint within this of a URDF limit
DUMP_CLEARANCE = 0.35        # mouth above the measured surface of the chosen cell ("carefully")
CARRY_CENTRE_R = 4.0         # radius of the bucket centre in the carry pose; inside the bed footprint at 90 deg

# where the bucket can usefully take from the pile with these joint limits (DERIVED from the FK probe)
DIG_R_MIN, DIG_R_MAX, DIG_Y_MAX = 3.3, 5.2, 1.6    # y widened 1.0 -> 1.6 (run 16): straight ahead the carried bucket
                                                   # hides the pile from the roof LiDAR; r widened 4.6 -> 5.2 (run 18):
                                                   # the near face's returns come back too long and are dropped as
                                                   # below-grade (open item), so what the LiDAR reliably sees is the
                                                   # crest and the far side; the entry pose clamps to the reach anyway


def carry_pose(model, bearing_deg, b=BUCKET):
    """Loaded-travel pose with the bucket centre at CARRY_CENTRE_R on the given bearing, CARRY_H above grade."""
    pose, e, _ = solve_bucket_point(model, "centre", _radial(CARRY_CENTRE_R, bearing_deg, GRADE_BASE + CARRY_H),
                                    None, MOUTH_CURL, b)
    return pose, e


def _radial(r, swing_deg, z):
    """Base-frame point at radius r (from the swing axis, along the house x) and height z."""
    a = math.radians(swing_deg)
    return np.array([r * math.cos(a), r * math.sin(a), z])


def choose_dig_point(terrain, h, r_min=DIG_R_MIN, r_max=DIG_R_MAX, y_max=DIG_Y_MAX):
    """Highest terrain cell whose centre is inside the reachable band in front of the machine.
    Returns (x, y, h) in the grid's frame, or None if nothing in the band is above 0.15 m."""
    best = None
    for i in range(terrain.n):
        for j in range(terrain.n):
            x = terrain.c[0] - terrain.half + (i + 0.5) * terrain.cell
            y = terrain.c[1] - terrain.half + (j + 0.5) * terrain.cell
            r = math.hypot(x, y)
            if r_min <= r <= r_max and abs(y) <= y_max and h[i, j] > 0.15:
                if best is None or h[i, j] > best[2]:
                    best = (x, y, float(h[i, j]))
    return best


TIP_MIN_ABOVE_GRADE = 0.35       # the joint limits keep the pin >= 1.18 m up; the tooth edge bottoms out near here
SCOOP_DEPTH = 0.55               # how far below the measured surface the edge is pulled. The LiDAR cell height is the
                                 # 90th percentile of a 0.2 m cell on a slope and the PBD pile keeps creeping down after
                                 # the survey, so 0.35 left the edge in the air (runs 11-13: < 10 particles)


def plan_dig(model, q_now, dig_xy_h, b=BUCKET, surface=None, base_h=None):
    """Trajectory from the current pose through: approach above the pile -> enter -> scoop -> lift to carry.
    dig_xy_h = (x, y, surface height above grade) of the chosen pile cell, base frame at swing 0 reference.
    surface(x, y) -> height above grade from the terrain map; the edge enters just above it on the far side
    and is pulled toward the machine SCOOP_DEPTH below it (never under TIP_MIN_ABOVE_GRADE)."""
    x, y, hs = dig_xy_h
    bearing = math.degrees(math.atan2(y, x))
    r = math.hypot(x, y)
    G = GRADE_BASE
    surf = surface if surface is not None else (lambda px, py: hs)
    base = SITE["bench_h"] if base_h is None else float(base_h)       # surface heights are above this
    r_in, r_low, r_end = r + 0.6, r - 0.1, max(r - 0.8, DIG_R_MIN - 0.3)
    p_in, p_low = _radial(r_in, bearing, 0.0), _radial(r_low, bearing, 0.0)
    p_end = _radial(r_end, bearing, 0.0)
    z_in = max(base + float(surf(p_in[0], p_in[1])) + 0.15, TIP_MIN_ABOVE_GRADE + 0.1)
    z_low = max(base + float(surf(p_low[0], p_low[1])) - SCOOP_DEPTH, TIP_MIN_ABOVE_GRADE, base + 0.05)
    # the curl finishes still under the surface at the near end of the sweep (run 12: ending 0.15 above the
    # low point left the edge in the air on the near slope and the bucket came up with 6 particles)
    z_end = max(base + float(surf(p_end[0], p_end[1])) - 0.2, TIP_MIN_ABOVE_GRADE, base + 0.05)
    enter = solve_bucket_point(model, "tip", _radial(r_in, bearing, G + z_in), None, MOUTH_DIG_ENTER, b)[0]
    # at the bottom of the scoop the mouth faces the cab (170 deg) so it leads the pull toward the machine
    low = solve_bucket_point(model, "tip", _radial(r_low, bearing, G + z_low), None, 170.0, b)[0]
    end = solve_bucket_point(model, "tip", _radial(r_end, bearing, G + z_end), None, MOUTH_CURL, b)[0]
    above = dict(enter, boom=max(enter["boom"] - 12.0, model.limits_deg("boom_joint")[0] + LIMIT_MARGIN_DEG))
    carry, _ = carry_pose(model, bearing, b)
    tr = JointTrajectory(q_now)
    tr.move(above, "approach")
    tr.move(enter, "enter", speed_scale=0.6)
    tr.move(low, "scoop", speed_scale=0.5)
    tr.move(end, "curl", speed_scale=0.6)
    tr.move(carry, "lift")
    return tr


def plan_dump(model, q_now, bed, cell, surface_h, b=BUCKET):
    """Trajectory: swing loaded to the truck, hover over the chosen cell, lower the mouth to
    surface + DUMP_CLEARANCE, pour slowly, close, lift away. Returns (trajectory, dump pose)."""
    carry, _ = carry_pose(model, SITE["truck_swing_deg"], b)
    pour = fit_in_bed(model, bed, cell, surface_h + DUMP_CLEARANCE, MOUTH_DUMP, b)
    hover = fit_in_bed(model, bed, cell, surface_h + DUMP_CLEARANCE + 0.15, MOUTH_CURL, b)
    closed = dict(pour, bucket=bucket_for_mouth_dir(model, pour["boom"], pour["arm"], MOUTH_CURL, b)[0])
    tr = JointTrajectory(q_now)
    tr.move(carry, "swing_to_truck")
    tr.move(hover, "over_cell")
    tr.move(pour, "pour", speed_scale=0.35)
    tr.hold(1.2, "drain")
    tr.move(closed, "close", speed_scale=0.6)
    tr.move(carry, "lift_away")
    return tr, pour


def box_extents_house(mouth_dir, b=BUCKET):
    """Corners of the bucket box relative to its interior centre, in the HOUSE frame, for a mouth direction.
    Orientation alone fixes these (the three pitch joints share the house y axis), so a pose can be aimed
    before it is solved."""
    W, D, H, t = b["width"], b["depth"], b["height"], b["wall"]
    T_B = bucket_T_output(b)
    # output-link pitch that gives mouth_dir at boom = arm = 0: bucket = c - mouth_dir, so the output link
    # rotation about y is Ry(-(c - mouth_dir)) relative to the house... computed numerically instead:
    n_o = mouth_normal_in_output(b)
    # find the rotation about house y that maps n_o onto the wanted direction
    cur = math.degrees(math.atan2(n_o[2], n_o[0]))
    R = Ry(math.radians(cur - mouth_dir))      # +theta about y lowers the direction angle
    centre = np.array(bucket_points(b)["centre"])
    out = []
    for x in (-t, H + t):
        for y in (-W / 2 - t, W / 2 + t):
            for z in (0.0, D + t):
                out.append(R @ (T_B[:3, :3] @ (np.array([x, y, z]) - centre)))
    return np.array(out)


def fit_in_bed(model, bed, cell, lowest_edge_above_floor, mouth_dir, b=BUCKET, margin=0.08, passes=3):
    """Pose with the bucket over bed cell `cell`, mouth at `mouth_dir`, the LOWEST corner of the bucket
    box at lowest_edge_above_floor, the whole box inside the bed walls (the target is pulled toward the bed
    axis when the box would overhang a wall) and the pin PIN_ABOVE_RAIL over the rail. The bed is in world
    (grade z = 0), the pose in base. Aimed from the orientation-only extents; because the bucket limit can
    clamp the direction, the aim is repeated with the direction actually achieved."""
    cx, cy, _ = bed.cell_centre_world(cell[0], cell[1], 0.0)
    bearing = math.atan2(cy, cx)
    rail_top = bed.floor_z + SITE["bed_rail_h"]
    want_dir = mouth_dir
    pose = None
    for _ in range(passes):
        ext = (Rz(bearing) @ box_extents_house(want_dir, b).T).T          # corners about the centre, world axes
        ext_bed = bed.to_bed(ext + np.array([*bed.c, bed.floor_z]))       # same corners along the bed axes
        centre_xy = np.array([cx, cy])
        for k, half in ((0, bed.L / 2), (1, bed.W / 2)):
            lo, hi = ext_bed[:, k].min(), ext_bed[:, k].max()
            cell_k = bed.to_bed(np.array([[cx, cy, bed.floor_z]]))[0, k]
            want_k = min(max(cell_k, -half + margin - lo), half - margin - hi)
            d = want_k - cell_k
            c_, s_ = math.cos(bed.yaw), math.sin(bed.yaw)
            centre_xy = centre_xy + (np.array([c_, s_]) if k == 0 else np.array([-s_, c_])) * d
        z_centre = bed.floor_z + lowest_edge_above_floor - ext[:, 2].min()
        tgt = np.array([centre_xy[0], centre_xy[1], GRADE_BASE + z_centre])
        pose = solve_bucket_point(model, "centre", tgt, None, want_dir, b)[0]
        pin_z = pin_base(model, pose["swing"], pose["boom"], pose["arm"])[2] - GRADE_BASE
        deficit = rail_top + PIN_ABOVE_RAIL - pin_z
        if deficit > 0:
            lowest_edge_above_floor += deficit
            tgt[2] += deficit
            pose = solve_bucket_point(model, "centre", tgt, None, want_dir, b)[0]
        got = mouth_dir_deg(model, **pose, b=b)
        if abs(((got - want_dir) + 180.0) % 360.0 - 180.0) < 1.0:
            break
        want_dir = got
    return pose


def plan_return(model, q_now, b=BUCKET):
    carry, _ = carry_pose(model, 0.0, b)
    return JointTrajectory(q_now).move(carry, "swing_back")


# -- threat-based stop: does the PLANNED motion meet the person's PREDICTED path? --------------------------
HOUSE_FOOTPRINT = (2.28, 2.46)       # lenUppX x lenUppY (sheet), centred on the swing axis: the tail swing hits people too


def machine_points_base(model, q, b=BUCKET):
    """x-y points (base frame) of what can hit a person at joint pose q (deg): bucket corners, bucket pin,
    boom mid, and the four house corners turned by the swing."""
    pts = [bucket_corners_base(model, **q, b=b)[:, :2]]
    pin = pin_base(model, q["swing"], q["boom"], q["arm"])
    T_bm = model.fk("boom_link", q_dict(q["swing"], q["boom"], q["arm"], q["bucket"]))
    mid = (T_bm @ np.array([1.8, 0.0, 0.0, 1.0]))[:3]
    pts.append(np.array([pin[:2], mid[:2]]))
    hx, hy = HOUSE_FOOTPRINT[0] / 2, HOUSE_FOOTPRINT[1] / 2
    Rs = Rz(math.radians(q["swing"]))[:2, :2]
    corners = np.array([[hx, hy], [hx, -hy], [-hx, hy], [-hx, -hy]]) @ Rs.T
    pts.append(corners)
    return np.vstack(pts)


class ThreatMonitor:
    """Speed-and-separation monitoring in the spirit of ISO 13855: look `horizon_s` ahead along the
    PLANNED trajectory and along the person's PREDICTED straight-line path; `stop` if any machine point
    comes within `margin` of the person, `slow` if within `margin + slow_band`, else `clear`.
    A person standing still behind the counterweight does not stop a dig in front; a person walking
    into the swing path stops it before the bucket arrives. Levels change with hysteresis in time
    (`hold_s`) so a flickering detection cannot chatter the machine."""

    def __init__(self, horizon_s=3.0, step_s=0.25, margin=1.5, slow_band=2.0, person_radius=0.4, hold_s=0.8):
        self.h, self.step = float(horizon_s), float(step_s)
        self.margin, self.slow_band, self.pr = float(margin), float(slow_band), float(person_radius)
        self.hold_s = float(hold_s)
        self.level = "clear"
        self.events = []
        self._last_change = -1e9
        self.last = None

    def assess(self, person_xy, person_v, machine_points_fn):
        """Return (level, min_distance_m, tau_of_min). machine_points_fn(tau) -> Nx2 points at +tau s, or None
        when the machine is not planning to move (then only tau = 0 is checked)."""
        if person_xy is None:
            return "clear", None, None
        p0 = np.asarray(person_xy, float)[:2]
        v = np.zeros(2) if person_v is None else np.asarray(person_v, float)[:2]
        pts0 = machine_points_fn(0.0)
        if pts0 is None:
            return "clear", None, None
        pts0 = np.asarray(pts0, float)[:, :2]
        person_moving = float(np.linalg.norm(v)) * self.h > 0.1
        best = (1e9, None)
        # Only MOTION is a threat: a machine point counts at +tau if it will have moved by then; a point that
        # stays put counts only if the person is walking (toward it or not -- the distance decides).
        for tau in np.arange(self.step, self.h + 1e-9, self.step):
            pts = machine_points_fn(tau)
            if pts is None:
                break
            pts = np.asarray(pts, float)[:, :2]
            moved = np.linalg.norm(pts - pts0, axis=1) > 0.1 if len(pts) == len(pts0) else np.ones(len(pts), bool)
            keep = moved | person_moving
            if not keep.any():
                continue
            d = float(np.min(np.linalg.norm(pts[keep] - (p0 + v * tau), axis=1))) - self.pr
            if d < best[0]:
                best = (d, float(tau))
        d, tau = best
        if tau is None:
            return "clear", None, None
        level = "stop" if d < self.margin else ("slow" if d < self.margin + self.slow_band else "clear")
        return level, d, tau

    def update(self, t, person_xy, person_v, machine_points_fn, source="camera"):
        level, d, tau = self.assess(person_xy, person_v, machine_points_fn)
        self.last = dict(level=level, min_dist=d, tau=tau)
        rank = dict(clear=0, slow=1, stop=2)
        # escalate immediately, de-escalate only after hold_s of a calmer assessment
        if rank[level] > rank[self.level] or (rank[level] < rank[self.level] and t - self._last_change >= self.hold_s):
            self.events.append(dict(t=float(t), event=f"{self.level}->{level}", min_dist=d, tau=tau,
                                    person_xy=None if person_xy is None else [float(x) for x in person_xy[:2]],
                                    person_v=None if person_v is None else [float(x) for x in person_v[:2]], source=source))
            self.level = level
            self._last_change = t
        elif rank[level] == rank[self.level]:
            self._last_change = t
        return self.level

    @property
    def speed_scale(self):
        return dict(clear=1.0, slow=0.3, stop=0.0)[self.level]
