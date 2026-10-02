"""
Planner for the dig-and-load demo (sim/loading_planner.py). Not firmware: these pin the geometry the
Isaac demo relies on, with the URDF FK only, so a joint-limit or bucket change that makes the demo
unreachable fails here in seconds instead of after a 10-minute Isaac run.
"""
import math
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

import numpy as np

from sil.urdf_fk import UrdfModel
from sim import loading_planner as lp

REPO = Path(__file__).resolve().parents[2]
XACRO = REPO / "assets/ecr88/urdf/ecr88.urdf.xacro"


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


@unittest.skipUnless(shutil.which("xacro"), "xacro not on PATH")
class PlannerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp()
        urdf = Path(cls.tmp) / "ecr88_demo.urdf"
        subprocess.run(["xacro", str(XACRO), "model_cylinders:=false", "-o", str(urdf)], check=True,
                       capture_output=True)
        cls.m = UrdfModel(urdf)
        cls.bed = lp.BedGrid(lp.SITE["bed_centre"], lp.SITE["bed_yaw_deg"], lp.SITE["bed_length"],
                             lp.SITE["bed_width"], lp.SITE["bed_floor_above_grade"], *lp.SITE["bed_cells"])

    # -- bucket orientation -----------------------------------------------------------------------
    def test_mouth_direction_is_c_minus_joint_sum(self):
        c = lp.mouth_offset_c(self.m)
        for bm, ar, bk in ((-50, 120, 43), (-50, 120, -60), (-50, 120, -126), (-60, 70, -100), (-31, 155, 0)):
            got = lp.mouth_dir_deg(self.m, 17.0, bm, ar, bk)        # swing must not matter
            self.assertAlmostEqual(wrap(got - (c - bm - ar - bk)), 0.0, places=6, msg=(bm, ar, bk))

    def test_full_curl_faces_up_and_full_dump_faces_down_with_the_arm_hanging(self):
        up = lp.mouth_dir_deg(self.m, 0, -50, 120, 43)
        down = lp.mouth_dir_deg(self.m, 0, -50, 120, -126)
        self.assertGreater(math.sin(math.radians(up)), math.sin(math.radians(50)))      # > 50 deg above horizontal
        self.assertLess(math.sin(math.radians(down)), -math.sin(math.radians(50)))      # > 50 deg below

    def test_bucket_walls_enclose_the_interior_and_leave_the_mouth_open(self):
        walls = lp.bucket_walls()
        b = lp.BUCKET
        # every wall centre sits outside the interior box by half its thickness on exactly one axis
        for name, (c, size) in walls.items():
            self.assertFalse(0 < c[0] < b["height"] and abs(c[1]) < b["width"] / 2 and 0 < c[2] < b["depth"], name)
        self.assertNotIn("mouth", walls)
        self.assertEqual(len(walls), 5)

    # -- reach ----------------------------------------------------------------------------------------
    def test_every_bed_cell_is_reachable_for_pouring_empty_and_half_full(self):
        for i in range(self.bed.nl):
            for j in range(self.bed.nw):
                for hs in (0.0, 0.5):
                    cx, cy, cz = self.bed.cell_centre_world(i, j, 0.0)
                    target = np.array([cx, cy, cz + hs + lp.DUMP_CLEARANCE + lp.GRADE_BASE])
                    pose, err, derr = lp.solve_bucket_point(self.m, "mouth", target, None, lp.MOUTH_DUMP)
                    self.assertLess(err, 0.15, (i, j, hs, pose, err))
                    # the bucket limit (-126 deg) can clamp the pour steeper than asked; any mouth between
                    # 55 and 130 deg below horizontal still pours
                    got = lp.mouth_dir_deg(self.m, **pose)
                    self.assertTrue(-130.0 <= got <= -55.0, (i, j, hs, pose, got))
                    lo, hi = self.m.limits_deg("bucket_joint")
                    self.assertTrue(lo <= pose["bucket"] <= hi)

    def test_pour_and_hover_keep_the_whole_bucket_inside_the_bed_and_above_the_load(self):
        bed = self.bed
        for i in range(bed.nl):
            for j in range(bed.nw):
                for hs in (0.0, 0.5):
                    tr, pour = lp.plan_dump(self.m, dict(swing=0, boom=-60, arm=120, bucket=43), bed, (i, j), hs)
                    hover = next(s for s in tr.segments if s[4] == "over_cell")[3]
                    for label, pose in (("pour", pour), ("hover", hover)):
                        corners = lp.bucket_corners_base(self.m, **pose) + np.array([0, 0, -lp.GRADE_BASE])
                        pb = bed.to_bed(corners)
                        self.assertTrue(np.all(np.abs(pb[:, 0]) < bed.L / 2 - 0.05), (i, j, hs, label, pb[:, 0]))
                        self.assertTrue(np.all(np.abs(pb[:, 1]) < bed.W / 2 - 0.05), (i, j, hs, label, pb[:, 1]))
                        self.assertTrue(np.all(pb[:, 2] > hs + 0.05), (i, j, hs, label, pb[:, 2]))

    def test_dig_poses_are_reachable_across_the_pile_face(self):
        for r in (3.4, 3.8, 4.2, 4.6):
            for y in (-0.8, 0.0, 0.8):
                x = math.sqrt(r * r - y * y)
                tr = lp.plan_dig(self.m, dict(swing=0, boom=-60, arm=120, bucket=43), (x, y, 0.9))
                labels = [s[4] for s in tr.segments]
                self.assertEqual(labels, ["approach", "enter", "scoop", "curl", "lift"])
                for _, _, _, q, label in tr.segments:
                    for k, j in (("boom", "boom_joint"), ("arm", "arm_joint"), ("bucket", "bucket_joint")):
                        lo, hi = self.m.limits_deg(j)
                        self.assertTrue(lo - 1e-6 <= q[k] <= hi + 1e-6, (r, y, label, k, q[k]))
                # the scoop really sweeps toward the machine and ends curled
                enter = next(s for s in tr.segments if s[4] == "enter")[3]
                curl = next(s for s in tr.segments if s[4] == "curl")[3]
                p0 = lp.bucket_point_base(self.m, "tip", **enter)
                p1 = lp.bucket_point_base(self.m, "tip", **curl)
                self.assertLess(math.hypot(*p1[:2]), math.hypot(*p0[:2]) - 0.6)
                self.assertGreater(math.sin(math.radians(lp.mouth_dir_deg(self.m, **curl))), 0.6)

    def test_carry_pose_clears_the_rail_in_front_and_over_the_truck(self):
        rail = lp.GRADE_BASE + lp.SITE["bed_floor_above_grade"] + lp.SITE["bed_rail_h"]
        for sw in (0.0, 45.0, 90.0):
            pose, err = lp.carry_pose(self.m, sw)
            self.assertLess(err, 0.05, (sw, pose, err))
            for name in ("tip", "heel", "centre"):
                self.assertGreater(lp.bucket_point_base(self.m, name, **pose)[2], rail + 0.3, (sw, name))

    # -- trajectory -------------------------------------------------------------------------------------
    def test_trajectory_respects_speed_caps_and_arrives(self):
        q0 = dict(swing=0.0, boom=-60.0, arm=120.0, bucket=43.0)
        tr = lp.JointTrajectory(q0).move(dict(swing=90.0, boom=-40.0), "a").hold(0.5).move(dict(bucket=-120.0), "b")
        self.assertAlmostEqual(tr.segments[0][1], 90.0 / lp.VMAX_DEG_S["swing"])
        ts = np.arange(0.0, tr.duration + 0.02, 0.01)
        prev, _ = tr.sample(0.0)
        for t in ts[1:]:
            q, _ = tr.sample(t)
            for k in q:
                self.assertLessEqual(abs(q[k] - prev[k]) / 0.01, lp.VMAX_DEG_S[k] * 1.5 + 1e-9, (t, k))   # smoothstep peak = 1.5x mean
            prev = q
        q, label = tr.sample(tr.duration + 1.0)
        self.assertEqual(label, "done")
        self.assertEqual(q, dict(swing=90.0, boom=-40.0, arm=120.0, bucket=-120.0))

    # -- grids ------------------------------------------------------------------------------------------
    def test_bed_grid_finds_the_lowest_cell_and_reports_evenness(self):
        bed = self.bed
        rng = np.random.default_rng(0)
        pts = []
        for i in range(bed.nl):
            for j in range(bed.nw):
                h = 0.6 if (i, j) == (0, 1) else (0.3 if (i, j) == (2, 0) else 0.1)
                c = bed.cell_centre_world(i, j, 0.0)
                p = c + np.column_stack([rng.uniform(-0.35, 0.35, 50), rng.uniform(-0.3, 0.3, 50), rng.uniform(0.0, h, 50)])   # inside WALL_MARGIN
                pts.append(p)
        pts = np.vstack(pts)
        pts = np.vstack([pts, [[50.0, 50.0, 3.0], [0.0, 4.4, 20.0]]])       # far away + absurdly high: ignored
        h = bed.heights(pts, pct=95)
        self.assertEqual(h.shape, (3, 2))
        self.assertAlmostEqual(h[0, 1], 0.6, delta=0.05)
        self.assertAlmostEqual(h[2, 0], 0.3, delta=0.05)
        self.assertNotEqual(bed.lowest_cell(h), (0, 1))
        self.assertLess(h[bed.lowest_cell(h)], 0.15)
        ev = bed.evenness(h)
        self.assertAlmostEqual(ev["range"], h.max() - h.min())
        self.assertEqual(int(bed.counts(pts).sum()), 300)

    def test_terrain_grid_measures_removed_volume_of_a_cone(self):
        g = lp.TerrainGrid((4.7, 0.0), 2.5, 0.1, 0.0)
        xs = np.arange(2.2, 7.2, 0.025)
        X, Y = np.meshgrid(xs, np.arange(-2.5, 2.5, 0.025))
        r = np.hypot(X - 4.7, Y)
        H = np.clip(1.35 * (1 - r / 2.0), 0, None)
        pts0 = np.column_stack([X.ravel(), Y.ravel(), H.ravel()])
        h0 = g.heights(pts0, pct=90)
        vol_true = math.pi * 2.0 ** 2 * 1.35 / 3
        self.assertAlmostEqual(g.volume(h0), vol_true, delta=0.08 * vol_true)
        H2 = H.copy()
        H2[(X > 3.5) & (X < 4.3) & (np.abs(Y) < 0.4)] = 0.2          # a scoop taken out of the near face
        removed_true = float(np.sum(np.clip(H - H2, 0, None)) * 0.025 * 0.025)
        h1 = g.heights(np.column_stack([X.ravel(), Y.ravel(), H2.ravel()]), pct=90)
        self.assertAlmostEqual(g.removed_volume(h0, h1), removed_true, delta=0.15 * removed_true)

    def test_choose_dig_point_prefers_the_highest_reachable_cell(self):
        g = lp.TerrainGrid((4.7, 0.0), 2.5, 0.2, 0.0)
        h = np.zeros((g.n, g.n))
        xs = g.c[0] - g.half + (np.arange(g.n) + 0.5) * g.cell
        ys = g.c[1] - g.half + (np.arange(g.n) + 0.5) * g.cell
        for i, x in enumerate(xs):
            for j, y in enumerate(ys):
                h[i, j] = max(0.0, 1.35 * (1 - math.hypot(x - 4.7, y) / 2.0))
        x, y, hh = lp.choose_dig_point(g, h)
        self.assertLessEqual(math.hypot(x, y), lp.DIG_R_MAX + 1e-9)
        self.assertGreaterEqual(math.hypot(x, y), lp.DIG_R_MIN - 1e-9)
        self.assertLess(abs(y), 0.21)                                   # the peak is on the axis
        self.assertGreater(hh, 1.2)                                     # near the apex side of the band
        self.assertIsNone(lp.choose_dig_point(g, np.zeros_like(h)))

    def test_safety_monitor_has_hysteresis(self):
        s = lp.SafetyMonitor(stop_r=6.0, resume_r=7.5)
        self.assertFalse(s.update(0.0, 10.0))
        self.assertTrue(s.update(1.0, 5.9))
        self.assertTrue(s.update(2.0, 7.0))         # between the radii: still stopped
        self.assertFalse(s.update(3.0, 7.6))
        self.assertEqual([e["event"] for e in s.events], ["stop", "resume"])
        self.assertFalse(s.update(4.0, None))
        # a stop whose detection then vanishes (person walked out of the camera's view) releases after lost_s
        s2 = lp.SafetyMonitor(stop_r=6.0, resume_r=7.5, lost_s=2.0)
        self.assertTrue(s2.update(10.0, 3.0))
        self.assertTrue(s2.update(11.0, None))
        self.assertFalse(s2.update(12.1, None))
        self.assertEqual(s2.events[-1]["event"], "resume_lost")


if __name__ == "__main__":
    unittest.main()


@unittest.skipUnless(shutil.which("xacro"), "xacro not on PATH")
class ThreatMonitorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp()
        urdf = Path(cls.tmp) / "ecr88_demo.urdf"
        subprocess.run(["xacro", str(XACRO), "model_cylinders:=false", "-o", str(urdf)], check=True, capture_output=True)
        cls.m = UrdfModel(urdf)
        q0 = lp.carry_pose(cls.m, 0.0)[0]
        cls.swing_traj = lp.JointTrajectory(q0).move(dict(swing=90.0), "swing")     # the house turns left over ~4 s
        cls.still = lp.JointTrajectory(q0).hold(5.0)

    def points_fn(self, traj, t_now):
        def fn(tau):
            q, label = traj.sample(t_now + tau)
            return lp.machine_points_base(self.m, q)
        return fn

    def test_person_in_the_swing_path_stops_before_the_bucket_arrives(self):
        mon = lp.ThreatMonitor()
        # the bucket will pass through bearing 45 deg at radius ~4 m in a couple of seconds; the person stands there
        p = np.array([4.0 * math.cos(math.radians(45)), 4.0 * math.sin(math.radians(45))])
        level = mon.update(0.0, p, np.zeros(2), self.points_fn(self.swing_traj, 0.0))
        self.assertEqual(level, "stop")
        self.assertGreater(mon.last["tau"], 0.5)          # it is the FUTURE motion that threatens, not the present pose

    def test_person_behind_the_machine_does_not_stop_a_dig_in_front(self):
        mon = lp.ThreatMonitor()
        dig = lp.plan_dig(self.m, lp.carry_pose(self.m, 0.0)[0], (4.3, 0.0, 0.9))
        p = np.array([-4.5, 0.0])                            # 4.5 m behind the swing axis, standing still
        level = mon.update(0.0, p, np.zeros(2), self.points_fn(dig, 0.0))
        self.assertEqual(level, "clear")
        # ... but a distance zone of 6 m would have stopped it
        self.assertTrue(lp.SafetyMonitor(6.0, 7.5).update(0.0, 4.5))

    def test_person_walking_into_the_path_of_a_still_machine_is_only_a_slow(self):
        mon = lp.ThreatMonitor()
        # machine holds the carry pose (bucket centre ~4 m ahead); person 8 m ahead walking in at 1.2 m/s
        p, v = np.array([8.0, 0.0]), np.array([-1.2, 0.0])
        level = mon.update(0.0, p, v, self.points_fn(self.still, 0.0))
        self.assertIn(level, ("slow", "stop"))
        # same person standing still 8 m away: clear
        self.assertEqual(lp.ThreatMonitor().update(0.0, p, np.zeros(2), self.points_fn(self.still, 0.0)), "clear")

    def test_levels_escalate_at_once_and_relax_only_after_the_hold(self):
        mon = lp.ThreatMonitor(hold_s=1.0)
        fn = self.points_fn(self.swing_traj, 0.0)                 # the house is about to swing left
        near = np.array([4.0 * math.cos(math.radians(45)), 4.0 * math.sin(math.radians(45))])
        self.assertEqual(mon.update(0.0, near, np.zeros(2), fn), "stop")
        self.assertEqual(mon.update(0.5, np.array([20.0, 0.0]), np.zeros(2), fn), "stop")     # too soon to relax
        self.assertEqual(mon.update(1.6, np.array([20.0, 0.0]), np.zeros(2), fn), "clear")
        self.assertEqual([e["event"] for e in mon.events], ["clear->stop", "stop->clear"])
        self.assertEqual(mon.speed_scale, 1.0)
