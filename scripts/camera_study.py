#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
camera_study.py -- which sensor mount actually sees the work, through the whole cycle.

THE QUESTION
------------
"Where should the camera go" is usually settled by pointing at a render. That answer
does not survive the machine moving. The boom swings through the cab's sightline, the
panel stack on the fork hides itself behind the backrest, the tool's own camera loses
the target the moment it gets close. All of that is geometry, and geometry can be
counted.

So: walk the pick-and-place cycle, and for every candidate mount in the URDF
(cam_*), report the fraction of the cycle in which each target is BOTH inside the
field of view AND not hidden behind the machine's own structure.

WHAT IT DOES NOT MODEL
----------------------
Lens distortion, exposure, motion blur, the module's own glass reflecting, and the
site (rows, piles, other machines) -- occlusion is tested against THIS MACHINE only.
It is a structural sightline test, not a rendering. A mount that fails here will not
be rescued by a better sensor; a mount that passes still has to be checked in the
viewer with real lighting.

OCCLUSION MODEL
---------------
Every link's visual primitives become oriented boxes (a cylinder becomes the box that
contains it -- deliberately pessimistic, since a sightline that survives the box
survives the cylinder). A target is occluded if the segment from the camera to it hits
any box other than those belonging to the camera's own link or the target's own link.

Run it on the FLAT URDF, not the xacro, and it needs no Isaac Sim:
    xacro assets/ecr88/urdf/ecr88.urdf.xacro -o build/ecr88.urdf
    python3 scripts/camera_study.py --urdf build/ecr88.urdf
"""
from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


# The same cycle animate_cycle.py authors: (swing, boom, arm, bucket) in degrees.
CYCLE = [
    ("parked",      0.0, -30.0, 110.0,  20.0),
    ("to stack",  -55.0, -32.0,  95.0,  10.0),
    ("at stack",  -55.0, -28.0,  70.0, -10.0),
    ("grip",      -55.0, -28.0,  70.0, -10.0),
    ("lift",      -55.0, -52.0,  80.0,   0.0),
    ("slew",       45.0, -52.0,  80.0,   0.0),
    ("set down",   45.0, -33.0,  55.0, -20.0),
    ("release",    45.0, -33.0,  55.0, -20.0),
    ("retract",    45.0, -58.0,  90.0,   0.0),
    ("home",        0.0, -30.0, 110.0,  20.0),
]
FOV_H_DEG, FOV_V_DEG = 80.0, 60.0
SUBSTEPS = 6            # interpolated poses between keyframes


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_R(a, t):
    a = np.asarray(a, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K


class Model:
    def __init__(self, urdf: Path):
        root = ET.parse(urdf).getroot()
        self.J, self.boxes = {}, []
        for j in root.findall("joint"):
            o, ax, mi = j.find("origin"), j.find("axis"), j.find("mimic")
            self.J[j.get("name")] = dict(
                type=j.get("type"), parent=j.find("parent").get("link"),
                child=j.find("child").get("link"),
                xyz=np.array([float(v) for v in (o.get("xyz", "0 0 0").split()
                                                 if o is not None else "0 0 0".split())]),
                rpy=np.array([float(v) for v in (o.get("rpy", "0 0 0").split()
                                                 if o is not None else "0 0 0".split())]),
                axis=(np.array([float(v) for v in ax.get("xyz").split()])
                      if ax is not None else np.array([1.0, 0, 0])),
                mimic=((mi.get("joint"), float(mi.get("multiplier")), float(mi.get("offset")))
                       if mi is not None else None))
        self.par = {v["child"]: k for k, v in self.J.items()}
        self.links = [l.get("name") for l in root.findall("link")]
        for l in root.findall("link"):
            for v in l.findall("visual"):
                g, o = v.find("geometry"), v.find("origin")
                kid = list(g)[0] if len(g) else None
                if kid is None:
                    continue
                xyz = np.array([float(x) for x in (o.get("xyz", "0 0 0").split()
                                                   if o is not None else "0 0 0".split())])
                rpy = np.array([float(x) for x in (o.get("rpy", "0 0 0").split()
                                                   if o is not None else "0 0 0".split())])
                if kid.tag == "box":
                    half = np.array([float(x) for x in kid.get("size").split()]) / 2.0
                elif kid.tag == "cylinder":
                    r, ln = float(kid.get("radius")), float(kid.get("length"))
                    half = np.array([r, r, ln / 2.0])      # cylinder's axis is +Z
                elif kid.tag == "sphere":
                    r = float(kid.get("radius"))
                    half = np.array([r, r, r])
                else:
                    continue
                # Markers are not structure; they must not block a sightline.
                if float(np.max(half)) <= 0.05:
                    continue
                self.boxes.append((l.get("name"), xyz, rpy_to_R(*rpy), half))

    def q(self, name, pose):
        j = self.J[name]
        if j["mimic"]:
            d, m, c = j["mimic"]
            return m * pose.get(d, 0.0) + c
        return pose.get(name, 0.0)

    def fk(self, link, pose):
        T, chain, cur = np.eye(4), [], link
        while cur in self.par:
            jn = self.par[cur]
            chain.append(jn)
            cur = self.J[jn]["parent"]
        for jn in reversed(chain):
            j = self.J[jn]
            A = np.eye(4)
            A[:3, :3] = rpy_to_R(*j["rpy"])
            A[:3, 3] = j["xyz"]
            v = self.q(jn, pose)
            if j["type"] in ("revolute", "continuous"):
                B = np.eye(4)
                B[:3, :3] = axis_R(j["axis"], v)
                A = A @ B
            elif j["type"] == "prismatic":
                B = np.eye(4)
                B[:3, 3] = j["axis"] * v
                A = A @ B
            T = T @ A
        return T


def seg_hits_obb(p0, p1, centre, R, half) -> bool:
    """Slab test in the box's own frame."""
    o = R.T @ (p0 - centre)
    d = R.T @ (p1 - p0)
    tmin, tmax = 0.0, 1.0
    for k in range(3):
        if abs(d[k]) < 1e-12:
            if abs(o[k]) > half[k]:
                return False
            continue
        t1 = (-half[k] - o[k]) / d[k]
        t2 = (half[k] - o[k]) / d[k]
        if t1 > t2:
            t1, t2 = t2, t1
        tmin, tmax = max(tmin, t1), min(tmax, t2)
        if tmin > tmax:
            return False
    return True


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", type=Path, required=True, help="FLAT urdf (expand the xacro first)")
    ap.add_argument("--fov-h", type=float, default=FOV_H_DEG)
    ap.add_argument("--fov-v", type=float, default=FOV_V_DEG)
    ap.add_argument("--ignore-tool-adapter", action="store_true",
                    help="do not let the tiltrotator adapter chain occlude. Those links "
                         "are drawn as solid cylinders spanning the whole tool axis, "
                         "which is far more solid than the real slim frame, and they "
                         "block every sightline to the cups. Run BOTH ways: the "
                         "difference is exactly how much a conclusion depends on tool "
                         "geometry we have not modelled properly.")
    args = ap.parse_args(argv)
    if not args.urdf.is_file():
        sys.stderr.write(f"\n[FATAL] no such urdf: {args.urdf}\n")
        return 2

    m = Model(args.urdf)
    cams = sorted(l for l in m.links if l.startswith("cam_"))
    # A sensor is bolted to a link's SURFACE, but that link is represented here by a
    # convex box that over-covers it, so the mount point lands INSIDE the proxy and
    # every ray leaving it is reported blocked by its own mount. First run of this
    # script scored 0% almost everywhere for exactly that reason. Exclude the link a
    # camera is mounted on; it cannot occlude a sensor sitting on its own skin.
    mount_of = {c: m.J[m.par[c]]["parent"] for c in cams if c in m.par}
    TOOL_ADAPTER = {"tilt_mount_link", "tilt_link", "rotator_link",
                    "attachment_link", "probe_link"}
    ignore = TOOL_ADAPTER if args.ignore_tool_adapter else set()
    if not cams:
        sys.stderr.write("\n[FATAL] no cam_* frames in this URDF. Build with "
                         "model_sensor_mounts:=true.\n")
        return 2

    # Targets. 'pick' and 'tcp' ride the machine; 'place' is a fixed point out on the
    # row, taken in the base frame so it does not move when the house slews.
    def targets(pose):
        t = {}
        if "panel_stack_link" in m.links:
            T = m.fk("panel_stack_link", pose)
            t["pick (stack top)"] = (T @ np.array([0, 0, 0.30, 1.0]))[:3], "panel_stack_link"
        T = m.fk("contact_surface_link", pose)
        t["tcp (cups)"] = (T @ np.array([0, 0, 0, 1.0]))[:3], "contact_surface_link"
        t["place (row)"] = np.array([5.0, 4.2, -0.35]), None
        return t

    poses = []
    for i in range(len(CYCLE) - 1):
        a, b = CYCLE[i], CYCLE[i + 1]
        for s in range(SUBSTEPS):
            f = s / SUBSTEPS
            vals = [a[k + 1] + (b[k + 1] - a[k + 1]) * f for k in range(4)]
            poses.append((a[0], dict(zip(
                ("swing_joint", "boom_joint", "arm_joint", "bucket_joint"),
                [math.radians(v) for v in vals]))))

    tan_h = math.tan(math.radians(args.fov_h) / 2.0)
    tan_v = math.tan(math.radians(args.fov_v) / 2.0)
    tnames = list(targets(poses[0][1]).keys())
    score = {c: {t: 0 for t in tnames} for c in cams}
    dists = {c: {t: [] for t in tnames} for c in cams}

    for _, pose in poses:
        world = {}
        for lk, xyz, R, half in m.boxes:
            T = m.fk(lk, pose)
            world.setdefault(lk, []).append(
                ((T @ np.append(xyz, 1))[:3], T[:3, :3] @ R, half))
        tg = targets(pose)
        for c in cams:
            Tc = m.fk(c, pose)
            eye, Rc = Tc[:3, 3], Tc[:3, :3]
            for tn, (pt, own) in tg.items():
                v = Rc.T @ (pt - eye)
                if v[0] <= 1e-6:
                    continue                              # behind the sensor
                if abs(v[1]) > tan_h * v[0] or abs(v[2]) > tan_v * v[0]:
                    continue                              # outside the FOV cone
                blocked = False
                for lk, lst in world.items():
                    if lk == c or lk == own or lk == mount_of.get(c) or lk in ignore:
                        continue
                    for centre, R, half in lst:
                        if seg_hits_obb(eye, pt, centre, R, half):
                            blocked = True
                            break
                    if blocked:
                        break
                if not blocked:
                    score[c][tn] += 1
                    dists[c][tn].append(float(np.linalg.norm(pt - eye)))

    n = len(poses)
    print("=" * 92)
    print(f"  ECR88 sensor mount study   {n} poses over the pick-and-place cycle")
    mode = ("tool adapter IGNORED as an occluder"
            if args.ignore_tool_adapter else "every link occludes, adapter included")
    print(f"  FOV {args.fov_h:g} x {args.fov_v:g} deg   {mode}")
    print("=" * 92)
    head = f"  {'mount':18s}" + "".join(f"{t:>22s}" for t in tnames) + f"{'총점':>8s}"
    print(head)
    print("  " + "-" * (len(head) - 2))
    ranked = sorted(cams, key=lambda c: -sum(score[c].values()))
    for c in ranked:
        row = f"  {c:18s}"
        for t in tnames:
            pct = 100.0 * score[c][t] / n
            d = np.mean(dists[c][t]) if dists[c][t] else float("nan")
            row += f"{pct:9.0f}% {('%.1fm' % d) if dists[c][t] else '  --':>11s}"
        row += f"{100.0 * sum(score[c].values()) / (n * len(tnames)):7.0f}%"
        print(row)
    print("=" * 92)
    print("  %  = fraction of the cycle where the target is in view AND unoccluded")
    print("  m  = mean distance to the target while it is visible")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
