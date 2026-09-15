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


# The work cycle and its scoring windows live in assets/ecr88/cycle.json, shared with
# animate_cycle.py so the poses scored here are exactly the poses the scene plays.
# (Both scripts used to carry their own copy of the keyframes.) Segment s runs from
# CYCLE[s] to CYCLE[s+1]. Scoring a target over the WHOLE cycle punishes a camera for
# not seeing something it has no business seeing yet, so each target is scored inside
# its window; the all-cycle number is kept beside it for situational awareness.
CYCLE_FILE = Path(__file__).resolve().parent.parent / "assets" / "ecr88" / "cycle.json"


def load_cycle(path=CYCLE_FILE):
    import json
    d = json.loads(Path(path).read_text(encoding="utf-8"))
    cycle = [(k["name"], k["swing"], k["boom"], k["arm"], k["bucket"]) for k in d["keyframes"]]
    phase = {t: tuple(v) for t, v in d["windows"].items() if not t.startswith("_")}
    return cycle, phase


CYCLE, PHASE = load_cycle()
FOV_H_DEG, FOV_V_DEG = 80.0, 60.0
SUBSTEPS = 6            # interpolated poses between keyframes


def targets(m, pose):
    """What a mount has to see, per pose: {name: (world point, link excluded as occluder)}.

    pick (panel edge): the middle of the outermost module's UPPER edge, 1 cm proud of its
        face. The firmware's pick frame panelTop (panel_top_link) is the face centre, but
        that point is under the suction pad from ~0.3 m out until release, so no camera
        can or needs to see it; what a camera can use to judge the approach is the edge
        showing around the pad. panelTop X points to the ground (ECR88D_*.m comment on
        distPanelTopToLiftPosn), so "up the face" is -X. The stack is NOT excluded as an
        occluder: the modules stand facing forward, away from the machine, so a camera
        behind the backrest or on the house cannot see the face, and should not score it.
    tcp (cups): the contact surface, excluding the tool it belongs to.
    place (row): a fixed point on the working row, in the base frame.
    """
    t = {}
    if "panel_top_link" in m.links:
        T = m.fk("panel_top_link", pose)
        half_h = next(h for lk, _, _, h in m.boxes if lk == "panel_stack_link")[2]
        t["pick (panel edge)"] = (T @ np.array([-half_h, 0.0, 0.01, 1.0]))[:3], None
    T = m.fk("contact_surface_link", pose)
    t["tcp (cups)"] = (T @ np.array([0, 0, 0, 1.0]))[:3], "contact_surface_link"
    t["place (row)"] = np.array([5.0, 4.2, -0.35]), None
    return t


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
    ap.add_argument("--sensors", type=Path, default=None,
                    help="per-camera optics json, shared with add_cameras.py. "
                         "Default: assets/ecr88/sensors.json")
    ap.add_argument("--fov-h", type=float, default=None,
                    help="override every camera's horizontal FOV, degrees")
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
    def targets_at(pose):
        return targets(m, pose)

    poses = []
    for i in range(len(CYCLE) - 1):
        a, b = CYCLE[i], CYCLE[i + 1]
        for s in range(SUBSTEPS):
            f = s / SUBSTEPS
            vals = [a[k + 1] + (b[k + 1] - a[k + 1]) * f for k in range(4)]
            poses.append((i, a[0], dict(zip(
                ("swing_joint", "boom_joint", "arm_joint", "bucket_joint"),
                [math.radians(v) for v in vals]))))

    import json
    spath = args.sensors or (Path(__file__).resolve().parent.parent
                             / "assets" / "ecr88" / "sensors.json")
    try:
        optics = {k: v for k, v in json.loads(
            Path(spath).read_text(encoding="utf-8")).items() if not k.startswith("_")}
    except OSError:
        optics = {"default": {"hfov": 90.0}}
        print(f"  (no sensors file at {spath}; 90 deg assumed for every camera)")

    def hfov_of(nm):
        if args.fov_h is not None:
            return args.fov_h
        return float(optics.get(nm, optics.get("default", {})).get(
            "hfov", optics.get("default", {}).get("hfov", 90.0)))

    # 4:3 sensor, so the vertical follows the horizontal rather than being set apart.
    tan = {c: (math.tan(math.radians(hfov_of(c)) / 2.0),
               math.tan(math.radians(hfov_of(c)) / 2.0) * 0.75) for c in cams}
    tnames = list(targets_at(poses[0][2]).keys())
    score = {c: {t: 0 for t in tnames} for c in cams}
    inwin = {c: {t: 0 for t in tnames} for c in cams}
    dists = {c: {t: [] for t in tnames} for c in cams}
    winlen = {t: sum(1 for seg, _, _ in poses if seg in PHASE.get(t, ()))
              for t in tnames}

    for seg, _, pose in poses:
        world = {}
        for lk, xyz, R, half in m.boxes:
            T = m.fk(lk, pose)
            world.setdefault(lk, []).append(
                ((T @ np.append(xyz, 1))[:3], T[:3, :3] @ R, half))
        tg = targets_at(pose)
        for c in cams:
            Tc = m.fk(c, pose)
            eye, Rc = Tc[:3, 3], Tc[:3, :3]
            for tn, (pt, own) in tg.items():
                v = Rc.T @ (pt - eye)
                if v[0] <= 1e-6:
                    continue                              # behind the sensor
                th, tv = tan[c]
                if abs(v[1]) > th * v[0] or abs(v[2]) > tv * v[0]:
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
                    if seg in PHASE.get(tn, ()):
                        inwin[c][tn] += 1
                    dists[c][tn].append(float(np.linalg.norm(pt - eye)))

    n = len(poses)
    print("=" * 92)
    print(f"  ECR88 sensor mount study   {n} poses over the pick-and-place cycle")
    mode = ("tool adapter IGNORED as an occluder"
            if args.ignore_tool_adapter else "every link occludes, adapter included")
    fovs = ", ".join(f"{c.replace('cam_',''):s} {hfov_of(c):.0f}" for c in
                     sorted(cams, key=lambda x: -hfov_of(x))[:3])
    print(f"  FOV from sensors.json (widest: {fovs} deg)   {mode}")
    print("=" * 92)
    print(f"  구간(window) = 그 타깃을 실제로 봐야 하는 사이클 구간만 집계")
    for t in tnames:
        segs = PHASE.get(t, ())
        names = " / ".join(CYCLE[s][0] for s in segs)
        print(f"     {t:18s} {winlen[t]:3d} poses  [{names}]")
    print("-" * 92)
    head = f"  {'mount':18s}" + "".join(f"{t:>26s}" for t in tnames)
    print(head)
    print(f"  {'':18s}" + "".join(f"{'구간':>10s}{'전체':>7s}{'거리':>9s}" for _ in tnames))
    print("  " + "-" * (len(head) - 2))

    def keyf(c):
        return -sum(inwin[c][t] / max(winlen[t], 1) for t in tnames)

    for c in sorted(cams, key=keyf):
        row = f"  {c:18s}"
        for t in tnames:
            w = 100.0 * inwin[c][t] / max(winlen[t], 1)
            a = 100.0 * score[c][t] / n
            d = np.mean(dists[c][t]) if dists[c][t] else float("nan")
            row += f"{w:9.0f}%{a:6.0f}%{('%.1fm' % d) if dists[c][t] else '   --':>9s}"
        print(row)
    print("=" * 92)
    print("  구간 = 그 타깃을 봐야 하는 구간에서 보이고 가려지지 않은 비율  <- 판단 기준")
    print("  전체 = 사이클 전 구간 기준 (상황인지용 참고)")
    print("  거리 = 보이는 동안의 평균 거리")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
