#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
optimize_cameras.py -- search mounting positions and angles instead of guessing them.

WHY THIS EXISTS
---------------
camera_study.py scores the mounts somebody already chose. That answers "is this one
any good", not "where should it go". The hand-picked set was hand-picked: cam_tool
looked hopeless until the lens got wider, and cam_house_front was pointed 20 deg into
the dirt because 20 deg sounded reasonable. Both were settled by argument, and both
were wrong.

The scoring function is cheap, so search it.

WHAT IS SEARCHED
----------------
Per mountable link, a grid of positions on plausible mounting surfaces crossed with
pitch and yaw. Each candidate is scored exactly the way camera_study.py scores one:
inside the field of view, and not hidden behind the machine's own structure, during
the part of the cycle where that target actually matters.

THE SET MATTERS MORE THAN THE MOUNT
-----------------------------------
"Best camera" is the wrong question; one camera cannot see the stack under the nose
AND the row six metres away. The real question is which TWO OR THREE together leave
nothing uncovered, so after ranking individual candidates this does a greedy set
cover over (target, cycle-pose) pairs. The second camera is chosen for what the first
one MISSES, which is not the same as the second-best camera.

WHAT IT STILL DOES NOT KNOW
---------------------------
Whether a spot is reachable for cabling, whether it survives vibration and mud, or
whether the operator's door opens into it. Everything here is sightlines. Treat the
output as a shortlist to walk the real machine with, not a bill of materials.
"""
from __future__ import annotations

import argparse
import importlib.util
import itertools
import json
import math
import sys
from pathlib import Path

import numpy as np


def _load_study():
    """Reuse camera_study's URDF model, FK and occlusion test rather than fork them."""
    here = Path(__file__).resolve().parent
    spec = importlib.util.spec_from_file_location("cstudy", here / "camera_study.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main(argv=None) -> int:
    cs = _load_study()

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", type=Path, required=True)
    ap.add_argument("--sensors", type=Path, default=None)
    ap.add_argument("--hfov", type=float, default=None,
                    help="evaluate every candidate at this FOV instead of sensors.json. "
                         "Worth sweeping: the lens changes the answer as much as the "
                         "mount does.")
    ap.add_argument("--ignore-tool-adapter", action="store_true",
                    help="same meaning as in camera_study.py")
    ap.add_argument("--top", type=int, default=3, help="candidates to list per link")
    ap.add_argument("--set-size", type=int, default=3, help="how many cameras to cover with")
    args = ap.parse_args(argv)
    if not args.urdf.is_file():
        sys.stderr.write(f"\n[FATAL] no such urdf: {args.urdf}\n")
        return 2

    m = cs.Model(args.urdf)

    default_hfov = 90.0
    if args.hfov is None:
        spath = args.sensors or (Path(__file__).resolve().parent.parent
                                 / "assets" / "ecr88" / "sensors.json")
        try:
            optics = {k: v for k, v in json.loads(
                spath.read_text(encoding="utf-8")).items() if not k.startswith("_")}
            default_hfov = float(optics.get("default", {}).get("hfov", 90.0))
        except OSError:
            pass
    hfov = args.hfov if args.hfov is not None else default_hfov
    tan_h = math.tan(math.radians(hfov) / 2.0)
    tan_v = tan_h * 0.75

    # ---- where a sensor could physically go -------------------------------- #
    # Anchors are in each link's own frame. They sit on surfaces a bracket could
    # reach: the house's front face and roof line, the outboard side of the boom and
    # arm, around the tool, and the fork backrest. Nothing is placed inside structure.
    P = m  # for brevity below

    def prop(*names):
        """Pull a numeric that the flat URDF still carries, via a known joint origin."""
        return None

    anchors = {
        "house_link": [
            ("front-low",  np.array([1.00, -0.55, 0.30])),
            ("front-high", np.array([1.00, -0.55, 0.75])),
            ("cab-roof",   np.array([0.55,  0.75, 1.30])),
            ("cab-side",   np.array([0.85,  0.20, 0.95])),
            ("front-mid",  np.array([1.05,  0.00, 0.55])),
        ],
        "boom_link": [
            ("root",  np.array([0.60, -0.28, 0.35])),
            ("mid",   np.array([1.60, -0.28, 0.85])),
            ("tip",   np.array([2.90, -0.28, 0.55])),
        ],
        "arm_link": [
            ("root", np.array([0.35, -0.24, 0.30])),
            ("mid",  np.array([1.10, -0.24, 0.25])),
            ("tip",  np.array([1.75, -0.24, 0.22])),
        ],
        "rotator_link": [
            ("side",  np.array([0.00, -0.30, -0.12])),
            ("side2", np.array([0.22, -0.22, -0.12])),
            ("axis",  np.array([0.00,  0.00, -0.12])),
        ],
        "fork_link": [
            ("mast-top", np.array([0.00,  0.50, 1.25])),
            ("mast-mid", np.array([0.00,  0.50, 0.85])),
            ("mast-ctr", np.array([0.00,  0.00, 1.25])),
        ],
    }
    anchors = {k: v for k, v in anchors.items() if k in m.links}
    pitches = [-10, 0, 10, 20, 30, 40, 50, 60, 75, 90]
    yaws = [-60, -30, -15, 0, 15, 30, 60]

    # ---- the cycle --------------------------------------------------------- #
    poses = []
    for i in range(len(cs.CYCLE) - 1):
        a, b = cs.CYCLE[i], cs.CYCLE[i + 1]
        for s in range(cs.SUBSTEPS):
            f = s / cs.SUBSTEPS
            vals = [a[k + 1] + (b[k + 1] - a[k + 1]) * f for k in range(4)]
            poses.append((i, dict(zip(
                ("swing_joint", "boom_joint", "arm_joint", "bucket_joint"),
                [math.radians(v) for v in vals]))))
    n = len(poses)

    TOOL_ADAPTER = {"tilt_mount_link", "tilt_link", "rotator_link",
                    "attachment_link", "probe_link"}
    ignore = TOOL_ADAPTER if args.ignore_tool_adapter else set()

    # Precompute per pose: world boxes, targets, and every mount link's transform.
    pre = []
    for seg, pose in poses:
        boxes = []
        for lk, xyz, R, half in m.boxes:
            T = m.fk(lk, pose)
            boxes.append((lk, (T @ np.append(xyz, 1))[:3], T[:3, :3] @ R, half))
        tg = cs.targets(m, pose)
        lt = {lk: m.fk(lk, pose) for lk in anchors}
        pre.append((seg, boxes, tg, lt))

    tnames = list(pre[0][2].keys())
    win = {t: [i for i, (seg, *_ ) in enumerate(pre) if seg in cs.PHASE.get(t, ())]
           for t in tnames}

    def rot_py(pitch_deg, yaw_deg):
        cp, sp = math.cos(math.radians(pitch_deg)), math.sin(math.radians(pitch_deg))
        cy, sy = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        return Rz @ Ry

    cands = []
    total = sum(len(v) for v in anchors.values()) * len(pitches) * len(yaws)
    print(f"  후보 {total}개 x {n} 포즈 평가 중 ...", file=sys.stderr)

    for link, alist in anchors.items():
        for aname, off in alist:
            for pit, yaw in itertools.product(pitches, yaws):
                Rl = rot_py(pit, yaw)
                cover = {t: set() for t in tnames}
                for idx, (seg, boxes, tg, lt) in enumerate(pre):
                    T = lt[link]
                    eye = (T @ np.append(off, 1))[:3]
                    Rc = T[:3, :3] @ Rl
                    for tn, (pt, own) in tg.items():
                        if idx not in win[tn]:
                            continue
                        v = Rc.T @ (pt - eye)
                        if v[0] <= 1e-6 or abs(v[1]) > tan_h * v[0] or abs(v[2]) > tan_v * v[0]:
                            continue
                        hit = False
                        for lk, ctr, R, half in boxes:
                            if lk == link or lk == own or lk in ignore:
                                continue
                            if cs.seg_hits_obb(eye, pt, ctr, R, half):
                                hit = True
                                break
                        if not hit:
                            cover[tn].add(idx)
                cands.append(dict(link=link, anchor=aname, pitch=pit, yaw=yaw,
                                  off=off, cover=cover,
                                  score=sum(len(cover[t]) / max(len(win[t]), 1)
                                            for t in tnames)))

    print("=" * 96)
    print(f"  카메라 배치 탐색   후보 {len(cands)}개, 사이클 {n} 포즈, 화각 {hfov:g} deg")
    print(f"  {'툴 어댑터 가림 제외' if args.ignore_tool_adapter else '모든 링크가 가림'}")
    print("=" * 96)

    for link in anchors:
        sub = sorted((c for c in cands if c["link"] == link),
                     key=lambda c: -c["score"])[:args.top]
        print(f"\n  [{link}]")
        for c in sub:
            cov = "  ".join(f"{t.split()[0]} {100*len(c['cover'][t])/max(len(win[t]),1):3.0f}%"
                            for t in tnames)
            print(f"    {c['anchor']:10s} pitch {c['pitch']:+4d} yaw {c['yaw']:+4d}   {cov}")

    # ---- greedy set cover -------------------------------------------------- #
    print("\n" + "=" * 96)
    print(f"  조합 탐색: 겹치지 않게 {args.set_size}대까지, 매번 '아직 못 보는 것'을 가장 많이 메우는 후보")
    print("=" * 96)
    need = {t: set(win[t]) for t in tnames}
    chosen = []
    for _ in range(args.set_size):
        best, gain = None, 0
        for c in cands:
            g = sum(len(need[t] & c["cover"][t]) for t in tnames)
            if g > gain:
                best, gain = c, g
        if best is None or gain == 0:
            break
        chosen.append((best, gain))
        for t in tnames:
            need[t] -= best["cover"][t]
    for k, (c, gain) in enumerate(chosen, 1):
        cov = "  ".join(f"{t.split()[0]} {100*len(c['cover'][t])/max(len(win[t]),1):3.0f}%"
                        for t in tnames)
        print(f"  {k}. {c['link']:14s} {c['anchor']:10s} pitch {c['pitch']:+4d} yaw {c['yaw']:+4d}")
        print(f"     xyz(링크 기준) = {np.round(c['off'], 3).tolist()}")
        print(f"     {cov}     새로 메운 (타깃,포즈) {gain}개")
    left = {t: len(need[t]) for t in tnames if need[t]}
    if left:
        print(f"\n  남은 미커버: " + ", ".join(f"{t} {v}/{len(win[t])}" for t, v in left.items()))
    else:
        print(f"\n  {len(chosen)}대로 전 구간 100% 커버")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
