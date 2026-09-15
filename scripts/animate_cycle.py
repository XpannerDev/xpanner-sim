#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
animate_cycle.py -- author a panel-lift work cycle onto an ECR88 scene.

WHY AN ANIMATION AND NOT A CONTROL SCRIPT
-----------------------------------------
The machine is already being watched through a live WebRTC session, and that
session is its own Isaac Sim process. A second process cannot drive it, and
starting one would fight this one for the GPU and the streaming ports on a shared
box. Time samples go INTO the stage instead: open the scene in the session that is
already running, press Play, and the machine moves. Nothing else has to change.

HOW IT DRIVES
-------------
Samples are written to each joint's drive:*:physics:targetPosition, not to its
state. The articulation is then doing what it would do under a real controller --
chasing a moving setpoint through the physics solver -- so the pose you watch is a
pose the machine can actually hold, and the hydraulic cylinders follow through
their mimic constraints without being animated themselves.

Mimic-slaved joints are deliberately NOT given samples. Their drives were zeroed
at conversion time, so a target on them does nothing; the mimic is what moves them.

THE CYCLE
---------
One pick-and-place the way the X1Exc firmware runs it (see cycle.json): tool raised,
approach the outermost module standing on the machine's own front fork, meet its face,
grip, lift clear of the stack, swing left to the working row, set down, release, return.
(Until 2026-09-14 it picked off a ground pallet on the right, which the machine does not
do: it carries its modules on the fork.)

Every keyframe is inside the joint limits this asset carries; the script checks
that and refuses rather than authoring a pose the machine would have to be dragged
into. Angles are degrees, which is what USD uses for angular drives.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path


# (frame, swing, boom, arm, bucket) in degrees, from assets/ecr88/cycle.json -- the same
# file camera_study.py scores, so the scene plays exactly the poses the sensor study used.
import json as _json
_CYCLE_FILE = Path(__file__).resolve().parent.parent / "assets" / "ecr88" / "cycle.json"
CYCLE = [(k["frame"], k["swing"], k["boom"], k["arm"], k["bucket"])
         for k in _json.loads(_CYCLE_FILE.read_text(encoding="utf-8"))["keyframes"]]
JOINTS = ("swing_joint", "boom_joint", "arm_joint", "bucket_joint")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stage", type=Path, required=True,
                    help="scene or robot USD to animate, edited in place")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--loops", type=int, default=3,
                    help="how many times to repeat the cycle (default 3)")
    args = ap.parse_args(argv)

    if not args.stage.is_file():
        sys.stderr.write(f"\n[FATAL] stage not found: {args.stage}\n")
        return 2

    try:
        from isaacsim import SimulationApp
    except ModuleNotFoundError as exc:
        sys.stderr.write("\n[FATAL] run this through /isaac-sim/python.sh inside the "
                         f"container. ({exc})\n")
        return 4
    app = SimulationApp({"headless": True})

    from pxr import Sdf, Usd, UsdPhysics

    stage = Usd.Stage.Open(str(args.stage))
    if stage is None:
        app.close()
        sys.stderr.write(f"\n[FATAL] could not open {args.stage}\n")
        return 2

    found, limits, mimicked = {}, {}, set()
    for prim in stage.Traverse():
        if not (prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint)):
            continue
        name = prim.GetName()
        rel = prim.GetRelationship("newton:mimicJoint")
        if rel and rel.GetTargets():
            mimicked.add(name)
        if name in JOINTS:
            found[name] = prim
            lo = prim.GetAttribute("physics:lowerLimit")
            hi = prim.GetAttribute("physics:upperLimit")
            limits[name] = (lo.Get() if lo and lo.HasAuthoredValue() else None,
                            hi.Get() if hi and hi.HasAuthoredValue() else None)

    missing = [j for j in JOINTS if j not in found]
    if missing:
        app.close()
        sys.stderr.write(f"\n[FATAL] joints not in this stage: {missing}\n"
                         f"        Is {args.stage.name} an ECR88 scene?\n")
        return 2

    # Refuse rather than author a pose the machine cannot hold.
    bad = []
    for frame, *vals in CYCLE:
        for j, v in zip(JOINTS, vals):
            lo, hi = limits[j]
            if lo is not None and hi is not None and not (lo - 1e-6 <= v <= hi + 1e-6):
                bad.append(f"frame {frame}: {j} = {v:g} outside [{lo:g}, {hi:g}]")
    if bad:
        app.close()
        sys.stderr.write("\n[FATAL] keyframes violate joint limits:\n  "
                         + "\n  ".join(bad) + "\n")
        return 3

    period = CYCLE[-1][0]
    stage.SetTimeCodesPerSecond(args.fps)
    stage.SetStartTimeCode(0.0)
    stage.SetEndTimeCode(float(period * args.loops))

    for name, prim in found.items():
        col = JOINTS.index(name) + 1
        attr = prim.GetAttribute("drive:angular:physics:targetPosition")
        if not attr:
            attr = prim.CreateAttribute("drive:angular:physics:targetPosition",
                                        Sdf.ValueTypeNames.Float)
        attr.Clear()
        for loop in range(args.loops):
            for kf in CYCLE:
                # The last key of one loop and the first of the next are the same
                # pose, so skip the duplicate or the machine pauses a frame there.
                if loop and kf[0] == 0:
                    continue
                attr.Set(float(kf[col]), Usd.TimeCode(loop * period + kf[0]))

    stage.GetRootLayer().Save()
    secs = period * args.loops / args.fps
    print(f"[anim] {args.stage}")
    print(f"[anim] {len(CYCLE)} keyframes x {args.loops} loops = "
          f"{period * args.loops} frames @ {args.fps:g} fps = {secs:.1f} s")
    print(f"[anim] animated: {', '.join(sorted(found))}")
    print(f"[anim] left to the mimic constraints: "
          f"{', '.join(sorted(mimicked)) if mimicked else 'none'}")
    print("[anim] open the stage, press Play (physics) and play the timeline")
    app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
