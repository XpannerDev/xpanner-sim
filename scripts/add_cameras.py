#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
add_cameras.py -- put real USD cameras on the cam_* mount frames so you can look
through them while the machine works.

WHY IT IS A SEPARATE STEP
-------------------------
The cam_* frames in the URDF are coordinate frames and nothing else. URDF has no way
to say "camera", so the importer gives them an Xform and a marker sphere, and the
viewport's camera list never hears about them. camera_study.py can reason about them
numerically because it only needs a pose; a human looking through one needs an actual
UsdGeom.Camera prim. This script adds those, in place, after the import.

THE AXIS FLIP THAT MATTERS
--------------------------
The mounts follow the robotics convention this project uses everywhere else: +X is the
optical axis, +Z is up. A USD camera looks down its own -Z with +Y up. Bolting a camera
onto a mount without rotating it points it at the machine's own side. The fixed
rotation below is that conversion, applied once per camera:

    camera +X  ->  mount -Y      (image right is the mount's left)
    camera +Y  ->  mount +Z      (image up is up)
    camera -Z  ->  mount +X      (it looks where the mount looks)

FOCAL LENGTH
------------
Given from the horizontal field of view so it matches what camera_study.py assumed:
f = (horizontal aperture / 2) / tan(HFOV / 2). Default 80 deg on the USD standard
20.955 mm aperture gives 12.5 mm. If you change --hfov here, change --fov-h there, or
the numbers and the picture stop describing the same sensor.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stage", type=Path, required=True, help="scene USD, edited in place")
    ap.add_argument("--hfov", type=float, default=80.0, help="horizontal FOV, degrees")
    ap.add_argument("--near", type=float, default=0.05)
    ap.add_argument("--far", type=float, default=200.0)
    args = ap.parse_args(argv)

    if not args.stage.is_file():
        sys.stderr.write(f"\n[FATAL] stage not found: {args.stage}\n")
        return 2
    try:
        from isaacsim import SimulationApp
    except ModuleNotFoundError as exc:
        sys.stderr.write(f"\n[FATAL] run through /isaac-sim/python.sh. ({exc})\n")
        return 4
    app = SimulationApp({"headless": True})

    from pxr import Gf, Sdf, Usd, UsdGeom

    stage = Usd.Stage.Open(str(args.stage))
    if stage is None:
        app.close()
        sys.stderr.write(f"\n[FATAL] could not open {args.stage}\n")
        return 2

    mounts = [p for p in stage.Traverse() if p.GetName().startswith("cam_")
              and not p.GetName().endswith("_joint")
              and not p.GetPath().pathString.endswith("/sensor")]
    if not mounts:
        app.close()
        sys.stderr.write("\n[FATAL] no cam_* prims in this stage. Rebuild the robot with "
                         "model_sensor_mounts:=true and re-run build_site.py.\n")
        return 2

    aperture = 20.955                       # USD default horizontal aperture, mm
    focal = (aperture / 2.0) / math.tan(math.radians(args.hfov) / 2.0)

    # ROWS, not columns. USD's Gf.Matrix4d is row-major and transforms ROW vectors
    # (v * M), so a change of basis puts the new axes in the rows. Writing them as
    # columns transposes the rotation, which is a real rotation too -- so nothing
    # errors, the cameras just end up 90 deg off looking at the machine's flank.
    # Caught by dotting each camera's -Z against its mount's +X: every one read 0.000.
    #   row 0 = camera +X in the mount frame = mount -Y
    #   row 1 = camera +Y in the mount frame = mount +Z
    #   row 2 = camera +Z in the mount frame = mount -X   (so -Z looks along mount +X)
    basis = Gf.Matrix4d(
        0.0, -1.0,  0.0, 0.0,
        0.0,  0.0,  1.0, 0.0,
        -1.0, 0.0,  0.0, 0.0,
        0.0,  0.0,  0.0, 1.0,
    )

    made = []
    for mp in mounts:
        path = mp.GetPath().AppendChild("sensor")
        cam = UsdGeom.Camera.Define(stage, path)
        cam.CreateFocalLengthAttr(focal)
        cam.CreateHorizontalApertureAttr(aperture)
        cam.CreateVerticalApertureAttr(aperture * 3.0 / 4.0)
        cam.CreateClippingRangeAttr(Gf.Vec2f(args.near, args.far))
        cam.CreateProjectionAttr(UsdGeom.Tokens.perspective)
        x = UsdGeom.Xformable(cam)
        x.ClearXformOpOrder()
        x.AddTransformOp().Set(basis)
        made.append(path.pathString)

    # SELF-CHECK. The basis above was wrong once (transposed) and nothing complained:
    # a transposed rotation is still a rotation, so the cameras pointed at the
    # machine's flank and the stage saved happily. Verify rather than trust, every run.
    import numpy as np
    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    bad = []
    for path in made:
        cam_prim = stage.GetPrimAtPath(path)
        mount = cam_prim.GetParent()
        Mm = np.array(xc.GetLocalToWorldTransform(mount)).T
        Mc = np.array(xc.GetLocalToWorldTransform(cam_prim)).T
        fwd = float(np.dot(Mm[:3, :3] @ np.array([1, 0, 0]),
                           Mc[:3, :3] @ np.array([0, 0, -1])))
        up = float(np.dot(Mm[:3, :3] @ np.array([0, 0, 1]),
                          Mc[:3, :3] @ np.array([0, 1, 0])))
        if fwd < 0.999 or up < 0.999:
            bad.append(f"{mount.GetName()}: fwd.fwd={fwd:+.4f} up.up={up:+.4f}")
    if bad:
        app.close()
        sys.stderr.write("\n[FATAL] cameras are not aligned with their mounts:\n  "
                         + "\n  ".join(bad)
                         + "\n        The basis matrix is wrong; nothing was saved.\n")
        return 5

    stage.GetRootLayer().Save()
    print(f"[cam] alignment verified: every camera's -Z is on its mount's +X")
    vfov = 2 * math.degrees(math.atan((aperture * 0.75 / 2.0) / focal))
    print(f"[cam] {args.stage}")
    print(f"[cam] {len(made)} cameras, f={focal:.2f} mm on a {aperture:.3f} mm aperture "
          f"= {args.hfov:g} x {vfov:.0f} deg")
    for p in made:
        print(f"[cam]   {p}")
    print("[cam] in the viewport: the camera dropdown at its top-left now lists these.")
    print("[cam] two at once: Window > Viewport > Viewport 2, then pick a camera in each.")
    app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
