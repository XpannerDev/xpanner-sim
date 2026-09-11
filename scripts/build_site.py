#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
build_site.py -- author a solar-farm panel-installation site around the ECR88.

WHY
---
Milestone 1 is "let people decide by eye where a camera or lidar goes". A machine
floating in an empty stage cannot answer that. The question is always relative to
something: can the camera see the pile it is about to set a panel on, does the
torque tube occlude it, does the operator's cab block the view, how far is the
panel stack. So the site has to exist before the sensor argument can happen.

WHAT IS MODELLED, AND WHY THOSE NUMBERS
---------------------------------------
Utility-scale single-axis tracker rows, which is what the X1 PanelLift works on:

  module          2278 x 1134 x 35 mm   standard 72-cell bifacial glass-glass
  modules per row 2 in portrait per bay
  pile            76 mm square section, driven, 1.9 m exposed
  pile pitch      5.0 m along the row
  torque tube     140 mm square, 1.8 m above grade
  row pitch       6.0 m  (typical GCR for a 1P tracker)

These are CLASS-TYPICAL for a US utility solar site, NOT measured from an Xpanner
job. They are here to give the camera argument a realistic scale, not to certify a
layout. Swap them for real site numbers before using this to size anything.

The site is deliberately mid-install, because that is when the machine is there:
  row 0   fully panelled          - what "done" looks like
  row 1   half panelled           - the working face, panels stop mid-row
  row 2   bare piles + torque tube - next up
  row 3   bare piles only          - not yet tubed
plus a delivered pallet stack of modules next to the machine, which is the pick
point, and a single module already on the tool.

GROUND PLANE NOTE
-----------------
base_link sits at the chassis reference, and the ECR88 asset puts grade at
z = lenBottomZ1 = -1.445 in its own frame (see T7 in ecr88.urdf.xacro). So the
robot is referenced in at z = +1.445 to stand its tracks on the site's z = 0.
Get this wrong and the machine is buried to the sprockets.

USAGE
-----
    ./run_isaac.sh convert ...            # build assets/ecr88/usd/ecr88.usd first
    docker exec <container> /isaac-sim/python.sh \
        /work/xpanner-sim/scripts/build_site.py \
        --robot /work/xpanner-sim/assets/ecr88/usd/ecr88.usd \
        --output /work/xpanner-sim/assets/site/solar_site.usd
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path


# --------------------------------------------------------------------------- #
# Site dimensions.  All metres. CLASS-TYPICAL -- see the module docstring.
# --------------------------------------------------------------------------- #
MODULE_L, MODULE_W, MODULE_T = 2.278, 1.134, 0.035
PILE_SECTION, PILE_EXPOSED = 0.076, 1.90
PILE_PITCH = 5.0
TUBE_SECTION, TUBE_HEIGHT = 0.140, 1.80
ROW_PITCH = 6.0
BAYS_PER_ROW = 8
MODULE_TILT_DEG = 25.0          # tracker parked at a working tilt
GRADE_Z = 0.0
ROBOT_LIFT_Z = 1.445            # ECR88 T7: grade is at -1.445 in the robot's frame

COL = {
    "ground":  (0.42, 0.38, 0.31),
    "pile":    (0.55, 0.57, 0.60),
    "tube":    (0.45, 0.47, 0.50),
    "module":  (0.09, 0.13, 0.28),
    "frame":   (0.72, 0.74, 0.78),
    "pallet":  (0.48, 0.36, 0.22),
}


def _fail(msg: str, code: int = 2) -> None:
    sys.stderr.write(f"\n[FATAL] {msg}\n")
    raise SystemExit(code)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", type=Path, required=True,
                    help="ecr88.usd to reference in")
    ap.add_argument("--output", "-o", type=Path, required=True,
                    help="site USD to write")
    ap.add_argument("--rows", type=int, default=4, help="tracker rows (default 4)")
    ap.add_argument("--no-robot", action="store_true",
                    help="author the site only, without referencing the machine")
    args = ap.parse_args(argv)

    # Isaac Sim's python is required only for the USD libraries here, but importing
    # SimulationApp first is what makes pxr importable inside the container.
    try:
        from isaacsim import SimulationApp
    except ModuleNotFoundError as exc:
        _fail("Isaac Sim python environment not active -- run this through "
              f"/isaac-sim/python.sh inside the container. ({exc})", 4)
    app = SimulationApp({"headless": True})

    from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics

    if not args.no_robot and not args.robot.is_file():
        app.close()
        _fail(f"robot USD not found: {args.robot}\n"
              f"        Build it first with scripts/urdf_to_usd.py.")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(args.output))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    def box(path, size, xyz, colour, rot_y_deg=0.0):
        c = UsdGeom.Cube.Define(stage, path)
        c.CreateSizeAttr(2.0)                       # unit cube spans -1..1
        c.CreateDisplayColorAttr([Gf.Vec3f(*colour)])
        x = UsdGeom.Xformable(c)
        x.AddTranslateOp().Set(Gf.Vec3d(*xyz))
        if rot_y_deg:
            x.AddRotateYOp().Set(rot_y_deg)
        x.AddScaleOp().Set(Gf.Vec3f(size[0] / 2.0, size[1] / 2.0, size[2] / 2.0))
        return c

    # ---- ground ---------------------------------------------------------- #
    extent = max(args.rows * ROW_PITCH, BAYS_PER_ROW * PILE_PITCH) + 30.0
    ground = box("/World/Ground", (extent, extent, 0.2),
                 (extent / 4.0, 0.0, GRADE_Z - 0.1), COL["ground"])
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

    # ---- lighting -------------------------------------------------------- #
    dome = UsdLux.DomeLight.Define(stage, "/World/Sky")
    dome.CreateIntensityAttr(1000.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(3000.0)
    sun.CreateAngleAttr(0.53)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 35.0))

    # ---- tracker rows ---------------------------------------------------- #
    # Rows run along +X in front of the machine; the machine faces +X.
    # Row 0 is nearest the machine.
    tilt = MODULE_TILT_DEG
    row_state = ["full", "half", "tubed", "bare"]
    rows = UsdGeom.Xform.Define(stage, "/World/TrackerRows")
    for r in range(args.rows):
        state = row_state[min(r, len(row_state) - 1)]
        y = 6.0 + r * ROW_PITCH
        grp = UsdGeom.Xform.Define(stage, f"/World/TrackerRows/Row_{r:02d}")
        for b in range(BAYS_PER_ROW):
            x = 4.0 + b * PILE_PITCH
            box(f"{grp.GetPath()}/pile_{b:02d}",
                (PILE_SECTION, PILE_SECTION, PILE_EXPOSED),
                (x, y, GRADE_Z + PILE_EXPOSED / 2.0), COL["pile"])
        if state in ("full", "half", "tubed"):
            length = (BAYS_PER_ROW - 1) * PILE_PITCH + 1.0
            box(f"{grp.GetPath()}/torque_tube",
                (length, TUBE_SECTION, TUBE_SECTION),
                (4.0 + length / 2.0 - 0.5, y, GRADE_Z + TUBE_HEIGHT), COL["tube"])
        if state in ("full", "half"):
            n = BAYS_PER_ROW - 1 if state == "full" else (BAYS_PER_ROW - 1) // 2
            for b in range(n):
                x = 4.0 + b * PILE_PITCH + PILE_PITCH / 2.0
                # Two modules per bay, one each side of the torque tube, both tilted
                # the SAME way: this is a single-axis tracker, so the whole row is one
                # rigid plane rotating about the tube. Mirroring the tilt per side
                # would draw a roof, not a tracker.
                for side, sgn in (("a", -1), ("b", +1)):
                    off = sgn * (MODULE_W / 2.0 + 0.02)
                    dy = off * math.cos(math.radians(tilt))
                    dz = off * math.sin(math.radians(tilt))
                    m = box(f"{grp.GetPath()}/module_{b:02d}{side}",
                            (MODULE_L, MODULE_W, MODULE_T),
                            (x, y + dy, GRADE_Z + TUBE_HEIGHT + 0.09 + dz),
                            COL["module"])
                    UsdGeom.Xformable(m).AddRotateXOp().Set(tilt)

    # ---- delivered pallet of modules, the pick point ---------------------- #
    pallet_x, pallet_y = 3.0, -4.2
    box("/World/Pallet/base", (2.6, 1.4, 0.14),
        (pallet_x, pallet_y, GRADE_Z + 0.07), COL["pallet"])
    for k in range(9):
        box(f"/World/Pallet/module_{k:02d}",
            (MODULE_L, MODULE_W, MODULE_T),
            (pallet_x, pallet_y, GRADE_Z + 0.14 + 0.045 * k + MODULE_T / 2.0),
            COL["module"])

    # ---- the machine ----------------------------------------------------- #
    if not args.no_robot:
        robot = UsdGeom.Xform.Define(stage, "/World/ECR88")
        robot.GetPrim().GetReferences().AddReference(str(args.robot))
        UsdGeom.Xformable(robot).AddTranslateOp().Set(
            Gf.Vec3d(0.0, 0.0, GRADE_Z + ROBOT_LIFT_Z))

    # ---- physics scene --------------------------------------------------- #
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(9.81)

    stage.GetRootLayer().Save()
    print(f"[site] {args.output}  "
          f"({args.output.stat().st_size / 1024:.1f} KiB, {args.rows} rows)")
    print(f"[site] grade z={GRADE_Z}, machine referenced at z={GRADE_Z + ROBOT_LIFT_Z} "
          f"(ECR88 T7: its own grade is -1.445)")
    app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
