#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
urdf_to_usd_isaac60.py -- ECR88 URDF -> USD converter for Isaac Sim 6.0.x.

WHY THIS EXISTS
---------------
scripts/urdf_to_usd.py targets the Isaac Sim 4.5/5.x importer API
(`from isaacsim.asset.importer.urdf import _urdf`, `ImportConfig`, the
`URDFParseAndImportFile` kit command).  In Isaac Sim 6.0.1 that binding is gone:

    ImportError: cannot import name '_urdf' from 'isaacsim.asset.importer.urdf'

6.0 ships isaacsim.asset.importer.urdf-3.11.2 with a dataclass API instead:
`URDFImporter` + `URDFImporterConfig` (see impl/converter.py, impl/config.py).
This script is the thin 6.0 equivalent.  It does NOT expand xacro -- the Isaac Sim
container has no xacro -- so expand on the host first:

    xacro assets/ecr88/urdf/ecr88.urdf.xacro -o build/ecr88.urdf

RUN (inside the Isaac Sim 6.0.1 container):
    /isaac-sim/python.sh /workspace/xpanner-sim/scripts/urdf_to_usd_isaac60.py \
        --urdf    /workspace/xpanner-sim/build/ecr88.urdf \
        --out-dir /workspace/xpanner-sim/assets/ecr88/usd

OUTPUT: <out-dir>/<robot_name>/<robot_name>.usda  (the importer picks the directory
and file name from the URDF's <robot name="...">; if that directory already exists
it writes <robot_name>_01 etc. rather than overwriting).

NOTE: merge_fixed_joints defaults to OFF on purpose -- turning it on deletes
gnss_*_link, the cylinder anchors, probe_link and contact_surface_link (the TCP).
"""
from __future__ import annotations

import argparse
from pathlib import Path


def parse_args():
    p = argparse.ArgumentParser(description="ECR88 URDF -> USD (Isaac Sim 6.0.x API)")
    p.add_argument("--urdf", required=True, type=Path, help="flat .urdf (expand xacro first)")
    p.add_argument("--out-dir", required=True, type=Path, help="output DIRECTORY, not a file")
    p.add_argument("--merge-fixed-joints", action="store_true",
                   help="collapse fixed-joint frames (DESTROYS TCP/GNSS frames)")
    p.add_argument("--floating", action="store_true", help="floating base instead of fixed base")
    p.add_argument("--self-collision", action="store_true")
    p.add_argument("--target-type", default="position", choices=("none", "position", "velocity"))
    p.add_argument("--stiffness", type=float, default=None, help="override joint stiffness")
    p.add_argument("--damping", type=float, default=None, help="override joint damping")
    p.add_argument("--gui", action="store_true", help="run with a window (debugging only)")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    urdf = args.urdf.resolve()
    if not urdf.is_file():
        raise SystemExit(f"[FATAL] --urdf not found: {urdf}")
    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    # SimulationApp must exist before any omni.* / isaacsim.asset.* import.
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": not args.gui})
    code = 0
    try:
        from isaacsim.core.utils.extensions import enable_extension

        if not enable_extension("isaacsim.asset.importer.urdf"):
            raise RuntimeError("could not enable isaacsim.asset.importer.urdf")
        app.update()

        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

        cfg = URDFImporterConfig(
            urdf_path=str(urdf),
            usd_path=str(out_dir),
            merge_fixed_joints=bool(args.merge_fixed_joints),
            allow_self_collision=bool(args.self_collision),
            fix_base=(False if args.floating else True),
            joint_target_type=args.target_type,
            override_joint_stiffness=args.stiffness,
            override_joint_damping=args.damping,
        )
        print(f"[import] urdf     : {urdf}")
        print(f"[import] out dir  : {out_dir}")
        print(f"[import] fix_base : {cfg.fix_base}   merge_fixed_joints: {cfg.merge_fixed_joints}")

        result = URDFImporter(cfg).import_urdf()
        print(f"[ok] USD written: {result}")
    except Exception as exc:  # noqa: BLE001
        import traceback

        traceback.print_exc()
        print(f"\n[FATAL] {type(exc).__name__}: {exc}")
        code = 1
    finally:
        try:
            app.close()
        except Exception:  # noqa: BLE001
            pass
    return code


if __name__ == "__main__":
    raise SystemExit(main())
