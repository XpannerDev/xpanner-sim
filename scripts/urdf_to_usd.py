#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
urdf_to_usd.py -- headless Isaac Sim converter for the Volvo ECR88 / Xpanner X1 PanelLift asset.

WHAT THIS DOES
--------------
1. Expands an xacro file to a flat URDF (subprocess `xacro`, else the python `xacro`
   module, else a clear, actionable error -- never a bare traceback).
2. Drives the Isaac Sim URDF importer headlessly and writes a .usd / .usda to disk.

HOW TO RUN
----------
This script MUST run inside Isaac Sim's own python environment.  It will NOT work
under a system python3, because `isaacsim` / `omni.*` are only importable there.

    cd ~/isaacsim                       # or wherever you unpacked Isaac Sim
    ./python.sh /home/ubuntu/jude/xpanner-sim/scripts/urdf_to_usd.py \
        --xacro  /home/ubuntu/jude/xpanner-sim/assets/ecr88/urdf/ecr88.urdf.xacro \
        --output /home/ubuntu/jude/xpanner-sim/assets/ecr88/usd/ecr88.usd \
        --fix-base --joint-drive-type position

Docker:

    docker run --rm --gpus all -v /home/ubuntu/jude:/work \
        nvcr.io/nvidia/isaac-sim:4.5.0 \
        /isaac-sim/python.sh /work/xpanner-sim/scripts/urdf_to_usd.py --xacro ... --output ...

IMPORTER API NOTE (version drift -- read before debugging import failures)
-------------------------------------------------------------------------
* Isaac Sim >= 4.5 (and 5.x): the extension is `isaacsim.asset.importer.urdf`,
  and the low-level bindings are `from isaacsim.asset.importer.urdf import _urdf`.
  Config object: `_urdf.ImportConfig()` (or the `URDFCreateImportConfig` kit command).
  Interface:     `_urdf.acquire_urdf_interface()`.
  Kit commands:  `URDFCreateImportConfig`, `URDFParseFile`, `URDFImportRobot`,
                 `URDFParseAndImportFile` (the one-shot command; honours `dest_path`).

* Isaac Sim <= 4.2 (LEGACY -- do not use for this project, kept here so that an
  older machine's failure is diagnosable): the extension was `omni.importer.urdf`
  and the import was:

      from omni.importer.urdf import _urdf
      urdf_interface = _urdf.acquire_urdf_interface()
      import_config  = _urdf.ImportConfig()
      status, robot_model = omni.kit.commands.execute(
          "URDFParseFile", urdf_path=..., import_config=import_config)
      status, prim_path   = omni.kit.commands.execute(
          "URDFImportRobot", urdf_robot=robot_model, import_config=import_config)

  The enum for the default drive type also moved: `_urdf.UrdfJointDriveType` on the
  old path vs `_urdf.UrdfJointTargetType` on the new one.  This script probes for
  both (see `_resolve_drive_type`) so it survives either spelling, but it only
  imports the NEW extension -- if you are on <= 4.2, upgrade rather than patching.

ECR88-SPECIFIC NOTES
--------------------
* `--distance-scale 1.0` is correct for this asset: every number in
  resources/ECR88_kinematic_parameters.md is already in METRES, and the xacro
  emits metres.  Only change it if you re-author the xacro in mm.
* `--merge-fixed-joints` collapses the many pure-frame links (gnss antennas,
  probe, contact_surface, cylinder attachment points).  That is good for sim
  performance but DESTROYS the frames you need to publish TCP / antenna poses to
  ROS 2.  Default is OFF for that reason.  Turn it on only for a
  performance-oriented build where contact_surface is not published.
* Mass / inertia in the xacro are ESTIMATE placeholders (see the TODO block at the
  top of ecr88.urdf.xacro).  `--import-inertia-tensor` is therefore ON by default so
  the importer uses what the URDF says rather than silently recomputing from the
  primitive collision shapes -- that way the placeholders stay visible and wrong in
  an obvious way, instead of being quietly replaced by plausible-looking numbers.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
import textwrap
from pathlib import Path

SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parent.parent


# --------------------------------------------------------------------------- #
# 0.  Environment guard -- fail loudly, not with a bare ImportError
# --------------------------------------------------------------------------- #
_ISAAC_HELP = """
================================================================================
  Isaac Sim python environment NOT active.
================================================================================
  This script imports `isaacsim` / `omni.kit`, which only exist inside the
  python interpreter that ships with Isaac Sim.  You appear to be running it
  under a plain python3.

      interpreter : {exe}
      sys.prefix  : {prefix}
      missing     : {missing}

  Run it through Isaac Sim's launcher instead:

      cd /path/to/isaacsim            # the dir that contains python.sh
      ./python.sh {script} --xacro <file.xacro> --output <file.usd>

  Common install locations to try:
      ~/isaacsim/python.sh
      ~/.local/share/ov/pkg/isaac-sim-*/python.sh
      /isaac-sim/python.sh                      (inside the NGC docker image)

  If you only want to sanity-check the URDF and do NOT need USD output, use the
  dependency-light validator instead -- it needs no Isaac Sim and no ROS:

      python3 {validator} --xacro <file.xacro>
================================================================================
"""


def _require_isaac_sim() -> None:
    """Import-probe Isaac Sim and exit(2) with an actionable message if absent."""
    try:
        import isaacsim  # noqa: F401
    except Exception as exc:  # ImportError, but also ModuleNotFoundError chains
        sys.stderr.write(
            _ISAAC_HELP.format(
                exe=sys.executable,
                prefix=sys.prefix,
                missing=f"{type(exc).__name__}: {exc}",
                script=SCRIPT,
                validator=SCRIPT.parent / "validate_urdf.py",
            )
        )
        sys.stderr.flush()
        raise SystemExit(2)


# --------------------------------------------------------------------------- #
# 1.  xacro expansion
# --------------------------------------------------------------------------- #
class XacroError(RuntimeError):
    pass


_XACRO_HELP = """Could not expand the xacro file: no xacro implementation is available.

Tried, in order:
  1. a `xacro` executable on PATH                      -> {p1}
  2. `python -m xacro` / the python `xacro` module      -> {p2}
  3. $XACRO / --xacro-bin override                      -> {p3}

Fix it with ONE of:
  * source a ROS 2 install that provides xacro:
        source /opt/ros/humble/setup.bash
  * pip-install it into a throwaway venv and point this script at it:
        python3 -m venv /tmp/xv && /tmp/xv/bin/pip install xacro
        {script} --xacro-bin /tmp/xv/bin/xacro ...
  * or pre-expand it yourself and pass the flat file with --urdf instead:
        xacro model.urdf.xacro > model.urdf
"""


def expand_xacro(
    xacro_path: Path,
    out_path: Path,
    mappings: list[str] | None = None,
    xacro_bin: str | None = None,
) -> Path:
    """Expand `xacro_path` into `out_path`. Returns out_path. Raises XacroError."""
    mappings = list(mappings or [])
    out_path.parent.mkdir(parents=True, exist_ok=True)

    candidates: list[tuple[str, list[str]]] = []
    explicit = xacro_bin or os.environ.get("XACRO")
    if explicit:
        candidates.append(("explicit (--xacro-bin/$XACRO)", [explicit]))
    on_path = shutil.which("xacro")
    if on_path:
        candidates.append(("PATH", [on_path]))
    candidates.append(("python module", [sys.executable, "-m", "xacro"]))

    tried: list[str] = []
    for label, base in candidates:
        cmd = base + [str(xacro_path), "-o", str(out_path)] + mappings
        try:
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
        except (OSError, subprocess.TimeoutExpired) as exc:
            tried.append(f"{label}: {type(exc).__name__}: {exc}")
            continue
        if proc.returncode == 0 and out_path.is_file() and out_path.stat().st_size > 0:
            print(f"[xacro] expanded via {label}: {' '.join(cmd)}")
            if proc.stderr.strip():
                print(textwrap.indent(proc.stderr.strip(), "[xacro]   "))
            return out_path
        tried.append(
            f"{label}: exit {proc.returncode}\n"
            + textwrap.indent((proc.stderr or proc.stdout or "").strip()[:4000], "    ")
        )

    raise XacroError(
        _XACRO_HELP.format(
            p1=tried[-2] if len(tried) >= 2 else "n/a",
            p2=tried[-1] if tried else "n/a",
            p3=tried[0] if explicit else "not set",
            script=SCRIPT,
        )
        + "\n--- attempts ---\n"
        + "\n".join(tried)
    )


# --------------------------------------------------------------------------- #
# 2.  Importer config plumbing
# --------------------------------------------------------------------------- #
def _resolve_drive_type(_urdf, name: str):
    """
    Map our CLI string onto whichever drive-type enum this Isaac Sim build ships.

    4.5/5.x : _urdf.UrdfJointTargetType.JOINT_DRIVE_{NONE,POSITION,VELOCITY}
    <=4.2   : _urdf.UrdfJointDriveType.JOINT_DRIVE_{NONE,POSITION,VELOCITY}
    """
    wanted = {
        "none": "JOINT_DRIVE_NONE",
        "position": "JOINT_DRIVE_POSITION",
        "velocity": "JOINT_DRIVE_VELOCITY",
    }[name]
    for enum_name in ("UrdfJointTargetType", "UrdfJointDriveType"):
        enum = getattr(_urdf, enum_name, None)
        if enum is None:
            continue
        member = getattr(enum, wanted, None)
        if member is not None:
            return enum_name, member
    raise RuntimeError(
        f"Neither _urdf.UrdfJointTargetType nor _urdf.UrdfJointDriveType exposes "
        f"{wanted!r}. Available attrs: {sorted(a for a in dir(_urdf) if 'Joint' in a)}"
    )


def _set_if_present(cfg, attr: str, value) -> bool:
    """Set cfg.attr, tolerating importer-config churn across Isaac Sim versions."""
    if not hasattr(cfg, attr):
        print(f"[import] note: this Isaac Sim build has no ImportConfig.{attr}; skipped")
        return False
    try:
        setattr(cfg, attr, value)
    except Exception as exc:
        print(f"[import] note: could not set ImportConfig.{attr}={value!r}: {exc}")
        return False
    return True


def build_import_config(_urdf, args):
    """Build and populate an ImportConfig, preferring the kit command when present."""
    cfg = None
    try:
        import omni.kit.commands

        ok, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
        if not ok:
            cfg = None
    except Exception:
        cfg = None
    if cfg is None:
        cfg = _urdf.ImportConfig()

    enum_name, drive = _resolve_drive_type(_urdf, args.joint_drive_type)
    print(f"[import] drive type: {enum_name}.{args.joint_drive_type.upper()}")

    _set_if_present(cfg, "merge_fixed_joints", bool(args.merge_fixed_joints))
    _set_if_present(cfg, "fix_base", bool(args.fix_base))
    _set_if_present(cfg, "distance_scale", float(args.distance_scale))
    _set_if_present(cfg, "self_collision", bool(args.self_collision))
    _set_if_present(cfg, "import_inertia_tensor", bool(args.import_inertia_tensor))
    # density 0.0 == "trust the URDF's <inertial>"; anything else silently overwrites
    # the ESTIMATE placeholders, which we do not want for this asset.
    _set_if_present(cfg, "density", float(args.density))
    _set_if_present(cfg, "convex_decomp", bool(args.convex_decomp))
    _set_if_present(cfg, "collision_from_visuals", bool(args.collision_from_visuals))
    _set_if_present(cfg, "create_physics_scene", bool(args.create_physics_scene))
    _set_if_present(cfg, "make_default_prim", True)
    _set_if_present(cfg, "parse_mimic", True)
    # both spellings have existed
    if not _set_if_present(cfg, "default_drive_type", drive):
        _set_if_present(cfg, "default_drive_target_type", drive)
    _set_if_present(cfg, "default_drive_strength", float(args.drive_strength))
    _set_if_present(cfg, "default_position_drive_damping", float(args.drive_damping))
    return cfg


def import_to_usd(urdf_path: Path, output: Path, args) -> str:
    """Run the importer. Returns the articulation prim path."""
    import omni.kit.commands
    from isaacsim.asset.importer.urdf import _urdf

    # acquire_urdf_interface() is what actually loads the native plugin; grabbing it
    # early turns a broken/absent extension into a clear failure here rather than an
    # opaque "command not found" from omni.kit.commands.execute below.
    urdf_interface = _urdf.acquire_urdf_interface()
    print(f"[import] urdf interface: {urdf_interface}")

    cfg = build_import_config(_urdf, args)
    output.parent.mkdir(parents=True, exist_ok=True)

    # Preferred: one-shot command, writes the USD itself via dest_path.
    try:
        ok, prim_path = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=str(urdf_path),
            import_config=cfg,
            get_articulation_root=True,
            dest_path=str(output),
        )
        if ok:
            print(f"[import] URDFParseAndImportFile -> {prim_path}")
            return prim_path
        print("[import] URDFParseAndImportFile returned False; falling back to "
              "URDFParseFile + URDFImportRobot")
    except Exception as exc:
        print(f"[import] URDFParseAndImportFile unavailable ({exc}); falling back")

    ok, robot_model = omni.kit.commands.execute(
        "URDFParseFile", urdf_path=str(urdf_path), import_config=cfg
    )
    if not ok:
        raise RuntimeError(f"URDFParseFile failed for {urdf_path}")
    ok, prim_path = omni.kit.commands.execute(
        "URDFImportRobot",
        urdf_robot=robot_model,
        import_config=cfg,
        get_articulation_root=True,
        dest_path=str(output),
    )
    if not ok:
        raise RuntimeError(f"URDFImportRobot failed for {urdf_path}")
    print(f"[import] URDFImportRobot -> {prim_path}")
    return prim_path


def ensure_saved(output: Path) -> None:
    """If the importer did not honour dest_path, export the live stage ourselves."""
    if output.is_file() and output.stat().st_size > 0:
        print(f"[usd] {output}  ({output.stat().st_size/1024:.1f} KiB)")
        return
    print("[usd] dest_path produced no file; exporting the live stage instead")
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No live USD stage to export and dest_path wrote nothing.")
    output.parent.mkdir(parents=True, exist_ok=True)
    stage.Export(str(output))
    if not output.is_file():
        raise RuntimeError(f"Stage.Export('{output}') produced no file.")
    print(f"[usd] {output}  ({output.stat().st_size/1024:.1f} KiB)")


def report_articulation(prim_path: str) -> None:
    """Print the joints/links the importer actually created -- cheap smoke test."""
    try:
        from pxr import Usd, UsdPhysics
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return
        root = stage.GetPrimAtPath(prim_path) if prim_path else stage.GetDefaultPrim()
        if not root or not root.IsValid():
            return
        links, joints = [], []
        for prim in Usd.PrimRange(root):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                links.append(prim.GetPath().pathString)
            if prim.IsA(UsdPhysics.Joint):
                joints.append(
                    (prim.GetPath().name, prim.GetTypeName())
                )
        print(f"\n[usd] articulation root : {prim_path}")
        print(f"[usd] rigid bodies      : {len(links)}")
        print(f"[usd] physics joints    : {len(joints)}")
        for name, typ in joints:
            print(f"[usd]     {typ:<22} {name}")
    except Exception as exc:
        print(f"[usd] (post-import report skipped: {exc})")


# --------------------------------------------------------------------------- #
# 3.  CLI
# --------------------------------------------------------------------------- #
def parse_args(argv=None):
    p = argparse.ArgumentParser(
        prog="urdf_to_usd.py",
        description="Headless Isaac Sim URDF/xacro -> USD converter (ECR88 X1 PanelLift).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=textwrap.dedent(
            """\
            examples:
              ./python.sh urdf_to_usd.py --xacro assets/ecr88/urdf/ecr88.urdf.xacro \\
                                         --output assets/ecr88/usd/ecr88.usd
              ./python.sh urdf_to_usd.py --urdf /tmp/ecr88.urdf --output /tmp/ecr88.usda \\
                                         --floating --merge-fixed-joints
              ./python.sh urdf_to_usd.py --xacro ... --output ... \\
                                         -D use_meshes:=true -D arm_variant:=2.1m
            """
        ),
    )
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--xacro", type=Path, help="xacro file to expand and import")
    src.add_argument("--urdf", type=Path, help="already-flat URDF to import")

    p.add_argument("--output", "-o", type=Path, required=True,
                   help="output .usd / .usda / .usdc path")

    base = p.add_mutually_exclusive_group()
    base.add_argument("--fix-base", dest="fix_base", action="store_true",
                      help="weld the chassis to the world (default)")
    base.add_argument("--floating", dest="fix_base", action="store_false",
                      help="free-floating base (needed for travel/undercarriage sim)")
    p.set_defaults(fix_base=True)

    p.add_argument("--merge-fixed-joints", dest="merge_fixed_joints",
                   action="store_true", default=False,
                   help="collapse fixed joints. WARNING: destroys the gnss/probe/"
                        "contact_surface frames you need for ROS 2 TF. Default off.")
    p.add_argument("--no-merge-fixed-joints", dest="merge_fixed_joints",
                   action="store_false", help=argparse.SUPPRESS)

    p.add_argument("--joint-drive-type", choices=("none", "position", "velocity"),
                   default="position",
                   help="default articulation drive mode (default: position)")
    p.add_argument("--drive-strength", type=float, default=1e7,
                   help="default drive stiffness/gain. PLACEHOLDER -- the real value "
                        "depends on hydraulic cylinder forces we do not have yet.")
    p.add_argument("--drive-damping", type=float, default=1e5,
                   help="default position-drive damping. PLACEHOLDER, see above.")

    p.add_argument("--distance-scale", type=float, default=1.0,
                   help="URDF length unit -> stage metersPerUnit (default 1.0; the "
                        "ECR88 parameter sheet is already in metres)")
    p.add_argument("--density", type=float, default=0.0,
                   help="0.0 = trust the URDF <inertial> blocks (default). Any other "
                        "value makes the importer recompute mass from geometry and "
                        "silently discard the xacro's ESTIMATE placeholders.")
    p.add_argument("--no-import-inertia-tensor", dest="import_inertia_tensor",
                   action="store_false", default=True,
                   help="let the importer recompute inertia instead of using the URDF's")
    p.add_argument("--self-collision", action="store_true", default=False,
                   help="enable self-collision (expensive; the 4-bar linkage will "
                        "self-collide with primitive shapes)")
    p.add_argument("--convex-decomp", action="store_true", default=False,
                   help="convex-decompose collision meshes (only useful once real "
                        "meshes replace the primitives)")
    p.add_argument("--collision-from-visuals", action="store_true", default=False,
                   help="derive collision from visual geometry")
    p.add_argument("--no-physics-scene", dest="create_physics_scene",
                   action="store_false", default=True,
                   help="do not add a PhysicsScene prim (use when compositing into "
                        "a larger stage that already has one)")

    p.add_argument("-D", "--mapping", dest="mappings", action="append", default=[],
                   metavar="NAME:=VALUE",
                   help="xacro argument, repeatable (e.g. -D use_meshes:=true)")
    p.add_argument("--xacro-bin", default=None,
                   help="explicit path to a xacro executable (or set $XACRO)")
    p.add_argument("--keep-urdf", action="store_true",
                   help="keep the expanded intermediate .urdf next to the output")
    p.add_argument("--renderer", default="RayTracedLighting",
                   help="Kit renderer (default RayTracedLighting)")
    p.add_argument("--gui", action="store_true",
                   help="run with a window instead of headless (debugging only)")
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    _require_isaac_sim()

    # SimulationApp MUST be constructed before any omni.* / isaacsim.asset.* import.
    from isaacsim import SimulationApp

    simulation_app = SimulationApp(
        {"headless": not args.gui, "renderer": args.renderer}
    )

    exit_code = 0
    tmpdir = None
    try:
        from isaacsim.core.utils.extensions import enable_extension

        if not enable_extension("isaacsim.asset.importer.urdf"):
            raise RuntimeError(
                "Could not enable extension 'isaacsim.asset.importer.urdf'.\n"
                "On Isaac Sim <= 4.2 this extension is named 'omni.importer.urdf' -- "
                "see the API NOTE in this file's module docstring; upgrade to >= 4.5."
            )
        simulation_app.update()

        # --- resolve the flat URDF -------------------------------------------
        if args.urdf is not None:
            urdf_path = args.urdf.resolve()
            if not urdf_path.is_file():
                raise FileNotFoundError(f"--urdf not found: {urdf_path}")
        else:
            xacro_path = args.xacro.resolve()
            if not xacro_path.is_file():
                raise FileNotFoundError(f"--xacro not found: {xacro_path}")
            # Expand NEXT TO the source so relative mesh paths keep resolving.
            if args.keep_urdf:
                urdf_path = args.output.resolve().with_suffix(".urdf")
            else:
                tmpdir = tempfile.mkdtemp(prefix="ecr88_xacro_", dir=str(xacro_path.parent))
                urdf_path = Path(tmpdir) / (xacro_path.stem.replace(".urdf", "") + ".urdf")
            expand_xacro(xacro_path, urdf_path, args.mappings, args.xacro_bin)

        print(f"[import] source   : {urdf_path}")
        print(f"[import] output   : {args.output.resolve()}")
        print(f"[import] fix_base : {args.fix_base}")
        print(f"[import] merge_fj : {args.merge_fixed_joints}")
        print(f"[import] dist_scl : {args.distance_scale}")

        prim_path = import_to_usd(urdf_path, args.output.resolve(), args)
        simulation_app.update()
        ensure_saved(args.output.resolve())
        report_articulation(prim_path)

    except XacroError as exc:
        sys.stderr.write(f"\n[FATAL] {exc}\n")
        exit_code = 3
    except Exception as exc:
        import traceback

        traceback.print_exc()
        sys.stderr.write(f"\n[FATAL] {type(exc).__name__}: {exc}\n")
        exit_code = 1
    finally:
        if tmpdir:
            shutil.rmtree(tmpdir, ignore_errors=True)
        try:
            simulation_app.close()
        except Exception:
            pass
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
