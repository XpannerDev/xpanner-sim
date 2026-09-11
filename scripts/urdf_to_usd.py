#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
urdf_to_usd.py -- headless Isaac Sim converter for the Volvo ECR88 / Xpanner X1 PanelLift asset.

PRIMARY TARGET: **Isaac Sim 6.0.1** (`nvcr.io/nvidia/isaac-sim:6.0.1`)
=====================================================================
This script is written against the Isaac Sim 6.0 URDF importer API and verified
against the 6.0.1 image installed on this machine
(`/isaac-sim/VERSION` == ``6.0.1-rc.7+release.42383.32955d8d.gl``,
extension ``isaacsim.asset.importer.urdf`` version ``3.11.2``).

A LEGACY fallback for Isaac Sim 4.5 / 5.x is kept (see `_import_legacy`) so that
running this on an older box degrades into a working import rather than a
traceback -- but 6.0.1 is the supported path and the only one exercised here.

WHAT THIS DOES
--------------
1. Expands an xacro file to a flat URDF (subprocess `xacro`, else the python `xacro`
   module, else a clear, actionable error -- never a bare traceback).
2. Drives the Isaac Sim URDF importer headlessly and writes a .usd / .usda to disk.

HOW TO RUN
----------
This script MUST run inside Isaac Sim's own python environment.  It will NOT work
under a system python3, because `isaacsim` / `omni.*` are only importable there.
On this machine Isaac Sim exists ONLY inside docker, so use the companion wrapper:

    scripts/run_isaac.sh convert \
        --xacro  /work/xpanner-sim/assets/ecr88/urdf/ecr88.urdf.xacro \
        --output /work/xpanner-sim/assets/ecr88/usd/ecr88.usd

or, equivalently, by hand inside the container:

    /isaac-sim/python.sh /work/xpanner-sim/scripts/urdf_to_usd.py \
        --xacro  /work/xpanner-sim/assets/ecr88/urdf/ecr88.urdf.xacro \
        --output /work/xpanner-sim/assets/ecr88/usd/ecr88.usd \
        --fix-base --joint-drive-type position

================================================================================
IMPORTER API NOTE -- 6.0 REWROTE THE URDF IMPORTER.  READ THIS FIRST.
================================================================================
Isaac Sim 6.0 replaced the C++/carb URDF importer with a pure-python one built on
the `urdf-usd-converter` pip package.  Almost every symbol the 4.5/5.x recipes use
is gone or is a deprecation shim that raises.  Verified by reading the extension
source out of the 6.0.1 image.

ENTRY POINT
    6.0.1   `from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig`
            cfg = URDFImporterConfig(urdf_path=..., usd_path=<DIRECTORY>, ...)
            out_file = URDFImporter(cfg).import_urdf()      # returns the written path
    <=5.x   `from isaacsim.asset.importer.urdf import _urdf`  (native bindings module)
            `_urdf.acquire_urdf_interface()` / `_urdf.ImportConfig()`
    <=4.2   `from omni.importer.urdf import _urdf`

    On 6.0 `_urdf` is NOT importable from the package root any more: it survives only
    as `isaacsim.asset.importer.urdf.impl._urdf`, and it is a *class* of deprecation
    shims (every method logs a warning; several raise outright).  Do not use it.

KIT COMMANDS -- ALL REMOVED OR NEUTERED IN 6.0
    URDFParseAndImportFile  -> DELETED ENTIRELY (no such command is registered).
    URDFParseFile           -> shim in `isaacsim.asset.importer.urdf.ui`; `do()`
                               raises RuntimeError("Parsing URDF files is no longer
                               supported.").
    URDFImportRobot         -> shim; works, but returns the USD *file path* (a str),
                               NOT a prim path, and `dest_path` is now a DIRECTORY.
    URDFCreateImportConfig  -> shim; returns a `URDFImporterConfig` dataclass.
    They also moved extension: the commands now live in `isaacsim.asset.importer.urdf.ui`,
    not in `isaacsim.asset.importer.urdf`.

EXTENSION-ENABLING HELPER
    6.0.1   `from isaacsim.core.experimental.utils.app import enable_extension`
    <=5.x   `from isaacsim.core.utils.extensions import enable_extension`
    `isaacsim.core.utils` DOES NOT EXIST in 6.0.1 -- the whole extension was dropped.
    This script therefore prefers the version-proof Kit API:
    `omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate()`.

IMPORT-CONFIG FIELD RENAMES (old ImportConfig attr  ->  6.0 URDFImporterConfig field)
    merge_fixed_joints              -> merge_fixed_joints          (unchanged)
    fix_base                        -> fix_base   (now tri-state bool|None; None =
                                       leave the source asset's base authoring alone)
    self_collision                  -> allow_self_collision
    density                         -> link_density  (None = no override; the old
                                       "0.0 means trust the URDF" sentinel is gone)
    convex_decomp=True              -> collision_type="Convex Decomposition"
                                       (also "Convex Hull"/"Bounding Sphere"/"Bounding Cube")
    collision_from_visuals          -> collision_from_visuals      (unchanged)
    default_drive_type (enum)       -> joint_target_type (str: "none"/"position"/"velocity")
                                       PLUS the new, separate joint_drive_type
                                       (str: "force"/"acceleration")
    default_drive_strength          -> override_joint_stiffness
    default_position_drive_damping  -> override_joint_damping
    dest_path (a FILE)              -> usd_path (a DIRECTORY)
    -- new in 6.0 --                   merge_mesh, robot_type, ros_package_paths,
                                       run_asset_transformer, run_multi_physics_conversion,
                                       debug_mode

IMPORT-CONFIG FIELDS DELETED IN 6.0 (setting them is impossible, not merely ignored)
    distance_scale          The converter is metres-only now.  Passing
                            --distance-scale != 1.0 is refused rather than silently dropped.
    import_inertia_tensor   The URDF <inertial> block is always used.  A MassAPI is only
                            authored when you set a density override.  --no-import-inertia-tensor
                            is now a no-op and says so.
    create_physics_scene    The converter is constructed as `Converter(scene=False)`; a
                            URDF import never emits a PhysicsScene.  --no-physics-scene is
                            now the only achievable behaviour.
    make_default_prim       The asset transformer profile sets the default prim.
    parse_mimic             Mimic joints are always emitted, now via NewtonMimicAPI
                            (`newton:mimicJoint`), not PhysxMimicJointAPI.

ENUM REMOVAL
    `_urdf.UrdfJointTargetType` / `_urdf.UrdfJointDriveType` and their
    JOINT_DRIVE_{NONE,POSITION,VELOCITY} members no longer exist on 6.0 -- drive and
    target types are plain lowercase strings.  `_resolve_drive_type()` below is kept
    for the legacy backend only.

OUTPUT SHAPE -- WHY --output IS POST-PROCESSED
    `URDFImporter.import_urdf()` does not write the single file you asked for.  It
    writes a *package directory* `<usd_path>/<robot_name>/` (asset-transformer layer
    structure: entry `.usda` plus sublayers and per-mesh USDs) and returns the entry
    file.  It also uniquifies the directory name if it already exists, so re-runs
    would pile up `robot`, `robot_01`, `robot_02`...
    To keep this script's `--output FILE` contract, we import into a deterministic
    staging directory (wiped first) and then FLATTEN the entry stage into the single
    file you named.  The package is kept next to it (it holds textures and the
    original layer structure) unless --no-keep-package.

POST-IMPORT STAGE INSPECTION
    The 6.0 importer works on a standalone `Usd.Stage.Open()` -- it never touches the
    Kit `omni.usd` context stage.  The old
    `omni.usd.get_context().get_stage()` post-import report therefore inspects an
    EMPTY stage on 6.0.  `report_articulation()` now reads the file that was written.

ECR88-SPECIFIC NOTES
--------------------
* `--distance-scale 1.0` is correct for this asset: every number in
  resources/ECR88_kinematic_parameters.md is already in METRES, and the xacro
  emits metres.  On 6.0 this is the only accepted value.
* `--merge-fixed-joints` collapses the many pure-frame links (gnss antennas,
  probe, contact_surface, cylinder attachment points).  That is good for sim
  performance but DESTROYS the frames you need to publish TCP / antenna poses to
  ROS 2.  Default is OFF for that reason.
* Mass / inertia in the xacro are ESTIMATE placeholders (see the TODO block at the
  top of ecr88.urdf.xacro).  On 6.0 the importer always honours the URDF's
  <inertial> block, and only authors a MassAPI override when --density is non-zero.
  Leave --density at 0.0 so the placeholders stay visible and wrong in an obvious
  way instead of being quietly replaced by plausible-looking numbers.
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

#: Importer backends this script knows how to drive.
BACKEND_60 = "isaacsim-6.0"          # URDFImporter / URDFImporterConfig
BACKEND_LEGACY = "isaacsim-legacy"   # _urdf bindings + kit commands (4.5 / 5.x)

#: Valid `URDFImporterConfig.collision_type` values on 6.0.
COLLISION_TYPES = ("Convex Hull", "Convex Decomposition", "Bounding Sphere", "Bounding Cube")


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

  On THIS machine Isaac Sim exists only inside docker (image
  nvcr.io/nvidia/isaac-sim:6.0.1); there is no host-side python.sh.  Use the
  companion wrapper, which mounts this repo at /work, picks a container name that
  will not collide with anyone else's session, and refuses to start on top of a
  container / GPU / port that somebody else is already using:

      {runner} convert --xacro <file.xacro> --output <file.usd>

  (the container name defaults to isaac-sim-$USER-$$; every human on this box logs
  in as the same unix account, so the $$ is what makes it unique. Set
  ISAAC_CONTAINER=<name> to pin one.)

  or drive the container by hand:

      {runner} shell
      # then, inside:
      /isaac-sim/python.sh /work/xpanner-sim/scripts/urdf_to_usd.py \\
          --xacro <file.xacro> --output <file.usd>

  On a machine with a native install, Isaac Sim's launcher is one of:
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
                runner=SCRIPT.parent / "run_isaac.sh",
                validator=SCRIPT.parent / "validate_urdf.py",
            )
        )
        sys.stderr.flush()
        raise SystemExit(2)


def _isaac_version_string() -> str:
    """Best-effort Isaac Sim version banner; never raises."""
    try:
        from isaacsim.core.version import get_version

        parts = get_version()
        return str(parts[0]) if parts and parts[0] else "unknown"
    except Exception:
        return "unknown"


# --------------------------------------------------------------------------- #
# 1.  xacro expansion  (unchanged across Isaac Sim versions -- pure subprocess)
# --------------------------------------------------------------------------- #
class XacroError(RuntimeError):
    pass


class ConfigError(RuntimeError):
    """A CLI option this Isaac Sim version cannot honour. Reported without a traceback."""


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
# 2.  Backend detection + extension enabling
# --------------------------------------------------------------------------- #
def enable_urdf_extension() -> None:
    """
    Enable the URDF importer extension.

    Uses the raw Kit extension-manager API, which is stable across 4.x/5.x/6.x.
    `isaacsim.core.utils.extensions.enable_extension` (the 5.x helper) is NOT used:
    the whole `isaacsim.core.utils` extension was deleted in 6.0.  Its 6.0
    replacement is `isaacsim.core.experimental.utils.app.enable_extension`, which
    is itself a one-line wrapper around the same manager call.
    """
    import omni.kit.app

    manager = omni.kit.app.get_app().get_extension_manager()
    for name in ("isaacsim.asset.importer.urdf", "omni.importer.urdf"):
        try:
            if manager.is_extension_enabled(name):
                print(f"[import] extension already enabled: {name}")
                return
            if manager.set_extension_enabled_immediate(name, True):
                print(f"[import] enabled extension: {name}")
                return
        except Exception as exc:
            # An unknown extension name can raise rather than return False.
            print(f"[import] note: could not enable {name}: {type(exc).__name__}: {exc}")
    raise RuntimeError(
        "Could not enable a URDF importer extension.\n"
        "  Tried 'isaacsim.asset.importer.urdf' (Isaac Sim >= 4.5, including 6.0)\n"
        "  and   'omni.importer.urdf'           (Isaac Sim <= 4.2).\n"
        "See the IMPORTER API NOTE in this file's module docstring."
    )


def detect_backend():
    """
    Decide which importer API this build exposes.

    Returns (backend_name, handles) where `handles` is a dict of the imported
    symbols the chosen backend needs.

    6.0 is probed FIRST and is the supported path.  The legacy probe exists only so
    that an older machine produces a working import instead of a traceback.
    """
    try:
        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

        return BACKEND_60, {"URDFImporter": URDFImporter, "URDFImporterConfig": URDFImporterConfig}
    except ImportError as exc:
        # Bind outside the handler: `exc` is unbound once the except block exits.
        why_not_60 = f"{type(exc).__name__}: {exc}"

    legacy_errors = [f"isaacsim.asset.importer.urdf (6.0 API): {why_not_60}"]
    for module_path in ("isaacsim.asset.importer.urdf", "omni.importer.urdf"):
        try:
            mod = __import__(module_path, fromlist=["_urdf"])
            return BACKEND_LEGACY, {"_urdf": mod._urdf, "module": module_path}
        except Exception as exc:
            legacy_errors.append(f"{module_path}._urdf (legacy API): {type(exc).__name__}: {exc}")

    raise RuntimeError(
        "No usable URDF importer API found in this Isaac Sim build.\n  "
        + "\n  ".join(legacy_errors)
        + "\n\nThis script targets Isaac Sim 6.0.1; see the IMPORTER API NOTE in the "
        "module docstring for what each version exposes."
    )


# --------------------------------------------------------------------------- #
# 3.  Isaac Sim 6.0 import path (PRIMARY)
# --------------------------------------------------------------------------- #
def _flag_notes_60(args) -> None:
    """Report CLI flags that 6.0 simply no longer has a config field for."""
    if not args.import_inertia_tensor:
        print(
            "[import] NOTE: --no-import-inertia-tensor is a no-op on Isaac Sim 6.0. "
            "The importer always honours the URDF <inertial> block; there is no "
            "import_inertia_tensor field any more."
        )
    if args.create_physics_scene:
        print(
            "[import] NOTE: Isaac Sim 6.0 never emits a PhysicsScene from a URDF import "
            "(the converter is built with scene=False), so --no-physics-scene is now the "
            "only achievable behaviour. Add a PhysicsScene in the stage that composes this asset."
        )


def build_config_60(URDFImporterConfig, urdf_path: Path, staging_dir: Path, args):
    """Build a 6.0 `URDFImporterConfig` from the CLI args."""
    collision_type = args.collision_type
    if args.convex_decomp and collision_type == "Convex Hull":
        # Legacy --convex-decomp is the 6.0 collision_type="Convex Decomposition".
        collision_type = "Convex Decomposition"

    ros_packages = []
    for spec in args.ros_packages:
        name, sep, path = spec.partition("=")
        if not sep or not name or not path:
            raise ConfigError(f"--ros-package expects NAME=PATH, got {spec!r}")
        ros_packages.append({"name": name, "path": str(Path(path).resolve())})

    # 6.0 splits the single old drive enum in two:
    #   joint_target_type  -- what the drive tracks   ("none"/"position"/"velocity")
    #   joint_drive_type   -- how the gains are meant ("force"/"acceleration")
    # Our long-standing --joint-drive-type maps onto joint_target_type.
    cfg = URDFImporterConfig(
        urdf_path=str(urdf_path),
        usd_path=str(staging_dir),                       # a DIRECTORY, not a file
        merge_fixed_joints=bool(args.merge_fixed_joints),
        merge_mesh=bool(args.merge_mesh),
        debug_mode=bool(args.debug_import),
        collision_from_visuals=bool(args.collision_from_visuals),
        collision_type=collision_type,
        allow_self_collision=bool(args.self_collision),  # was: self_collision
        ros_package_paths=ros_packages,
        fix_base=args.fix_base,                          # tri-state: True/False/None
        link_density=(None if args.density == 0.0 else float(args.density)),  # was: density
        joint_drive_type=args.joint_drive_mode,          # NEW in 6.0
        joint_target_type=args.joint_drive_type,         # was: default_drive_type (enum)
        override_joint_stiffness=float(args.drive_strength),   # was: default_drive_strength
        override_joint_damping=float(args.drive_damping),      # was: default_position_drive_damping
    )
    print(f"[import] collision_type    : {collision_type}")
    print(f"[import] joint_target_type : {cfg.joint_target_type}")
    print(f"[import] joint_drive_type  : {cfg.joint_drive_type}")
    print(f"[import] link_density      : {cfg.link_density}")
    return cfg


def _import_60(handles, urdf_path: Path, output: Path, args) -> tuple[Path, Path]:
    """
    Run the Isaac Sim 6.0 importer.

    Returns (entry_usd_written_by_importer, package_root_dir).
    """
    URDFImporter = handles["URDFImporter"]
    URDFImporterConfig = handles["URDFImporterConfig"]

    _flag_notes_60(args)

    # import_urdf() writes a PACKAGE DIRECTORY under usd_path and uniquifies the
    # name on collision, so re-runs would accumulate robot/robot_01/robot_02.
    # Use a deterministic staging dir and wipe it first.
    staging_dir = output.parent / f"{output.stem}_pkg"
    if staging_dir.exists():
        shutil.rmtree(staging_dir)
    staging_dir.mkdir(parents=True, exist_ok=True)

    cfg = build_config_60(URDFImporterConfig, urdf_path, staging_dir, args)
    entry = Path(URDFImporter(cfg).import_urdf()).resolve()
    print(f"[import] URDFImporter.import_urdf() -> {entry}")
    if not entry.is_file():
        raise RuntimeError(f"import_urdf() reported {entry}, but no such file exists.")
    return entry, staging_dir


def flatten_to_output(entry: Path, output: Path) -> None:
    """
    Compose the importer's multi-layer package down into the single file the CLI asked for.

    `--output` has always meant "one USD file at this path".  The 6.0 importer emits a
    layered package instead, so flatten the composed stage (this inlines sublayers,
    references and payloads, including the per-mesh USDs) and export it.
    """
    from pxr import Usd

    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.Open(str(entry))
    if stage is None:
        raise RuntimeError(f"Could not open the imported stage at {entry}")
    stage.Load()  # make sure payloads are composed in before flattening
    flat = stage.Flatten()
    if not flat.Export(str(output)):
        raise RuntimeError(f"Flatten/Export to {output} failed.")
    print(f"[usd] {output}  ({output.stat().st_size / 1024:.1f} KiB, flattened from {entry.name})")


# --------------------------------------------------------------------------- #
# 4.  LEGACY import path (Isaac Sim 4.5 / 5.x) -- fallback only
# --------------------------------------------------------------------------- #
def _resolve_drive_type(_urdf, name: str):
    """
    LEGACY ONLY (Isaac Sim <= 5.x).  Map our CLI string onto whichever drive-type
    enum that build ships.  Both enums were DELETED in 6.0 -- drive/target types are
    plain strings there, handled in `build_config_60`.

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


def build_import_config_legacy(_urdf, args):
    """LEGACY ONLY. Build and populate a 4.5/5.x `_urdf.ImportConfig()`."""
    cfg = _urdf.ImportConfig()

    enum_name, drive = _resolve_drive_type(_urdf, args.joint_drive_type)
    print(f"[import] drive type: {enum_name}.{args.joint_drive_type.upper()}")

    # fix_base is tri-state only on 6.0; legacy builds take a plain bool, and their
    # historical default is True. Do not let None collapse into "floating".
    fix_base = True if args.fix_base is None else bool(args.fix_base)
    if args.fix_base is None:
        print("[import] note: --base-as-authored is 6.0-only; using fix_base=True on this build")

    _set_if_present(cfg, "merge_fixed_joints", bool(args.merge_fixed_joints))
    _set_if_present(cfg, "fix_base", fix_base)
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


def _import_legacy(handles, urdf_path: Path, output: Path, args) -> tuple[Path, Path | None]:
    """
    LEGACY fallback for Isaac Sim 4.5 / 5.x.

    Uses the `URDFParseFile` + `URDFImportRobot` kit commands, which on those builds
    populate the live `omni.usd` stage and honour `dest_path` as a FILE.  Both
    commands are deprecation shims on 6.0 (`URDFParseFile.do()` raises), which is why
    this path is never taken there.
    """
    import omni.kit.commands

    _urdf = handles["_urdf"]
    print(f"[import] LEGACY backend via {handles['module']} (Isaac Sim <= 5.x)")

    urdf_interface = _urdf.acquire_urdf_interface()
    print(f"[import] urdf interface: {urdf_interface}")

    cfg = build_import_config_legacy(_urdf, args)
    output.parent.mkdir(parents=True, exist_ok=True)

    ok, robot_model = omni.kit.commands.execute(
        "URDFParseFile", urdf_path=str(urdf_path), import_config=cfg
    )
    if not ok:
        raise RuntimeError(f"URDFParseFile failed for {urdf_path}")
    ok, _ = omni.kit.commands.execute(
        "URDFImportRobot",
        urdf_robot=robot_model,
        import_config=cfg,
        dest_path=str(output),
    )
    if not ok:
        raise RuntimeError(f"URDFImportRobot failed for {urdf_path}")

    if not (output.is_file() and output.stat().st_size > 0):
        # Old builds sometimes ignored dest_path and only populated the live stage.
        print("[usd] dest_path produced no file; exporting the live stage instead")
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("No live USD stage to export and dest_path wrote nothing.")
        stage.Export(str(output))
    if not output.is_file():
        raise RuntimeError(f"Legacy import produced no file at {output}")
    print(f"[usd] {output}  ({output.stat().st_size / 1024:.1f} KiB)")
    return output, None


# --------------------------------------------------------------------------- #
# 5.  Post-import report
# --------------------------------------------------------------------------- #
def report_articulation(usd_file: Path) -> None:
    """
    Print the joints/links actually authored -- cheap smoke test.

    Reads the WRITTEN FILE, not `omni.usd.get_context().get_stage()`.  On 6.0 the
    importer operates on a standalone `Usd.Stage.Open()` and never populates the Kit
    context stage, so the old context-based report always saw an empty stage.
    """
    try:
        from pxr import Usd, UsdPhysics

        stage = Usd.Stage.Open(str(usd_file))
        if stage is None:
            print(f"[usd] (post-import report skipped: could not open {usd_file})")
            return

        roots, links, joints = [], [], []
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                roots.append(prim.GetPath().pathString)
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                links.append(prim.GetPath().pathString)
            if prim.IsA(UsdPhysics.Joint):
                joints.append((prim.GetPath().name, prim.GetTypeName()))

        default_prim = stage.GetDefaultPrim()
        print(f"\n[usd] file              : {usd_file}")
        print(f"[usd] default prim      : {default_prim.GetPath() if default_prim else '(none)'}")
        print(f"[usd] articulation roots: {roots or '(none)'}")
        print(f"[usd] rigid bodies      : {len(links)}")
        print(f"[usd] physics joints    : {len(joints)}")
        for name, typ in joints:
            print(f"[usd]     {typ:<22} {name}")
        if not roots:
            print("[usd] WARNING: no UsdPhysics.ArticulationRootAPI found -- this asset "
                  "will not simulate as an articulation.")
    except Exception as exc:
        print(f"[usd] (post-import report skipped: {exc})")


# --------------------------------------------------------------------------- #
# 6.  CLI
# --------------------------------------------------------------------------- #
def parse_args(argv=None):
    p = argparse.ArgumentParser(
        prog="urdf_to_usd.py",
        description="Headless Isaac Sim 6.0.1 URDF/xacro -> USD converter (ECR88 X1 PanelLift).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=textwrap.dedent(
            """\
            examples:
              scripts/run_isaac.sh convert --xacro assets/ecr88/urdf/ecr88.urdf.xacro \\
                                           --output assets/ecr88/usd/ecr88.usd
              /isaac-sim/python.sh urdf_to_usd.py --urdf /tmp/ecr88.urdf \\
                                           --output /tmp/ecr88.usda --floating --merge-fixed-joints
              /isaac-sim/python.sh urdf_to_usd.py --xacro ... --output ... \\
                                           -D use_meshes:=true -D arm_variant:=2.1m
            """
        ),
    )
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--xacro", type=Path, help="xacro file to expand and import")
    src.add_argument("--urdf", type=Path, help="already-flat URDF to import")

    p.add_argument("--output", "-o", type=Path, required=True,
                   help="output .usd / .usda / .usdc path (single flattened file)")

    base = p.add_mutually_exclusive_group()
    base.add_argument("--fix-base", dest="fix_base", action="store_true",
                      help="weld the chassis to the world (default)")
    base.add_argument("--floating", dest="fix_base", action="store_false",
                      help="free-floating base (needed for travel/undercarriage sim)")
    base.add_argument("--base-as-authored", dest="fix_base", action="store_const", const=None,
                      help="Isaac Sim 6.0 only: leave the URDF's base authoring untouched "
                           "(fix_base=None). Ignored on the legacy backend.")
    p.set_defaults(fix_base=True)

    p.add_argument("--merge-fixed-joints", dest="merge_fixed_joints",
                   action="store_true", default=False,
                   help="collapse fixed joints. WARNING: destroys the gnss/probe/"
                        "contact_surface frames you need for ROS 2 TF. Default off.")
    p.add_argument("--no-merge-fixed-joints", dest="merge_fixed_joints",
                   action="store_false", help=argparse.SUPPRESS)

    p.add_argument("--joint-drive-type", choices=("none", "position", "velocity"),
                   default="position",
                   help="what the joint drives track. Isaac Sim 6.0 field: "
                        "joint_target_type (default: position)")
    p.add_argument("--joint-drive-mode", choices=("force", "acceleration"), default=None,
                   help="Isaac Sim 6.0 only: joint_drive_type, i.e. how the gains are "
                        "interpreted. Default None = leave the converter's choice alone.")
    p.add_argument("--drive-strength", type=float, default=1e7,
                   help="default drive stiffness/gain (6.0 field: override_joint_stiffness). "
                        "PLACEHOLDER -- the real value depends on hydraulic cylinder forces "
                        "we do not have yet.")
    p.add_argument("--drive-damping", type=float, default=1e5,
                   help="default drive damping (6.0 field: override_joint_damping). "
                        "PLACEHOLDER, see above.")

    p.add_argument("--distance-scale", type=float, default=1.0,
                   help="URDF length unit -> stage metersPerUnit. Isaac Sim 6.0 is "
                        "metres-only and has no such field, so only 1.0 is accepted there; "
                        "other values are honoured on the legacy backend only.")
    p.add_argument("--density", type=float, default=0.0,
                   help="0.0 = trust the URDF <inertial> blocks (default). Any other value "
                        "sets a link density override (6.0 field: link_density) and makes "
                        "the importer author mass from geometry, silently discarding the "
                        "xacro's ESTIMATE placeholders.")
    p.add_argument("--no-import-inertia-tensor", dest="import_inertia_tensor",
                   action="store_false", default=True,
                   help="legacy backends only: let the importer recompute inertia instead of "
                        "using the URDF's. No-op on Isaac Sim 6.0, which always uses the URDF.")
    p.add_argument("--self-collision", action="store_true", default=False,
                   help="enable self-collision (6.0 field: allow_self_collision). Expensive; "
                        "the 4-bar linkage will self-collide with primitive shapes.")
    p.add_argument("--convex-decomp", action="store_true", default=False,
                   help="convex-decompose collision meshes. On 6.0 this is shorthand for "
                        "--collision-type 'Convex Decomposition'.")
    p.add_argument("--collision-type", choices=COLLISION_TYPES, default="Convex Hull",
                   help="Isaac Sim 6.0 only: collision approximation (default: Convex Hull)")
    p.add_argument("--collision-from-visuals", action="store_true", default=False,
                   help="derive collision from visual geometry")
    p.add_argument("--merge-mesh", action="store_true", default=False,
                   help="Isaac Sim 6.0 only: merge per-link meshes (config: merge_mesh)")
    p.add_argument("--no-physics-scene", dest="create_physics_scene",
                   action="store_false", default=True,
                   help="do not add a PhysicsScene prim. On Isaac Sim 6.0 a URDF import never "
                        "emits one, so this is already the behaviour.")
    p.add_argument("--ros-package", dest="ros_packages", action="append", default=[],
                   metavar="NAME=PATH",
                   help="Isaac Sim 6.0 only: resolve package:// URIs, repeatable "
                        "(config: ros_package_paths)")

    p.add_argument("-D", "--mapping", dest="mappings", action="append", default=[],
                   metavar="NAME:=VALUE",
                   help="xacro argument, repeatable (e.g. -D use_meshes:=true)")
    p.add_argument("--xacro-bin", default=None,
                   help="explicit path to a xacro executable (or set $XACRO)")
    p.add_argument("--keep-urdf", action="store_true",
                   help="keep the expanded intermediate .urdf next to the output")
    p.add_argument("--no-keep-package", dest="keep_package", action="store_false", default=True,
                   help="Isaac Sim 6.0 only: delete the importer's layered package directory "
                        "(<output-stem>_pkg) after flattening. It holds textures and the "
                        "original layer structure, so it is kept by default.")
    p.add_argument("--debug-import", action="store_true", default=False,
                   help="Isaac Sim 6.0 only: config debug_mode -- keep the importer's "
                        "intermediate artifacts next to the output for inspection")
    p.add_argument("--renderer", default="MinimalRendering",
                   help="Kit renderer. Isaac Sim 6.0 accepts RaytracedLighting, PathTracing, "
                        "RealTimePathTracing or MinimalRendering (default: MinimalRendering -- "
                        "nothing is rendered during a conversion).")
    p.add_argument("--gui", action="store_true",
                   help="run with a window instead of headless (debugging only)")
    return p.parse_args(argv)


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    # Probe Isaac Sim BEFORE argparse so that a plain `python3 urdf_to_usd.py ...`
    # gets the actionable environment message instead of an argparse usage error or
    # a bare ImportError.  --help still works everywhere.
    if not any(a in ("-h", "--help") for a in argv):
        _require_isaac_sim()
    args = parse_args(argv)

    # SimulationApp MUST be constructed before any omni.* / isaacsim.asset.* import.
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": not args.gui, "renderer": args.renderer})

    exit_code = 0
    tmpdir = None
    try:
        print(f"[import] Isaac Sim version: {_isaac_version_string()}")
        enable_urdf_extension()
        simulation_app.update()

        backend, handles = detect_backend()
        print(f"[import] backend  : {backend}")

        if backend == BACKEND_60 and args.distance_scale != 1.0:
            raise ConfigError(
                f"--distance-scale {args.distance_scale} is not supported on Isaac Sim 6.0.\n"
                "The 6.0 URDF converter is metres-only; URDFImporterConfig has no "
                "distance_scale field, so the value would be silently ignored rather than "
                "applied. Re-author the URDF in metres (the ECR88 xacro already is) or set "
                "metersPerUnit on the composing stage afterwards."
            )

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

        output = args.output.resolve()
        print(f"[import] source   : {urdf_path}")
        print(f"[import] output   : {output}")
        print(f"[import] fix_base : {args.fix_base}")
        print(f"[import] merge_fj : {args.merge_fixed_joints}")

        if backend == BACKEND_60:
            entry, package_root = _import_60(handles, urdf_path, output, args)
            flatten_to_output(entry, output)
            if package_root is not None:
                if args.keep_package:
                    print(f"[usd] importer package kept at: {package_root}")
                else:
                    shutil.rmtree(package_root, ignore_errors=True)
                    print(f"[usd] importer package removed: {package_root}")
        else:
            _import_legacy(handles, urdf_path, output, args)

        simulation_app.update()
        report_articulation(output)

    except XacroError as exc:
        sys.stderr.write(f"\n[FATAL] {exc}\n")
        exit_code = 3
    except ConfigError as exc:
        # A user-facing option clash: actionable message, no traceback.
        sys.stderr.write(f"\n[FATAL] {exc}\n")
        exit_code = 4
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
