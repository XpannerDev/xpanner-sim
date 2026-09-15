#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
validate_urdf.py -- dependency-light structural + kinematic validator for the
Volvo ECR88 / Xpanner X1 PanelLift URDF.

DESIGN CONSTRAINT: runs on python3 stdlib ALONE.  No Isaac Sim, no ROS, no
urdfdom, no PyKDL.  numpy is used if importable and pure-python 3x3/4x4 math is
used if it is not; the results are identical either way.  This is deliberate --
the whole point is that CI, a laptop, or a colleague without an Omniverse
install can check the asset before anyone spends 4 GB on Isaac Sim.

WHAT IT CHECKS
--------------
structure   single root link; no cycles; every joint parent/child resolves;
            no duplicate link or joint names; every link reachable from the root
inertial    no zero / missing mass in the articulated chain (evaluated per
            fixed-joint rigid cluster, which is what PhysX actually merges);
            positive inertia diagonal; triangle inequality on the principal moments
joints      revolute + prismatic joints carry <limit> with lower < upper and
            non-zero effort/velocity; joint axes are unit vectors; joint types
            are legal; <mimic> targets exist
geometry    referenced mesh files exist on disk (so the "flip one flag to meshes"
            path fails here rather than inside Isaac Sim)
kinematics  forward kinematics at the ZERO POSE, printed as a tree with
            cumulative XYZ, so a human can eyeball boom tip / arm tip /
            contact_surface against the parameter sheet
groundtruth every length and offset in resources/ECR88_kinematic_parameters.md
            (sheet KinematicPara_new, column "ECR88 기장") is searched for among
            the joint origins and reported MATCHED / NOT FOUND

EXIT CODES
----------
    0   passed (warnings may still be present -- read them)
    1   one or more HARD failures
    2   could not get as far as validating (no xacro, unreadable file, bad XML)

USAGE
-----
    python3 scripts/validate_urdf.py --xacro assets/ecr88/urdf/ecr88.urdf.xacro
    python3 scripts/validate_urdf.py --urdf  /tmp/ecr88.urdf --print-urdf
    python3 scripts/validate_urdf.py --xacro ... -D use_meshes:=true
    XACRO=/tmp/xv/bin/xacro python3 scripts/validate_urdf.py --xacro ...

XACRO EXPANSION
---------------
Tried in order: $XACRO / --xacro-bin, a `xacro` on PATH, the python `xacro`
module, then a BUILT-IN minimal expander (properties / args / if / unless /
include / macros / insert_block / ${} / $(arg)).  The built-in is a reduced
subset and says so loudly every time it is used -- it exists so this script
still works on a bare box, not so that anyone ships without real xacro.
"""

from __future__ import annotations

import argparse
import copy
import math
import os
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parent.parent

# --------------------------------------------------------------------------- #
# optional numpy
# --------------------------------------------------------------------------- #
try:
    import numpy as _np  # noqa: N813

    HAVE_NUMPY = True
except Exception:
    _np = None
    HAVE_NUMPY = False

AXIS_TOL = 1e-6         # |axis| - 1 tolerance
MATCH_TOL = 5e-4        # 0.5 mm -- ground-truth offset matching


# --------------------------------------------------------------------------- #
# ground truth  (resources/ECR88_kinematic_parameters.md, sheet KinematicPara_new).
# Rows marked Delete are intentionally absent.
#
# The sheet has one column PER MACHINE VARIANT and the xacro picks one with
# `machine_variant`.  Checking a 2.1 m build against the 기장 column made the
# validator warn about lenArm and distArmToInpLink on a perfectly correct asset --
# false warnings are worse than none, because they train people to skip the block.
# So: the base table is the 기장 column, and VARIANT_OVERRIDES carries the rows that
# actually differ.  `--variant` selects one; it defaults to what the xacro is set to,
# read out of ecr88_params.xacro, so the default run is always self-consistent.
# --------------------------------------------------------------------------- #
VARIANT_OVERRIDES = {
    # KinematicPara_new col "ECR88 2.1m 미국#1" and "... 신규흡착기".
    # Only rows whose value actually moves are listed.
    "ECR88_US1_2P1M": {
        "offsets": {"distArmToInpLink": (1.841, 0.0, 0.0185)},
        "lengths": {"lenArm": 2.1},
    },
    "ECR88_US1_2P1M_NEWSUCTION": {
        "offsets": {"distArmToInpLink": (1.841, 0.0, 0.0185),
                    "distAttToProbe": (0.255, 0.0, -0.57),
                    "distProbeToContactSurface": (0.1, 0.0, -0.262)},
        "lengths": {"lenArm": 2.1},
        "sum_check": (0.355, 0.0, -0.832),
    },
    "ECR88_KIJANG": {"offsets": {}, "lengths": {}},
}

# --------------------------------------------------------------------------- #
# A SECOND ground truth, and it outranks the sheet.
#
# XpannerLab/X1Exc ships the machine's own parameter files -- ControlModel/Data/
# ECR88D_LongArm.m and ECR88D_ShortArm.m -- using the SAME parameter names as the
# sheet. Comparing the two: 21 of 22 shared scalars and 10 of 11 shared vectors agree
# to 1e-6, which is a strong check on the whole geometry chain. The rows below are the
# ones that do NOT agree, and in each case the firmware is what actually runs on the
# machine, so the asset follows the firmware and the validator has to as well --
# otherwise a correct asset warns forever and people learn to ignore the block.
#
# Raise these with the OEM team rather than quietly living with them: for the antenna
# offset the 71 mm gap is bigger than the firmware's own placement tolerance.
FIRMWARE_OVERRIDES = {
    "offsets": {
        # sheet -1.416 for all three variants; both firmware variants say -1.345.
        # X and Y agree exactly, so it is a mount stack height, not a re-survey.
        "distAntMainToChs": (0.57, -0.087, -1.345),
    },
    "lengths": {},
    # Unit-specific firmware values. ECR88D_ShortArm.m (the 1.7 m unit, compiled into the repo's
    # binary) carries the new suction head; the sheet's 기장 column still has the old one. The
    # Isaac step-28 SIL run found this as a 92 mm contact-surface offset.
    "by_variant": {
        "ECR88_KIJANG": {
            "distAttToProbe": (0.255, 0.0, -0.57),
            "distProbeToContactSurface": (0.1, 0.0, -0.262),
            "distAttToContactSurface": (0.355, 0.0, -0.832),   # the sum of the two above
        },
    },
}


def apply_firmware(variant=None):
    """Fold the firmware's values over the sheet. Call AFTER apply_variant."""
    GT_OFFSETS.update(FIRMWARE_OVERRIDES["offsets"])
    GT_LENGTHS.update(FIRMWARE_OVERRIDES["lengths"])
    extra = dict(FIRMWARE_OVERRIDES["by_variant"].get(variant, {}))
    if "distAttToContactSurface" in extra:          # a sum, checked by GT_SUM_CHECK, not a joint origin
        global GT_SUM_CHECK
        GT_SUM_CHECK = (GT_SUM_CHECK[0], extra.pop("distAttToContactSurface"), GT_SUM_CHECK[2])
    GT_OFFSETS.update(extra)
    return len(FIRMWARE_OVERRIDES["offsets"]) + len(FIRMWARE_OVERRIDES["lengths"]) + len(extra)


def read_variant_from_params(urdf_dir):
    """Return the machine_variant the xacro is actually set to, or None."""
    import re as _re
    p = urdf_dir / "ecr88_params.xacro"
    try:
        m = _re.search(r'<xacro:property\s+name="machine_variant"\s+value="([^"]+)"',
                       p.read_text(encoding="utf-8", errors="replace"))
    except OSError:
        return None
    return m.group(1) if m else None


def apply_variant(name):
    """Fold the variant's overrides into the module-level ground-truth tables."""
    ov = VARIANT_OVERRIDES.get(name)
    if ov is None:
        return False
    GT_OFFSETS.update(ov.get("offsets", {}))
    GT_LENGTHS.update(ov.get("lengths", {}))
    if "sum_check" in ov:
        global GT_SUM_CHECK
        GT_SUM_CHECK = (GT_SUM_CHECK[0], ov["sum_check"], GT_SUM_CHECK[2])
    return True


GT_OFFSETS = {
    "distAntMainToAntAux":      (0.682, 0.959, 0.0),
    "distAntMainToChs":         (0.57, -0.087, -1.416),
    "distChsToBmMnt":           (0.94, -0.15, 0.0),
    "distBmMntToBm1":           (0.0, 0.0, 0.0),
    "distArmToInpLink":         (1.441, 0.0, 0.0185),
    "distOutpLinkToTiltMnt":    (0.0, 0.0, 0.0),
    "distTiltMntToTilt":        (0.28, 0.0, -0.22),
    "distRotToAtt":             (0.0, 0.0, -0.2615),
    "distAttToProbe":           (0.35, 0.0, -0.572),
    "distProbeToContactSurface": (0.1, 0.0, -0.264),
    "distCylLrgBm1":            (0.272, 0.0, 0.391),
    "distCylSmlBm1":            (1.715, 0.0, 0.58),
    "distCylLrgArm":            (-1.508, 0.0, 1.08),
}
# Pure scalar link lengths -- matched against |joint origin| along the chain.
GT_LENGTHS = {
    "lenBm1": 3.55,
    "lenArm": 1.7,
    "lenInpLink": 0.42,
    "lenConnRod": 0.4,
    "lenOutpLink": 0.33,
    "lenGndLink": 0.2597,
}
# Zero-valued rows exist in the sheet but cannot be distinguished from "absent",
# so they are reported as informational rather than searched for.
GT_ZERO_ROWS = {"distBmMntToBm1", "distOutpLinkToTiltMnt", "lenBm2"}
# distAttToContactSurface is the sum of the two frames above it; the sheet states
# the additive rule explicitly, so we verify the sum rather than look for a joint.
GT_SUM_CHECK = ("distAttToContactSurface", (0.45, 0.0, -0.836),
                ["distAttToProbe", "distProbeToContactSurface"])


# =========================================================================== #
# 1.  xacro expansion
# =========================================================================== #
XNS = "http://www.ros.org/wiki/xacro"
XP = "{%s}" % XNS


class XacroUnsupported(RuntimeError):
    """The built-in mini-expander hit a construct it refuses to guess at."""


class XacroFailed(RuntimeError):
    """Could not expand -- environment problem; caller should SKIP (exit 2)."""


class XacroRejected(XacroFailed):
    """A real xacro ran and refused the file -- asset fault; caller FAILS (exit 1)."""


def _safe_eval_env(scope: dict) -> dict:
    env = {
        k: v for k, v in vars(math).items() if not k.startswith("_")
    }
    env.update({"pi": math.pi, "True": True, "False": False, "None": None,
                "min": min, "max": max, "abs": abs, "round": round,
                "int": int, "float": float, "str": str, "len": len})
    for k, v in scope.items():
        env[k] = _coerce(v)
    return env


def _coerce(v):
    if not isinstance(v, str):
        return v
    s = v.strip()
    if s in ("True", "true"):
        return True
    if s in ("False", "false"):
        return False
    try:
        return int(s)
    except ValueError:
        pass
    try:
        return float(s)
    except ValueError:
        pass
    return v


def _eval_expr(expr: str, scope: dict, where: str):
    try:
        return eval(expr, {"__builtins__": {}}, _safe_eval_env(scope))  # noqa: S307
    except Exception as exc:
        raise XacroUnsupported(
            f"cannot evaluate ${{{expr}}} in {where}: {type(exc).__name__}: {exc}"
        ) from exc


def _subst(text: str, scope: dict, args: dict, where: str) -> str:
    """Expand ${...} and $(arg ...) / $(find ...) in a string."""
    if text is None or "$" not in text:
        return text
    out, i, n = [], 0, len(text)
    while i < n:
        c = text[i]
        if c != "$":
            out.append(c)
            i += 1
            continue
        if i + 1 < n and text[i + 1] == "$":       # $$ escape
            out.append("$")
            i += 2
            continue
        if i + 1 < n and text[i + 1] in "{(":
            opener = text[i + 1]
            closer = "}" if opener == "{" else ")"
            depth, j = 1, i + 2
            while j < n and depth:
                if text[j] == opener:
                    depth += 1
                elif text[j] == closer:
                    depth -= 1
                j += 1
            if depth:
                raise XacroUnsupported(f"unbalanced '{opener}' in {where}: {text!r}")
            inner = text[i + 2: j - 1]
            if opener == "{":
                val = _eval_expr(inner, scope, where)
                out.append(_fmt(val))
            else:
                parts = inner.split(None, 1)
                kind = parts[0] if parts else ""
                rest = parts[1].strip() if len(parts) > 1 else ""
                if kind == "arg":
                    if rest not in args:
                        raise XacroUnsupported(
                            f"$(arg {rest}) used in {where} but never declared "
                            f"via <xacro:arg> and not passed with -D")
                    out.append(str(args[rest]))
                elif kind == "find":
                    out.append(_find_pkg(rest))
                elif kind == "eval":
                    out.append(_fmt(_eval_expr(rest, scope, where)))
                elif kind == "optenv":
                    bits = rest.split(None, 1)
                    out.append(os.environ.get(bits[0], bits[1] if len(bits) > 1 else ""))
                elif kind == "env":
                    out.append(os.environ[rest])
                else:
                    raise XacroUnsupported(f"$({kind} ...) unsupported in {where}")
            i = j
            continue
        out.append(c)
        i += 1
    return "".join(out)


def _fmt(v) -> str:
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, float):
        if v == int(v) and abs(v) < 1e15:
            return str(int(v)) if abs(v) >= 1 or v == 0 else repr(v)
        return repr(v)
    return str(v)


_PKG_CACHE: dict[str, str] = {}


def _find_pkg(pkg: str) -> str:
    """Best-effort $(find pkg): look for a package.xml named `pkg` under the repo."""
    if pkg in _PKG_CACHE:
        return _PKG_CACHE[pkg]
    for root in (REPO_ROOT, REPO_ROOT.parent):
        for cand in root.rglob("package.xml"):
            try:
                nm = ET.parse(cand).getroot().findtext("name")
            except Exception:
                continue
            if nm and nm.strip() == pkg:
                _PKG_CACHE[pkg] = str(cand.parent)
                return _PKG_CACHE[pkg]
    for root in (REPO_ROOT, REPO_ROOT.parent):
        d = root / pkg
        if d.is_dir():
            _PKG_CACHE[pkg] = str(d)
            return str(d)
    raise XacroUnsupported(
        f"$(find {pkg}) could not be resolved: no package.xml naming '{pkg}' and no "
        f"directory '{pkg}' under {REPO_ROOT}. Install real xacro with a sourced "
        f"ROS 2 workspace, or use a relative <xacro:include filename=...>.")


def _local(tag: str) -> str:
    return tag.split("}")[-1] if "}" in tag else tag


def _is_x(el) -> bool:
    return isinstance(el.tag, str) and el.tag.startswith(XP)


class MiniXacro:
    """A deliberately small xacro subset. Raises XacroUnsupported on anything else."""

    SUPPORTED = ("property", "arg", "include", "macro", "if", "unless",
                 "insert_block", "element", "attribute")

    def __init__(self, cli_args: dict):
        self.args: dict = dict(cli_args)
        self.macros: dict = {}
        self.notes: list[str] = []

    # -- public ----------------------------------------------------------- #
    def expand(self, path: Path) -> ET.Element:
        try:
            root = ET.parse(path).getroot()
        except ET.ParseError as exc:
            raise XacroFailed(f"XML parse error in {path}: {exc}") from exc
        if _local(root.tag) != "robot":
            raise XacroFailed(f"{path}: root element is <{_local(root.tag)}>, expected <robot>")
        scope: dict = {}
        out = ET.Element("robot", {k: v for k, v in root.attrib.items()})
        # a first sweep for <xacro:arg>, so $(arg) works regardless of ordering
        self._collect_args(root, path)
        kids = self._process(list(root), scope, path)
        for k in kids:
            out.append(k)
        if "name" in root.attrib:
            out.set("name", _subst(root.attrib["name"], scope, self.args, str(path)))
        return out

    # -- internals -------------------------------------------------------- #
    def _collect_args(self, parent, path: Path):
        for el in parent:
            if _is_x(el) and _local(el.tag) == "arg":
                nm = el.attrib["name"]
                if nm not in self.args:
                    if "default" not in el.attrib:
                        raise XacroUnsupported(
                            f"<xacro:arg name='{nm}'> has no default and was not "
                            f"supplied with -D {nm}:=<value>")
                    self.args[nm] = el.attrib["default"]
            elif _is_x(el) and _local(el.tag) == "include":
                inc = self._resolve_include(el, {}, path)
                if inc and inc.is_file():
                    try:
                        self._collect_args(ET.parse(inc).getroot(), inc)
                    except Exception:
                        pass
            else:
                self._collect_args(el, path)

    def _resolve_include(self, el, scope, path: Path):
        fn = el.attrib.get("filename")
        if fn is None:
            return None
        try:
            fn = _subst(fn, scope, self.args, f"{path}:<xacro:include>")
        except XacroUnsupported:
            return None
        p = Path(fn)
        if not p.is_absolute():
            p = (path.parent / p)
        return p.resolve()

    def _process(self, nodes, scope: dict, path: Path) -> list:
        out = []
        for el in nodes:
            if not isinstance(el.tag, str):     # comment / PI
                continue
            if _is_x(el):
                out.extend(self._xacro_node(el, scope, path))
            elif _local(el.tag) in self.macros and el.tag == _local(el.tag):
                out.extend(self._call_macro(_local(el.tag), el, scope, path))
            else:
                out.append(self._plain(el, scope, path))
        return out

    def _plain(self, el, scope, path: Path):
        where = f"{path}:<{_local(el.tag)}>"
        new = ET.Element(_local(el.tag))
        for k, v in el.attrib.items():
            new.set(_local(k), _subst(v, scope, self.args, where))
        if el.text and el.text.strip():
            new.text = _subst(el.text, scope, self.args, where)
        for child in self._process(list(el), scope, path):
            new.append(child)
        return new

    def _xacro_node(self, el, scope, path: Path) -> list:
        name = _local(el.tag)
        where = f"{path}:<xacro:{name}>"

        if name == "property":
            pn = el.attrib["name"]
            if "value" in el.attrib:
                scope[pn] = _subst(el.attrib["value"], scope, self.args, where)
            elif "default" in el.attrib:
                scope.setdefault(pn, _subst(el.attrib["default"], scope, self.args, where))
            else:
                raise XacroUnsupported(
                    f"{where} name='{pn}' is a block property (no value=); the "
                    f"built-in expander does not implement block properties.")
            return []

        if name == "arg":
            return []                                     # handled in _collect_args

        if name == "include":
            inc = self._resolve_include(el, scope, path)
            if inc is None or not inc.is_file():
                raise XacroFailed(f"{where} cannot find {el.attrib.get('filename')!r} "
                                  f"(resolved to {inc})")
            sub = ET.parse(inc).getroot()
            self._collect_args(sub, inc)
            return self._process(list(sub), scope, inc)

        if name == "macro":
            mn = el.attrib["name"]
            params = el.attrib.get("params", "").split()
            self.macros[mn] = (params, list(el))
            return []

        if name in ("if", "unless"):
            raw = el.attrib.get("value", el.attrib.get("cond", ""))
            val = _coerce(_subst(raw, scope, self.args, where))
            truthy = bool(val) if not isinstance(val, str) else val.strip().lower() in (
                "1", "true", "yes")
            if (name == "if") == truthy:
                return self._process(list(el), scope, path)
            return []

        if name == "insert_block":
            bn = el.attrib["name"]
            block = scope.get("__block__" + bn)
            if block is None:
                raise XacroUnsupported(f"{where} name='{bn}' is not a bound block param")
            return self._process([copy.deepcopy(b) for b in block], scope, path)

        if name in self.macros:
            return self._call_macro(name, el, scope, path)

        raise XacroUnsupported(
            f"{where} is not implemented by the built-in expander "
            f"(supported: {', '.join('xacro:' + s for s in self.SUPPORTED)}). "
            f"Install real xacro.")

    def _call_macro(self, mname: str, el, scope, path: Path) -> list:
        params, body = self.macros[mname]
        where = f"{path}:<xacro:{mname}>"
        local = dict(scope)
        given = {k: _subst(v, scope, self.args, where) for k, v in el.attrib.items()}
        blocks = [c for c in list(el) if isinstance(c.tag, str)]
        bi = 0
        for p in params:
            if p.startswith("**"):
                raise XacroUnsupported(f"{where}: ** params not implemented")
            if p.startswith("*"):
                pn = p[1:]
                if bi >= len(blocks):
                    raise XacroUnsupported(
                        f"{where}: block param *{pn} has no corresponding child element")
                local["__block__" + pn] = [blocks[bi]]
                bi += 1
                continue
            if ":=" in p:
                pn, default = p.split(":=", 1)
                default = default.strip("'\"")
                local[pn] = given.get(pn, _subst(default, local, self.args, where))
            elif "=" in p:
                pn, default = p.split("=", 1)
                local[pn] = given.get(pn, _subst(default.strip("'\""), local, self.args, where))
            else:
                pn = p
                if pn not in given:
                    raise XacroUnsupported(
                        f"{where}: required macro param '{pn}' not supplied")
                local[pn] = given[pn]
        return self._process([copy.deepcopy(b) for b in body], local, path)


def _xml_wellformed_report(path: Path) -> str | None:
    """
    Return a precise, actionable diagnostic if `path` is not well-formed XML.

    Both ElementTree ("not well-formed (invalid token)") and xacro ("Check that
    your XML is well-formed") report malformed XML uselessly, with no idea what
    is actually wrong.  The overwhelmingly common cause in a heavily commented
    URDF is a literal '--' inside an XML comment, which the XML spec forbids
    outright (XML 1.0 s2.5) -- an ASCII-art rule or an arrow like '<-- note'.
    """
    try:
        ET.parse(path)
        return None
    except ET.ParseError as exc:
        # Python deletes the `as` name when the except block exits, so stash it
        # before falling through to the reporting code below.
        parse_error = exc
    except OSError as exc:
        return f"{path}: cannot read: {exc}"

    exc = parse_error
    lineno, col = getattr(exc, "position", (0, 0))
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        lines = []
    out = [f"{path}:{lineno}:{col}: XML is not well-formed -- {exc}"]
    if 0 < lineno <= len(lines):
        src = lines[lineno - 1]
        out.append(f"    {src.rstrip()}")
        out.append("    " + " " * max(col - 1, 0) + "^")

    # scan comments for the forbidden '--'
    text = "\n".join(lines)
    hits = []
    pos = 0
    while True:
        s = text.find("<!--", pos)
        if s < 0:
            break
        e = text.find("-->", s + 4)
        body = text[s + 4: e if e >= 0 else len(text)]
        for m in re.finditer(r"--", body):
            ln = text[:s + 4 + m.start()].count("\n") + 1
            hits.append(ln)
        if e < 0:
            out.append(f"    note: comment opened at line "
                       f"{text[:s].count(chr(10)) + 1} is never closed with '-->'")
            break
        pos = e + 3
    if hits:
        uniq = sorted(set(hits))
        out.append(f"    CAUSE: '--' appears inside an XML comment at line(s) "
                   f"{', '.join(str(h) for h in uniq[:12])}"
                   + (" ..." if len(uniq) > 12 else ""))
        for ln in uniq[:6]:
            if 0 < ln <= len(lines):
                out.append(f"      {ln}: {lines[ln - 1].strip()[:96]}")
        out.append("    XML forbids '--' inside <!-- -->. Fix by rewriting the text:")
        out.append("      '<-- note'      ->  '<== note'  or  '&lt;-- note'")
        out.append("      '-----' rules   ->  '====='")
    if "&" in text and not hits:
        out.append("    hint: an unescaped '&' must be written '&amp;'")
    return "\n".join(out)


def _undefined_symbol_report(paths: list[Path]) -> str | None:
    """
    Statically list EVERY undefined ${symbol} across the xacro set.

    xacro aborts on the first undefined name, so fixing a file with many missing
    properties otherwise means N expand-fail-edit cycles.  This scan reports them
    all at once, and separates the two failure modes that look identical in
    xacro's output but need completely different fixes:

      (A) a spelling mismatch -- the property IS defined, under another spelling
          (e.g. ${dist..._x} used but dist...X defined). A mechanical rename.
      (B) genuinely absent -- no definition under any spelling. Someone has to
          author the value, which for masses and joint limits means an ESTIMATE
          that must be labelled as one.
    """
    defined: set[str] = set()
    used: dict[str, list[tuple[str, int]]] = {}
    for p in paths:
        try:
            s = p.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        defined |= set(re.findall(r'<xacro:property\s+name="([^"]+)"', s))
        defined |= set(re.findall(r'<xacro:arg\s+name="([^"]+)"', s))
        # macro parameters are bound locally at call time, not global properties
        local: set[str] = set()
        for mp in re.findall(r'<xacro:macro[^>]*params="([^"]*)"', s):
            for tok in mp.split():
                local.add(tok.lstrip("*").split(":=")[0].split("=")[0])
        for m in re.finditer(r"\$\{([^}]*)\}", s):
            ln = s[: m.start()].count("\n") + 1
            # Strip quoted string literals first: in ${machine_variant ==
            # 'ECR88_KIJANG'} the quoted token is a VALUE, not a symbol
            # reference, and reporting it as undefined is a false positive.
            expr = re.sub(r"'[^']*'|\"[^\"]*\"", " ", m.group(1))
            for sym in re.findall(r"\b([A-Za-z_]\w*)\b", expr):
                if sym not in local:
                    used.setdefault(sym, []).append((p.name, ln))

    builtin = set(dir(math)) | {"pi", "True", "False", "None", "min", "max", "abs",
                                "round", "int", "float", "str", "len", "radians",
                                "degrees"}
    missing = sorted(set(used) - defined - builtin)
    if not missing:
        return None

    norm = lambda s: s.lower().replace("_", "")          # noqa: E731
    dmap: dict[str, str] = {}
    for dn in defined:
        dmap.setdefault(norm(dn), dn)

    rename = [(mm, dmap[norm(mm)]) for mm in missing if norm(mm) in dmap]
    absent = [mm for mm in missing if norm(mm) not in dmap]

    out = [f"STATIC SCAN: {len(missing)} undefined ${{symbol}}s across "
           f"{len(paths)} file(s). xacro only ever reports the first one."]
    if rename:
        out.append("")
        out.append(f"  (A) SPELLING MISMATCH -- defined, but under another name "
                   f"({len(rename)}). Mechanical rename:")
        for a, b in rename[:14]:
            loc = used[a][0]
            out.append(f"        {loc[0]}:{loc[1]}: ${{{a}}}   ->   defined as  {b}")
        if len(rename) > 14:
            out.append(f"        ... and {len(rename) - 14} more with the same pattern")
    if absent:
        groups: dict[str, list[str]] = {}
        for a in absent:
            groups.setdefault(a.split("_")[0] if "_" in a else a, []).append(a)
        out.append("")
        out.append(f"  (B) GENUINELY ABSENT -- no definition under any spelling "
                   f"({len(absent)}). These must be authored:")
        for g, v in sorted(groups.items()):
            out.append(f"        {g + '_*':<18} {len(v):>3}   "
                       f"{', '.join(sorted(v)[:5])}{' ...' if len(v) > 5 else ''}")
    return "\n".join(out)


def expand_xacro(xacro_path: Path, mappings: list[str], xacro_bin: str | None,
                 allow_builtin: bool, log) -> tuple[ET.Element, str]:
    """Return (robot_element, how). Raises XacroFailed with an actionable message."""
    mappings = list(mappings or [])
    attempts: list[str] = []

    # Pre-flight: well-formedness of the entry file and every file it includes.
    # Doing this up front turns an opaque expander error into an exact line number.
    to_scan = [xacro_path]
    try:
        txt = xacro_path.read_text(encoding="utf-8", errors="replace")
        for m in re.finditer(r'filename\s*=\s*"([^"]+)"', txt):
            fn = m.group(1)
            if "$(" in fn or "${" in fn:
                fn = re.sub(r"\$\([^)]*\)|\$\{[^}]*\}", "", fn)
            cand = (xacro_path.parent / Path(fn).name)
            if cand.is_file() and cand not in to_scan:
                to_scan.append(cand)
    except OSError:
        pass
    bad = [r for r in (_xml_wellformed_report(p) for p in to_scan) if r]
    if bad:
        raise XacroRejected(
            "The xacro source is not valid XML, so no expander can read it.\n\n"
            + "\n\n".join(bad))

    cands: list[tuple[str, list[str]]] = []
    explicit = xacro_bin or os.environ.get("XACRO")
    if explicit:
        cands.append(("$XACRO/--xacro-bin", [explicit]))
    if shutil.which("xacro"):
        cands.append(("xacro on PATH", [shutil.which("xacro")]))
    cands.append(("python -m xacro", [sys.executable, "-m", "xacro"]))

    # Distinguish "no expander is installed" (an environment problem) from "a real
    # expander ran and rejected the file" (a problem in the xacro).  Conflating the
    # two sends whoever is debugging off to reinstall ROS when the actual fault is
    # an undefined property three files away.
    real_expander_errors: list[tuple[str, str]] = []

    for label, base in cands:
        cmd = base + [str(xacro_path)] + mappings
        try:
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
        except (OSError, subprocess.TimeoutExpired) as exc:
            attempts.append(f"  {label}: {type(exc).__name__}: {exc}")
            continue
        if proc.returncode == 0 and proc.stdout.strip():
            try:
                return ET.fromstring(proc.stdout), label
            except ET.ParseError as exc:
                raise XacroFailed(f"{label} produced unparseable XML: {exc}")
        err = (proc.stderr or proc.stdout or "").strip()
        # "No module named xacro" means the expander is absent, not that it judged
        # the file; anything else means a real xacro ran and refused this input.
        absent = ("No module named xacro" in err
                  or "command not found" in err.lower())
        if not absent:
            real_expander_errors.append((label, err))
        attempts.append(f"  {label}: exit {proc.returncode}"
                        + ("\n      " + "\n      ".join(err.splitlines()[-6:])
                           if err else ""))

    if real_expander_errors:
        label, err = real_expander_errors[0]
        msg = (f"A real xacro ({label}) ran and REJECTED this file. The fault is in the\n"
               f"xacro source, not in your environment or in this validator:\n\n"
               + "\n".join("    " + l for l in err.splitlines()[:20]))
        if "not defined" in err or "NameError" in err:
            rep = _undefined_symbol_report(to_scan)
            if rep:
                msg += "\n\n" + rep
        raise XacroRejected(
            msg + "\n\nNothing to validate until the xacro expands. "
                  "This is a hard failure of the asset, not a skip.")

    if allow_builtin:
        cli = {}
        for m in mappings:
            if ":=" in m:
                k, v = m.split(":=", 1)
                cli[k.lstrip("-")] = v
        try:
            root = MiniXacro(cli).expand(xacro_path)
        except XacroUnsupported as exc:
            raise XacroFailed(
                "No real xacro available, and the built-in fallback expander cannot "
                f"handle this file:\n    {exc}\n\n" + _install_hint(attempts))
        banner = (
            "  !! expanded with the BUILT-IN minimal xacro subset, NOT real xacro.\n"
            "  !! It implements property/arg/if/unless/include/macro/insert_block and\n"
            "  !! ${} $(arg) $(find) only. Re-run with a real xacro before release:\n"
            "  !!     source /opt/ros/humble/setup.bash    # or set $XACRO")
        print("  " + "!" * 104)
        print(banner)
        print("  " + "!" * 104)
        log.check3("xacro expansion", "WARN", "built-in minimal subset, not real xacro")
        log.warn("XACRO",
                 "expanded with the BUILT-IN minimal xacro subset, not real xacro -- "
                 "re-run with a real xacro before trusting this for release")
        return root, "built-in minimal expander"

    raise XacroFailed("Could not expand xacro.\n" + _install_hint(attempts))


def _install_hint(attempts: list[str]) -> str:
    return (
        "Tried:\n" + ("\n".join(attempts) if attempts else "  (nothing)") + "\n\n"
        "Fix with ONE of:\n"
        "  source /opt/ros/humble/setup.bash\n"
        "  python3 -m venv /tmp/xv && /tmp/xv/bin/pip install xacro && \\\n"
        "      XACRO=/tmp/xv/bin/xacro python3 " + str(SCRIPT) + " --xacro <file>\n"
        "  xacro model.urdf.xacro > model.urdf && python3 " + str(SCRIPT) +
        " --urdf model.urdf\n")


# =========================================================================== #
# 2.  small linear algebra (numpy if present, pure python otherwise)
# =========================================================================== #
def rpy_to_mat(r: float, p: float, y: float):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    m = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ]
    return _np.array(m, dtype=float) if HAVE_NUMPY else m


def mat_mul(a, b):
    if HAVE_NUMPY:
        return a @ b
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_vec(a, v):
    if HAVE_NUMPY:
        return a @ v
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def vec_add(a, b):
    if HAVE_NUMPY:
        return a + b
    return [a[i] + b[i] for i in range(3)]


def vec(x, y, z):
    return _np.array([x, y, z], dtype=float) if HAVE_NUMPY else [float(x), float(y), float(z)]


def identity3():
    return _np.eye(3) if HAVE_NUMPY else [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]


def norm(v) -> float:
    return math.sqrt(sum(float(c) * float(c) for c in v))


# =========================================================================== #
# 3.  URDF model
# =========================================================================== #
class Log:
    def __init__(self):
        self.fails: list[tuple[str, str]] = []
        self.warns: list[tuple[str, str]] = []
        self.infos: list[tuple[str, str]] = []
        self.checks: list[tuple[str, str, str]] = []   # (check, status, detail)

    def fail(self, cat, msg):
        self.fails.append((cat, msg))

    def warn(self, cat, msg):
        self.warns.append((cat, msg))

    def info(self, cat, msg):
        self.infos.append((cat, msg))

    def check(self, name, ok, detail=""):
        self.checks.append((name, "PASS" if ok else "FAIL", detail))
        return ok

    def check3(self, name, status, detail=""):
        self.checks.append((name, status, detail))


def _floats(s: str, n: int, default):
    if s is None:
        return list(default)
    parts = s.replace(",", " ").split()
    if len(parts) != n:
        raise ValueError(f"expected {n} numbers, got {len(parts)}: {s!r}")
    return [float(p) for p in parts]


class Joint:
    __slots__ = ("name", "type", "parent", "child", "xyz", "rpy", "axis",
                 "limit", "mimic", "el")

    def __init__(self, el):
        self.el = el
        self.name = el.get("name", "")
        self.type = el.get("type", "")
        pe, ce = el.find("parent"), el.find("child")
        self.parent = pe.get("link") if pe is not None else None
        self.child = ce.get("link") if ce is not None else None
        o = el.find("origin")
        self.xyz = _floats(o.get("xyz") if o is not None else None, 3, (0, 0, 0))
        self.rpy = _floats(o.get("rpy") if o is not None else None, 3, (0, 0, 0))
        a = el.find("axis")
        self.axis = _floats(a.get("xyz") if a is not None else None, 3, (1, 0, 0)) \
            if a is not None else None
        self.limit = el.find("limit")
        self.mimic = el.find("mimic")


class Link:
    __slots__ = ("name", "mass", "inertia", "com", "has_inertial",
                 "visuals", "collisions", "meshes", "el")

    def __init__(self, el):
        self.el = el
        self.name = el.get("name", "")
        self.has_inertial = False
        self.mass = 0.0
        self.inertia = None
        self.com = [0.0, 0.0, 0.0]
        i = el.find("inertial")
        if i is not None:
            self.has_inertial = True
            m = i.find("mass")
            if m is not None and m.get("value") is not None:
                self.mass = float(m.get("value"))
            o = i.find("origin")
            if o is not None:
                self.com = _floats(o.get("xyz"), 3, (0, 0, 0))
            inz = i.find("inertia")
            if inz is not None:
                self.inertia = {k: float(inz.get(k, "0")) for k in
                                ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")}
        self.visuals = el.findall("visual")
        self.collisions = el.findall("collision")
        self.meshes = [m.get("filename") for m in el.iter("mesh") if m.get("filename")]


class Model:
    def __init__(self, root: ET.Element):
        self.root_el = root
        self.name = root.get("name", "<unnamed>")
        self.links: dict[str, Link] = {}
        self.joints: dict[str, Joint] = {}
        self.dup_links: list[str] = []
        self.dup_joints: list[str] = []
        for el in root.findall("link"):
            lk = Link(el)
            if lk.name in self.links:
                self.dup_links.append(lk.name)
            self.links[lk.name] = lk
        # `all_joints` keeps EVERY parsed <joint>, including same-named duplicates.
        # Checks must iterate this list, not the dict: a duplicate name silently
        # overwrites its twin in the dict and would mask that twin's defects
        # (a bad axis, a second parent for a link) behind the duplicate-name error.
        self.all_joints: list[Joint] = []
        for el in root.findall("joint"):
            jt = Joint(el)
            self.all_joints.append(jt)
            if jt.name in self.joints:
                self.dup_joints.append(jt.name)
            else:
                self.joints[jt.name] = jt


# =========================================================================== #
# 4.  checks
# =========================================================================== #
VALID_TYPES = {"revolute", "continuous", "prismatic", "fixed", "floating", "planar"}
NEEDS_LIMIT = {"revolute", "prismatic"}
NEEDS_AXIS = {"revolute", "continuous", "prismatic", "planar"}


def check_structure(m: Model, log: Log):
    log.check("no duplicate link names", not m.dup_links,
              ", ".join(sorted(set(m.dup_links))) or f"{len(m.links)} links")
    for d in set(m.dup_links):
        log.fail("STRUCT", f"duplicate <link name='{d}'>")
    log.check("no duplicate joint names", not m.dup_joints,
              ", ".join(sorted(set(m.dup_joints))) or f"{len(m.all_joints)} joints")
    for d in set(m.dup_joints):
        log.fail("STRUCT", f"duplicate <joint name='{d}'>")

    bad_refs = []
    for j in m.all_joints:
        if not j.parent:
            bad_refs.append(f"{j.name}: missing <parent>")
        elif j.parent not in m.links:
            bad_refs.append(f"{j.name}: parent link '{j.parent}' undefined")
        if not j.child:
            bad_refs.append(f"{j.name}: missing <child>")
        elif j.child not in m.links:
            bad_refs.append(f"{j.name}: child link '{j.child}' undefined")
    log.check("joint parent/child links all resolve", not bad_refs,
              f"{len(bad_refs)} unresolved" if bad_refs else f"{len(m.all_joints)} joints")
    for b in bad_refs:
        log.fail("STRUCT", b)

    # each link at most one parent joint -> tree, not graph
    parent_of: dict[str, str] = {}
    multi = []
    for j in m.all_joints:
        if not j.child:
            continue
        if j.child in parent_of:
            multi.append(f"link '{j.child}' is the child of both "
                         f"'{parent_of[j.child]}' and '{j.name}'")
        else:
            parent_of[j.child] = j.name
    log.check("every link has at most one parent joint", not multi,
              f"{len(multi)} violations" if multi else "tree topology")
    for msg in multi:
        log.fail("STRUCT", msg)

    roots = [n for n in m.links if n not in parent_of]
    log.check("exactly one root link", len(roots) == 1,
              (roots[0] if len(roots) == 1 else f"{len(roots)}: {', '.join(sorted(roots))}"))
    if len(roots) != 1:
        log.fail("STRUCT",
                 f"expected exactly 1 root link, found {len(roots)}: {sorted(roots)}"
                 + ("  (0 roots means a cycle)" if not roots else ""))
    root = roots[0] if len(roots) == 1 else (sorted(roots)[0] if roots else None)

    # cycle detection / reachability
    children: dict[str, list[Joint]] = {}
    for j in m.all_joints:
        children.setdefault(j.parent, []).append(j)
    seen, cyc = set(), []
    if root:
        stack = [(root, [root])]
        while stack:
            node, path = stack.pop()
            if node in seen:
                continue
            seen.add(node)
            for j in children.get(node, []):
                if j.child in path:
                    cyc.append(" -> ".join(path + [j.child]))
                    continue
                stack.append((j.child, path + [j.child]))
    # A cycle with no link outside it is unreachable from the root, so the walk
    # above cannot see it. Any link that is neither reached nor a root is either
    # in, or hangs off, such a component.
    orphan_cycle = sorted(set(m.links) - seen - set(roots))
    if orphan_cycle and not cyc:
        cyc.append("disconnected component containing " + ", ".join(orphan_cycle[:6]))
    log.check("no cycles in the kinematic tree", not cyc, "; ".join(cyc)[:70] or "acyclic")
    for c in cyc:
        log.fail("STRUCT", f"cycle: {c}")

    unreach = sorted(set(m.links) - seen)
    log.check("all links reachable from root", not unreach,
              ", ".join(unreach)[:70] if unreach else f"{len(seen)}/{len(m.links)} reached")
    for u in unreach:
        log.fail("STRUCT", f"link '{u}' is not connected to the root '{root}'")

    return root, parent_of, children, bool(cyc)


def check_joints(m: Model, log: Log):
    bad_type, no_limit, bad_limit, bad_axis, bad_mimic, zero_effort = [], [], [], [], [], []
    for j in m.all_joints:
        if j.type not in VALID_TYPES:
            bad_type.append(f"{j.name}: type='{j.type}'")
        if j.type in NEEDS_LIMIT:
            if j.limit is None:
                no_limit.append(j.name)
            else:
                lo = j.limit.get("lower")
                hi = j.limit.get("upper")
                ef = j.limit.get("effort")
                ve = j.limit.get("velocity")
                if lo is None or hi is None:
                    bad_limit.append(f"{j.name}: <limit> missing lower/upper")
                elif float(lo) >= float(hi):
                    bad_limit.append(f"{j.name}: lower={lo} >= upper={hi}")
                if ef is None or ve is None:
                    bad_limit.append(f"{j.name}: <limit> missing effort/velocity "
                                     f"(required by the URDF spec for {j.type})")
                else:
                    if float(ef) <= 0:
                        zero_effort.append(f"{j.name}: effort={ef}")
                    if float(ve) <= 0:
                        zero_effort.append(f"{j.name}: velocity={ve}")
        if j.type in NEEDS_AXIS:
            if j.axis is None:
                # URDF default axis is (1,0,0); legal but almost always a mistake here
                log.warn("JOINT", f"{j.name} ({j.type}) has no <axis>; URDF defaults to "
                                  "(1 0 0). The parameter sheet gives NO axis directions, "
                                  "so make this explicit.")
            else:
                nrm = norm(j.axis)
                if nrm < 1e-12:
                    bad_axis.append(f"{j.name}: axis is the zero vector")
                elif abs(nrm - 1.0) > AXIS_TOL:
                    bad_axis.append(f"{j.name}: |axis|={nrm:.9f} (axis={j.axis})")
        if j.mimic is not None:
            tgt = j.mimic.get("joint")
            if tgt not in m.joints:
                bad_mimic.append(f"{j.name}: <mimic joint='{tgt}'> does not exist")

    log.check("joint types legal", not bad_type,
              "; ".join(bad_type)[:70] or f"{len(m.all_joints)} joints")
    for x in bad_type:
        log.fail("JOINT", x)

    n_lim = sum(1 for j in m.all_joints if j.type in NEEDS_LIMIT)
    log.check("revolute/prismatic joints have <limit>", not no_limit,
              ", ".join(no_limit) if no_limit else f"{n_lim}/{n_lim} limited")
    for x in no_limit:
        log.fail("JOINT", f"{x}: joint has no <limit>. Joint limits "
                          "are NOT in the parameter sheet -- they must be a clearly "
                          "labelled ESTIMATE, not silently omitted.")

    log.check("limit lower < upper, effort/velocity present", not bad_limit,
              "; ".join(bad_limit) or "ok")
    for x in bad_limit:
        log.fail("JOINT", x)

    log.check("joint axes are unit vectors", not bad_axis,
              "; ".join(bad_axis) or f"tol {AXIS_TOL:g}")
    for x in bad_axis:
        log.fail("JOINT", x)

    log.check("<mimic> targets exist", not bad_mimic, "; ".join(bad_mimic) or "ok")
    for x in bad_mimic:
        log.fail("JOINT", x)

    for x in zero_effort:
        log.warn("JOINT", f"{x} is non-positive -- Isaac will import a drive that cannot move")


def check_inertial(m: Model, log: Log, root: str, parent_of: dict, children: dict):
    """
    Mass is checked per FIXED-JOINT RIGID CLUSTER, because that is the body PhysX
    actually simulates: a massless frame welded to a massive parent is fine, a
    whole cluster with no mass is not.
    """
    # union-find over fixed joints
    par = {n: n for n in m.links}

    def find(a):
        while par[a] != a:
            par[a] = par[par[a]]
            a = par[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            par[rb] = ra

    for j in m.all_joints:
        if j.type == "fixed" and j.parent in par and j.child in par:
            union(j.parent, j.child)

    clusters: dict[str, list[str]] = {}
    for n in m.links:
        clusters.setdefault(find(n), []).append(n)

    bad_clusters, massless_links, no_inertial = [], [], []
    for rep, members in clusters.items():
        total = sum(m.links[n].mass for n in members)
        if total <= 0.0:
            bad_clusters.append((sorted(members), total))
        for n in members:
            if not m.links[n].has_inertial:
                no_inertial.append(n)
            elif m.links[n].mass <= 0.0:
                massless_links.append(n)

    log.check("no zero-mass rigid body in the articulated chain", not bad_clusters,
              f"{len(clusters)} rigid clusters" if not bad_clusters
              else f"{len(bad_clusters)} massless clusters")
    for members, total in bad_clusters:
        log.fail("INERTIA",
                 f"rigid cluster {members} has total mass {total} kg. PhysX cannot "
                 f"simulate a dynamic body with zero mass.")

    for n in sorted(set(massless_links)):
        log.warn("INERTIA", f"link '{n}' has mass=0 but is welded to a cluster that has "
                            f"mass -- fine WITH --merge-fixed-joints, but the importer "
                            f"will invent a default mass without it.")
    for n in sorted(set(no_inertial)):
        log.warn("INERTIA", f"link '{n}' has no <inertial> block at all")

    # inertia tensor sanity
    bad_diag, tri = [], []
    for lk in m.links.values():
        if not lk.inertia:
            continue
        ixx, iyy, izz = lk.inertia["ixx"], lk.inertia["iyy"], lk.inertia["izz"]
        if lk.mass > 0 and min(ixx, iyy, izz) <= 0:
            bad_diag.append(f"{lk.name}: ixx={ixx} iyy={iyy} izz={izz}")
        elif lk.mass > 0:
            for a, b, c, nm in ((ixx, iyy, izz, "ixx+iyy>=izz"),
                                (iyy, izz, ixx, "iyy+izz>=ixx"),
                                (izz, ixx, iyy, "izz+ixx>=iyy")):
                if a + b < c * (1 - 1e-9):
                    tri.append(f"{lk.name}: {nm} violated ({a:g}+{b:g} < {c:g})")
    log.check("inertia diagonals positive for massive links", not bad_diag,
              "; ".join(bad_diag) or "ok")
    for x in bad_diag:
        log.fail("INERTIA", x)
    log.check3("inertia triangle inequality", "WARN" if tri else "PASS",
               "; ".join(tri) or "ok")
    for x in tri:
        log.warn("INERTIA", f"{x} -- physically impossible inertia tensor; PhysX may reject it")

    # ESTIMATE bookkeeping
    total_mass = sum(lk.mass for lk in m.links.values())
    log.info("INERTIA", f"total model mass = {total_mass:.1f} kg "
                        f"(ECR88 operating weight is ~8.5 t -- if this is far off, the "
                        f"ESTIMATE placeholders need revisiting)")


def check_geometry(m: Model, log: Log, urdf_dir: Path):
    missing, checked = [], 0
    for lk in m.links.values():
        for fn in lk.meshes:
            checked += 1
            p = _resolve_mesh(fn, urdf_dir)
            if p is None or not p.is_file():
                missing.append(f"{lk.name}: {fn}")
    if checked == 0:
        log.check3("mesh files exist", "SKIP",
                   "no <mesh> refs -- primitive geometry build")
        log.info("GEOM", "no meshes referenced: this is the primitive-geometry build "
                         "derived from the LenBottom*/LenUpp* boundary numbers, as "
                         "expected until the ECR88 CAD lands.")
    else:
        log.check("mesh files exist", not missing,
                  f"{checked - len(missing)}/{checked} found")
        for x in missing:
            log.fail("GEOM", f"missing mesh -> {x}")

    no_coll = sorted(n for n, lk in m.links.items()
                     if not lk.collisions and lk.mass > 0)
    for n in no_coll:
        log.warn("GEOM", f"link '{n}' has mass but no <collision> geometry")


def _resolve_mesh(fn: str, urdf_dir: Path):
    if fn.startswith("package://"):
        rest = fn[len("package://"):]
        pkg, _, tail = rest.partition("/")
        try:
            return Path(_find_pkg(pkg)) / tail
        except Exception:
            for base in (REPO_ROOT, urdf_dir, urdf_dir.parent, urdf_dir.parent.parent):
                cand = base / tail
                if cand.is_file():
                    return cand
            return None
    if fn.startswith("file://"):
        return Path(fn[len("file://"):])
    p = Path(fn)
    return p if p.is_absolute() else (urdf_dir / p)


# =========================================================================== #
# 5.  forward kinematics at the zero pose
# =========================================================================== #
def forward_kinematics(m: Model, root: str, children: dict):
    """
    Zero pose: every revolute/prismatic joint is at q=0, so each joint contributes
    only its <origin>.  Returns {link: (R, p)} plus an ordered walk for printing.
    """
    poses = {root: (identity3(), vec(0, 0, 0))}
    walk: list[tuple[int, str, Joint | None]] = [(0, root, None)]
    visited = {root}

    def rec(link: str, depth: int):
        for j in sorted(children.get(link, []), key=lambda x: x.name):
            if j.child in visited:      # belt-and-braces: never recurse into a cycle
                continue
            visited.add(j.child)
            R, p = poses[link]
            Rj = rpy_to_mat(*j.rpy)
            pj = vec(*j.xyz)
            Rc = mat_mul(R, Rj)
            pc = vec_add(p, mat_vec(R, pj))
            poses[j.child] = (Rc, pc)
            walk.append((depth + 1, j.child, j))
            rec(j.child, depth + 1)

    rec(root, 0)
    return poses, walk


def print_fk(m: Model, root: str, children: dict, poses, walk, out=print):
    # Column widths are computed from the data so that no link or joint name is
    # ever silently truncated -- a truncated name is exactly the kind of thing a
    # human eyeballing the chain would misread.
    names = [("  " * d) + l for d, l, _ in walk]
    jds = [("(root)" if j is None else f"{j.name} ({j.type})") for _, _, j in walk]
    wn = max([len(s) for s in names] + [len("LINK")]) + 2
    wj = max([len(s) for s in jds] + [len("JOINT (type)")]) + 2
    width = wn + wj + 30 + 9 + 10

    out("")
    out("  FORWARD KINEMATICS @ ZERO POSE  (all q = 0; each joint contributes only "
        "its <origin>)")
    out("  " + "-" * width)
    out(f"  {'LINK':<{wn}}{'JOINT (type)':<{wj}}{'cumulative XYZ [m]':<30}"
        f"{'|seg|':>9}{'mass kg':>10}")
    out("  " + "-" * width)
    for (depth, link, j), name, jd in zip(walk, names, jds):
        seg = "" if j is None else f"{norm(j.xyz):9.4f}"
        p = poses[link][1]
        xyz = f"({float(p[0]):8.4f},{float(p[1]):9.4f},{float(p[2]):9.4f})"
        mass = m.links[link].mass if link in m.links else 0.0
        out(f"  {name:<{wn}}{jd:<{wj}}{xyz:<30}{seg:>9}{mass:10.1f}")
    out("  " + "-" * width)

    # leaf + named-frame reach summary -- the numbers a human actually eyeballs
    leaves = [lk for lk in m.links if not children.get(lk)]
    interesting = [lk for lk in m.links if any(
        k in lk.lower() for k in ("boom", "arm", "tilt", "rot", "att", "probe",
                                  "contact", "bucket", "bkt", "ant", "gnss"))]
    rows = [lk for lk in set(interesting + leaves) if lk in poses]
    out("")
    out("  REACH @ ZERO POSE   (straight-line distance from root '%s')" % root)
    out("  " + "-" * width)
    wl = max([len(s) for s in rows] + [4]) + 2
    for lk in sorted(rows, key=lambda n: norm(poses[n][1])):
        p = poses[lk][1]
        out(f"    {lk:<{wl}} |p| = {norm(p):8.4f} m    "
            f"x={float(p[0]):8.4f}  y={float(p[1]):8.4f}  z={float(p[2]):8.4f}")
    out("  " + "-" * width)
    out("    ^ compare against the parameter sheet: lenBm1 = 3.55 m, lenArm = 1.7 m,")
    out("      distAttToContactSurface = (0.45, 0, -0.836) m from the attachment frame.")


def check_reach(m: Model, log: Log, poses, root: str):
    """Eyeball checks that the printed chain is the machine the sheet describes."""
    def find_link(*keys):
        for n in sorted(poses):
            low = n.lower()
            if all(k in low for k in keys):
                return n
        return None

    cs = find_link("contact", "surface") or find_link("contact")
    if cs:
        p = poses[cs][1]
        log.info("REACH", f"TCP frame '{cs}' at zero pose: "
                          f"({float(p[0]):.4f}, {float(p[1]):.4f}, {float(p[2]):.4f}) "
                          f"|p|={norm(p):.4f} m from '{root}'")
    else:
        log.warn("REACH", "no link whose name contains 'contact' -- the parameter sheet "
                          "makes ContactSurface the natural TCP frame to publish")

    # Boom+arm fully extended is the sanity number a human knows by heart:
    # 3.55 + 1.7 = 5.25 m from the boom foot, before the tiltrotator/tool.
    att = find_link("att") or find_link("tilt")
    bm = find_link("boom_mount") or find_link("bmmnt") or find_link("boom")
    if att and bm:
        pa, pb = poses[att][1], poses[bm][1]
        d = norm([float(pa[i]) - float(pb[i]) for i in range(3)])
        log.info("REACH", f"'{bm}' -> '{att}' at zero pose = {d:.4f} m "
                          f"(lenBm1 + lenArm = {3.55 + 1.7:.2f} m plus the tiltrotator "
                          f"offsets; a wildly different number means the chain is wrong)")


def check_ground_truth(m: Model, log: Log):
    """
    Search every joint <origin xyz> for the offsets in the parameter sheet.
    Sign is allowed to flip, because parent->child direction in the URDF may be the
    reverse of the sheet's naming (e.g. distAntMainToChs vs a chassis->antenna joint).
    """
    origins = [(j.name, j.xyz) for j in m.all_joints]
    found, missing = [], []
    for pname, want in GT_OFFSETS.items():
        if pname in GT_ZERO_ROWS:
            continue
        hit = None
        for jn, xyz in origins:
            if all(abs(xyz[i] - want[i]) <= MATCH_TOL for i in range(3)):
                hit = (jn, "+")
                break
            if all(abs(xyz[i] + want[i]) <= MATCH_TOL for i in range(3)):
                hit = (jn, "-")
                break
        if hit:
            found.append(f"{pname:<26} -> {hit[0]} ({hit[1]})")
        else:
            missing.append(pname)

    len_found, len_missing = [], []
    for pname, want in GT_LENGTHS.items():
        hit = None
        for jn, xyz in origins:
            if abs(norm(xyz) - want) <= MATCH_TOL:
                hit = jn
                break
        if hit:
            len_found.append(f"{pname:<26} -> {hit}  (|origin| = {want})")
        else:
            len_missing.append(f"{pname}={want}")

    n_ok = len(found) + len(len_found)
    n_tot = n_ok + len(missing) + len(len_missing)
    log.check3("parameter-sheet offsets present in joint origins",
               "PASS" if not (missing or len_missing) else "WARN",
               f"{n_ok}/{n_tot} matched (tol {MATCH_TOL*1000:g} mm)")
    for f in found + len_found:
        log.info("GTRUTH", f)
    for mss in missing:
        log.warn("GTRUTH", f"{mss} = {GT_OFFSETS[mss]} not found among joint origins "
                           f"(may be legitimately folded into another frame -- check)")
    for mss in len_missing:
        log.warn("GTRUTH", f"{mss} not found as any |joint origin| "
                           f"(may be legitimately folded into another frame -- check)")

    # additive rule stated verbatim in the source deck
    nm, want, parts = GT_SUM_CHECK
    s = [sum(GT_OFFSETS[p][i] for p in parts) for i in range(3)]
    ok = all(abs(s[i] - want[i]) <= MATCH_TOL for i in range(3))
    log.check(f"{nm} == {' + '.join(parts)}", ok,
              f"{tuple(round(v,4) for v in s)} vs {want}")
    if not ok:
        log.fail("GTRUTH", f"{nm} additive rule broken: {s} != {want}")


# =========================================================================== #
# 6.  main
# =========================================================================== #
def render_summary(log: Log, out=print) -> None:
    out("")
    out("=" * 108)
    out("  SUMMARY")
    out("=" * 108)
    w = max([len(c[0]) for c in log.checks] + [10])
    for name, status, detail in log.checks:
        mark = {"PASS": "[ ok ]", "FAIL": "[FAIL]", "WARN": "[warn]", "SKIP": "[skip]"}[status]
        out(f"  {mark}  {name:<{w}}  {detail}")
    out("-" * 108)
    out(f"  checks: {sum(1 for c in log.checks if c[1]=='PASS')} pass, "
        f"{sum(1 for c in log.checks if c[1]=='FAIL')} fail, "
        f"{sum(1 for c in log.checks if c[1]=='WARN')} warn, "
        f"{sum(1 for c in log.checks if c[1]=='SKIP')} skip"
        f"   |   {len(log.fails)} hard failures, {len(log.warns)} warnings")
    out("=" * 108)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="validate_urdf.py",
        description="Dependency-light URDF/xacro validator (no Isaac Sim, no ROS).")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--xacro", type=Path, help="xacro file to expand and validate")
    src.add_argument("--urdf", type=Path, help="already-flat URDF to validate")
    ap.add_argument("-D", "--mapping", dest="mappings", action="append", default=[],
                    metavar="NAME:=VALUE", help="xacro argument, repeatable")
    ap.add_argument("--xacro-bin", default=None, help="path to a xacro executable (or $XACRO)")
    ap.add_argument("--no-builtin-xacro", action="store_true",
                    help="refuse the built-in minimal expander; require real xacro")
    ap.add_argument("--print-urdf", action="store_true",
                    help="dump the expanded URDF to stdout before validating")
    ap.add_argument("--save-urdf", type=Path, default=None,
                    help="write the expanded URDF here")
    ap.add_argument("--warn-as-error", action="store_true",
                    help="exit non-zero on warnings too (use in CI once the TODOs close)")
    ap.add_argument("--variant", default=None, choices=sorted(VARIANT_OVERRIDES),
                    help="parameter-sheet column to check against. Default: whatever "
                         "machine_variant ecr88_params.xacro is set to.")
    args = ap.parse_args(argv)

    # ---- ground-truth variant --------------------------------------------- #
    # Must happen before any GTRUTH check reads GT_OFFSETS / GT_LENGTHS.
    src_path = args.urdf if args.urdf is not None else args.xacro
    variant = args.variant or read_variant_from_params(src_path.resolve().parent)
    variant_src = "--variant" if args.variant else "ecr88_params.xacro"
    if variant is None:
        variant, variant_src = "ECR88_KIJANG", "fallback (machine_variant not readable)"
    if not apply_variant(variant):
        print(f"[SKIP] unknown machine_variant {variant!r}; ground truth left at 기장",
              file=sys.stderr)
        variant, variant_src = "ECR88_KIJANG", f"fallback (unknown {variant})"
    n_fw = apply_firmware(variant)

    log = Log()
    print("=" * 108)
    print("  ECR88 / X1 PanelLift  URDF validator")
    print(f"  numpy: {'yes (' + _np.__version__ + ')' if HAVE_NUMPY else 'NO -- using pure-python matrix math (same results)'}")
    print(f"  ground truth: KinematicPara_new / {variant}   (from {variant_src})")
    print("=" * 108)

    # ---- source ----------------------------------------------------------- #
    if args.urdf is not None:
        path = args.urdf.resolve()
        if not path.is_file():
            print(f"[SKIP] --urdf not found: {path}", file=sys.stderr)
            return 2
        try:
            root_el = ET.parse(path).getroot()
        except ET.ParseError as exc:
            print(f"[FAIL] XML parse error in {path}: {exc}", file=sys.stderr)
            return 2
        how = "flat URDF (no expansion)"
    else:
        path = args.xacro.resolve()
        if not path.is_file():
            print(f"[SKIP] --xacro not found: {path}\n"
                  f"       The xacro has not been written yet; nothing to validate.",
                  file=sys.stderr)
            return 2
        try:
            root_el, how = expand_xacro(path, args.mappings, args.xacro_bin,
                                        allow_builtin=not args.no_builtin_xacro, log=log)
        except XacroRejected as exc:
            # The asset itself is broken -> exit 1, so CI fails rather than
            # silently treating an unbuildable model as "not checked".
            print(f"[FAIL] {exc}", file=sys.stderr)
            return 1
        except XacroFailed as exc:
            print(f"[SKIP] {exc}", file=sys.stderr)
            return 2
        except XacroUnsupported as exc:
            print(f"[SKIP] {exc}", file=sys.stderr)
            return 2

    print(f"  source : {path}")
    print(f"  expand : {how}")
    if "built-in" not in how:
        log.check3("xacro expansion", "PASS", how)

    if _local(root_el.tag) != "robot":
        print(f"[FAIL] root element is <{_local(root_el.tag)}>, expected <robot>",
              file=sys.stderr)
        return 2

    xml_text = ET.tostring(root_el, encoding="unicode")
    if args.save_urdf:
        args.save_urdf.parent.mkdir(parents=True, exist_ok=True)
        args.save_urdf.write_text('<?xml version="1.0"?>\n' + xml_text)
        print(f"  saved  : {args.save_urdf}")
    if args.print_urdf:
        print(xml_text)

    try:
        m = Model(root_el)
    except ValueError as exc:
        print(f"[FAIL] malformed numeric attribute: {exc}", file=sys.stderr)
        return 2

    print(f"  robot  : {m.name}   ({len(m.links)} links, {len(m.all_joints)} joints)")
    jtypes: dict[str, int] = {}
    for j in m.all_joints:
        jtypes[j.type] = jtypes.get(j.type, 0) + 1
    print(f"  joints : " + ", ".join(f"{v}x {k}" for k, v in sorted(jtypes.items())))

    # ---- checks ----------------------------------------------------------- #
    root, parent_of, children, has_cycle = check_structure(m, log)
    check_joints(m, log)
    if root:
        check_inertial(m, log, root, parent_of, children)
    check_geometry(m, log, path.parent)
    check_ground_truth(m, log)

    if root and not has_cycle:
        try:
            poses, walk = forward_kinematics(m, root, children)
            print_fk(m, root, children, poses, walk)
            check_reach(m, log, poses, root)
            log.check("forward kinematics @ zero pose computed", True,
                      f"{len(poses)}/{len(m.links)} frames placed")
        except Exception as exc:
            log.check("forward kinematics @ zero pose computed", False,
                      f"{type(exc).__name__}: {exc}")
            log.fail("FK", f"{type(exc).__name__}: {exc}")

    # ---- report ----------------------------------------------------------- #
    if log.infos:
        print("")
        print("  NOTES")
        print("  " + "-" * 104)
        for cat, msg in log.infos:
            print(f"    [{cat}] {msg}")
    if log.warns:
        print("")
        print("  WARNINGS  (not fatal, but every one of these is a real gap)")
        print("  " + "-" * 104)
        for cat, msg in log.warns:
            print(f"    [{cat}] {msg}")
    if log.fails:
        print("")
        print("  HARD FAILURES")
        print("  " + "-" * 104)
        for cat, msg in log.fails:
            print(f"    [{cat}] {msg}")

    render_summary(log)

    if log.fails:
        return 1
    if args.warn_as_error and log.warns:
        print("  (--warn-as-error: failing because warnings are present)")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
