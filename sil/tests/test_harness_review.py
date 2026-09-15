"""
Adversarial review of the SIL core itself (sil/build.py, sil/firmware.py, sil/harness.py) and
of the shared plant libraries every scenario and the Isaac plant use (sil/kinematics.py,
sil/geodesy.py).

The question every test here asks is "can the harness make a SIL result WRONG while it
still LOOKS right?".

  * Where the code is correct the test asserts the correct behaviour and passes. Ten of these
    started life as expected failures in the first review round (boolean normalisation,
    integer range checks, step-before-reset, request_step re-edge and latch, run_until
    pre-check, run_seconds rounding, trace restart and precision). The core was fixed; they
    are regression tests now, each with a "fixed: was ..." line.
  * Where the code is still wrong the test asserts the correct behaviour and is marked
    @unittest.expectedFailure with a LIBRARY BUG comment, so the suite passes today and
    reports an "unexpected success" the day it is fixed (then drop the decorator).

Tests that decide through a _probe_*() function: the probe RETURNS a bool and raises
RuntimeError when its own precondition does not hold. TestProbeSanity calls every probe, so an
expected failure can never be a typo'd signal name or a broken setup in disguise.

Run:  cd xpanner-sim && python3 -m unittest sil.tests.test_harness_review -v
"""
import ctypes
import functools
import glob
import hashlib
import json
import math
import re
import struct
import subprocess
import sys
import unittest
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

import numpy as np

from sil import geodesy as geo
from sil import kinematics as kin
from sil.harness import DT, RE_EDGE_STEP, Harness, SaveHandshake, StepTimeout, firmware

REPO = Path(__file__).resolve().parents[2]
BUILD = REPO / "build" / "sil"

# Every writable data object the firmware owns, per the definitions at MdlApp.c:373
# (parLocalTest), :1597-1606 (B, DW, U, Y) and rt_nonfinite.c:18-23. ConstB/ConstP are const.
FIRMWARE_STATE = {"MdlApp_B", "MdlApp_DW", "MdlApp_U", "MdlApp_Y", "parLocalTest",
                  "rtInf", "rtInfF", "rtMinusInf", "rtMinusInfF", "rtNaN", "rtNaNF"}

X86_64_LINUX = sys.platform == "linux" and struct.calcsize("P") == 8


# ----------------------------------------------------------------------------------------
# helpers
# ----------------------------------------------------------------------------------------
def _x1exc():
    return Path(firmware().manifest["x1exc_dir"])


def _gen():
    return _x1exc() / "Asw" / "GeneratedCode"


def _read_latin1(p):
    return Path(p).read_text(encoding="latin-1")


@functools.lru_cache(maxsize=None)
def _mdlapp_c():
    return _read_latin1(_gen() / "MdlApp_ert_rtw/MdlApp.c")


def _nm_data_symbols(path, dynamic):
    """{name: size} of sized writable data symbols (B/b/D/d/C) in an object or .so."""
    args = ["nm", "-S", "--defined-only"] + (["-D"] if dynamic else []) + [str(path)]
    try:
        out = subprocess.run(args, capture_output=True, text=True, check=True).stdout
    except (OSError, subprocess.CalledProcessError) as e:
        raise unittest.SkipTest(f"nm unavailable: {e}")
    syms = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[2] in "BbDdCc":
            syms[parts[3]] = int(parts[1], 16)
    return syms


def _state_sizes(fw):
    syms = _nm_data_symbols(fw.lib._name, dynamic=True)
    return {n: syms[n] for n in FIRMWARE_STATE}


def _snapshot(fw, sizes):
    return {n: ctypes.string_at(ctypes.addressof(ctypes.c_char.in_dll(fw.lib, n)), s)
            for n, s in sizes.items()}


def _combined_digest(fw, sizes):
    snap = _snapshot(fw, sizes)
    return hashlib.sha256(b"".join(snap[n] for n in sorted(snap))).hexdigest()


def _reset_digests(fw):
    """Used in a child process too: reset, then sha256 of every firmware state object."""
    fw.reset()
    return {n: hashlib.sha256(b).hexdigest() for n, b in _snapshot(fw, _state_sizes(fw)).items()}


def _run_child(code):
    r = subprocess.run([sys.executable, "-c", code], cwd=str(REPO), capture_output=True, text=True,
                       timeout=180)
    if r.returncode:
        raise RuntimeError(f"child failed: {r.stderr[-2000:]}")
    return json.loads(r.stdout.strip().splitlines()[-1])


def _plant(h):
    """Deterministic, non-trivial sensor motion so filters/integrators in DW evolve."""
    k = h.tick_count
    a = 0.3 * math.sin(k * 0.02)
    b = 0.2 * math.sin(k * 0.031)
    h.fw["u.bm1ImuQuat"] = [math.cos(a / 2), 0.0, math.sin(a / 2), 0.0]
    h.fw["u.armImuQuat"] = [math.cos(b / 2), 0.0, math.sin(b / 2), 0.0]
    h.fw["u.chsImuAngRate"] = [0.0, 0.0, 0.01 * math.cos(k * 0.05)]
    h.fw["u.rcvPiPrs.bm1Up"] = 5.0 * max(0.0, math.sin(k * 0.07))


class _RecordingHarness(Harness):
    """Harness that records one digest of all firmware state objects after every step,
    including the hidden ticks inside pulse() / request_step() / run_until()."""

    def __init__(self, sizes, **kw):
        super().__init__(**kw)
        self.sizes = sizes
        self.digests = []
        self.visited = set()

    def tick(self, n=1):
        for _ in range(n):
            super().tick(1)
            self.digests.append(_combined_digest(self.fw, self.sizes))
            self.visited.add((self.curr_step(), bool(self.fw["y.autoCtrl_StartStopSts"])))
        return self


def _eventful_scenario(h):
    """target -> Standby -> Positioning -> run -> pause -> resume -> stop, with sensor motion."""
    h.nominal_inputs()
    h.pulse("u.isSwingAligned")
    h.gnss_rtk_fixed()
    h.set_target_panel()
    h.tick(5)
    h.request_step("Standby")
    h.run_until(lambda h: h.curr_step() == "Standby", 0.5, "Standby")
    h.request_step("Positioning")
    h.tick(5)
    h.pulse("u.jstAutoReq_StartPause")
    h.tick(100)
    h.pulse("u.jstAutoReq_StartPause")          # pause
    h.tick(30)
    h.pulse("u.jstAutoReq_StartPause")          # resume
    h.tick(5)
    h.pulse("u.tabletAutoReq_Stop")
    h.tick(20)
    return h


def _burn_prior_scenario():
    """A DIFFERENT firmware history (tilted pose, calibration run) to leave other bytes on the
    C stack and in the heap before a trajectory is recorded."""
    h = Harness().reset().nominal_inputs()
    h.set_pose(R_chs=kin.Rz(0.4) @ kin.Rx(0.07), q_bm1=math.radians(-25), q_arm=math.radians(120),
               q_inp=math.radians(-20), q_tilt=0.3)
    h.set_swing_aligned(True)
    h.gnss_rtk_fixed()
    h.tick(3)
    h.request_step("CalibForkRefPose")
    h.tick()
    h.pulse("u.jstAutoReq_StartPause")
    h.tick(300)


def _trajectory(prior=False):
    """Used in a child process too."""
    if prior:
        _burn_prior_scenario()
    h = _RecordingHarness(_state_sizes(firmware()), plant=_plant).reset()
    _eventful_scenario(h)
    return {"digests": h.digests, "tick_count": h.tick_count,
            "visited": sorted([s, r] for s, r in h.visited)}


def _booted(target=True):
    """Healthy machine idle in NoTarget: swing switch held closed, RTK fixed, optional target."""
    h = Harness().reset().nominal_inputs()
    h.set_swing_aligned(True)
    h.gnss_rtk_fixed()
    if target:
        h.set_target_panel(panel_id=7)
    h.tick(5)
    return h


def _standby():
    h = _booted()
    h.request_step("Standby")
    h.run_until(lambda h: h.curr_step() == "Standby", 0.5, "Standby")
    return h


def _positioning_paused():
    h = _standby()
    h.request_step("Positioning")
    h.run_until(lambda h: h.main_state() == "PositioningPaused", 0.5, "PositioningPaused")
    if h.is_running():
        raise RuntimeError(f"precondition: {h.describe()}")
    return h


def _parse_header_structs_and_enums():
    """Independent of build.py: brace-matching parse of every generated header."""
    hdrs = sorted(glob.glob(str(_gen() / "slprj/ert/_sharedutils/*.h"))) + [str(_gen() / "MdlApp_ert_rtw/MdlApp.h")]
    text = re.sub(r"/\*.*?\*/", " ", "\n".join(_read_latin1(h) for h in hdrs), flags=re.S)
    text = re.sub(r"//[^\n]*", " ", text)
    structs, enums = {}, {}
    for m in re.finditer(r"typedef\s+(struct|enum)\s*\w*\s*\{", text):
        i, depth = m.end(), 1
        while depth:
            depth += (text[i] == "{") - (text[i] == "}")
            i += 1
        body = text[m.end():i - 1]
        name = re.match(r"\s*(\w+)\s*;", text[i:]).group(1)
        if m.group(1) == "struct":
            structs[name] = None if "{" in body else [" ".join(s.split()) for s in body.split(";") if s.strip()]
        else:
            vals, nxt = {}, 0
            for item in (x.strip() for x in body.split(",")):
                if not item:
                    continue
                if "=" in item:
                    k, v = (x.strip() for x in item.split("=", 1))
                    nxt = int(re.sub(r"\(\w+\)", "", v).rstrip("uUlL"), 0)
                else:
                    k = item
                vals[k] = nxt
                nxt += 1
            enums[name] = vals
    return structs, enums


def _expected_leaves(structs, tname, prefix):
    out = []
    members = structs[tname]
    if members is None:
        raise AssertionError(f"{tname} has a nested body; reachable from {prefix}")
    for s in members:
        m = re.match(r"^(?:const )?(\w+) (\w+)((?:\[\d+\])*)$", s)
        if not m:
            raise AssertionError(f"unparsed member {s!r} in {tname}")
        t, n, dims = m.group(1), m.group(2), [int(x) for x in re.findall(r"\d+", m.group(3))]
        p = f"{prefix}.{n}"
        if t in structs:
            if dims:
                for k in range(dims[0]):
                    out += _expected_leaves(structs, t, f"{p}[{k}]")
            else:
                out += _expected_leaves(structs, t, p)
        else:
            out.append(p)
    return out


_PRIM_SIZE = {  # rtwtypes.h:42-63
    "int8_T": 1, "uint8_T": 1, "boolean_T": 1, "char_T": 1, "int16_T": 2, "uint16_T": 2,
    "int32_T": 4, "uint32_T": 4, "int_T": 4, "uint_T": 4, "real32_T": 4,
    "real64_T": 8, "real_T": 8, "time_T": 8,
}


def _abi_layout(structs, enums, tname, prefix):
    """(size, align, {leaf path: (offset, nbytes)}) for a struct, x86-64 SysV layout."""
    off, align, leaves = 0, 1, {}
    for s in structs[tname]:
        m = re.match(r"^(?:const )?(\w+) (\w+)((?:\[\d+\])*)$", s)
        t, n = m.group(1), m.group(2)
        count = 1
        for d in re.findall(r"\d+", m.group(3)):
            count *= int(d)
        p = f"{prefix}.{n}"
        if t in structs:
            esize, ealign, sub = _abi_layout(structs, enums, t, "")
        else:
            esize = _PRIM_SIZE[t] if t in _PRIM_SIZE else (4 if t in enums else None)
            if esize is None:
                raise AssertionError(f"unknown type {t} at {p}")
            ealign, sub = esize, None
        off = -(-off // ealign) * ealign
        align = max(align, ealign)
        if sub is None:
            leaves[p] = (off, esize * count)
        elif m.group(3):
            for k in range(count):
                for sp, (so, sn) in sub.items():
                    leaves[f"{p}[{k}]{sp}"] = (off + k * esize + so, sn)
        else:
            for sp, (so, sn) in sub.items():
                leaves[f"{p}{sp}"] = (off + so, sn)
        off += esize * count
    return -(-off // align) * align, align, leaves


def _stateflow_chart(file_number):
    """(chart path, [uncommented state names], [commented state names]) of the Stateflow chart
    whose chartFileNumber is `file_number` (-> is_c<N>_MdlApp in generated code). Read from the
    .slx zip in memory; X1Exc is never written."""
    with zipfile.ZipFile(_x1exc() / "ControlModel" / "Models" / "MdlApp.slx") as z:
        for n in z.namelist():
            if not re.match(r"simulink/stateflow/chart_\d+\.xml$", n):
                continue
            root = ET.fromstring(z.read(n))
            props = {p.get("Name"): p.text for p in root.findall("P")}
            if props.get("chartFileNumber") != str(file_number):
                continue
            live, commented = [], []
            for s in root.iter("state"):
                name = (s.findtext("P[@Name='labelString']") or "").split("\n")[0].strip()
                (commented if s.find("comment") is not None else live).append(name)
            return props["name"], live, commented
    raise AssertionError(f"no Stateflow chart with chartFileNumber {file_number}")


def _glue_persisted_y_paths():
    """y.* leaves the ECU glue copies into INTP / kin_s / actuators0x_s while isCalibrating
    (AppCtrlIf.c:801-1018): exactly what SaveInternalParam() + WriteToNVM() persist when
    SaveMchCalibData() sees the request edge (main.c:405-406, InternalParam.c:72-86)."""
    text = _read_latin1(_x1exc() / "Asw" / "ModelInterface" / "AppCtrlIf.c")
    i = text.index("if (MdlApp_Y.isCalibrating)")
    body = text[i:text.index("// MdlApp_Y.calibStep", i)]
    return sorted({"y." + m.group(1) for m in re.finditer(r"MdlApp_Y\.([\w.]+)", body)} - {"y.isCalibrating"})


def _wrap(a):
    return math.remainder(a, 2 * math.pi)


def _yaw312(R):
    """R = Rz(psi) Rx(phi) Ry(theta): psi = atan2(-R01, R11)."""
    return math.atan2(-R[0, 1], R[1, 1])


def _eul312(R):
    return [math.asin(R[2, 1]), math.atan2(-R[2, 0], R[2, 2]), _yaw312(R)]


def _quat_to_R(q):
    """Independent of kinematics.py: the matrix the firmware builds, MdlApp.c:10917-10925."""
    w, x, y, z = q
    return np.array([[w * w + x * x - y * y - z * z, 2 * (x * y - w * z), 2 * (w * y + x * z)],
                     [2 * (x * y + w * z), w * w - x * x + y * y - z * z, 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (w * x + y * z), w * w - x * x - y * y + z * z]])


# ----------------------------------------------------------------------------------------
# probes: each returns True iff the code behaves correctly
# ----------------------------------------------------------------------------------------
def _probe_bool_truthy_value_reaches_firmware_as_true():
    h = Harness().reset().nominal_inputs()
    h.tick(2)
    if "BIT_SWING_NOT_INIT" not in h.inhibit_names():
        raise RuntimeError("precondition: swing should not be initialised yet")
    h.fw["u.isSwingAligned"] = 2          # truthy; e.g. a plant writing `status & 0x2`
    h.tick(3)
    return "BIT_SWING_NOT_INIT" not in h.inhibit_names()


def _probe_integer_overflow_is_rejected():
    fw = Harness().reset().fw
    rejected = 0
    for v in (256, -1):
        try:
            fw["u.autoReqStep"] = v
        except (OverflowError, ValueError, TypeError):
            rejected += 1
    return rejected == 2


def _probe_non_integral_float_to_int_is_rejected():
    fw = Harness().reset().fw
    try:
        fw["u.autoReqStep"] = 1.9
    except (ValueError, TypeError):
        return True
    return False


def _probe_request_notarget_does_not_request_standby():
    h = _booted()
    if h.curr_step() != "NoTarget" or h.fw["y.tarPanelIdAck"] != 7 or h.fw["u.autoReqStep"] != 0:
        raise RuntimeError(f"precondition: {h.describe()}")
    h.request_step("NoTarget")
    h.tick(5)
    return h.curr_step() == "NoTarget"


def _probe_request_step_edges_against_latched_value():
    h = Harness().reset().nominal_inputs()
    h.pulse("u.isSwingAligned")
    h.gnss_rtk_fixed()
    h.fw["u.autoReqStep"] = 1                  # latched by the firmware while no target exists
    h.tick(2)
    h.set_target_panel()
    h.tick(3)
    if h.curr_step() != "NoTarget":
        raise RuntimeError(f"precondition: {h.describe()}")
    h.fw["u.autoReqStep"] = 9                  # written, never stepped
    h.request_step("Standby")
    h.tick(5)
    return h.curr_step() == "Standby"


def _probe_run_until_sees_condition_true_on_entry():
    h = Harness().reset().nominal_inputs()
    h.tick(1)
    entry = h.tick_count
    got = h.run_until(lambda h: h.tick_count >= entry, 0.1, "already true")
    return got == entry and h.tick_count == entry


def _probe_run_seconds_is_strictly_monotone_on_half_ticks():
    counts = []
    for s in (0.005, 0.015, 0.025, 0.035, 0.045):
        counts.append(Harness().reset().run_seconds(s).tick_count)
    return all(b > a for a, b in zip(counts, counts[1:]))


def _probe_trace_rows_stay_rectangular():
    h = Harness().reset()
    h.trace("y.autoCtrl_CurrStep")
    h.tick(2)
    h.trace("y.autoCtrl_CurrStep", "y.autoCtrl_InhibitSts")
    h.tick(2)
    return all(len(r) == 1 + len(h.trace_paths) for r in h.trace_rows)


def _probe_trace_array_floats_keep_precision():
    h = Harness().reset()
    h.fw["u.chsImuQuat"] = [1.0000001, 0.1234567, 0.0, 0.0]
    h.trace("u.chsImuQuat")
    h.tick()
    cell = h.trace_rows[-1][1]
    parsed = [float(x) for x in (cell.split() if isinstance(cell, str) else cell)]
    return parsed == h.fw["u.chsImuQuat"]


_CHILD_NO_RESET = r"""
import ctypes, json
from sil.firmware import Firmware
from sil.harness import Harness
MODE = %r
fw = Firmware()                     # fresh process: MdlApp_initialize() never ran
h = Harness(fw=fw)
out = {}
for name, call in (("fw_step", fw.step), ("h_tick", h.tick)):
    try:
        call()
        out[name] = None
    except RuntimeError as e:
        out[name] = str(e)
out["tick_count"] = h.tick_count
h.nominal_inputs()                  # writes are allowed before reset
if MODE == "nan":
    fw.lib.rt_InitInfAndNaN.argtypes = [ctypes.c_size_t]
    fw.lib.rt_InitInfAndNaN(8)      # the first line of MdlApp_initialize, MdlApp.c:52620
for _ in range(3):
    fw.lib.MdlApp_step()            # raw: bypasses the guard to show what it prevents
out["y"] = {p: repr(fw[p]) for p in fw.paths("y.")}
print(json.dumps(out))
"""


def _probe_stepping_without_reset_is_refused():
    child = _run_child(_CHILD_NO_RESET % "raw")
    return bool(child["fw_step"]) and bool(child["h_tick"]) and child["tick_count"] == 0


def _probe_float32_overflow_is_rejected():
    fw = Harness().reset().fw
    fw["u.gnssPosStdDevZ"] = 0.5
    try:
        fw["u.gnssPosStdDevZ"] = 1e40          # finite double, not representable in real32_T
    except (OverflowError, ValueError):
        return fw["u.gnssPosStdDevZ"] == 0.5
    if not math.isinf(fw["u.gnssPosStdDevZ"]):
        raise RuntimeError("precondition: expected the silent float32 overflow to store inf")
    return False


_CHILD_SSE_ROUNDING = r"""
import ctypes, json, math, mmap
from sil.harness import Harness
try:
    buf = mmap.mmap(-1, mmap.PAGESIZE, prot=mmap.PROT_READ | mmap.PROT_WRITE | mmap.PROT_EXEC)
except (OSError, ValueError, AttributeError) as e:
    print(json.dumps({"skip": "no executable mmap: %%s" %% e}))
    raise SystemExit(0)
buf.write(bytes([0x0F, 0xAE, 0x17, 0xC3, 0x0F, 0xAE, 0x1F, 0xC3]))  # ldmxcsr (%%rdi); ret / stxcsr (%%rdi); ret
addr = ctypes.addressof(ctypes.c_char.from_buffer(buf))
fn = ctypes.CFUNCTYPE(None, ctypes.POINTER(ctypes.c_uint32))
ldmxcsr, stxcsr = fn(addr), fn(addr + 4)
def csr():
    v = ctypes.c_uint32(); stxcsr(ctypes.byref(v)); return v.value
def set_csr(v):
    w = ctypes.c_uint32(v); ldmxcsr(ctypes.byref(w))
def posed():
    h = Harness().reset().nominal_inputs()   # inputs computed with the clean MXCSR
    h.set_pose(q_bm1=math.radians(-33.3), q_arm=math.radians(97.1), q_inp=math.radians(-51.7), q_tilt=0.123)
    return h
def links(h):
    return [repr(h.fw[p]) for p in h.fw.paths("y.links.")]
MODE = %d                           # MXCSR rounding-control bits to set
base = csr()
h = posed(); h.tick(50); clean = links(h)
h = posed()
res = {}
set_csr(base | MODE)                  # SSE rounding only; the x87 control word is untouched
try:
    res["csr"] = csr()
    res["x87_round"] = ctypes.CDLL("libm.so.6").fegetround()
    res["violation"] = h.fw.lib.sil_fp_env_violation()
    for _ in range(50):
        h.fw.lib.MdlApp_step()          # raw, to measure what the firmware computes in this mode
finally:
    set_csr(base)                     # restore before any Python float formatting
sse = links(h)
set_csr(base | MODE)
try:
    try:
        h.fw.step()
        res["raised"] = False
    except RuntimeError:
        res["raised"] = True
finally:
    set_csr(base)
h2 = posed(); h2.tick(50)
res["outputs_differ"] = sse != clean
res["clean_repeatable"] = links(h2) == clean
print(json.dumps(res))
"""


def _probe_fp_guard_sees_sse_rounding_mode():
    if not X86_64_LINUX:
        raise unittest.SkipTest("MXCSR is x86-64 specific")
    child = _run_child(_CHILD_SSE_ROUNDING % 0x4000)
    if "skip" in child:
        raise unittest.SkipTest(child["skip"])
    if not (child["csr"] & 0x6000 == 0x4000 and child["x87_round"] == 0
            and child["outputs_differ"] and child["clean_repeatable"]):
        raise RuntimeError(f"precondition: {child}")
    return child["raised"]


def _probe_save_snapshot_covers_every_persisted_output():
    fw = firmware()
    paths = _glue_persisted_y_paths()
    unknown = [p for p in paths if p not in fw]
    if unknown or len(paths) < 100:
        raise RuntimeError(f"precondition: glue parse gave {len(paths)} paths, unknown {unknown[:5]}")
    return all(p.startswith(SaveHandshake.SNAPSHOT_PREFIXES) for p in paths)


ALL_PROBES = [
    _probe_bool_truthy_value_reaches_firmware_as_true,
    _probe_integer_overflow_is_rejected,
    _probe_non_integral_float_to_int_is_rejected,
    _probe_request_notarget_does_not_request_standby,
    _probe_request_step_edges_against_latched_value,
    _probe_run_until_sees_condition_true_on_entry,
    _probe_run_seconds_is_strictly_monotone_on_half_ticks,
    _probe_trace_rows_stay_rectangular,
    _probe_trace_array_floats_keep_precision,
    _probe_stepping_without_reset_is_refused,
    _probe_float32_overflow_is_rejected,
    _probe_fp_guard_sees_sse_rounding_mode,
    _probe_save_snapshot_covers_every_persisted_output,
]


class TestProbeSanity(unittest.TestCase):
    def test_every_probe_runs_cleanly_and_returns_a_bool(self):
        # WHY: an expectedFailure that fails on a KeyError hides a broken test as a known bug.
        # Citation: the probes' own preconditions (each raises RuntimeError if its setup is off).
        for probe in ALL_PROBES:
            with self.subTest(probe=probe.__name__):
                try:
                    self.assertIsInstance(probe(), bool)
                except unittest.SkipTest:
                    pass


# ----------------------------------------------------------------------------------------
# build.py: the signal table and the manifest
# ----------------------------------------------------------------------------------------
class TestSignalTable(unittest.TestCase):
    def test_table_paths_equal_an_independent_header_parse(self):
        # WHY: a field silently missing from the table is a sensor the plant can never drive.
        # Citation: MdlApp.h:985-1079 (ExtU, 91 members), :1082-1175 (ExtY, 90), ParLocal_t.h.
        # build.py parses with a non-greedy regex; this re-parses with brace matching.
        structs, _ = _parse_header_structs_and_enums()
        fw = firmware()
        for prefix, ctype in (("u", "ExtU_MdlApp_T"), ("y", "ExtY_MdlApp_T"), ("par", "ParLocal_t")):
            with self.subTest(root=prefix):
                expected = _expected_leaves(structs, ctype, prefix)
                self.assertEqual(len(expected), len(set(expected)), "duplicate paths")
                self.assertEqual(set(expected), set(fw.paths(prefix + ".")))
                self.assertEqual(len(expected), fw.manifest["leaves"][prefix])
        self.assertEqual(len(structs["ExtU_MdlApp_T"]), 91)
        self.assertEqual(len(structs["ExtY_MdlApp_T"]), 90)

    def test_every_leaf_sits_at_its_abi_offset_computed_from_the_headers(self):
        # WHY: an overlapping or mis-offset leaf feeds the right-looking name the wrong bytes.
        # The table uses compiler offsetof(); this recomputes every offset from the headers with
        # the x86-64 SysV rules (align = size for rtwtypes primitives, enums int-sized), checks
        # every element size (enum leaves included) and each root size against the linked object.
        # Citation: rtwtypes.h:42-63 (primitive widths); MdlApp.h:985-1175; nm sizes of
        # MdlApp_U/MdlApp_Y/parLocalTest (MdlApp.c:1603, :1606, :373).
        structs, enums = _parse_header_structs_and_enums()
        fw = firmware()
        nm_sizes = _state_sizes(fw)
        roots = [("u", "ExtU_MdlApp_T", "MdlApp_U"), ("y", "ExtY_MdlApp_T", "MdlApp_Y"),
                 ("par", "ParLocal_t", "parLocalTest")]
        for b, (prefix, ctype, sym) in enumerate(roots):
            base, size = fw.lib.sil_base(b), fw.lib.sil_base_size(b)
            abi_size, _align, offsets = _abi_layout(structs, enums, ctype, prefix)
            self.assertEqual(size, nm_sizes[sym], sym)
            self.assertEqual(abi_size, size, sym)
            self.assertEqual(base, ctypes.addressof(ctypes.c_char.in_dll(fw.lib, sym)), sym)
            spans = []
            for p in fw.paths(prefix + "."):
                addr, ct, count, _code = fw._sigs[p]
                self.assertEqual(addr - base, offsets[p][0], p)
                self.assertEqual(ctypes.sizeof(ct) * count, offsets[p][1], p)
                spans.append((addr - base, addr - base + offsets[p][1], p))
            spans.sort()
            for (_, e0, p0), (s1, _, p1) in zip(spans, spans[1:]):
                self.assertLessEqual(e0, s1, f"{p0} overlaps {p1}")

    def test_enum_values_match_headers(self):
        # WHY: fw["y.autoCtrl_CurrStep"] is decoded with these numbers; a shifted enum turns
        # "Standby" into "Positioning" in every test report. (Enum leaf WIDTH is covered by the
        # ABI offset test above; the old sizeof(c_int32) loop here was a tautology and is gone.)
        # Citation: AutoCtrlStep.h:15-37 (CalibSwingStopCoeff = 32); sil_table.c static asserts.
        _, enums = _parse_header_structs_and_enums()
        fw = firmware()
        for name, vals in fw.enums.items():
            self.assertEqual(vals, enums[name], name)
        table_c = (BUILD / "sil_table.c").read_text()
        for name in fw.enums:
            self.assertIn(f"_Static_assert(sizeof({name}) == 4u", table_c)
        self.assertEqual(fw.enums["AutoCtrlStep"]["CalibSwingStopCoeff"], 32)

    def test_both_inhibit_masks_match_the_literals_in_code(self):
        # WHY: auto_inhibited()/inhibit_names() and every calib-inhibit test trust these masks.
        # Citation: MdlApp.c:39675 (& 51086U) and :39679 (& 13976U).
        src = _mdlapp_c()
        auto = re.search(r"LogicalOperator_cfu0 = \(\(\(\(uint32_T\)status\) & \(\(uint32_T\)(\d+)U\)\)", src)
        calib = re.search(r"isCalibInhibited = \(\(\(\(uint32_T\)status\) & \(\(uint32_T\)(\d+)U\)\)", src)
        masks = firmware().manifest["inhibit_masks"]
        self.assertEqual(int(auto.group(1)), masks["AUTO_INHIBIT_MASK"]["value"])
        self.assertEqual(int(calib.group(1)), masks["CALIB_INHIBIT_MASK"]["value"])


class TestManifest(unittest.TestCase):
    def test_chart_state_tables_match_generated_assignments_and_stateflow_xml(self):
        # WHY: h.main_state() is how tests tell PositioningPaused from PositioningInhibited; a
        # table built from the wrong #define block would name the wrong state with confidence.
        # Two independent sources: (1) every constant the generated code ASSIGNS to the getter's
        # variable (is_c75_MdlApp / is_c11_MdlApp, build.py INTERNALS) resolved through all
        # #defines in MdlApp.c; (2) the uncommented states of the Stateflow chart whose
        # chartFileNumber is 75 / 11 in MdlApp.slx (chart_2537 AutoStsMgr/Chart: 57 states, the
        # six CalibForkPnt2/3 states commented; chart_1210 CalibStepMgr: 152, six ForkDownPnt2/3).
        fw = firmware()
        src = _mdlapp_c()
        table_c = (BUILD / "sil_table.c").read_text()
        defs = {m.group(1): int(m.group(2))
                for m in re.finditer(r"#define\s+(Mdl\w*?_IN_\w+)\s+\(\(uint8_T\)(\d+)U\)", src)}
        for key, var, number, n_states, n_commented in (("main", "is_c75_MdlApp", 75, 51, 6),
                                                        ("calib", "is_c11_MdlApp", 11, 146, 6)):
            with self.subTest(chart=key):
                table = fw.chart_states[key]
                self.assertEqual(table[0], "NO_ACTIVE_CHILD")
                self.assertEqual(sorted(table), list(range(n_states + 1)))
                assigned = {}
                for c in set(re.findall(var + r"\s*=\s*(Mdl\w*?_IN_\w+)\s*;", src)):
                    assigned.setdefault(defs[c], set()).add(re.sub(r"^Mdl\w*?_IN_", "", c))
                self.assertEqual(sorted(assigned), sorted(table))
                for idx, names in assigned.items():
                    self.assertEqual(len(names), 1, idx)
                    (cname,) = names
                    self.assertTrue(cname == table[idx] or re.fullmatch(re.escape(table[idx]) + r"_[a-z0-9]{4}", cname),
                                    f"{var}={idx}: code {cname} vs table {table[idx]}")
                self.assertIn(f"unsigned int sil_int_{key}_chart_state(void) "
                              f"{{ return MdlApp_DW.bitsForTID0.{var}; }}", table_c)
                _name, live, commented = _stateflow_chart(number)
                self.assertEqual(len(commented), n_commented)
                self.assertEqual(sorted(live), sorted(v for k, v in table.items() if k))
        self.assertEqual(_stateflow_chart(75)[0], "Subsystem/AutoStsMgr/Chart")
        self.assertEqual(_stateflow_chart(11)[0], "Subsystem/MachCalib/CalibStepMgr")

    def test_every_manifest_internal_is_callable_typed_and_means_what_its_name_says(self):
        # WHY: internals are read-only debug getters over bitfields and B/DW members; a getter on
        # the wrong member returns a plausible 0/1 forever. Each is called and cross-checked
        # against an observable effect.
        # Citation: build.py INTERNALS; swing latch MdlApp.c:12066-12070; accuracy state
        # :39081-39106 (SysPar.m:57 on-delay 20); cup contact on-delay SysPar.m:465
        # CntSuctionContactConfirmDly = 20 (MdlApp.c:39026); enh_LocalMain/Aux from Localization.
        h = Harness().reset().nominal_inputs()
        fw = h.fw
        h.tick()
        names = [n for n, _ in fw.manifest["internals"]]
        self.assertEqual(len(names), len(set(names)))
        for name, ctype in fw.manifest["internals"]:
            with self.subTest(internal=name):
                v = fw.internal(name)
                self.assertIsInstance(v, float if ctype == "float" else int)
                if ctype == "unsigned int" and name not in ("main_chart_state", "calib_chart_state"):
                    self.assertIn(v, (0, 1))
        self.assertEqual(h.main_state(), "NoTarget")
        self.assertEqual(fw.chart_states["calib"][fw.internal("calib_chart_state")], "Standby")
        for step in ("positioning_step", "picking_step", "placing_step"):
            self.assertFalse(getattr(h, step)().startswith("<"), step)
        # swing_init mirrors BIT_SWING_NOT_INIT
        self.assertEqual(fw.internal("swing_init"), 0)
        self.assertTrue(h.inhibit_bit("SWING_NOT_INIT"))
        h.set_swing_aligned(True)
        h.tick(2)
        self.assertEqual(fw.internal("swing_init"), 1)
        self.assertFalse(h.inhibit_bit("SWING_NOT_INIT"))
        # low_vertical_accuracy follows the GNSS gate
        self.assertEqual(fw.internal("low_vertical_accuracy"), 1)
        h.gnss_rtk_fixed()
        h.tick(2)
        self.assertEqual(fw.internal("low_vertical_accuracy"), 0)
        # all_cups_in_contact: exactly the 20-tick on-delay after all four contacts close
        fw["u.isSuctionCupContact"] = [1, 1, 1, 1]
        t0 = h.tick_count
        h.run_until(lambda h: h.fw.internal("all_cups_in_contact") == 1, 0.5, "cups")
        self.assertEqual(h.tick_count - t0, 20)
        # enh_local_*: the firmware's local main/aux antenna positions == geodesy's world points
        R = kin.Rz(-0.7) @ kin.Ry(-0.05) @ kin.Rx(0.06)
        h.set_pose(R_chs=R, **Harness.NOMINAL_POSE)
        h.gnss_site()
        main = h.place_chassis((123.4, -56.7, 2.5), R)
        aux = main + R @ np.asarray(fw["par.parKin.distAntMainToAntAux"], dtype=float)
        h.tick(3)
        for c, i in (("e", 0), ("n", 1), ("h", 2)):
            self.assertAlmostEqual(fw.internal(f"enh_local_main_{c}"), main[i], delta=5e-5)
            self.assertAlmostEqual(fw.internal(f"enh_local_aux_{c}"), aux[i], delta=5e-5)


# ----------------------------------------------------------------------------------------
# reset(): completeness and determinism
# ----------------------------------------------------------------------------------------
class TestResetCompleteness(unittest.TestCase):
    def test_firmware_owns_no_mutable_state_beyond_U_Y_B_DW_par_and_nonfinites(self):
        # WHY: any static outside what initialize()/reset() rewrite would leak across scenarios.
        # Citation: MdlApp.c:1597-1606, :373; rt_nonfinite.c:18-23; function-local statics in
        # MdlApp.c are all `static const` (e.g. :2837, :38972). Checked on the object files.
        found = set()
        for obj in sorted(BUILD.glob("*.o")):
            if obj.name == "sil_table.o":
                continue
            found |= set(_nm_data_symbols(obj, dynamic=False))
        self.assertEqual(found, FIRMWARE_STATE)

    def test_reset_after_eventful_run_zeroes_U_Y_and_restores_par_bitwise(self):
        # WHY: a scenario that patches par.* or leaves inputs set must not bias the next one.
        # Citation: MdlApp.c:52638 memset U, :52641 memset Y; firmware.py:106-109 restores par.
        # Spec A2 ("initialize zeroes U; parLocalTest untouched") -- confirmed.
        fw = firmware()
        sizes = _state_sizes(fw)
        h = _eventful_scenario(Harness(plant=_plant).reset())
        fw["par.parKin.lenArm"] = 2.1
        h.reset()
        snap = _snapshot(fw, sizes)
        self.assertEqual(snap["MdlApp_U"], bytes(sizes["MdlApp_U"]))
        self.assertEqual(snap["MdlApp_Y"], bytes(sizes["MdlApp_Y"]))
        self.assertEqual(snap["parLocalTest"], fw._par_initial)
        self.assertEqual(h.tick_count, 0)
        self.assertEqual(h.fw["u.chsImuQuat"], [0.0, 0.0, 0.0, 0.0])   # reset() does not re-apply nominal_inputs()
        self.assertTrue(math.isnan(ctypes.c_double.in_dll(fw.lib, "rtNaN").value))

    def test_reset_state_is_bit_identical_to_a_fresh_process(self):
        # WHY: proves reset() is as good as restarting the ECU, so test order cannot matter.
        # Citation: MdlApp.c:52615-52649 (initialize: rt_InitInfAndNaN, memset B/DW/U/Y,
        # MdlApp_Subsystem_Init); firmware.py:106-109.
        fw = firmware()
        _eventful_scenario(Harness(plant=_plant).reset())
        mine = _reset_digests(fw)
        child = _run_child("import json; from sil.firmware import Firmware; "
                           "from sil.tests.test_harness_review import _reset_digests; "
                           "print(json.dumps(_reset_digests(Firmware())))")
        self.assertEqual(mine, child)

    def test_eventful_trajectory_is_bit_identical_across_resets_and_processes(self):
        # WHY: SIL pass/fail verdicts are only reproducible if the firmware is deterministic
        # under the harness. Compared byte-for-byte on U, Y, B, DW, par and nonfinites after
        # every step. The second comparison runs in a CHILD process that first drives a
        # different history (tilted pose, calibration run), so a read of an uninitialised C
        # local would see different stack bytes there and break the equality.
        # Citation: chart_2537 T284 (NoTarget->Standby, MdlApp.c:22940-22944), StartPause edge
        # latch at MdlApp.c:39724-39744; initialize :52615-52649.
        a = _trajectory()
        b = _trajectory()
        c = _run_child("import json; from sil.tests.test_harness_review import _trajectory; "
                       "print(json.dumps(_trajectory(prior=True)))")
        self.assertEqual(len(a["digests"]), a["tick_count"])
        self.assertEqual(a["tick_count"], b["tick_count"])
        self.assertEqual(a["tick_count"], c["tick_count"])
        self.assertEqual(a["digests"], b["digests"])
        self.assertEqual(a["digests"], c["digests"])
        # the trajectory really was eventful
        self.assertIn(["Standby", False], a["visited"])
        self.assertIn(["Positioning", True], a["visited"])
        self.assertIn(["Positioning", False], a["visited"])
        self.assertGreater(len(set(a["digests"])), 150)

    def test_stepping_before_any_reset_is_refused_in_a_fresh_process(self):
        # WHY: an Isaac plant that builds Harness() and starts ticking would get plausible but
        # wrong kinematics. fixed: was silently allowed (firmware.py never required initialize).
        # The divergence it prevents is the nonfinites: rtNaN/rtInf are 0.0 until
        # rt_InitInfAndNaN (MdlApp.c:52620; used e.g. :10344 rtInfF), and three raw steps then
        # give y.euAng_ChsEstm_z = -2.356 rad and ~45 differing y leaves. Calling ONLY
        # rt_InitInfAndNaN removes every visible difference over those 3 steps; the non-zero
        # MdlApp_Subsystem_Init states (e.g. lowVerticalAccuracyState = true, :38340) exist but do
        # not reach y that early. Guard: firmware.py:111-113.
        self.assertTrue(_probe_stepping_without_reset_is_refused())
        raw = _run_child(_CHILD_NO_RESET % "raw")
        nan = _run_child(_CHILD_NO_RESET % "nan")
        for child in (raw, nan):
            self.assertIn("reset()", child["fw_step"])
            self.assertIn("reset()", child["h_tick"])
            self.assertEqual(child["tick_count"], 0)
        h = Harness().reset().nominal_inputs()
        h.tick(3)
        ref = {p: repr(h.fw[p]) for p in h.fw.paths("y.")}
        self.assertEqual(float(ref["y.euAng_ChsEstm_z"]), 0.0)
        self.assertAlmostEqual(float(raw["y"]["y.euAng_ChsEstm_z"]), -2.356194, places=5)
        self.assertGreater(len([p for p in ref if raw["y"][p] != ref[p]]), 30)
        self.assertEqual([p for p in ref if nan["y"][p] != ref[p]], [])


class TestParameterPath(unittest.TestCase):
    def setUp(self):
        self.h = Harness().reset().nominal_inputs()

    def test_par_patch_reaches_firmware_and_stored_inport_does_not(self):
        # WHY: reset() restores parLocalTest precisely because par.* is the live parameter set;
        # if the firmware read u.parKinStored instead, every par.* patch would be a no-op.
        # Citation: MdlApp.c:41669 (B.lenArm = parLocalTest.parKin.lenArm), :46000 (Y.parKin.lenArm).
        # Spec A2 ("*Stored inports are dead") -- confirmed.
        fw = self.h.fw
        fw["par.parKin.lenArm"] = 2.1
        self.h.tick()
        self.assertAlmostEqual(fw["y.parKin.lenArm"], 2.1, places=6)
        fw["u.parKinStored.lenArm"] = 2.3
        self.h.tick(2)
        self.assertAlmostEqual(fw["y.parKin.lenArm"], 2.1, places=6)


# ----------------------------------------------------------------------------------------
# firmware.py: value handling
# ----------------------------------------------------------------------------------------
class TestValueHandling(unittest.TestCase):
    def setUp(self):
        self.h = Harness().reset()
        self.fw = self.h.fw

    def test_float32_write_reads_back_the_float32_the_firmware_compares(self):
        # WHY: thresholds are compared in single precision; two doubles that collapse to one
        # float32 are the same stimulus, and readback must show that, not the Python literal.
        # Citation: MdlApp.h:1037 real32_T gnssPosStdDevZ; MdlApp.c:39089-39106 (float32 compare).
        f32 = lambda v: struct.unpack("<f", struct.pack("<f", v))[0]
        self.fw["u.gnssPosStdDevZ"] = 0.02
        self.assertEqual(self.fw["u.gnssPosStdDevZ"], f32(0.02))
        self.assertNotEqual(self.fw["u.gnssPosStdDevZ"], 0.02)
        self.fw["u.gnssPosStdDevZ"] = 0.0200000001
        self.assertEqual(self.fw["u.gnssPosStdDevZ"], f32(0.02))

    def test_float64_fields_keep_double_precision(self):
        # WHY: geodetic inputs (lat/lon in rad) lose centimetres if narrowed to float32.
        # Citation: MdlApp.h:1040 real_T blh_Main[3].
        v = [0.6283185307179586, 2.2689280275926285, 102.123456789]
        self.fw["u.blh_Main"] = v
        self.assertEqual(self.fw["u.blh_Main"], v)

    def test_array_length_mismatch_and_unknown_path_are_loud(self):
        # WHY: a 3-vector written into a 4-quaternion must not silently leave w stale.
        # Citation: firmware.py:155-156; MdlApp.h:987 real32_T chsImuQuat[4].
        with self.assertRaises(ValueError):
            self.fw["u.chsImuQuat"] = [1.0, 0.0, 0.0]
        with self.assertRaises(KeyError):
            self.fw["u.chsImuQuaternion"] = [1.0, 0.0, 0.0, 0.0]

    def test_enum_field_round_trips_the_full_int32_range(self):
        # WHY: enum inports are int32 in the ABI; the write path must neither truncate nor wrap.
        # (u.autoStepRenderingAck itself has ZERO reads in the generated code -- the rendering
        # watchdog compares two delays of autoCtrl_CurrStep, MdlApp.c:39399-39407, :52226, :52234,
        # spec A8.6 -- so this is purely a harness write-path check.)
        # Citation: MdlApp.h:1008 AutoCtrlStep autoStepRenderingAck; firmware.py:40-43.
        for v in (self.fw.enum_value("AutoCtrlStep", "CalibSwingStopCoeff"), -1, -2 ** 31, 2 ** 31 - 1):
            self.fw["u.autoStepRenderingAck"] = v
            self.assertEqual(self.fw["u.autoStepRenderingAck"], v)

    def test_boolean_writes_are_normalised_to_the_bit_the_firmware_reads(self):
        # WHY: a plant that writes `sensor_word & 0x2` or a numpy count into a boolean inport
        # must drive the firmware the way fw[...] reads back.
        # fixed: was stored raw (2) and read back True while the firmware saw FALSE: it ANDs
        # booleans and keeps them in 1-bit bitfields, MdlApp.c:12066-12070 `(!wasSwingAligned) &
        # U.isSwingAligned` (1 & 2 == 0) and :12153 into `uint_T wasSwingAligned:1` (MdlApp.h:600).
        # Real glue writes only A_ON/A_OFF (PrePostProc_If.c:290). Now firmware.py:161-162, and a
        # raw non-0/1 byte from outside the harness is refused on read (firmware.py:142-146).
        self.assertTrue(_probe_bool_truthy_value_reaches_firmware_as_true())
        fw = self.fw
        addr = fw._sigs["u.isSwingAligned"][0]
        for v, bit in ((2, 1), (0.5, 1), (np.bool_(True), 1), (np.int64(4), 1), (0, 0), (False, 0), (0.0, 0)):
            fw["u.isSwingAligned"] = v
            self.assertEqual(ctypes.c_uint8.from_address(addr).value, bit, repr(v))
        ctypes.c_uint8.from_address(addr).value = 2
        with self.assertRaises(ValueError):
            fw["u.isSwingAligned"]
        ctypes.c_uint8.from_address(addr).value = 0
        fw["u.isSuctionCupContact"] = [1, 1, 1, 1]
        with self.assertRaises(TypeError):
            fw["u.isSuctionCupContact"] = [0, 0, 0, "0"]
        self.assertEqual(fw["u.isSuctionCupContact"], [True] * 4)     # a refused write changes nothing

    def test_out_of_range_integer_write_is_rejected(self):
        # WHY: fw["u.autoReqStep"] = 256 would silently become 0 (NoTarget) and -1 become 255;
        # a sweep over request values would test different stimuli than it reports.
        # fixed: was wrapped modulo 2^n by the ctypes array store. Now firmware.py:163-171.
        # Citation: MdlApp.h:1007 uint8_T autoReqStep, :1077 uint32_T cntCatcherSigRst, :1008 enum.
        self.assertTrue(_probe_integer_overflow_is_rejected())
        fw = self.fw
        for path, bad, lo, hi in (("u.autoReqStep", (256, -1), 0, 255),
                                  ("u.cntCatcherSigRst", (-1, 2 ** 32), 0, 2 ** 32 - 1),
                                  ("u.autoStepRenderingAck", (2 ** 31, -2 ** 31 - 1), -2 ** 31, 2 ** 31 - 1)):
            with self.subTest(path=path):
                fw[path] = 7
                for v in bad:
                    with self.assertRaises(OverflowError):
                        fw[path] = v
                    self.assertEqual(fw[path], 7)
                for v in (lo, hi):
                    fw[path] = v
                    self.assertEqual(fw[path], v)

    def test_non_integral_or_non_numeric_writes_are_rejected(self):
        # WHY: a plant computing a step or id as float (1.9) would get 1 written with no warning.
        # fixed: was int(v) truncation. Now firmware.py:159-166; integral floats still pass.
        # Citation: MdlApp.h:1007 uint8_T autoReqStep.
        self.assertTrue(_probe_non_integral_float_to_int_is_rejected())
        fw = self.fw
        for v in (1.9, np.float32(1.5), np.float64(0.1), float("nan")):
            with self.assertRaises(ValueError, msg=repr(v)):
                fw["u.autoReqStep"] = v
        with self.assertRaises((ValueError, OverflowError)):
            fw["u.autoReqStep"] = float("inf")
        for v, want in ((3.0, 3), (np.float32(4.0), 4), (np.float64(9.0), 9), (True, 1), (np.uint8(5), 5)):
            fw["u.autoReqStep"] = v
            self.assertEqual(fw["u.autoReqStep"], want)
        for path, v in (("u.autoReqStep", "1"), ("u.gnssPosStdDevZ", "0.02"), ("u.isSwingAligned", "1"),
                        ("u.gnssPosStdDevZ", b"\x01")):
            with self.assertRaises(TypeError, msg=f"{path}={v!r}"):
                fw[path] = v

    def test_finite_double_outside_float32_range_is_rejected(self):
        # WHY: the same stimulus-reporting hazard the integer checks close: a plant value of
        # 1e40 (a unit slip, a diverged physics step) reaches a real32_T inport as +inf, and a
        # trace shows inf with no hint that the write overflowed. inf itself stays writable.
        # LIBRARY BUG (low): firmware.py:173 converts with float(v) and the c_float array store
        # rounds anything above FLT_MAX (3.4028235e38) to inf without error.
        # Citation: MdlApp.h:1037 real32_T gnssPosStdDevZ (compared at MdlApp.c:39089-39106).
        self.assertTrue(_probe_float32_overflow_is_rejected())


# ----------------------------------------------------------------------------------------
# harness.py: input idioms, stepping, trace
# ----------------------------------------------------------------------------------------
class TestHarnessIdioms(unittest.TestCase):
    def setUp(self):
        self.h = Harness().reset().nominal_inputs()

    def test_tick_count_equals_firmware_step_count_including_hidden_ticks(self):
        # WHY: every firmware dwell is a step count; if tick_count drifted from real steps
        # (hidden ticks in pulse/request_step) every timing assertion would be off.
        # Counted twice, independently of tick(): a proxy around fw.step(), and a firmware-side
        # clock -- the GNSS accuracy on-delay CntAccuracyChkDly = 20 (SysPar.m:57, counted at
        # MdlApp.c:39370-39386) must set the auto-inhibit aggregate at tick 20 whether the ticks
        # come from tick() or from pulse()/request_step() internals. Tick 1 is inhibited for a
        # different reason: swing latch and target validity reach the inhibit word a step late
        # (BIT_SWING_NOT_INIT, BIT_NO_TARGET). The 0.01F literal is SysPar.m:1 SampleTime.
        self.assertIn("MdlApp_LPF1st_JntAngs(jntAngFilt, 0.01F, 3.0F, false);", _mdlapp_c())
        self.assertEqual(DT, 0.01)

        def first_inhibit(drive):
            h = Harness().reset().nominal_inputs()
            h.set_swing_aligned(True)
            h.set_target_panel()
            calls, hidden = [], []
            real_step = h.fw.step
            h.fw.step = lambda: (calls.append(h.tick_count), real_step())
            try:
                h.trace("y.autoCtrl_InhibitSts")
                h.tick()
                self.assertTrue({"BIT_SWING_NOT_INIT", "BIT_NO_TARGET"} <= set(h.inhibit_names()))
                while h.tick_count < 26:
                    drive(h, hidden)
            finally:
                del h.fw.step
            self.assertEqual(calls, list(range(h.tick_count)))
            self.assertEqual([r[0] for r in h.trace_rows], list(range(1, h.tick_count + 1)))
            first = next(r[0] for r in h.trace_rows if r[0] > 1 and r[1] & 0x2)
            return first, len(hidden)

        def plain(h, hidden):
            h.tick()

        def mixed(h, hidden):
            k = h.tick_count
            if k % 5 == 1:
                before = h.tick_count
                h.pulse("u.tabletAutoReq_Stop")                  # 1 high + 1 trailing low
                hidden.append(h.tick_count - before)
            elif k % 5 == 3:
                before = h.tick_count
                h.request_step("NoTarget")                        # latched 0: one hidden 255 tick
                hidden.append(h.tick_count - before)
            else:
                h.tick()

        self.assertEqual(first_inhibit(plain), (20, 0))
        first, n_hidden = first_inhibit(mixed)
        self.assertEqual(first, 20)
        self.assertGreater(n_hidden, 4)

    def test_pulse_gives_a_rising_edge_even_if_left_high(self):
        # WHY: StartPause guards are rising-edge only; a level left high must not swallow the
        # next pulse. Asserted on the FIRMWARE's reaction (StartStopSts), not on the inport the
        # harness wrote -- the old version traced u.jstAutoReq_StartPause and was a tautology.
        # Contract (harness.py:206-216): lower+tick if high, 1 for `ticks`, then 0 and ONE MORE
        # tick. Citation: chart_2537 guard [hasChanged(autoReq_StartPause) && autoReq_StartPause],
        # OR of jst|rmt at MdlApp.c:39724-39725, latched into a 1-bit field at :39743-39744.
        h = _positioning_paused()
        h.trace("u.jstAutoReq_StartPause", "y.autoCtrl_StartStopSts")
        h.fw["u.jstAutoReq_StartPause"] = 1          # a level write is its own edge: runs
        h.tick()
        self.assertTrue(h.is_running())
        t0 = h.tick_count
        h.pulse("u.jstAutoReq_StartPause")           # input still high from the level write
        self.assertEqual(h.tick_count - t0, 3)
        self.assertEqual([r[1:] for r in h.trace_rows],
                         [[True, True], [False, True], [True, False], [False, False]])
        self.assertEqual(h.main_state(), "PositioningPaused")

    def test_back_to_back_pulses_are_two_edges(self):
        # WHY: pause-then-resume, or a joystick press followed by a remote press, must be two
        # commands. Without the trailing low tick the second pulse would find the firmware
        # still latched high (jst and rmt are OR-ed) and see no edge.
        # fixed: was one merged edge (pulse() left the input low without ticking).
        # Citation: MdlApp.c:39724-39725 (OR), :39743-39744 (latch); harness.py:206-216.
        for second in ("u.jstAutoReq_StartPause", "u.rmtAutoReq_StartPause"):
            with self.subTest(second=second):
                h = _positioning_paused()
                h.trace("y.autoCtrl_StartStopSts")
                h.pulse("u.jstAutoReq_StartPause")
                h.pulse(second)
                self.assertEqual([r[1] for r in h.trace_rows], [True, True, False, False])
                self.assertEqual(h.main_state(), "PositioningPaused")

    def test_request_step_re_edges_through_255_which_no_guard_compares(self):
        # WHY: the one-tick "re-edge" value must never be a request itself.
        # fixed: was `0 if val else 1`, so request_step('NoTarget') from 0 injected a real Standby
        # request (T284 NoTarget -> Standby, chart_2537 SSID 284, MdlApp.c:22940-22944).
        # Source proof that 255 is inert: every read of MdlApp_U.autoReqStep is either the
        # hasChanged latch (:39741-39742) or `== ((int32_T)<AutoCtrlStep name>)`, never NoTarget
        # and never a number; the latch DW.autoReq_Step_start is only initialised, copied to
        # _prev, or compared with `!=`. Harness: RE_EDGE_STEP harness.py:45, request_step :225-232.
        fw = firmware()
        src = _mdlapp_c()
        reads = re.findall(r"MdlApp_U\s*\.\s*autoReqStep\b", src)
        compared = re.findall(r"\(\(int32_T\)\s*MdlApp_U\s*\.\s*autoReqStep\s*\)\s*==\s*\(\(int32_T\)\s*(\w+)\s*\)", src)
        self.assertEqual(len(reads), len(compared) + 1)
        self.assertTrue(set(compared) <= set(fw.enums["AutoCtrlStep"]))
        self.assertNotIn("NoTarget", compared)
        flat = re.sub(r"\s+", " ", re.sub(r"/\*.*?\*/", " ", src, flags=re.S))
        for m in re.finditer(r"MdlApp_DW\.autoReq_Step_start", flat):
            ctx = flat[m.start() - 30:m.end() + 30]
            self.assertTrue(re.search(r"!= MdlApp_DW\.autoReq_Step_start|autoReq_Step_prev = MdlApp_DW\.autoReq_Step_start;"
                                      r"|MdlApp_DW\.autoReq_Step_start = (MdlApp_U\.autoReqStep|0U);", ctx), ctx)
        self.assertEqual(RE_EDGE_STEP, 255)
        self.assertNotIn(RE_EDGE_STEP, fw.enums["AutoCtrlStep"].values())
        # behaviour: same value re-requested from Standby
        h = _standby()
        h.trace("u.autoReqStep", "y.autoCtrl_CurrStep")
        h.request_step("Standby")
        self.assertEqual([r[1] for r in h.trace_rows], [RE_EDGE_STEP])
        h.tick()
        self.assertEqual([r[1] for r in h.trace_rows], [RE_EDGE_STEP, 1])
        self.assertEqual({r[2] for r in h.trace_rows}, {fw.enum_value("AutoCtrlStep", "Standby")})
        # a different value inserts no hidden tick
        t0 = h.tick_count
        h.request_step("Positioning")
        self.assertEqual(h.tick_count, t0)
        # NoTarget from a latched 0 re-edges through 255 as well, and stays NoTarget
        self.assertTrue(_probe_request_notarget_does_not_request_standby())
        h = _booted()
        h.trace("u.autoReqStep", "y.autoCtrl_CurrStep")
        h.request_step("NoTarget")
        h.tick(3)
        self.assertEqual([r[1] for r in h.trace_rows], [RE_EDGE_STEP, 0, 0, 0])
        self.assertEqual({r[2] for r in h.trace_rows}, {0})

    def test_request_step_edges_against_the_latched_value_not_the_unticked_inport(self):
        # WHY: request_step promises "hasChanged() fires".
        # fixed: was a comparison against the current inport byte. hasChanged compares against
        # the value latched at the LAST STEP (MdlApp.c:39741-39742, autoReq_Step_start): latched
        # 1, test wrote 9 without stepping, request_step(1) saw 9 != 1, skipped the re-edge, and
        # the step saw 1 -> 1. Now harness.py:176 records the latched value in tick().
        self.assertTrue(_probe_request_step_edges_against_latched_value())

    def test_re_requesting_the_current_step_stays_in_standby_with_no_state_effect(self):
        # WHY: the re-edge DOES fire a firmware transition -- T159, a Standby self-loop with
        # re-entry (chart_2537 SSID 159, MdlApp.c:25157-25170, in MdlApp_Standby :25138) -- so
        # "nothing happens" is not the claim. The claim is that the re-entry rewrites the same
        # outputs and leaves no trace: Y, B and DW digests after it are byte-identical, tick for
        # tick, to a control run that spends the hidden tick as a plain tick instead.
        fw = firmware()
        sizes = {k: v for k, v in _state_sizes(fw).items() if k in ("MdlApp_Y", "MdlApp_B", "MdlApp_DW")}

        def run(rereq):
            h = _standby()
            h.tick(3)
            h.trace("u.autoReqStep")
            if rereq:
                h.request_step("Standby")
            else:
                h.tick()
            rows = []
            for _ in range(6):
                h.tick()
                rows.append((h.curr_step(), _combined_digest(fw, sizes)))
            return h, rows

        ha, a = run(True)
        hb, b = run(False)
        self.assertEqual([r[1] for r in ha.trace_rows[:2]], [RE_EDGE_STEP, 1])   # the edge happened
        self.assertEqual(ha.tick_count, hb.tick_count)
        self.assertEqual({s for s, _ in a}, {"Standby"})
        self.assertEqual(a, b)

    def test_run_until_times_out_after_exactly_the_requested_ticks(self):
        # WHY: timeouts are the fail verdict of every scenario; an off-by-one changes verdicts
        # at dwell boundaries. Citation: harness.py:191-203 (ticks_for :184-186).
        h = self.h
        with self.assertRaises(StepTimeout):
            h.run_until(lambda h: False, 0.05, "never")
        self.assertEqual(h.tick_count, 5)
        got = h.run_until(lambda h: h.tick_count == 8, 0.05, "tick 8")
        self.assertEqual(got, 8)
        with self.assertRaises(ValueError):
            h.run_until(lambda h: False, 0.004, "below one tick")
        self.assertEqual(h.tick_count, 8)

    def test_run_until_returns_immediately_when_condition_already_holds(self):
        # WHY: "the tick at which the condition first held" -- a one-tick output event visible
        # on entry must not be stepped past.
        # fixed: was a tick before the first pred() check. Now harness.py:192-193.
        # Citation: harness.py:21-23 (ONE-TICK SKEW doc).
        self.assertTrue(_probe_run_until_sees_condition_true_on_entry())

    def test_run_until_sees_only_completed_steps_so_a_pending_write_is_not_yet_consumed(self):
        # WHY: the price of the pre-check, pinned so nobody relies on the opposite. On entry pred
        # reads the outputs of the LAST step; inputs written since have not been stepped. A
        # request followed by run_until(<condition that is already true>) returns without the
        # firmware ever seeing the request. Wait for the condition the request should CHANGE.
        # Citation: harness.py:191-193; T284 NoTarget -> Standby consumes u.autoReqStep in-step
        # (MdlApp.c:22940-22944, latched at :39741-39742).
        h = _booted()
        t0 = h.tick_count
        h.request_step("Standby")                           # latched 0 -> no hidden tick
        self.assertEqual(h.run_until(lambda h: h.curr_step() == "NoTarget", 0.5), t0)
        self.assertEqual(h.tick_count, t0)
        self.assertEqual(h.fw["u.autoReqStep"], h.fw.enum_value("AutoCtrlStep", "Standby"))
        self.assertEqual(h.run_until(lambda h: h.curr_step() == "Standby", 0.05), t0 + 1)

    def test_run_seconds_rounds_half_ticks_up(self):
        # WHY: a delay sweep in 10 ms steps offset by 5 ms must test distinct tick counts.
        # fixed: was banker's rounding (0.015 -> 2, 0.025 -> 2, 0.005 -> 0).
        # Citation: harness.py:184-189; DT = 0.01 (harness.py:44, SysPar.m:1 SampleTime).
        self.assertTrue(_probe_run_seconds_is_strictly_monotone_on_half_ticks())
        for s, n in ((0.004, 0), (0.005, 1), (0.015, 2), (0.025, 3), (0.035, 4), (0.045, 5), (0.1, 10), (1.0, 100)):
            self.assertEqual(Harness.ticks_for(s), n, s)

    def test_trace_rows_are_post_step_and_one_based(self):
        # WHY: trace CSVs are SIL evidence; row N must be the output of the N-th step. The whole
        # column is asserted: a pre-step sampler would give [0, 7, 7, 7].
        # Citation: harness.py:172-181; in NoTarget y.tarPanelIdAck follows tarPanelData.id in the
        # same step (target latch runs after the chart, MdlApp.c:40922-40957, ack at :40949).
        h = self.h
        h.trace("y.tarPanelIdAck")
        h.set_target_panel(panel_id=7)
        h.tick(4)
        self.assertEqual([r[0] for r in h.trace_rows], [1, 2, 3, 4])
        self.assertEqual([r[1] for r in h.trace_rows], [7, 7, 7, 7])

    def test_trace_rows_stay_rectangular_when_paths_change(self):
        # WHY: save_trace() writes one header; earlier rows would sit under the wrong columns.
        # fixed: was trace() replacing trace_paths but keeping trace_rows. Now harness.py:336-340.
        # Citation: MdlApp.h:1082-1175 (the y.* columns traced).
        self.assertTrue(_probe_trace_rows_stay_rectangular())

    def test_trace_array_floats_keep_float32_precision(self):
        # WHY: trace CSVs are compared across runs/firmware revisions.
        # fixed: was f"{x:g}" (6 significant digits, 1.0000001 -> "1"). Now repr, harness.py:343-346.
        # Citation: MdlApp.h:987 real32_T chsImuQuat[4].
        self.assertTrue(_probe_trace_array_floats_keep_precision())

    def test_plants_run_in_list_order_before_every_step(self):
        # WHY: SaveHandshake must see Y of the previous step and write U before the next one, and
        # an Isaac plant stacked with it must not reorder that. Citation: harness.py:96, :172-178;
        # the ECU runs SaveMchCalibData after MdlApp_step in the same task (main.c:180-199).
        seen = []
        h = Harness(plant=[lambda h: seen.append(("f", h.tick_count)),
                           lambda h: seen.append(("g", h.tick_count))]).reset()
        h.tick(2)
        self.assertEqual(seen, [("f", 0), ("g", 0), ("f", 1), ("g", 1)])
        single = []
        Harness(plant=lambda h: single.append(h.tick_count)).reset().tick(2)
        self.assertEqual(single, [0, 1])


# ----------------------------------------------------------------------------------------
# harness.SaveHandshake (main.c:394-419)
# ----------------------------------------------------------------------------------------
class TestSaveHandshake(unittest.TestCase):
    def test_snapshot_on_rising_edge_only_and_level_ack_like_main_c(self):
        # WHY: the emulation's edge logic is what makes a calibration finish fast; a level or
        # repeated snapshot would record the wrong NVM history. Driven by writing y.* directly,
        # no firmware step, so only SaveHandshake is under test.
        # Citation: main.c:399-418 (clear when no request; SaveInternalParam+WriteToNVM on rising
        # edge; ReadInternalParam on falling edge; isCalibDataSaved = wasCalibDataSaved).
        emu = SaveHandshake()
        h = Harness(plant=emu).reset()
        fw = h.fw
        emu(h)
        self.assertEqual((emu.saved, emu.reloads, fw["u.isCalibDataSaved"]), ([], 0, False))
        fw["y.isCalibDataSaveReq"] = 1
        fw["y.parKin.lenArm"] = 2.5
        emu(h)
        emu(h)
        self.assertEqual(len(emu.saved), 1)
        tick, snap = emu.saved[0]
        self.assertEqual(tick, 0)
        self.assertEqual(snap["y.parKin.lenArm"], 2.5)
        self.assertEqual(len(snap), 55 + 72 + 40 + 1)   # + y.jntAngRotZeroOffs (AppCtrlIf.c:882)
        self.assertTrue(fw["u.isCalibDataSaved"])
        fw["y.isCalibDataSaveReq"] = 0
        emu(h)
        self.assertEqual((len(emu.saved), emu.reloads, fw["u.isCalibDataSaved"]), (1, 1, False))
        fw["y.isCalibDataSaveReq"] = 1
        emu(h)
        self.assertEqual((len(emu.saved), emu.reloads, fw["u.isCalibDataSaved"]), (2, 1, True))

    def test_fork_ref_pose_save_is_snapshotted_on_the_request_edge_and_acked_next_step(self):
        # WHY: step 28 is the SIL v1 scenario; its pass criterion is "the save happened with these
        # values". The snapshot must be Y of the step that raised the request, and the firmware
        # must leave _save one step later instead of waiting out the 1.0 s fallback.
        # Citation: request is a during action of ForkRefPose_save (MdlApp.c:30082-30083); exit on
        # hasChanged(isCalibDataSaved) && isCalibDataSaved (:30056-30061, CntCalib_save = 100
        # SysPar.m:107 fallback); main.c:394-419.
        emu = SaveHandshake()
        req_y = []

        def recorder(h):          # runs BEFORE emu, sees the same Y
            if h.fw["y.isCalibDataSaveReq"]:
                req_y.append((h.tick_count, {p: h.fw[p] for p in h.fw.paths("y.")
                                             if p.startswith(SaveHandshake.SNAPSHOT_PREFIXES)}))

        h = Harness(plant=[recorder, emu]).reset().nominal_inputs()
        h.set_swing_aligned(True)
        h.gnss_rtk_fixed()
        h.tick(3)
        h.request_step("CalibForkRefPose")
        h.tick()
        h.pulse("u.jstAutoReq_StartPause")
        h.trace("y.calibStep", "y.isCalibDataSaveReq")
        h.run_until(lambda h: h.calib_step() == "CalibStandby", 12.0, "calibration done")
        h.tick(3)
        req_rows = [r[0] for r in h.trace_rows if r[2]]
        self.assertEqual(len(req_rows), 1)                 # acked on the very next step
        (n_req,) = req_rows
        self.assertEqual(len(emu.saved), 1)
        self.assertEqual(emu.saved[0][0], n_req)
        self.assertEqual(req_y[0], emu.saved[0])
        self.assertEqual(emu.reloads, 1)
        self.assertFalse(h.fw["u.isCalibDataSaved"])
        calib = h.fw.enums["CalibStep"]
        self.assertEqual(next(r[1] for r in h.trace_rows if r[0] == n_req), calib["ForkRefPose_save"])
        self.assertEqual(next(r[1] for r in h.trace_rows if r[0] == n_req + 1), calib["CalibStandby"])

    def test_snapshot_covers_every_output_the_ecu_persists(self):
        # WHY: `saved` stands for SaveInternalParam() + WriteToNVM(); a calibration result the ECU
        # persists but the snapshot omits cannot be checked in SIL.
        # LIBRARY BUG (low): harness.py:69 SNAPSHOT_PREFIXES has y.parKin / y.imuMntOri /
        # y.tblReqSpdToActCmd but not y.jntAngRotZeroOffs, which the glue copies into INTP while
        # isCalibrating (AppCtrlIf.c:882) alongside the rest (AppCtrlIf.c:801-1018). Fix: add
        # "y.jntAngRotZeroOffs" to SNAPSHOT_PREFIXES. (This is the CalibRot step-26 result;
        # firmware writes it at MdlApp.c:43357-43372.)
        self.assertTrue(_probe_save_snapshot_covers_every_persisted_output())


# ----------------------------------------------------------------------------------------
# word size / host arithmetic
# ----------------------------------------------------------------------------------------
class TestHostArithmetic(unittest.TestCase):
    def test_no_long_size_t_or_packing_dependence_in_firmware_sources(self):
        # WHY: the build bypasses the ERT long-size guard by redefining LONG_MAX/ULONG_MAX
        # (build/sil/wordsize_shim.h); that is only safe if nothing but the guard uses them, or
        # long itself. Citation: MdlApp.c:361-369 (#if ULONG_MAX/LONG_MAX ... #error, the only
        # LONG_MAX use); rtwtypes.h:42-63 maps fixed-width types to char/short/int; build.py:21-26.
        files = [_gen() / "MdlApp_ert_rtw/MdlApp.c"] + [Path(p) for p in sorted(glob.glob(str(_gen() / "slprj/ert/_sharedutils/*.[ch]")))]
        offenders = []
        guard_lines = 0
        for f in files:
            code = re.sub(r"/\*.*?\*/", " ", _read_latin1(f), flags=re.S)
            code, n = re.subn(r"#if \( ULONG_MAX != \(0xFFFFFFFFU\) \) \|\| \( LONG_MAX != \(0x7FFFFFFF\) \)\s*\n", "\n", code)
            guard_lines += n
            code = re.sub(r"#error[^\n]*(\\\n[^\n]*)*", " ", code)
            for m in re.finditer(r"\b(long|ulong_T|ptrdiff_t|intptr_t|uintptr_t|U?LONG_MAX|LONG_MIN)\b|#\s*pragma\s+pack|__attribute__", code):
                offenders.append((f.name, m.group(0)))
            for m in re.finditer(r"\bsize_t\b[^;]*;", code):
                if f.name not in ("rt_nonfinite.c", "rt_nonfinite.h", "rtGetNaN.c", "rtGetInf.c"):
                    offenders.append((f.name, m.group(0)))
        # rtwtypes.h:60 typedefs ulong_T but nothing uses it
        offenders = [o for o in offenders if not (o[0] == "rtwtypes.h" and o[1] in ("long", "ulong_T"))]
        self.assertEqual(guard_lines, 1)
        self.assertEqual(offenders, [])

    def test_host_fpu_baseline_is_ieee_nearest_without_ftz_or_daz(self):
        # WHY: TriCore evaluates real32_T in IEEE single, round-to-nearest, no fused multiply-add.
        # BASELINE ONLY: this checks the test process at test time; co-simulation protection is
        # the per-step guard tested below. The exception-mask check proves the MXCSR field was
        # really read (a zero buffer would pass the FTZ/DAZ checks).
        # Citation: build.py:274 -ffp-contract=off; MdlApp.h:11 "Embedded hardware selection:
        # Infineon->TriCore".
        self.assertIn("-ffp-contract=off", firmware().manifest["flags"])
        if not X86_64_LINUX:
            self.skipTest("MXCSR layout check is x86-64 glibc specific")
        try:
            libm = ctypes.CDLL("libm.so.6")
        except OSError as e:
            self.skipTest(str(e))
        self.assertEqual(libm.fegetround(), 0)          # FE_TONEAREST on x86
        env = (ctypes.c_char * 32)()
        self.assertEqual(libm.fegetenv(env), 0)
        mxcsr = struct.unpack_from("<I", bytes(env), 28)[0]
        self.assertEqual(mxcsr & 0x1F80, 0x1F80, f"exception masks not default: MXCSR=0x{mxcsr:04X}")
        self.assertEqual(mxcsr & 0x8040, 0, f"FTZ/DAZ set: MXCSR=0x{mxcsr:04X}")
        self.assertEqual(mxcsr & 0x6000, 0, f"SSE rounding not nearest: MXCSR=0x{mxcsr:04X}")
        self.assertEqual(firmware().lib.sil_fp_env_violation(), 0)

    def test_fp_guard_refuses_to_step_with_ftz_daz_or_changed_rounding(self):
        # WHY: a physics runtime in the same process may enable flush-to-zero/denormals-are-zero
        # or change rounding; the firmware's filters would change with no visible error. Set in a
        # CHILD process through glibc fesetenv (MXCSR at fenv_t+28) and fesetround(FE_UPWARD).
        # Citation: build.py:261-265 sil_fp_env_violation; firmware.py:114-117.
        if not X86_64_LINUX:
            self.skipTest("MXCSR is x86-64 specific")
        child = _run_child(r"""
import ctypes, json, struct
from sil.firmware import Firmware
fw = Firmware(); fw.reset()
libm = ctypes.CDLL("libm.so.6")
def env():
    e = (ctypes.c_char * 32)(); libm.fegetenv(e); return e
def attempt():
    r = {"violation": fw.lib.sil_fp_env_violation()}
    try:
        fw.step(); r["raised"] = None
    except RuntimeError as ex:
        r["raised"] = str(ex)
    return r
res = {"clean": fw.lib.sil_fp_env_violation()}
saved = env()
for name, bits in (("FTZ", 0x8000), ("DAZ", 0x0040)):
    e = bytearray(bytes(saved))
    struct.pack_into("<I", e, 28, struct.unpack_from("<I", e, 28)[0] | bits)
    libm.fesetenv((ctypes.c_char * 32).from_buffer(e))
    r = attempt()
    r["set"] = bool(struct.unpack_from("<I", bytes(env()), 28)[0] & bits)
    libm.fesetenv(saved)
    res[name] = r
libm.fesetround(0x800)                 # FE_UPWARD
res["UPWARD"] = attempt()
libm.fesetround(0)
res["restored"] = fw.lib.sil_fp_env_violation()
fw.step()
print(json.dumps(res))
""")
        self.assertEqual(child["clean"], 0)
        # fesetround(FE_UPWARD) sets the x87 AND the SSE rounding field, so since the guard reads
        # MXCSR RC (0x6000) as well, UPWARD reports both: 0x4000 (RC = up) | 1<<31 (fegetround).
        for name, bits in (("FTZ", 0x8000), ("DAZ", 0x0040), ("UPWARD", 0x80004000)):
            with self.subTest(mode=name):
                self.assertTrue(child[name].get("set", True))
                self.assertEqual(child[name]["violation"], bits)
                self.assertIsNotNone(child[name]["raised"], "step() ran")
                self.assertIn("floating-point environment", child[name]["raised"])
        self.assertEqual(child["restored"], 0)

    def test_fp_guard_sees_an_sse_only_rounding_change(self):
        # WHY: the firmware's real32_T arithmetic runs on SSE, so the SSE rounding field is the
        # one that matters. Measured in a child: MXCSR RC = round-up written directly (what
        # _MM_SET_ROUNDING_MODE / _mm_setcsr do -- the same register the guard already watches for
        # FTZ/DAZ) changes the last bits of y.links.* after 50 steps, while fegetround() still
        # reports FE_TONEAREST and the guard returns 0.
        # LIBRARY BUG (medium): build.py:262-263 checks `_mm_getcsr() & 0x8040` (FTZ|DAZ) plus
        # fegetround(), and glibc's x86-64 fegetround reads only the x87 control word. The SSE
        # rounding bits 0x6000 are never checked. Fix: `_mm_getcsr() & 0xE040u`.
        # Citation: MdlApp.h:11 TriCore single precision; firmware.py:23-25 claims "the rounding mode".
        self.assertTrue(_probe_fp_guard_sees_sse_rounding_mode())


# ----------------------------------------------------------------------------------------
# sil/kinematics.py: the IMU publisher every plant shares
# ----------------------------------------------------------------------------------------
class TestKinematicsLibrary(unittest.TestCase):
    def test_rot_to_quat_is_exact_on_all_four_pivot_branches(self):
        # WHY: every published IMU goes through rot_to_quat; a sign slip in one Shepperd branch
        # only shows once that branch is selected -- rotations of about 90 deg or more, which the
        # mount matrices themselves are (IMU boards sit ~90 deg to their links): nominal_inputs()
        # alone goes through branches 0 (bm1), 1 (arm, bkt) and 3 (chs, tilt). Checked against the
        # matrix the firmware builds from a quaternion, MdlApp.c:10917-10925.
        # Citation: kinematics.py:62-80.
        rng = np.random.default_rng(7)
        hits = {0: 0, 1: 0, 2: 0, 3: 0}
        worst = 0.0
        for _ in range(4000):
            q = rng.normal(size=4)
            q /= np.linalg.norm(q)
            R = _quat_to_R(q)
            t = [1 + R[0, 0] + R[1, 1] + R[2, 2], 1 + R[0, 0] - R[1, 1] - R[2, 2],
                 1 - R[0, 0] + R[1, 1] - R[2, 2], 1 - R[0, 0] - R[1, 1] + R[2, 2]]
            hits[int(np.argmax(t))] += 1
            out = kin.rot_to_quat(R)
            worst = max(worst, np.abs(_quat_to_R(out) - R).max(), abs(np.linalg.norm(out) - 1))
        self.assertTrue(all(n > 500 for n in hits.values()), hits)
        self.assertLess(worst, 1e-12)
        for R, axis in ((kin.Rx(math.pi), 1), (kin.Ry(math.pi), 2), (kin.Rz(math.pi), 3)):
            q = kin.rot_to_quat(R)
            self.assertAlmostEqual(abs(q[axis]), 1.0, places=12)

    def test_mirror_is_the_firmware_input_mirror_a_180_deg_turn_about_y(self):
        # WHY: the publisher pre-applies the firmware's left->right-handed mirror; if the two
        # disagreed every link would be read in a mirrored frame.
        # Citation: MdlApp.c:10910-10915 (qx = -quat(2), qz = -quat(4), imuAcc/imuAngRate
        # [-x; y; -z]); kinematics.py:83-88.
        src = _mdlapp_c()
        for line in ("'<S164>:1:6' qx = -quat(2);", "'<S164>:1:7' qy = quat(3);", "'<S164>:1:8' qz = -quat(4);",
                     "imuAcc = [-imuAccRaw(1); imuAccRaw(2); -imuAccRaw(3)];",
                     "imuAngRate = [-imuAngRateRaw(1); imuAngRateRaw(2); -imuAngRateRaw(3)];"):
            self.assertIn(line, src)
        rng = np.random.default_rng(3)
        Ry180 = kin.Ry(math.pi)
        for _ in range(50):
            q = rng.normal(size=4)
            q /= np.linalg.norm(q)
            v = rng.normal(size=3)
            np.testing.assert_allclose(_quat_to_R(kin.mirror(q)), Ry180 @ _quat_to_R(q) @ Ry180.T, atol=1e-12)
            np.testing.assert_allclose(kin.mirror(v), Ry180 @ v, atol=1e-12)
            np.testing.assert_allclose(kin.mirror(kin.mirror(q)), q)

    def test_publish_imus_port_to_mount_pairing_matches_the_firmware_call_sites(self):
        # WHY: the bucket port carries the four-bar INPUT link IMU (imuLink), not a bucket IMU; a
        # pairing slip would publish every link through another link's mount.
        # Citation: MdlApp.c:41868-41970, MdlApp_ImuToLink_Arm(MdlApp_U.<port>ImuQuat, ...,
        # parLocalTest.<mount>.a11, ...); kinematics.py:37.
        flat = re.sub(r"\s+", " ", _mdlapp_c())
        pairs = dict(re.findall(r"MdlApp_ImuToLink_Arm\(MdlApp_U\.(\w+)ImuQuat, MdlApp_U\.\w+ImuAcc, "
                                r"MdlApp_U\.\w+ImuAngRate, parLocalTest\.(\w+)\.a11", flat))
        self.assertEqual(pairs.pop("bm2"), "imuBm2")        # real ECU sends zeros; not published
        self.assertEqual(pairs, kin.PORT_MOUNT)

    def test_set_pose_round_trips_through_the_firmware_at_a_tilted_moving_pose(self):
        # WHY: set_pose/publish_imus is THE sensor path of the Isaac plant. A tilted, yawed chassis
        # with body rates on every link exercises quaternion, mount transpose, mirror and gyro
        # mapping at once; the firmware must hand back the same attitude (312 Euler), rotation
        # matrix, joint angles and joint rates.
        # Citation: linkOri = imuOri*mntOri MdlApp.c:10940; chassis 312 Euler y.chs.euAngSeq;
        # joint angles/rates from IMU differences (MdlApp.c:12411 boom swing pinned 0);
        # kinematics.py:151-176.
        h = Harness().reset().nominal_inputs()
        fw = h.fw
        R = kin.Rz(0.3) @ kin.Ry(0.1) @ kin.Rx(-0.08)
        q_bm1, q_arm, q_inp, q_tilt = math.radians(-30), math.radians(100), math.radians(-40), math.radians(12)
        qd_bm1, qd_arm, qd_inp = 0.2, -0.3, 0.4
        w_chs = np.array([0.01, -0.02, 0.05])
        w_bm1 = kin.Ry(q_bm1).T @ w_chs + [0.0, qd_bm1, 0.0]
        w_arm = kin.Ry(q_arm).T @ w_bm1 + [0.0, qd_arm, 0.0]
        w_inp = kin.Ry(q_inp).T @ w_arm + [0.0, qd_inp, 0.0]
        h.set_pose(R_chs=R, q_bm1=q_bm1, q_arm=q_arm, q_inp=q_inp, q_tilt=q_tilt,
                   rates={"chs": w_chs, "bm1": w_bm1, "arm": w_arm, "bkt": w_inp})
        h.tick(400)                                         # joint-angle LPF settles
        self.assertEqual(fw["y.chs.euAngSeq"], 312)
        np.testing.assert_allclose(fw["y.chs.euAng"], _eul312(R), atol=2e-6)
        np.testing.assert_allclose(np.array(fw["y.chs.R"]).reshape(3, 3).T, R, atol=2e-6)   # column-major
        np.testing.assert_allclose(fw["y.chs.angVel"], w_chs, atol=1e-6)
        for path, want in (("BmMntToBm1.q", q_bm1), ("Bm2ToArm.q", q_arm), ("ArmToInpLink.q", q_inp),
                           ("TiltMntToTilt.q", q_tilt), ("BmMntToBm1.qDot", qd_bm1),
                           ("Bm2ToArm.qDot", qd_arm), ("ArmToInpLink.qDot", qd_inp)):
            self.assertAlmostEqual(fw["y.jnts." + path], want, delta=1e-5, msg=path)
        self.assertAlmostEqual(fw["y.jnts.ArmToOutpLink.q"], kin.fourbar_output(fw, q_inp), delta=1e-5)

    def test_fourbar_output_matches_the_firmware_and_none_marks_the_non_closing_region(self):
        # WHY: link_frames() places the tilt IMU through fourbar_output(); a wrong branch would
        # publish the tilt link on the other assembly mode. The firmware solves the four-bar from
        # the RAW input angle and only then low-pass filters every joint angle (MdlApp.c:12410-12425,
        # 3 Hz), so comparing against y.jnts.ArmToInpLink.q needs the filter settled per sample.
        # Non-closing region (det < 0): the firmware zeroes angOutpLink RELATIVE to the ground
        # link, so y.jnts.ArmToOutpLink.q reads angArmToGndLink (0.0716 rad), not 0 -- the
        # "firmware would output 0" wording in kinematics.py:136-137 is loose (reported as a doc
        # issue; the None return itself is fine). The compiled geometry is Grashof double-crank
        # and always closes, so the region is produced by patching lenConnRod.
        # Citation: MdlApp.c:11145-11200 CalcAngLinkOutp (det < 0 -> angOutpLink = 0 at :11178-11182),
        # :11789-11808 (+ angArmToGndLink, WrapToPi); kinematics.py:134-148.
        h = Harness().reset().nominal_inputs()
        fw = h.fw
        R_bm1 = kin.Ry(math.radians(-40))
        R_arm = R_bm1 @ kin.Ry(math.radians(90))
        worst = 0.0
        for deg in range(-180, 180, 15):
            kin.publish_imus(fw, {"chs": np.eye(3), "bm1": R_bm1, "arm": R_arm,
                                  "bkt": R_arm @ kin.Ry(math.radians(deg))})
            h.tick(300)
            fin = fw["y.jnts.ArmToInpLink.q"]
            self.assertAlmostEqual(_wrap(fin - math.radians(deg)), 0.0, delta=1e-5)
            out = kin.fourbar_output(fw, fin)
            self.assertIsNotNone(out, deg)
            worst = max(worst, abs(_wrap(out - fw["y.jnts.ArmToOutpLink.q"])))
        self.assertLess(worst, 3e-5)          # float32 Freudenstein terms: 1.3e-5 rad worst, at -105 deg
        # non-closing region
        fw["par.parKin.lenConnRod"] = 0.05
        q = math.radians(-5)
        self.assertIsNone(kin.fourbar_output(fw, q))
        kin.publish_imus(fw, {"chs": np.eye(3), "bm1": R_bm1, "arm": R_arm, "bkt": R_arm @ kin.Ry(q)})
        h.tick(300)
        self.assertIsNone(kin.fourbar_output(fw, fw["y.jnts.ArmToInpLink.q"]))
        self.assertAlmostEqual(fw["y.jnts.ArmToOutpLink.q"], fw["par.parKin.angArmToGndLink"], delta=1e-6)  # LPF-settled
        self.assertGreater(fw["par.parKin.angArmToGndLink"], 0.07)
        with self.assertRaises(ValueError):
            h.set_pose(q_bm1=math.radians(-40), q_arm=math.radians(90), q_inp=q)

    def test_mount_files_parse_and_identify_the_loaded_set(self):
        # WHY: mounts are per-unit calibration; running a LongArm URDF on ShortArm mounts misreads
        # arm/link by degrees with no fault. The parser must read both files as rotations and
        # identify_mount_source must name the set actually in par.* -- before and after
        # load_imu_mounts(), and again after reset().
        # Citation: SysPar.m:4 (compiled set = ECR88D_ShortArm); ControlModel/Data/ECR88D_*.m imu*
        # blocks; kinematics.py:92-130; harness.py:139-146.
        h = Harness().reset()
        fw = h.fw
        data = _x1exc() / "ControlModel" / "Data"
        for name in ("ECR88D_ShortArm.m", "ECR88D_LongArm.m"):
            mounts = kin.mounts_from_param_file(data / name)
            self.assertEqual(sorted(mounts), sorted(kin.MOUNT_NAMES))
            for mname, M in mounts.items():
                np.testing.assert_allclose(M @ M.T, np.eye(3), atol=1e-4, err_msg=f"{name}:{mname}")
                self.assertAlmostEqual(np.linalg.det(M), 1.0, delta=1e-4)
        self.assertEqual(kin.identify_mount_source(fw, data), "ECR88D_ShortArm.m")
        h.load_imu_mounts("ECR88D_LongArm.m")
        self.assertEqual(kin.identify_mount_source(fw, data), "ECR88D_LongArm.m")
        long_mounts = kin.mounts_from_param_file(data / "ECR88D_LongArm.m")
        for mname, M in kin.mounts_from_fw(fw).items():
            np.testing.assert_allclose(M, long_mounts[mname], atol=1e-7)
        h.reset()
        self.assertEqual(kin.identify_mount_source(fw, data), "ECR88D_ShortArm.m")


# ----------------------------------------------------------------------------------------
# sil/geodesy.py: world metres -> the geodetic inports
# ----------------------------------------------------------------------------------------
class TestGeodesyLibrary(unittest.TestCase):
    def test_krueger_tm_is_self_consistent_and_matches_the_meridian_arc(self):
        # WHY: place_antennas() feeds the firmware's Localization through KruegerTM; a wrong series
        # coefficient shows as a scale error that grows with distance from the site origin.
        # Independent checks: forward(inverse) round trip; northing on the central meridian equals
        # the numerically integrated meridian arc; point scale on the central meridian is k0.
        # Citation: geodesy.py:31-96 (Karney 2011 eqs. 35/36); firmware side checked in the
        # place_chassis test below. WGS84 a/1/f as written by Site.write (geodesy.py:110-114).
        lat0 = math.radians(32.9)
        tm = geo.KruegerTM(lat0, math.radians(-96.8))
        worst = 0.0
        for e, n in ((0.0, 0.0), (1234.5, -2345.6), (-50000.0, 80000.0), (300000.0, 1.0e6)):
            e2, n2 = tm.forward(*tm.inverse(e, n))
            worst = max(worst, abs(e2 - e), abs(n2 - n))
        self.assertLess(worst, 1e-6)
        a, esq = geo.WGS84_A, geo.WGS84_ESQ

        def arc(phi, steps=4000):
            f = lambda t: a * (1 - esq) / (1 - esq * math.sin(t) ** 2) ** 1.5
            hstep = phi / steps
            s = f(0) + f(phi) + sum((4 if i % 2 else 2) * f(i * hstep) for i in range(1, steps))
            return s * hstep / 3
        for lat_deg in (30.0, 32.9, 40.0):
            lat = math.radians(lat_deg)
            e, n = tm.forward(lat, tm.lon0)
            self.assertAlmostEqual(e, 0.0, delta=1e-9)
            self.assertAlmostEqual(n, arc(lat) - arc(lat0), delta=1e-4)
        lat, d = math.radians(33.2), 1e-7
        de = (tm.forward(lat, tm.lon0 + d)[0] - tm.forward(lat, tm.lon0 - d)[0]) / (2 * d)
        N = a / math.sqrt(1 - esq * math.sin(lat) ** 2)
        self.assertAlmostEqual(de / (N * math.cos(lat)), 1.0, delta=1e-6)

    def test_place_chassis_reproduces_position_and_attitude_with_a_false_origin(self):
        # WHY: place_chassis/place_antennas define the GNSS half of the Isaac plant. The firmware
        # must return the chassis origin (y.links.chs.p), attitude (y.links.chs.R) and heading
        # that were placed, for a tilted and yawed chassis, with and without a false origin and a
        # siteOrigin offset, near the origin and kilometres away. Far away the firmware's own
        # real32_T local coordinates set the resolution: enh_LocalMain/Aux (MdlApp.h:315-316) are
        # float32 steps of 0.25 mm at 3.5 km, which over the 1.18 m antenna baseline is up to
        # ~4e-4 rad of heading (4.2e-5 rad measured). Negative control: mirroring the aux antenna's
        # Y offset (a wrong chassis-axis convention) must visibly move heading and position.
        # Citation: links.chs.p = mainAnt + R_chs*distAntMainToChs (spec A3); Localization inports
        # MdlApp.c:41996-42020; heading = pi/2 - euAng_ChsEstm_z, :13422-13442, baseline angle
        # :11768-11772; geodesy.py:132-153.
        R = kin.Rz(-0.7) @ kin.Ry(-0.05) @ kin.Rx(0.06)
        sites = (geo.Site(), geo.Site(false_e=500000.0, false_n=-3000.0, site_origin=[500123.0, -2500.0, 150.0]),
                 geo.Site(lat0_deg=37.5, lon0_deg=127.0, h0=40.0))
        for org, tol_p, tol_ang in (((12.3, -45.6, 1.25), 2e-5, 2e-6), ((2345.6, -3456.7, 12.5), 5e-4, 5e-4)):
            for i, site in enumerate(sites):
                with self.subTest(site=i, org=org):
                    h = Harness().reset().nominal_inputs()
                    h.set_pose(R_chs=R, **Harness.NOMINAL_POSE)
                    h.gnss_rtk_fixed()
                    h.gnss_site(site)
                    h.place_chassis(org, R)
                    h.tick(300)
                    np.testing.assert_allclose(h.fw["y.links.chs.p"], org, atol=tol_p)
                    np.testing.assert_allclose(np.array(h.fw["y.links.chs.R"]).reshape(3, 3).T, R, atol=tol_ang)
                    self.assertAlmostEqual(_wrap(h.fw["y.machHeading"] - (math.pi / 2 - _yaw312(R))), 0.0,
                                           delta=tol_ang)
        # negative control
        org = np.array([12.3, -45.6, 1.25])
        h = Harness().reset().nominal_inputs()
        h.set_pose(R_chs=R, **Harness.NOMINAL_POSE)
        h.gnss_rtk_fixed()
        site = h.gnss_site()
        main = geo.main_antenna_for_chassis(h.fw, org, R)
        d = np.asarray(h.fw["par.parKin.distAntMainToAntAux"], dtype=float) * [1.0, -1.0, 1.0]
        h.fw["u.blh_Main"] = site.blh(main)
        h.fw["u.blh_Aux"] = site.blh(main + R @ d)
        h.tick(300)
        self.assertGreater(abs(_wrap(h.fw["y.machHeading"] - (math.pi / 2 - _yaw312(R)))), math.radians(10))
        self.assertGreater(np.abs(np.array(h.fw["y.links.chs.p"]) - org).max(), 0.1)

    def test_machheading_is_pi_over_2_minus_the_312_yaw_not_the_azimuth_of_chassis_x(self):
        # WHY: a plant or scenario that computes the expected heading as "the clockwise-from-North
        # azimuth of chassis +X" (geodesy.py:17-18) is off by 0.46 deg at roll -4.6 / pitch 5.7 deg.
        # What the firmware reports is wrap(pi/2 - psi) with psi the 312 yaw (atan2(-R01, R11), the
        # azimuth of chassis +Y minus 90 deg), which equals the +X azimuth only for a level chassis.
        # Reported as a docstring issue; no library function computes heading.
        # Citation: machHeading = WrapToPi(pi/2 - euAng_ChsEstm_z) MdlApp.c:13422-13442, where
        # euAng_ChsEstm_z is the tilt-compensated antenna-baseline angle (:11768-11772) and matches
        # y.chs.euAng[2] of the 312 sequence (y.chs.euAngSeq, :11099-11100); geodesy.py:17-18.
        R = kin.Rz(1.2) @ kin.Ry(0.1) @ kin.Rx(-0.08)
        h = Harness().reset().nominal_inputs()
        h.set_pose(R_chs=R, **Harness.NOMINAL_POSE)
        h.gnss_rtk_fixed()
        h.gnss_site()
        h.place_chassis((0.0, 0.0, 0.0), R)
        h.tick(300)
        heading = h.fw["y.machHeading"]
        self.assertAlmostEqual(h.fw["y.chs.euAng"][2], _yaw312(R), delta=2e-6)
        self.assertAlmostEqual(_wrap(heading - (math.pi / 2 - _yaw312(R))), 0.0, delta=2e-6)
        az_x = math.atan2(R[1, 0], R[0, 0])                 # == 1.2 exactly for Rz Ry Rx
        self.assertGreater(abs(_wrap(heading - (math.pi / 2 - az_x))), math.radians(0.4))


if __name__ == "__main__":
    unittest.main()
