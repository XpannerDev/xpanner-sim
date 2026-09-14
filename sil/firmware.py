"""
firmware.py -- ctypes access to the compiled X1Exc MdlApp (build with sil/build.py).

    fw = Firmware()                     # loads build/sil/libx1exc_sil.so
    fw.reset()                          # parLocalTest restored, MdlApp_initialize()
    fw["u.tarPanelData.id"] = 5
    fw.step()
    fw["y.autoCtrl_CurrStep"]           # -> 0
    fw.enum_name("AutoCtrlStep", fw["y.autoCtrl_CurrStep"])   # -> "NoTarget"

ONE FIRMWARE PER PROCESS. MdlApp keeps all of its state in C globals (MdlApp_U/Y/B/DW
and parLocalTest), so two Firmware objects in one process would be the same machine.
The constructor refuses a second instance. Run parallel scenarios as separate processes.

RESET IS MORE THAN MdlApp_initialize(). initialize() zeroes U, Y, B and DW but does NOT
touch parLocalTest, which is an ordinary writable global the harness is expected to
overwrite. reset() therefore restores the bytes parLocalTest had at load (its static
initializer, i.e. the compiled parameter set) before initializing, so a scenario that
patched a parameter cannot leak into the next one.
"""
import ctypes
import json
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
BUILD = REPO / "build" / "sil"

_CTYPES = {
    "b": ctypes.c_int8, "B": ctypes.c_uint8, "?": ctypes.c_uint8,
    "h": ctypes.c_int16, "H": ctypes.c_uint16,
    "i": ctypes.c_int32, "I": ctypes.c_uint32, "e": ctypes.c_int32,
    "f": ctypes.c_float, "d": ctypes.c_double,
}


class _Sig(ctypes.Structure):
    _fields_ = [("path", ctypes.c_char_p), ("base", ctypes.c_uint8), ("type", ctypes.c_uint8),
                ("count", ctypes.c_uint), ("offset", ctypes.c_ulong), ("elem_size", ctypes.c_uint)]


class Firmware:
    _instance_exists = False

    def __init__(self, build_dir=BUILD):
        if Firmware._instance_exists:
            raise RuntimeError("one Firmware per process: MdlApp state is C-global")
        build_dir = Path(build_dir)
        lib_path = build_dir / "libx1exc_sil.so"
        if not lib_path.is_file():
            raise FileNotFoundError(f"{lib_path} missing -- run: python3 sil/build.py")
        self.manifest = json.loads((build_dir / "manifest.json").read_text())
        self.lib = ctypes.CDLL(str(lib_path))
        self.lib.MdlApp_initialize.restype = None
        self.lib.MdlApp_step.restype = None
        self.lib.sil_base.restype = ctypes.c_void_p
        self.lib.sil_base.argtypes = [ctypes.c_uint]
        self.lib.sil_base_size.restype = ctypes.c_ulong
        self.lib.sil_base_size.argtypes = [ctypes.c_uint]

        n = ctypes.c_uint.in_dll(self.lib, "sil_signal_count").value
        table = (_Sig * n).in_dll(self.lib, "sil_signals")
        bases = [self.lib.sil_base(i) for i in range(3)]
        self._sigs = {}
        for s in table:
            ct = _CTYPES[chr(s.type)]
            if ctypes.sizeof(ct) != s.elem_size:
                raise RuntimeError(f"size mismatch for {s.path.decode()}")
            self._sigs[s.path.decode()] = (bases[s.base] + s.offset, ct, s.count, chr(s.type))

        par_size = self.lib.sil_base_size(2)
        self._par_addr = bases[2]
        self._par_initial = ctypes.string_at(self._par_addr, par_size)
        self.enums = self.manifest["enums"]
        self._enum_names = {e: {v: k for k, v in vals.items()} for e, vals in self.enums.items()}
        self.inhibit_bits = self.manifest["inhibit_bits"]
        Firmware._instance_exists = True

    # -- lifecycle -------------------------------------------------------------------
    def reset(self):
        ctypes.memmove(self._par_addr, self._par_initial, len(self._par_initial))
        self.lib.MdlApp_initialize()

    def step(self):
        self.lib.MdlApp_step()

    # -- signal access ---------------------------------------------------------------
    def paths(self, prefix=""):
        return sorted(p for p in self._sigs if p.startswith(prefix))

    def __contains__(self, path):
        return path in self._sigs

    def _lookup(self, path):
        try:
            return self._sigs[path]
        except KeyError:
            near = [p for p in self._sigs if path.split(".")[-1].lower() in p.lower()][:8]
            raise KeyError(f"{path!r} is not a firmware signal. Similar: {near}") from None

    def __getitem__(self, path):
        addr, ct, count, code = self._lookup(path)
        arr = (ct * count).from_address(addr)
        vals = [bool(v) if code == "?" else v for v in arr]
        return vals[0] if count == 1 else vals

    def __setitem__(self, path, value):
        addr, ct, count, code = self._lookup(path)
        arr = (ct * count).from_address(addr)
        if count == 1:
            value = [value]
        value = list(value)
        if len(value) != count:
            raise ValueError(f"{path} takes {count} values, got {len(value)}")
        for i, v in enumerate(value):
            arr[i] = int(v) if code in "bBhHiIe?" else float(v)

    # -- enums -----------------------------------------------------------------------
    def enum_value(self, enum, name):
        return self.enums[enum][name]

    def enum_name(self, enum, value):
        return self._enum_names[enum].get(value, f"<{enum} {value}>")
