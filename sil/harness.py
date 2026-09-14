"""
harness.py -- 10 ms lockstep driver around Firmware, with the input idioms the
X1Exc state machines actually respond to.

    h = Harness()
    h.reset()
    h.nominal_inputs()                  # healthy-machine defaults, see below
    h.pulse("u.isSwingAligned")         # rising edge latches swing init
    h.request_step("Standby")           # autoReqStep must CHANGE to fire
    h.run_until(lambda h: h.curr_step() == "Standby", timeout_s=0.5)

WHY PULSE AND REQUEST_STEP EXIST (resources/X1Exc_SIL_spec.md A6)
    The main chart uses hasChanged(): true for exactly one tick after a value changes.
    Holding jstAutoReq_StartPause high produces no further edges, and writing the same
    autoReqStep twice is a no-op. Harness methods encode that so tests cannot
    accidentally rely on a held level.

ONE-TICK SKEW
    Every derived flag (isTarPanelValid, isTarActuatorReached, pickingStep, ...) reaches
    the main chart through a unit delay. run_until() therefore polls AFTER each step and
    reports the tick at which the condition first held; do not assert same-tick causality.

PLANT HOOK
    plant(h) is called before every MdlApp_step() to fill ExtU. Isaac Sim will be one; a
    test can pass a lambda. Without one, inputs hold whatever was last written.
"""
import csv

from .firmware import Firmware

DT = 0.01
_fw_singleton = None


def firmware():
    global _fw_singleton
    if _fw_singleton is None:
        _fw_singleton = Firmware()
    return _fw_singleton


class StepTimeout(AssertionError):
    pass


class Harness:
    def __init__(self, fw=None, plant=None):
        self.fw = fw or firmware()
        self.plant = plant
        self.tick_count = 0
        self.trace_paths = []
        self.trace_rows = []

    # -- lifecycle -------------------------------------------------------------------
    def reset(self):
        self.fw.reset()
        self.tick_count = 0
        self.trace_rows = []
        return self

    def nominal_inputs(self):
        """Healthy-machine inputs from spec A3.4/A3.7. Writes only; call after reset().
        Deliberately does NOT set isSwingAligned or any GNSS field: those are the gates
        a scenario is usually testing, so each test decides them explicitly."""
        fw = self.fw
        fw["u.isRmtOk"] = 1
        # ECU interface parameters, not measurements: AppCtrlIf.c:651-652 copies them from
        # INTP. They are INPORTS, so after initialize() they are 0 and "stdDevZ < 0" can
        # never be true -- GNSS would read as poor forever. Model defaults, SysPar.m:59-60.
        fw["u.verticalAccuracyGoodThld"] = 0.02
        fw["u.verticalAccuracyPoorThld"] = 0.04
        for n in ("chs", "bm1", "arm", "bkt", "tilt"):
            fw[f"u.{n}ImuQuat"] = [1.0, 0.0, 0.0, 0.0]   # identity, not all-zero (A2)
        fw["u.bm2ImuQuat"] = [0.0, 0.0, 0.0, 0.0]          # the real ECU sends zeros
        for p in self.fw.paths("u."):
            if p.startswith(("u.isChsImuFault", "u.isBm1ImuFault", "u.isArmImuFault",
                             "u.isBktImuFault", "u.isTiltImuFault", "u.isBm2ImuFault")):
                fw[p] = 0
        return self

    def gnss_rtk_fixed(self, std_dev_z=0.008):
        """Both antennas RTK fixed (methodGnss == 4) with a good vertical sigma. Position
        (blh_*) is left alone: the accuracy gate does not read it (chart_2496)."""
        self.fw["u.methodGnss_Main"] = 4
        self.fw["u.methodGnss_Aux"] = 4
        self.fw["u.gnssPosStdDevZ"] = std_dev_z
        return self

    # -- stepping --------------------------------------------------------------------
    def tick(self, n=1):
        for _ in range(n):
            if self.plant:
                self.plant(self)
            self.fw.step()
            self.tick_count += 1
            if self.trace_paths:
                self.trace_rows.append([self.tick_count] + [self._flat(self.fw[p]) for p in self.trace_paths])
        return self

    def run_seconds(self, s):
        return self.tick(int(round(s / DT)))

    def run_until(self, pred, timeout_s, what=None):
        limit = self.tick_count + int(round(timeout_s / DT))
        while self.tick_count < limit:
            self.tick()
            if pred(self):
                return self.tick_count
        raise StepTimeout(f"timed out after {timeout_s} s waiting for {what or pred}; "
                          f"state: {self.describe()}")

    # -- input idioms ----------------------------------------------------------------
    def pulse(self, path, ticks=1):
        """0 -> 1 for `ticks`, then back to 0. Guarantees a rising edge even if the
        input was left high."""
        if self.fw[path]:
            self.fw[path] = 0
            self.tick()
        self.fw[path] = 1
        self.tick(ticks)
        self.fw[path] = 0
        return self

    def request_step(self, step):
        """Write autoReqStep so hasChanged() fires. Accepts an AutoCtrlStep name or int."""
        val = step if isinstance(step, int) else self.fw.enum_value("AutoCtrlStep", step)
        if self.fw["u.autoReqStep"] == val:
            self.fw["u.autoReqStep"] = 0 if val else 1
            self.tick()
        self.fw["u.autoReqStep"] = val
        return self

    def set_target_panel(self, panel_id=1, north=0.0, east=4.35, hgt=0.0,
                         row1=((-5.0, 4.35), (5.0, 4.35)), row2=None, hgt_offs_release=0.0):
        """Write a non-degenerate tarPanelData. Rows are ((E0,N0),(E1,N1)) site metres.
        row2=None marks row 2 invalid (id 255)."""
        fw = self.fw
        fw["u.tarPanelData.id"] = panel_id
        fw["u.tarPanelData.relNorthing"] = north
        fw["u.tarPanelData.relEasting"] = east
        fw["u.tarPanelData.relHgt"] = hgt
        fw["u.tarPanelData.hgtOffsRelease"] = hgt_offs_release
        fw["u.tarPanelData.row1Id"] = 1
        (e0, n0), (e1, n1) = row1
        fw["u.tarPanelData.row1StartRelEasting"], fw["u.tarPanelData.row1StartRelNorthing"] = e0, n0
        fw["u.tarPanelData.row1EndRelEasting"], fw["u.tarPanelData.row1EndRelNorthing"] = e1, n1
        if row2 is None:
            fw["u.tarPanelData.row2Id"] = 255
        else:
            fw["u.tarPanelData.row2Id"] = 2
            (e0, n0), (e1, n1) = row2
            fw["u.tarPanelData.row2StartRelEasting"], fw["u.tarPanelData.row2StartRelNorthing"] = e0, n0
            fw["u.tarPanelData.row2EndRelEasting"], fw["u.tarPanelData.row2EndRelNorthing"] = e1, n1
        return self

    # -- observation -----------------------------------------------------------------
    def curr_step(self):
        return self.fw.enum_name("AutoCtrlStep", self.fw["y.autoCtrl_CurrStep"])

    def calib_step(self):
        return self.fw.enum_name("CalibStep", self.fw["y.calibStep"])

    def inhibit_status(self):
        return self.fw["y.autoCtrl_InhibitSts"]

    def auto_inhibited(self):
        """y.autoCtrl_InhibitSts bit 1. NOT poor accuracy: InhibitStsMgr overwrites bit 1
        of the OUTPUT word with isAutoCtrlInhibited = (status & AUTO_INHIBIT_MASK) != 0,
        source comment "Temporary solution to show inhibit status on the tablet"
        (MdlApp.c:39688-39706). The real POOR_ACCURACY bit is only inside the chart."""
        return bool(self.inhibit_status() & 0x2)

    def inhibit_names(self, status=None):
        """Decode y.autoCtrl_InhibitSts. Bit 1 is reported as AUTO_INHIBITED, see above."""
        status = self.inhibit_status() if status is None else status
        names = []
        for n, b in sorted(self.fw.inhibit_bits.items(), key=lambda kv: kv[1]):
            if status >> b & 1:
                names.append("AUTO_INHIBITED(any)" if b == 1 else n)
        return names

    def describe(self):
        return (f"tick={self.tick_count} step={self.curr_step()} "
                f"startStop={self.fw['y.autoCtrl_StartStopSts']} calibrating={self.fw['y.isCalibrating']} "
                f"calibStep={self.calib_step()} inhibit=0x{self.inhibit_status():04X} {self.inhibit_names()}")

    # -- trace -----------------------------------------------------------------------
    def trace(self, *paths):
        self.trace_paths = list(paths)
        return self

    @staticmethod
    def _flat(v):
        return " ".join(f"{x:g}" if isinstance(x, float) else str(x) for x in v) if isinstance(v, list) else v

    def save_trace(self, path):
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["tick"] + self.trace_paths)
            w.writerows(self.trace_rows)
