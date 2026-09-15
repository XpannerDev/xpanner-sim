"""
harness.py -- 10 ms lockstep driver around Firmware, with the input idioms the X1Exc
state machines actually respond to.

    h = Harness().reset().nominal_inputs()   # healthy machine at a physical rest pose
    h.set_swing_aligned(True)                # proximity switch closed (edge latches swing init)
    h.gnss_rtk_fixed()
    h.set_target_panel(panel_id=7)
    h.tick(3)
    h.request_step("Standby")                # autoReqStep must CHANGE to fire
    h.run_until(lambda h: h.curr_step() == "Standby", timeout_s=0.5)

EDGE-DETECTED INPUTS (spec A6; test_core, test_main_cycle)
    The main chart uses hasChanged(): true for exactly one tick after the value the firmware
    SAMPLED changes. pulse() raises an input for n ticks and then lowers it AND ticks, so two
    pulses in a row are two edges (jst and rmt StartPause are OR-ed: back-to-back pulses on
    both would otherwise merge into one). request_step() compares against the value the
    firmware latched at the last step, not the unticked inport, and re-edges through 255 --
    a value no guard compares against -- never through a real step number.

ONE-TICK SKEW
    Derived flags reach the main chart through unit delays. run_until() checks the condition
    once BEFORE ticking (returns immediately if it already holds) and then after every tick.

PLANTS
    Harness(plant=f) or Harness(plant=[f, g]) -- each called as f(h) before every
    MdlApp_step(), in order, to fill ExtU. SaveHandshake below is one; Isaac Sim is another.

NOMINAL POSE
    nominal_inputs() publishes IMUs for a physically possible rest pose (level house, boom
    -40, arm 90, input link -60, tilt 0 deg, firmware convention). Identity quaternions are
    NOT a pose through the compiled mounts: they read boom +71, arm +165, tilt -107 deg with
    the tool about 1.5 m below the chassis origin.
"""
import csv
import math
from pathlib import Path

import numpy as np

from . import kinematics as kin
from .firmware import Firmware

DT = 0.01
RE_EDGE_STEP = 255          # autoReqStep is only compared against AutoCtrlStep values (<= 32)
_fw_singleton = None


def firmware():
    global _fw_singleton
    if _fw_singleton is None:
        _fw_singleton = Firmware()
    return _fw_singleton


class StepTimeout(AssertionError):
    pass


class SaveHandshake:
    """Emulates SaveMchCalibData(), X1Exc Asw/main.c:394-419, as a plant.

    main.c runs it after MdlApp_step() in the same task, so what it writes is first seen by
    the NEXT step -- exactly what a plant called before each step sees. On each rising edge of
    y.isCalibDataSaveReq the calibration outputs are snapshotted into `saved` (standing for
    SaveInternalParam + WriteToNVM). Without this plant every _save state waits out its 1.0 s
    fallback (CntCalib_save = 100)."""

    # Everything AppCtrlIf.c:801-1018 copies into NVM while isCalibrating, incl. the CalibRot
    # zero offset (AppCtrlIf.c:882).
    SNAPSHOT_PREFIXES = ("y.parKin.", "y.imuMntOri", "y.tblReqSpdToActCmd.", "y.jntAngRotZeroOffs")

    def __init__(self):
        self.prev_req = False
        self.was_saved = False
        self.saved = []            # (tick, {path: value})
        self.reloads = 0

    def __call__(self, h):
        fw = h.fw
        req = bool(fw["y.isCalibDataSaveReq"])
        if not req:
            fw["u.isCalibDataSaved"] = 0
        if req and not self.prev_req:
            self.saved.append((h.tick_count, {p: fw[p] for p in fw.paths("y.")
                                              if p.startswith(self.SNAPSHOT_PREFIXES)}))
            self.was_saved = True
        if not req and self.prev_req:
            self.reloads += 1
            self.was_saved = False
        fw["u.isCalibDataSaved"] = int(self.was_saved)
        self.prev_req = req


class Harness:
    def __init__(self, fw=None, plant=None):
        self.fw = fw or firmware()
        self.plants = [] if plant is None else (list(plant) if isinstance(plant, (list, tuple)) else [plant])
        self.tick_count = 0
        self.trace_paths = []
        self.trace_rows = []
        self._latched_req_step = 0

    # -- lifecycle -------------------------------------------------------------------
    def reset(self):
        self.fw.reset()
        self.tick_count = 0
        self.trace_rows = []
        self._latched_req_step = 0
        return self

    NOMINAL_POSE = dict(q_bm1=math.radians(-40), q_arm=math.radians(90),
                        q_inp=math.radians(-60), q_tilt=0.0)

    def nominal_inputs(self):
        """Healthy-machine inputs. Writes only; call after reset(). Deliberately does NOT set
        isSwingAligned, GNSS measurements or the site calibration: those are the gates a
        scenario usually tests, so each test decides them explicitly."""
        fw = self.fw
        fw["u.isRmtOk"] = 1
        # ECU interface parameters, not measurements: AppCtrlIf.c:651-652 copies them from
        # INTP. They are INPORTS, so after initialize() they are 0 and "stdDevZ < 0" can
        # never be true -- GNSS would read as poor forever. Model defaults, SysPar.m:59-60.
        fw["u.verticalAccuracyGoodThld"] = 0.02
        fw["u.verticalAccuracyPoorThld"] = 0.04
        self.set_pose(**self.NOMINAL_POSE)
        fw["u.bm2ImuQuat"] = [0.0, 0.0, 0.0, 0.0]          # the real ECU sends zeros
        for p in ("u.isChsImuFault", "u.isBm1ImuFault", "u.isArmImuFault", "u.isBktImuFault",
                  "u.isTiltImuFault", "u.isBm2ImuFault", "u.isJntAngTiltFault", "u.isJntAngRotFault"):
            fw[p] = 0
        return self

    # -- sensors -----------------------------------------------------------------------
    def set_pose(self, R_chs=None, q_bm1=0.0, q_arm=0.0, q_inp=0.0, q_tilt=0.0, rates=None, accel=True):
        """Publish the five IMUs for a machine pose (firmware joint convention, radians).
        rates: optional port -> link body angular velocity (link coordinates)."""
        frames = kin.link_frames(self.fw, R_chs, q_bm1, q_arm, q_inp, q_tilt)
        kin.publish_imus(self.fw, frames, rates=rates, accel=accel)
        return frames

    def load_imu_mounts(self, param_file):
        """Patch par.imu* from an X1Exc parameter file name ('ECR88D_LongArm.m') or path.
        Mounts are per-unit calibration: use the set that matches the URDF being simulated."""
        path = Path(param_file)
        if not path.is_absolute():
            path = Path(self.fw.manifest["x1exc_dir"]) / "ControlModel" / "Data" / path
        kin.write_mounts(self.fw, kin.mounts_from_param_file(path))
        return self

    def gnss_rtk_fixed(self, std_dev_z=0.008):
        """Both antennas RTK fixed (methodGnss == 4) with a good vertical sigma. Position
        (blh_*) is left alone: the accuracy gate does not read it (chart_2496)."""
        self.fw["u.methodGnss_Main"] = 4
        self.fw["u.methodGnss_Aux"] = 4
        self.fw["u.gnssPosStdDevZ"] = std_dev_z
        return self

    def gnss_site(self, site=None):
        """Write a bare-TM site calibration (see sil/geodesy.py) and keep it for place_chassis()."""
        from .geodesy import Site
        self.site = site or Site()
        self.site.write(self.fw)
        return self.site

    def place_chassis(self, chs_origin_world=(0.0, 0.0, 0.0), R_chs=None):
        """Antenna fixes for a chassis origin/attitude in site-world metres. Requires gnss_site()."""
        from .geodesy import main_antenna_for_chassis, place_antennas
        R = np.eye(3) if R_chs is None else np.asarray(R_chs, dtype=float)
        main = main_antenna_for_chassis(self.fw, chs_origin_world, R)
        place_antennas(self.fw, self.site, main, R)
        return main

    # -- stepping --------------------------------------------------------------------
    def tick(self, n=1):
        for _ in range(n):
            for plant in self.plants:
                plant(self)
            self._latched_req_step = self.fw["u.autoReqStep"]
            self.fw.step()
            self.tick_count += 1
            if self.trace_paths:
                self.trace_rows.append([self.tick_count] + [self._cell(self.fw[p]) for p in self.trace_paths])
        return self

    @staticmethod
    def ticks_for(s):
        """Seconds -> ticks, round half up (banker's rounding collapsed 0.015 and 0.025 to 2)."""
        return int(math.floor(s / DT + 0.5 + 1e-9))

    def run_seconds(self, s):
        return self.tick(self.ticks_for(s))

    def run_until(self, pred, timeout_s, what=None):
        if pred(self):
            return self.tick_count
        n = self.ticks_for(timeout_s)
        if n <= 0:
            raise ValueError(f"timeout_s={timeout_s} is less than one tick")
        limit = self.tick_count + n
        while self.tick_count < limit:
            self.tick()
            if pred(self):
                return self.tick_count
        raise StepTimeout(f"timed out after {timeout_s} s waiting for {what or pred}; "
                          f"state: {self.describe()}")

    # -- input idioms ----------------------------------------------------------------
    def pulse(self, path, ticks=1):
        """0 -> 1 for `ticks`, then 0 and one more tick, so the firmware samples the low level.
        Guarantees a rising edge even if the input was left high."""
        if self.fw[path]:
            self.fw[path] = 0
            self.tick()
        self.fw[path] = 1
        self.tick(ticks)
        self.fw[path] = 0
        self.tick()
        return self

    def set_swing_aligned(self, aligned=True):
        """Level write of the swing-alignment proximity switch. The boot latch needs a rising
        edge (a machine powered up aligned counts), but fork calibrations 28/29 gate on the
        LIVE level (MdlApp.c:39714): a pulse leaves the switch open and they never start."""
        self.fw["u.isSwingAligned"] = 1 if aligned else 0
        return self

    def request_step(self, step):
        """Write autoReqStep so hasChanged() fires. Accepts an AutoCtrlStep name or int."""
        val = step if isinstance(step, int) else self.fw.enum_value("AutoCtrlStep", step)
        if self._latched_req_step == val:
            self.fw["u.autoReqStep"] = RE_EDGE_STEP
            self.tick()
        self.fw["u.autoReqStep"] = val
        return self

    def jump_to_step(self, step, start=True):
        """The tablet step jump. For CYCLE steps (Positioning .. Releasing): from Standby or any
        <X>Paused / <X>Inhibited, changing autoReqStep lands in <X>Paused without that step's
        entry guard (chart_2537); a StartPause edge then runs it. CALIBRATION steps (20-29) are
        accepted ONLY from NoTarget: from Standby the request is dropped and the StartPause edge
        starts the PANEL CYCLE instead (test_calib_request_in_standby_then_auto_press_starts_the_
        panel_cycle). From a RUNNING state it does nothing at all -- no write,
        no tick: the firmware ignores the step change there, and the StartPause edge would
        PAUSE the run (MdlApp.c:18960) instead of starting anything. Pause first."""
        # A running calibration reports isCalibrating, not autoCtrl_StartStopSts.
        if self.is_running() or self.fw["y.isCalibrating"]:
            return self
        self.request_step(step)
        self.tick()
        if start:
            self.pulse("u.jstAutoReq_StartPause")
        return self

    def set_target_panel(self, panel_id=1, north=0.0, east=4.35, hgt=0.0,
                         row1=((-5.0, 4.35), (5.0, 4.35)), row2=None, hgt_offs_release=0.0):
        """Write a non-degenerate tarPanelData. Rows are ((E0,N0),(E1,N1)) site metres.
        row2=None marks row 2 invalid (id 255). id 0 and 0xFFFFFF mean "no target"."""
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

    def is_running(self):
        return bool(self.fw["y.autoCtrl_StartStopSts"])

    def main_state(self):
        """Active state of the main chart, e.g. 'PositioningPaused' vs 'PositioningInhibited'
        (they report the same autoCtrl_CurrStep and StartStopSts)."""
        return self.fw.chart_states["main"].get(self.fw.internal("main_chart_state"), "?")

    def calib_step(self):
        return self.fw.enum_name("CalibStep", self.fw["y.calibStep"])

    def positioning_step(self):
        return self.fw.enum_name("PositioningStep", self.fw.internal("positioning_step"))

    def picking_step(self):
        return self.fw.enum_name("PickingStep", self.fw.internal("picking_step"))

    def placing_step(self):
        return self.fw.enum_name("PlacingStep", self.fw.internal("placing_step"))

    def valves(self):
        """Non-zero y.propVlvCmd ports, {port: percent}."""
        out = {}
        for p in self.fw.paths("y.propVlvCmd."):
            v = self.fw[p]
            if v:
                out[p.rsplit(".", 1)[1]] = v
        return out

    def inhibit_status(self):
        return self.fw["y.autoCtrl_InhibitSts"]

    def inhibit_bit(self, name):
        """One bit of y.autoCtrl_InhibitSts by name ('BIT_PANEL_ATTACHED' or 'PANEL_ATTACHED').
        Bit 1 is the aggregate, see auto_inhibited()."""
        name = name if name.startswith("BIT_") else "BIT_" + name
        return bool(self.inhibit_status() >> self.fw.inhibit_bits[name] & 1)

    def auto_inhibited(self):
        """y.autoCtrl_InhibitSts bit 1. NOT poor accuracy: InhibitStsMgr overwrites bit 1
        of the OUTPUT word with isAutoCtrlInhibited = (status & AUTO_INHIBIT_MASK) != 0,
        source comment "Temporary solution to show inhibit status on the tablet"
        (MdlApp.c:39688-39706). The real accuracy state is fw.internal('low_vertical_accuracy')."""
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
        fw = self.fw
        chs = fw["y.links.chs.p"]
        return (f"tick={self.tick_count} step={self.curr_step()} state={self.main_state()} "
                f"running={self.is_running()} sub=({self.positioning_step()},{self.picking_step()},"
                f"{self.placing_step()}) calibrating={fw['y.isCalibrating']} calibStep={self.calib_step()} "
                f"inhibit=0x{self.inhibit_status():04X} {self.inhibit_names()} "
                f"chs.p=[{chs[0]:.3f},{chs[1]:.3f},{chs[2]:.3f}] heading={math.degrees(fw['y.machHeading']):.1f}deg "
                f"valves={self.valves()}")

    # -- trace -----------------------------------------------------------------------
    def trace(self, *paths):
        """Start (or restart) a trace. Existing rows are discarded so the CSV stays rectangular."""
        self.trace_paths = list(paths)
        self.trace_rows = []
        return self

    @staticmethod
    def _cell(v):
        if isinstance(v, list):
            return " ".join(repr(x) for x in v)
        return v

    def save_trace(self, path):
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["tick"] + self.trace_paths)
            w.writerows(self.trace_rows)
