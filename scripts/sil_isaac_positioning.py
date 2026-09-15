#!/usr/bin/env python3
"""
sil_isaac_positioning.py -- Positioning in Isaac: the first scenario where the firmware swings the house, so
inertia matters (the kinematic plant has none). Mirrors sil/tests/test_valve_plant.py
test_positioning_raise_swing_and_align_converge:

  boot at swing 0 (the swing switch latches) -> the operator's remote lever swings the house ~13 deg off square
  -> Standby -> Auto -> Raise (task space: tool to 0.8 m above the chassis) -> Swing (swing, tilt, rotator in
  joint space to 0) -> Align (travel; never completes: the undercarriage does not move) -> pause -> static check.

Report: first tick of each positioning step; tool height at Swing entry; swing trajectory with overshoot and
settling; firmware swing estimate vs Isaac; tilt/rotator end errors; tool position at rest.

    docker exec isaac-sim-jude /isaac-sim/python.sh /work/xpanner-sim/scripts/sil_isaac_positioning.py \
        --usd /work/xpanner-sim/build/isaac/ecr88_kijang_step28.usd --report /work/xpanner-sim/build/isaac/positioning_report.json
"""
import argparse
import json
import math
import sys

sys.path.insert(0, "/work/xpanner-sim")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--usd", required=True)
    ap.add_argument("--report", required=True)
    ap.add_argument("--budget-s", type=float, default=40.0)
    ap.add_argument("--lever-pct", type=float, default=-30.0)
    ap.add_argument("--lever-ticks", type=int, default=300)
    ap.add_argument("--align-dwell-s", type=float, default=20.0)
    args = ap.parse_args()

    from sil.isaac_scene import IsaacScene
    scene = IsaacScene(args.usd)

    import numpy as np
    h, plant = scene.boot(q0=dict(tilt=6.0, rotator=-10.0))
    boot_aligned = int(h.fw["u.isSwingAligned"])

    # the operator's remote lever (a real firmware path: u.rmtLvrDmd.swing -> propVlvCmd.swingLe)
    h.fw["u.rmtLvrDmd.swing"] = args.lever_pct
    h.tick(args.lever_ticks)
    h.fw["u.rmtLvrDmd.swing"] = 0.0
    lever_release_swing = math.degrees(plant.q["swing"])
    h.tick(100)
    offset = math.degrees(plant.q["swing"])
    coast = offset - lever_release_swing

    h.set_target_panel(panel_id=7)
    h.tick(3)
    h.request_step("Standby")
    h.run_until(lambda h: h.curr_step() == "Standby", 0.5, "Standby")
    h.tick(90)
    h.pulse("u.jstAutoReq_StartPause")

    first, cs_z, rows = {}, None, []
    align_until = None
    for _ in range(int(args.budget_s / 0.01)):
        h.tick()
        step = h.positioning_step()
        if step not in first:
            first[step] = h.tick_count
            if step == "PositioningStep_Swing":
                cs_z = float(scene.tool_in_chassis(h.fw)[0][2])
            if step == "PositioningStep_Align":
                align_until = h.tick_count + int(args.align_dwell_s / 0.01)
        if h.tick_count % 5 == 0:
            fq = scene.firmware_joints_deg(h.fw)
            v = h.valves()
            rows.append(dict(t=h.tick_count, step=step,
                             swing=math.degrees(plant.q["swing"]), swing_fw=fq["swing"],
                             swing_rate=math.degrees(plant.qdot["swing"]),
                             tilt=math.degrees(plant.q["tilt"]), rot=math.degrees(plant.q["rotator"]),
                             swingLe=v.get("swingLe", 0.0), swingRi=v.get("swingRi", 0.0)))
        if align_until is not None and h.tick_count >= align_until:
            break

    swing_rows = [r for r in rows if "PositioningStep_Swing" in first and r["t"] >= first["PositioningStep_Swing"]]
    sw = np.array([r["swing"] for r in swing_rows]) if swing_rows else np.array([])
    # overshoot = how far past 0 the house went, on the side opposite to where it started
    overshoot = float(max(0.0, -sw.min())) if (sw.size and offset > 0) else (float(max(0.0, sw.max())) if sw.size else None)
    end = dict(swing=math.degrees(plant.q["swing"]), tilt=math.degrees(plant.q["tilt"]), rotator=math.degrees(plant.q["rotator"]))
    diff = max((abs(r["swing"] - r["swing_fw"]) for r in rows[4:]), default=None)
    static = scene.static_check(h, plant)
    report = dict(
        settle_drift_deg=scene.settle_drift_deg, boot_swing_aligned=boot_aligned,
        usd_joint_friction_before_zeroing=plant.friction_before,
        lever=dict(pct=args.lever_pct, ticks=args.lever_ticks, swing_at_release_deg=lever_release_swing,
                   coast_after_release_deg=coast, offset_deg=offset),
        first_tick=first, tool_z_in_chassis_at_swing_entry_m=cs_z,
        swing_to_align_ticks=(first.get("PositioningStep_Align", 0) - first.get("PositioningStep_Swing", 0)
                              if "PositioningStep_Align" in first else None),
        swing_overshoot_deg=overshoot, peak_swing_rate_deg_s=float(max((abs(r["swing_rate"]) for r in rows), default=0.0)),
        end_deg=end, max_swing_diff_firmware_vs_isaac_deg=diff,
        inhibit=h.fw["y.autoCtrl_InhibitSts"], paused_static=static, final_state=h.describe(), rows_every_50ms=rows,
    )
    json.dump(report, open(args.report, "w"), indent=1)
    for r in rows[::20]:
        print("ROW", json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()}))
    scene.close("SILPOS " + json.dumps({k: v for k, v in report.items() if k not in ("rows_every_50ms",)}))


if __name__ == "__main__":
    main()
