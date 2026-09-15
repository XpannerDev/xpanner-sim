#!/usr/bin/env python3
"""
sil_isaac_prepare_pick.py -- the firmware MOVES the machine in Isaac: Picking / PreparePick drives boom, arm
and link in joint space through the valve model (sil.isaac_plant.IsaacPlant) until PreparePoseCtrlTol holds for
0.3 s and the sub-step advances to ApproachPanel; ApproachPanel (task space) then runs for --approach-s with no
panel or cups in the scene; finally the cycle is paused and the firmware's belief is compared with Isaac at rest.

Report: ApproachPanel reached and when; stroke errors against the firmware's own targets (debug outports
P11/P13/P15, as sil/tests/test_valve_plant.py); firmware joint estimate vs Isaac while moving; tool position
firmware vs Isaac at rest.

    docker exec isaac-sim-jude /isaac-sim/python.sh /work/xpanner-sim/scripts/sil_isaac_prepare_pick.py \
        --usd /work/xpanner-sim/build/isaac/ecr88_kijang_step28.usd --report /work/xpanner-sim/build/isaac/prepare_pick_report.json
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
    ap.add_argument("--timeout-s", type=float, default=40.0)
    ap.add_argument("--approach-s", type=float, default=15.0,
                    help="keep running ApproachPanel (task space, no cups in the scene) this long and record it")
    args = ap.parse_args()

    from sil.isaac_scene import IsaacScene
    scene = IsaacScene(args.usd)
    from sil import valves as vlv

    trace = []
    plant_ref = {}

    def tracer(h):
        p = plant_ref.get("p")
        if p is not None and h.tick_count % 10 == 0 and p.isaac_q:
            trace.append(dict(tick=h.tick_count, sub=h.picking_step(),
                              isaac={k: math.degrees(v) for k, v in p.q.items()},
                              firmware=scene.firmware_joints_deg(h.fw), valves=h.valves()))

    h, plant = scene.boot(q0=dict(boom=-50.0, arm=70.0, input_link=-90.0), extra=[tracer])
    plant_ref["p"] = plant
    h.set_target_panel(panel_id=7)
    h.tick(3)
    h.request_step("Standby")
    h.run_until(lambda h: h.curr_step() == "Standby", 0.5, "Standby")
    h.tick(90)
    h.jump_to_step("Picking")
    start_sub = h.picking_step()
    start_q = dict(plant.q)
    reached, err = None, None
    try:
        reached = h.run_until(lambda h: h.picking_step() == "PickingStep_ApproachPanel", args.timeout_s, "ApproachPanel")
    except Exception as e:                  # StepTimeout: report it instead of dying
        err = f"{type(e).__name__}: {e}"
    fw = h.fw
    tol = {"boom": 2.0, "arm": 1.0, "input_link": 1.0}
    # --- at the tick PreparePick completed ---
    strokes = plant.strokes()
    stroke_err = {}
    for axis, p in (("boom", "P11"), ("arm", "P13"), ("input_link", "P15")):
        e = abs(strokes[axis] - fw[f"y.dbg_F64_{p}"]) / abs(vlv.stroke_jacobian(plant.cyls[axis], plant.q[axis]))
        stroke_err[axis] = math.degrees(e)
    fwq = scene.firmware_joints_deg(fw)
    joint_diff = {k: fwq[k] - math.degrees(plant.q[k]) for k in ("boom", "arm", "input_link", "tilt", "rotator")}
    moved = {k: math.degrees(plant.q[k] - start_q[k]) for k in ("boom", "arm", "input_link", "rotator")}
    track = [max(abs(r["isaac"][k] - r["firmware"][k]) for k in ("boom", "arm", "input_link", "tilt")) for r in trace[5:]]
    cs_fw, cs_is = scene.tool_in_chassis(fw)
    # --- ApproachPanel: task space, no panel or cups in the scene, so it never completes ---
    approach = []
    if reached is not None and args.approach_s > 0:
        for _ in range(int(args.approach_s / 0.5)):
            h.run_seconds(0.5)
            a_fw, a_is = scene.tool_in_chassis(fw)
            approach.append(dict(t=h.tick_count, state=h.main_state(), sub=h.picking_step(),
                                 firmware_m=a_fw.round(4).tolist(), isaac_m=a_is.round(4).tolist(),
                                 diff_mm=(1000 * (a_fw - a_is)).round(2).tolist(), valves=h.valves()))
    static = scene.static_check(h, plant) if reached is not None else None
    report = dict(
        start_substep=start_sub, reached_tick=reached, error=err, timeout_s=args.timeout_s,
        settle_drift_deg=scene.settle_drift_deg,
        usd_joint_friction_before_zeroing=plant.friction_before,
        moved_deg=moved, stroke_error_deg=stroke_err, stroke_tolerance_deg=tol,
        joint_diff_firmware_minus_isaac_at_reach_deg=joint_diff,
        max_tracking_diff_deg=max(track) if track else None,
        contact_surface_in_chassis_at_reach_m=dict(firmware=cs_fw.tolist(), isaac=cs_is.tolist(), diff_mm=(1000 * (cs_fw - cs_is)).tolist()),
        approach_panel_every_0p5s=approach, paused_static=static,
        final_state=h.describe(), trace_every_10_ticks=trace[::5],
    )
    json.dump(report, open(args.report, "w"), indent=1)
    for a in approach:
        print("APPROACH", json.dumps({k: a[k] for k in ("t", "sub", "firmware_m", "diff_mm", "valves")}))
    scene.close("SILPP " + json.dumps({k: report[k] for k in (
        "start_substep", "reached_tick", "error", "settle_drift_deg", "usd_joint_friction_before_zeroing", "moved_deg", "stroke_error_deg",
        "joint_diff_firmware_minus_isaac_at_reach_deg", "max_tracking_diff_deg",
        "contact_surface_in_chassis_at_reach_m", "paused_static")}))


if __name__ == "__main__":
    main()
