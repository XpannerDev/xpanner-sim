#!/usr/bin/env python3
"""
sil_isaac_prepare_pick.py -- second firmware-in-the-loop scenario in Isaac Sim, the first one where the
firmware MOVES the machine: Picking / PreparePick drives boom, arm and link in joint space through the
valve model (sil.isaac_plant.IsaacPlant) until PreparePoseCtrlTol holds for 0.3 s and the sub-step
advances to ApproachPanel.

Pass criteria (reported in --report): ApproachPanel reached within the timeout; the firmware's joint
estimate (from IMUs published off Isaac's joint state) tracks Isaac's joints while moving; the final
cylinder strokes are within the firmware's own tolerance of its stroke targets (debug outports P11/P13/P15,
the same check as sil/tests/test_valve_plant.py on the kinematic plant).

    docker exec isaac-sim-jude /isaac-sim/python.sh /work/xpanner-sim/scripts/sil_isaac_prepare_pick.py \
        --usd /work/xpanner-sim/build/isaac/ecr88_kijang_step28.usd --report /work/xpanner-sim/build/isaac/prepare_pick_report.json
"""
import argparse
import json
import math
import sys

REPO = "/work/xpanner-sim"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--usd", required=True)
    ap.add_argument("--report", required=True)
    ap.add_argument("--timeout-s", type=float, default=40.0)
    ap.add_argument("--approach-s", type=float, default=15.0,
                    help="keep running ApproachPanel (task space, no cups in the scene) this long and record it")
    args = ap.parse_args()

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    import numpy as np
    from pxr import Gf, UsdGeom
    import isaacsim.core.utils.stage as su
    from isaacsim.core.api import World
    from isaacsim.core.prims import Articulation
    from isaacsim.core.utils.stage import add_reference_to_stage

    sys.path.insert(0, REPO)
    from sil import valves as vlv
    from sil.harness import Harness
    from sil.isaac_plant import IsaacPlant
    from sil.plant import FW_JOINT, Hardware

    world = World(stage_units_in_meters=1.0, physics_dt=0.005, rendering_dt=0.01)
    world.scene.add_default_ground_plane()
    add_reference_to_stage(args.usd, "/World/ECR88")
    UsdGeom.Xformable(su.get_current_stage().GetPrimAtPath("/World/ECR88")).AddTranslateOp().Set(Gf.Vec3d(0, 0, 1.445))
    world.reset()
    art = Articulation("/World/ECR88")
    art.initialize()

    plant = IsaacPlant(world, art, q0=dict(boom=-50.0, arm=70.0, input_link=-90.0), degrees=True)
    trace = []
    body_names = list(art.body_names)

    def isaac_link(name):
        """(R, p) of an articulation link in world, from the articulation's own tensor view (see step 28)."""
        tf = art._physics_view.get_link_transforms()
        px, py, pz, qx, qy, qz, qw = np.asarray(tf.numpy() if hasattr(tf, "numpy") else tf, float)[0][body_names.index(name)]
        w, x, y, z = np.array([qw, qx, qy, qz]) / math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
        R = np.array([[w*w+x*x-y*y-z*z, 2*(x*y-w*z), 2*(w*y+x*z)], [2*(x*y+w*z), w*w-x*x+y*y-z*z, 2*(y*z-w*x)],
                      [2*(x*z-w*y), 2*(w*x+y*z), w*w-x*x-y*y+z*z]])
        return R, np.array([px, py, pz])

    def tool_in_chassis(fw):
        """contact surface in the chassis frame: firmware belief (y.links) and Isaac truth (house_link)."""
        R_chs = np.array(fw["y.links.chs.R"], float).reshape(3, 3, order="F")
        cs_fw = R_chs.T @ (np.array(fw["y.links.contactSurface.p"], float) - np.array(fw["y.links.chs.p"], float))
        R_h, p_h = isaac_link("house_link")
        cs_is = R_h.T @ (isaac_link("contact_surface_link")[1] - p_h)
        return cs_fw, cs_is

    def firmware_estimate(fw):
        """y.jnts.*.q in plant names and degrees (ChsToUc = -swing)."""
        return {k: math.degrees(-fw[f"y.jnts.{n}.q"] if k == "swing" else fw[f"y.jnts.{n}.q"]) for k, n in FW_JOINT.items()}

    def tracer(h):
        if h.tick_count % 10 == 0 and plant.isaac_q:
            trace.append(dict(tick=h.tick_count, sub=h.picking_step(),
                              isaac={k: math.degrees(v) for k, v in plant.q.items()},
                              firmware=firmware_estimate(h.fw), valves=h.valves()))

    h = Harness(plant=[plant, tracer]).reset()
    plant.set_hardware(Hardware.compiled(h.fw))
    plant.push_pose()
    plant.configure_drives()
    for _ in range(40):
        world.step(render=False)
    plant.sync_from_isaac()
    settled = dict(plant.q)
    h.nominal_inputs()
    h.gnss_rtk_fixed()
    h.tick(3)
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
    fwq = firmware_estimate(fw)
    joint_diff = {k: fwq[k] - math.degrees(plant.q[k]) for k in ("boom", "arm", "input_link", "tilt", "rotator")}
    moved = {k: math.degrees(plant.q[k] - start_q[k]) for k in ("boom", "arm", "input_link", "rotator")}
    track = [max(abs(r["isaac"][k] - r["firmware"][k]) for k in ("boom", "arm", "input_link", "tilt")) for r in trace[5:]]
    cs_fw, cs_is = tool_in_chassis(fw)
    # --- ApproachPanel: task space, no panel or cups in the scene, so it never completes ---
    approach = []
    if reached is not None and args.approach_s > 0:
        for _ in range(int(args.approach_s / 0.5)):
            h.run_seconds(0.5)
            a_fw, a_is = tool_in_chassis(fw)
            approach.append(dict(t=h.tick_count, state=h.main_state(), sub=h.picking_step(),
                                 firmware_m=a_fw.round(4).tolist(), isaac_m=a_is.round(4).tolist(),
                                 diff_mm=(1000 * (a_fw - a_is)).round(2).tolist(), valves=h.valves()))
    # --- pause, let the machine come to rest, compare statically (no estimator lag) ---
    static = None
    if reached is not None:
        h.pulse("u.jstAutoReq_StartPause")
        h.run_seconds(2.0)
        s_fw, s_is = tool_in_chassis(fw)
        fq = firmware_estimate(fw)
        static = dict(state=h.main_state(), valves=h.valves(),
                      joint_rates_deg_s={k: math.degrees(v) for k, v in plant.qdot.items()},
                      joint_diff_firmware_minus_isaac_deg={k: fq[k] - math.degrees(plant.q[k]) for k in ("swing", "boom", "arm", "input_link", "tilt", "rotator")},
                      contact_surface_in_chassis_m=dict(firmware=s_fw.tolist(), isaac=s_is.tolist(), diff_mm=(1000 * (s_fw - s_is)).tolist()))
    report = dict(
        start_substep=start_sub, reached_tick=reached, error=err, timeout_s=args.timeout_s,
        settle_drift_deg={k: math.degrees(settled[k] - v) for k, v in dict(boom=-50.0 * math.pi / 180, arm=70.0 * math.pi / 180,
                                                                            input_link=-90.0 * math.pi / 180).items()},
        moved_deg=moved, stroke_error_deg=stroke_err, stroke_tolerance_deg=tol,
        joint_diff_firmware_minus_isaac_at_reach_deg=joint_diff,
        max_tracking_diff_deg=max(track) if track else None,
        contact_surface_in_chassis_at_reach_m=dict(firmware=cs_fw.tolist(), isaac=cs_is.tolist(), diff_mm=(1000 * (cs_fw - cs_is)).tolist()),
        approach_panel_every_0p5s=approach, paused_static=static,
        final_state=h.describe(), trace_every_10_ticks=trace[::5],
    )
    json.dump(report, open(args.report, "w"), indent=1)
    print("SILPP " + json.dumps({k: report[k] for k in ("start_substep", "reached_tick", "error", "settle_drift_deg", "moved_deg", "stroke_error_deg",
                                                         "joint_diff_firmware_minus_isaac_at_reach_deg", "max_tracking_diff_deg",
                                                         "contact_surface_in_chassis_at_reach_m", "paused_static")}))
    for a in approach:
        print("APPROACH", json.dumps({k: a[k] for k in ("t", "sub", "firmware_m", "diff_mm", "valves")}))
    sys.stdout.flush()
    import os
    os._exit(0)


if __name__ == "__main__":
    main()
