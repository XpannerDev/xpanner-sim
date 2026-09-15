#!/usr/bin/env python3
"""
sil_isaac_step28.py -- first firmware-in-the-loop run: X1Exc calibration step 28
(CalibForkRefPose) with Isaac Sim as the plant.

WHAT IT PROVES
    The compiled firmware, fed ONLY simulated IMUs, reconstructs the machine's pose and the
    fork reference it records matches what Isaac's own ground truth says it should record.
    No valves, no GNSS, no dynamics are needed -- step 28 is stb (8 s) -> log (1 s) -> save of
    a pure forward-kinematics read (Stateflow chart_2352 UpdateForKin):
        R_forkBack_W  = contactSurface.R * Ry(-pi/2)
        R_forkBack_uc = uc.R' * R_forkBack_W
        angForkUpLimit   = -atan2(R_forkBack_uc(3,1), R_forkBack_uc(1,1))
        distUcToForkBack = [x; 0; z] of uc.R' * (contactSurface.p - uc.p)
    The oracle applies that same formula to Isaac's link poses. Any difference is the
    simulation and the firmware disagreeing about where the tool is.

HOW IT RUNS (inside the Isaac container; see sil/README.md for the host prep)
    /isaac-sim/python.sh /work/xpanner-sim/scripts/sil_isaac_step28.py \\
        --usd /work/xpanner-sim/build/isaac/ecr88_kijang_step28.usd \\
        --pose /work/xpanner-sim/build/isaac/step28_pose.json \\
        --report /work/xpanner-sim/build/isaac/step28_report.json

    Lockstep: every 10 ms firmware tick advances physics by 10 ms first (2 x 5 ms substeps),
    then the plant publishes the five IMUs from Isaac's link poses, then MdlApp_step().
    The 1.7 m variant is used because the repo's binary is compiled with ECR88D_ShortArm.m,
    whose IMU mounts that variant's URDF frames carry (sil/tests/test_imu_kinematics).

THE FOUR-BAR
    The URDF's input_link is a 1:1 mimic placeholder, not a closed loop; the firmware reads the
    INPUT link and solves the four-bar for the output link. So the bktImu attitude published is
    arm * Ry(q_inp) with q_inp = kinematics.fourbar_input(q_outp) of Isaac's OUTPUT link -- what
    the board would read on the real linkage. Publishing the placeholder's own attitude would
    make the firmware compute a tool attitude Isaac does not have.
"""
import argparse
import json
import math
import sys

REPO = "/work/xpanner-sim"
LINKS = ("base_link", "house_link", "boom_link", "arm_link", "output_link", "tilt_link",
         "contact_surface_link")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--usd", required=True)
    ap.add_argument("--pose", required=True, help="json swing/boom/arm/bucket in degrees")
    ap.add_argument("--report", required=True)
    ap.add_argument("--settle-s", type=float, default=3.0)
    ap.add_argument("--timeout-s", type=float, default=15.0)
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
    from sil import kinematics as kin
    from sil.harness import Harness, SaveHandshake

    pose = json.load(open(args.pose))
    world = World(stage_units_in_meters=1.0, physics_dt=0.005, rendering_dt=0.01)
    world.scene.add_default_ground_plane()
    add_reference_to_stage(args.usd, "/World/ECR88")
    stage = su.get_current_stage()
    UsdGeom.Xformable(stage.GetPrimAtPath("/World/ECR88")).AddTranslateOp().Set(Gf.Vec3d(0, 0, 1.445))
    world.reset()

    art = Articulation("/World/ECR88")
    art.initialize()
    names = list(art.dof_names)
    q0 = art.get_joint_positions()[0].copy()
    for j, key in (("swing_joint", "swing"), ("boom_joint", "boom"), ("arm_joint", "arm"), ("bucket_joint", "bucket")):
        q0[names.index(j)] = math.radians(pose[key])
    art.set_joint_positions(q0.reshape(1, -1))
    art.set_joint_position_targets(q0.reshape(1, -1))

    # Link poses come from the articulation's own tensor view. A separate RigidPrim view over
    # articulation links, created after world.reset(), invalidates the simulation view.
    body_names = list(art.body_names)
    missing = [k for k in LINKS if k not in body_names]
    if missing:
        raise SystemExit(f"links not in the articulation: {missing}; have {body_names}")
    link_idx = {k: body_names.index(k) for k in LINKS}

    def quat_wxyz_to_R(q):
        w, x, y, z = q
        return np.array([[w*w+x*x-y*y-z*z, 2*(x*y-w*z), 2*(w*y+x*z)],
                         [2*(x*y+w*z), w*w-x*x+y*y-z*z, 2*(y*z-w*x)],
                         [2*(x*z-w*y), 2*(w*x+y*z), w*w-x*x-y*y+z*z]])

    def truth():
        tf = art._physics_view.get_link_transforms()        # (count, links, 7): px py pz qx qy qz qw
        tf = np.asarray(tf.numpy() if hasattr(tf, "numpy") else tf, float)[0]
        out = {}
        for k, i in link_idx.items():
            px, py, pz, qx, qy, qz, qw = tf[i]
            q = np.array([qw, qx, qy, qz])
            out[k] = (quat_wxyz_to_R(q / np.linalg.norm(q)), np.array([px, py, pz]))
        return out

    for _ in range(int(args.settle_s / 0.005)):
        world.step(render=False)

    h = Harness()
    fw = h.fw
    state = {"prev": None, "ticks": 0}

    def isaac_plant(hh):
        world.step(render=False)
        world.step(render=False)
        T = truth()
        R_arm = T["arm_link"][0]
        R_out_rel = R_arm.T @ T["output_link"][0]
        q_outp = math.atan2(R_out_rel[0, 2], R_out_rel[0, 0])
        q_inp = kin.fourbar_input(fw, q_outp)
        frames = {"chs": T["house_link"][0], "bm1": T["boom_link"][0], "arm": R_arm,
                  "bkt": R_arm @ kin.Ry(q_inp), "tilt": T["tilt_link"][0]}
        rates = None
        if state["prev"] is not None:
            rates = {}
            for port, R in frames.items():
                dR = state["prev"][port].T @ R
                rates[port] = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]]) / (2 * 0.01)
        kin.publish_imus(fw, frames, rates=rates)
        state["prev"] = frames
        state["truth"] = T
        state["q_outp"], state["q_inp"] = q_outp, q_inp

    save = SaveHandshake()
    h.plants = [isaac_plant, save]
    h.reset().nominal_inputs()
    h.set_swing_aligned(True)
    h.run_seconds(1.0)
    boot = h.describe()

    h.jump_to_step("CalibForkRefPose")
    left = h.run_until(lambda hh: hh.calib_step() != "CalibStandby", 1.0, "calibration started")
    done = h.run_until(lambda hh: hh.calib_step() == "CalibStandby" and hh.curr_step() == "NoTarget",
                       args.timeout_s, "step 28 finished")

    # Oracle: chart_2352 applied to Isaac ground truth (uc = base_link; swing is 0 here).
    T = state["truth"]
    R_uc, p_uc = T["base_link"]
    R_cs, p_cs = T["contact_surface_link"]
    R_fb_uc = R_uc.T @ (R_cs @ kin.Ry(-math.pi / 2))
    ang_exp = -math.atan2(R_fb_uc[2, 0], R_fb_uc[0, 0])
    v = R_uc.T @ (p_cs - p_uc)
    dist_exp = [float(v[0]), 0.0, float(v[2])]

    snap = save.saved[-1][1] if save.saved else {}
    ang_fw = snap.get("y.parKin.angForkUpLimit", fw["y.parKin.angForkUpLimit"])
    dist_fw = snap.get("y.parKin.distUcToForkBack", fw["y.parKin.distUcToForkBack"])

    # Where the firmware believes the tool is, in its chassis frame, vs Isaac in the house frame.
    R_chs_fw = np.array(fw["y.links.chs.R"]).reshape(3, 3, order="F")
    cs_fw = R_chs_fw.T @ (np.array(fw["y.links.contactSurface.p"]) - np.array(fw["y.links.chs.p"]))
    R_h, p_h = T["house_link"]
    cs_is = R_h.T @ (p_cs - p_h)
    q_is = art.get_joint_positions()[0]
    joints = {
        "BmMntToBm1": (fw["y.jnts.BmMntToBm1.q"], float(q_is[names.index("boom_joint")])),
        "Bm2ToArm": (fw["y.jnts.Bm2ToArm.q"], float(q_is[names.index("arm_joint")])),
        "ArmToOutpLink": (fw["y.jnts.ArmToOutpLink.q"], state["q_outp"]),
        "ArmToInpLink": (fw["y.jnts.ArmToInpLink.q"], state["q_inp"]),
        "TiltMntToTilt": (fw["y.jnts.TiltMntToTilt.q"], float(q_is[names.index("tilt_joint")])),
    }
    report = {
        "boot_state": boot,
        "calib_started_tick": left, "calib_done_tick": done,
        "saves": len(save.saved),
        "angForkUpLimit_deg": {"firmware": math.degrees(ang_fw), "isaac_oracle": math.degrees(ang_exp),
                               "diff": math.degrees(ang_fw - ang_exp)},
        "distUcToForkBack_m": {"firmware": list(map(float, dist_fw)), "isaac_oracle": dist_exp,
                               "diff_xz_mm": [1000 * (dist_fw[0] - dist_exp[0]), 1000 * (dist_fw[2] - dist_exp[2])]},
        "contact_surface_in_chassis_m": {"firmware": cs_fw.tolist(), "isaac": cs_is.tolist(),
                                         "diff_mm": (1000 * (cs_fw - cs_is)).tolist()},
        "joints_deg": {k: {"firmware": math.degrees(a), "isaac": math.degrees(b),
                           "diff": math.degrees(math.atan2(math.sin(a - b), math.cos(a - b)))}
                       for k, (a, b) in joints.items()},
        "commanded_pose_deg": pose,
        "final_state": h.describe(),
    }
    with open(args.report, "w") as f:
        json.dump(report, f, indent=1)
    print("SIL28 " + json.dumps({k: report[k] for k in ("calib_started_tick", "calib_done_tick", "saves",
                                                         "angForkUpLimit_deg", "distUcToForkBack_m",
                                                         "contact_surface_in_chassis_m")}))
    sys.stdout.flush()
    import os
    os._exit(0)


if __name__ == "__main__":
    main()
