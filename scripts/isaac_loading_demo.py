#!/usr/bin/env python3
"""
isaac_loading_demo.py -- dig a stockpile and load a dump truck EVENLY, with perception choosing both the
dig point and the dump cell, and a person walking by stopping the machine. Isaac Sim 6.0.1, PhysX.

What is in the scene
  * the ECR88 (assets/ecr88/usd/ecr88_physics.usd) with a CLASS-TYPICAL 0.25 m^3 bucket hung on the
    firmware's output link in place of the PanelLift tool (sim/loading_planner.BUCKET); the tool, fork
    and panel stack are hidden and switched off for collision
  * a stockpile of PhysX PBD particles (granular) in front of the machine (sim/isaac_soil.py)
  * a tipper truck of primitives on the machine's left, bed = 3 x 2 cells (sim/isaac_site.py)
  * a roof LiDAR mounted VERTICALLY (spin axis along the house y) so the swing sweeps a vertical fan over
    the pile and the bed -- the Bedrock-Robotics-style survey; a 120-deg semantic camera for people;
    a spectator camera for the recording (sim/isaac_sensors.py)
  * a person (capsule, label "person") who walks across the site once

The loop (per cycle)
  1. terrain map from the LiDAR cloud -> highest reachable pile cell -> scripted dig (enter, scoop, curl, lift)
  2. swing loaded to the truck; during the swing the fan crosses the bed
  3. bed heightmap from the LiDAR cloud -> LOWEST cell -> hover, lower the mouth to surface + 0.35 m, pour
     slowly, close, lift away
  4. swing back; report LiDAR estimates against particle ground truth (volumes, heights, spillage)
  A person inside stop_radius (camera detection, GT logged alongside) freezes the trajectory clock;
  it resumes when they are beyond resume_radius.

    ./scripts/run_isaac.sh python /work/xpanner-sim/scripts/isaac_loading_demo.py [--stream] [--cycles 6] \
        [--record /work/xpanner-sim/build/isaac/loading_demo] [--report ...json]

--stream turns the WebRTC livestream on first (same settings as isaac_stream_scene.py) so the viewer sees
the run live; without it the run is headless and only the report / recording come out.
"""
import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, "/work/xpanner-sim")

import numpy as np                                                                   # noqa: E402

REPO = "/work/xpanner-sim"
BASE_Z = 1.445                                                                       # machine base above grade
MACHINE = "/World/ECR88"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--usd", default=f"{REPO}/assets/ecr88/usd/ecr88_physics.usd")
    ap.add_argument("--urdf", default=f"{REPO}/build/ecr88_demo.urdf",
                    help="URDF for the planner FK (host: xacro ecr88.urdf.xacro model_cylinders:=false -o build/ecr88_demo.urdf)")
    ap.add_argument("--cycles", type=int, default=6)
    ap.add_argument("--max-sim-s", type=float, default=420.0)
    ap.add_argument("--physics-hz", type=float, default=120.0)
    ap.add_argument("--render-hz", type=float, default=20.0)
    ap.add_argument("--spacing", type=float, default=0.08, help="particle spacing m (0.08 -> ~11k particles)")
    ap.add_argument("--no-person", action="store_true")
    ap.add_argument("--stream", action="store_true", help="WebRTC livestream (host/ports from ISAACSIM_* env)")
    ap.add_argument("--host", default=os.environ.get("ISAACSIM_HOST", ""))
    ap.add_argument("--signal-port", type=int, default=int(os.environ.get("ISAACSIM_SIGNAL_PORT", 49100)))
    ap.add_argument("--stream-port", type=int, default=int(os.environ.get("ISAACSIM_STREAM_PORT", 47998)))
    ap.add_argument("--record", default="", help="directory for the mp4 + snapshots (empty = none)")
    ap.add_argument("--video-fps", type=float, default=10.0)
    ap.add_argument("--report", default=f"{REPO}/build/isaac/loading_demo_report.json")
    ap.add_argument("--idle-after", action="store_true", help="keep the app (and stream) alive after the cycles")
    ap.add_argument("--safety", choices=("threat", "zone"), default="threat",
                    help="threat: stop only when the PLANNED motion meets the person's PREDICTED path (slow first); "
                         "zone: stop whenever a person is inside stop_radius")
    args = ap.parse_args()

    t_wall0 = time.time()
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True, "width": 1280, "height": 720})

    import carb
    from isaacsim.core.utils.extensions import enable_extension
    if args.stream:
        s = carb.settings.get_settings()
        if args.host:
            s.set("/exts/omni.kit.livestream.app/primaryStream/publicIp", args.host)
        s.set("/exts/omni.kit.livestream.app/primaryStream/signalPort", args.signal_port)
        s.set("/exts/omni.kit.livestream.app/primaryStream/streamPort", args.stream_port)
        s.set("/app/window/drawMouse", True)
        enable_extension("omni.kit.livestream.app")
        app.update()

    from pxr import Gf, UsdGeom
    import isaacsim.core.utils.stage as su
    from isaacsim.core.api import World
    from isaacsim.core.prims import Articulation
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.semantics import add_labels
    from isaacsim.core.utils.viewports import set_camera_view

    from sil.urdf_fk import UrdfModel
    from sim import loading_planner as lp
    from sim import isaac_site as site
    from sim.isaac_soil import Soil
    from sim.isaac_sensors import RoofLidar, RoofCamera, Spectator

    S = lp.SITE
    log = lambda *a: print("[loading_demo]", *a, flush=True)                     # noqa: E731

    # ---------------------------------------------------------------- scene
    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / args.physics_hz, rendering_dt=1.0 / args.render_hz)
    pc = world.get_physics_context()
    pc.enable_gpu_dynamics(True)
    pc.set_broadphase_type("GPU")
    world.scene.add_default_ground_plane()
    stage = su.get_current_stage()

    add_reference_to_stage(args.usd, MACHINE)
    UsdGeom.Xformable(stage.GetPrimAtPath(MACHINE)).AddTranslateOp().Set(Gf.Vec3d(0, 0, BASE_Z))
    hidden = site.hide_panellift(stage, MACHINE)
    site.add_bucket(stage, MACHINE)
    site.add_truck(stage, "/World/Truck", S["bed_centre"], S["bed_yaw_deg"])
    site.add_lights(stage)

    person = None
    if not args.no_person:
        person = site.add_person(stage, "/World/Person", (*S["person_start"], 0.0))
        add_labels(person.GetPrim(), ["person"], instance_name="class")
        add_labels(stage.GetPrimAtPath("/World/Person/body"), ["person"], instance_name="class")

    site.add_bench(stage, "/World/Bench", S["mound_centre"], S["bench_half"], S["bench_h"])
    mound = site.mound_positions(S["mound_centre"], S["mound_radius"], S["mound_height"], args.spacing, z0=S["bench_h"])
    soil = Soil(stage, str(pc.prim_path) if hasattr(pc, "prim_path") else "/physicsScene", mound, spacing=args.spacing)
    log(f"scene: {len(mound)} soil particles, {len(hidden)} PanelLift gprims hidden")

    # Start the articulation AT the carry pose, authored in USD before the first physics step. The importer's
    # default (all joints 0) lays the arm straight out through the stockpile; run 3 let world.reset() take one
    # step like that and the bucket blew the pile apart before the teleport to the carry pose.
    model = UrdfModel(args.urdf)
    q_now = lp.carry_pose(model, 0.0)[0]
    from pxr import PhysxSchema, UsdPhysics
    for jname, k in (("swing_joint", "swing"), ("boom_joint", "boom"), ("arm_joint", "arm"),
                     ("bucket_joint", "bucket"), ("input_link_joint", "bucket")):
        jp = site.find_prim(stage, MACHINE, jname)
        st = PhysxSchema.JointStateAPI.Apply(jp, "angular")
        st.CreatePositionAttr().Set(float(q_now[k]))
        st.CreateVelocityAttr().Set(0.0)
        drv = UsdPhysics.DriveAPI.Get(jp, "angular")
        if drv:
            drv.CreateTargetPositionAttr().Set(float(q_now[k]))

    world.reset()
    art = Articulation(MACHINE)
    art.initialize()
    names = list(art.dof_names)
    J = dict(swing="swing_joint", boom="boom_joint", arm="arm_joint", bucket="bucket_joint")
    idx = {k: names.index(v) for k, v in J.items()}
    i_inp = names.index("input_link_joint") if "input_link_joint" in names else None

    # position drives: stiff on the four working joints, the rest keep the importer's drives
    kps, kds = (np.asarray(g.numpy() if hasattr(g, "numpy") else g, float).reshape(1, -1).copy() for g in art.get_gains())
    for k, i in idx.items():
        kps[0, i], kds[0, i] = (2.0e7, 1.5e6) if k in ("swing", "boom", "arm") else (4.0e6, 2.0e5)
    if i_inp is not None:
        kps[0, i_inp], kds[0, i_inp] = 4.0e6, 2.0e5
    art.set_gains(kps=kps, kds=kds)
    fr = art.get_friction_coefficients()
    fr = np.asarray(fr.numpy() if hasattr(fr, "numpy") else fr, float).reshape(1, -1).copy()
    fr[:] = 0.0
    art.set_friction_coefficients(fr)

    bed = lp.BedGrid(S["bed_centre"], S["bed_yaw_deg"], S["bed_length"], S["bed_width"],
                     S["bed_floor_above_grade"], *S["bed_cells"])
    terrain = lp.TerrainGrid(S["mound_centre"], S["mound_radius"] + 0.6, 0.20, S["bench_h"], max_h=S["mound_height"] + 0.2)   # heights above the bench
    safety = lp.SafetyMonitor(S["stop_radius"], S["resume_radius"])     # the distance zone, always logged
    threat = lp.ThreatMonitor()                                         # planned motion vs predicted person path
    person_track = dict(t=None, xy=None, v=None)

    def targets_array(qdeg):
        arr = np.asarray(art.get_joint_positions()[0], float).copy()
        for k, i in idx.items():
            arr[i] = math.radians(qdeg[k])
        if i_inp is not None:
            arr[i_inp] = math.radians(qdeg["bucket"])            # the four-bar placeholder follows the bucket
        return arr.reshape(1, -1)

    arr = targets_array(q_now)
    got = np.asarray(art.get_joint_positions()[0], float)
    log("pose after reset vs carry pose (deg): " + ", ".join(f"{k} {math.degrees(got[i]):.1f}/{q_now[k]:.1f}" for k, i in idx.items()))
    art.set_joint_positions(arr)
    art.set_joint_velocities(np.zeros_like(arr))
    art.set_joint_position_targets(arr)
    for _ in range(int(args.physics_hz)):                        # 1 s settle, particles land, drives hold
        world.step(render=False)

    # ---------------------------------------------------------------- sensors (after the timeline runs)
    # cab roof (house frame): cab box centre (0.64, 0.77), top at lenUppZ1 + geo_house_h = -0.67 + 1.962 = +1.29 m.
    # The first run put both sensors at z 1.0-1.05, INSIDE the cab box: the camera saw the box's inner faces.
    house = str(site.find_prim(stage, MACHINE, "house_link").GetPath())
    lidar = RoofLidar(house, translation=(0.64, 0.77, 1.42))                    # 13 cm above the cab roof, vertical fan
    # two 120-deg semantic cameras: front-left (pile + truck) and rear-right, 240 deg between them. Run 4 lost
    # the person 2.75 m from the machine as they walked out of the single camera's view, and the stop released.
    cam = RoofCamera(house, translation=(1.10, 0.77, 1.36), rpy_deg=(0.0, 12.0, 40.0), hfov_deg=120.0)
    cam.initialize()
    cam2 = RoofCamera(house, name="roof_cam_rear", translation=(0.20, 0.77, 1.36), rpy_deg=(0.0, 12.0, -140.0), hfov_deg=120.0)
    cam2.initialize()
    EYE, TGT = (-9.0, -13.0, 8.5), (2.5, 2.5, 0.8)
    spectator = Spectator(eye=EYE, target=TGT, resolution=(960, 540))
    spectator.initialize()
    set_camera_view(eye=list(EYE), target=list(TGT), camera_prim_path="/OmniverseKit_Persp")
    for _ in range(10):
        world.render()

    writer = None
    if args.record:
        import cv2
        os.makedirs(args.record, exist_ok=True)
        writer = cv2.VideoWriter(os.path.join(args.record, "loading_demo.mp4"), cv2.VideoWriter_fourcc(*"mp4v"),
                                 args.video_fps, (960, 540))

    # ---------------------------------------------------------------- helpers
    dt = 1.0 / args.physics_hz
    render_every = max(1, int(round(args.physics_hz / args.render_hz)))
    video_every = max(1, int(round(args.render_hz / args.video_fps)))
    state = dict(t=0.0, tick=0, frames=0, paused=False, speed=1.0, person_dist_gt=None, person_dist_cam=None, person_px=0)
    report = dict(config=vars(args), site=S, bucket=lp.BUCKET, particles=int(soil.n),
                  particle_volume_m3=soil.particle_volume, cycles=[], safety=[], notes=[])

    def person_xyz(t):
        if person is None:
            return None
        t0, v = S["person_start_t"], S["person_speed"]
        a, b = np.array(S["person_start"], float), np.array(S["person_end"], float)
        L = np.linalg.norm(b - a)
        s = min(max((t - t0) * v, 0.0), L)
        p = a + (b - a) / L * s
        return np.array([p[0], p[1], 0.0])

    body_names = list(art.body_names)

    def link_T(name):
        """World 4x4 of an articulation link from the tensor view (see sil/isaac_scene.py)."""
        from scipy.spatial.transform import Rotation as R
        tf = art._physics_view.get_link_transforms()
        px, py, pz, qx, qy, qz, qw = np.asarray(tf.numpy() if hasattr(tf, "numpy") else tf, float)[0][body_names.index(name)]
        T = np.eye(4)
        T[:3, :3] = R.from_quat([qx, qy, qz, qw]).as_matrix()
        T[:3, 3] = (px, py, pz)
        return T

    def in_bucket(p):
        """Particles inside the bucket's interior box (a 5 cm tolerance), using the live output_link pose."""
        T = link_T("output_link") @ lp.bucket_T_output()
        q = (np.linalg.inv(T) @ np.column_stack([p, np.ones(len(p))]).T).T[:, :3]
        b = lp.BUCKET
        return int(np.sum((q[:, 0] > -0.05) & (q[:, 0] < b["height"] + 0.05) & (np.abs(q[:, 1]) < b["width"] / 2 + 0.05)
                          & (q[:, 2] > -0.05) & (q[:, 2] < b["depth"] + 0.05)))

    def soil_stats():
        p_all = soil.positions()
        # particles riding in the bucket are not load yet: keep them out of the bed / terrain ground truth
        T = link_T("output_link") @ lp.bucket_T_output()
        qb = (np.linalg.inv(T) @ np.column_stack([p_all, np.ones(len(p_all))]).T).T[:, :3]
        bk = lp.BUCKET
        riding = ((qb[:, 0] > -0.1) & (qb[:, 0] < bk["height"] + 0.1) & (np.abs(qb[:, 1]) < bk["width"] / 2 + 0.1)
                  & (qb[:, 2] > -0.1) & (qb[:, 2] < bk["depth"] + 0.1))
        p = p_all[~riding]
        pb = bed.to_bed(p)
        in_bed = bed.inside(pb)
        zone = (np.abs(pb[:, 0]) < bed.L / 2 + 1.5) & (np.abs(pb[:, 1]) < bed.W / 2 + 1.5)
        spilled = zone & ~in_bed & (p[:, 2] < bed.floor_z - 0.3)
        r = np.linalg.norm(p[:, :2] - np.array(S["mound_centre"]), axis=1)
        in_mound = (r < S["mound_radius"] + 0.5) & (p[:, 2] < S["bench_h"] + 2.0)
        return dict(in_bed=int(in_bed.sum()), spilled_near_truck=int(spilled.sum()), in_mound=int(in_mound.sum()),
                    in_bucket=in_bucket(p_all), z_max=float(p_all[:, 2].max()) if len(p_all) else None,
                    loaded_m3=float(in_bed.sum() * soil.particle_volume), bed_heights_gt=bed.heights(p, pct=95).tolist(),
                    terrain_gt=terrain.heights(p, pct=95))

    def advance(traj, label_cb=None, max_s=120.0):
        """Run one trajectory to completion (the clock freezes while the safety monitor says stop)."""
        t_traj, t_start = 0.0, state["t"]
        done = False

        def machine_points(tau):
            q_f, _ = traj.sample(t_traj + tau)
            return lp.machine_points_base(model, q_f)

        while not done and state["t"] - t_start < max_s:
            t_traj += dt * state["speed"]
            q, label = traj.sample(t_traj)
            art.set_joint_position_targets(targets_array(q))
            world.step(render=False)
            state["t"] += dt
            state["tick"] += 1
            if person is not None:
                site.set_translate(person.GetPrim(), person_xyz(state["t"]))
            if state["tick"] % render_every == 0:
                world.render()
                state["frames"] += 1
                T_boom, T_arm, T_out = link_T("boom_link"), link_T("arm_link"), link_T("output_link")
                pin = T_out[:3, 3]
                b_c = (T_out @ lp.bucket_T_output() @ np.array([*lp.bucket_points()["centre"], 1.0]))[:3]
                lidar.read(state["t"], parent_T=link_T("house_link"),
                           exclude=[(T_boom[:3, 3], T_arm[:3, 3], 0.55), (T_arm[:3, 3], pin, 0.5), (pin, b_c, 0.75)])
                if state["frames"] % 2 == 0:
                    d1, n1 = cam.person()
                    d2, n2 = cam2.person()
                    seen = [(d, c) for d, c in ((d1, cam), (d2, cam2)) if d is not None]
                    d_cam, which = min(seen, key=lambda x: x[0]) if seen else (None, None)
                    npx = n1 + n2
                    state["person_dist_cam"], state["person_px"] = d_cam, npx
                    pxyz = person_xyz(state["t"])
                    state["person_dist_gt"] = float(np.linalg.norm(pxyz[:2])) if pxyz is not None else None
                    # the camera sits ~0.7 m from the swing axis: its depth is the distance from the machine
                    zone_paused = safety.update(state["t"], d_cam, source="camera")
                    # person track in the world from the camera (position + straight-line velocity)
                    pw = which.world_xyz if which is not None else None
                    if pw is not None:
                        if person_track["t"] is not None and not person_track.get("coasting") and 0.05 < state["t"] - person_track["t"] < 1.0:
                            v = (pw[:2] - person_track["xy"]) / (state["t"] - person_track["t"])
                            v = v if np.linalg.norm(v) < 3.0 else v / np.linalg.norm(v) * 3.0
                            person_track["v"] = 0.5 * v + 0.5 * person_track["v"] if person_track["v"] is not None else v
                        person_track.update(t=state["t"], xy=pw[:2].copy(), coasting=False)
                        track_xy = person_track["xy"]
                    elif person_track["t"] is not None and state["t"] - person_track["t"] <= 6.0:
                        # out of view: dead-reckon along the last velocity rather than declaring the site clear
                        v = person_track["v"] if person_track["v"] is not None else np.zeros(2)
                        track_xy = person_track["xy"] + v * (state["t"] - person_track["t"])
                        person_track["coasting"] = True
                    else:
                        person_track.update(t=None, xy=None, v=None, coasting=False)   # lost for 6 s: forget
                        track_xy = None
                    level = threat.update(state["t"], track_xy, person_track["v"], machine_points)
                    was_level, was = state.get("level", "clear"), state["paused"]
                    if args.safety == "threat":
                        state["speed"] = threat.speed_scale
                        state["paused"] = level == "stop"
                    else:
                        state["speed"] = 0.0 if zone_paused else 1.0
                        state["paused"] = zone_paused
                    state["level"] = level
                    if state["paused"] != was or level != was_level:
                        log(f"t={state['t']:.1f}s threat={level} zone={'stop' if zone_paused else 'clear'} speed={state['speed']:.1f} "
                            f"person cam={None if d_cam is None else round(d_cam, 2)} gt={None if state['person_dist_gt'] is None else round(state['person_dist_gt'], 2)} "
                            f"xy={None if track_xy is None else np.round(track_xy, 2).tolist()}{' (coasting)' if person_track.get('coasting') else ''} "
                            f"v={None if person_track['v'] is None else np.round(person_track['v'], 2).tolist()} "
                            f"cam={'front' if which is cam else ('rear' if which is cam2 else None)}")
                        if safety.events and "camera_debug" not in safety.events[-1]:
                            safety.events[-1]["camera_debug"] = dict(getattr(cam, "debug", {}))
                        if args.record:
                            import cv2
                            img = cam.rgb()
                            if img is not None and img.size:
                                cv2.imwrite(os.path.join(args.record, f"person_{'stop' if state['paused'] else 'resume'}_{state['t']:.0f}s.png"),
                                            cv2.cvtColor(np.ascontiguousarray(img), cv2.COLOR_RGB2BGR))
                if writer is not None and state["frames"] % video_every == 0:
                    import cv2
                    frame = spectator.rgb()
                    if frame is not None and frame.size:
                        writer.write(cv2.cvtColor(np.ascontiguousarray(frame), cv2.COLOR_RGB2BGR))
                if label != state.get("label"):
                    state["label"] = label
                    qa = joints_now()
                    T_out = link_T("output_link")
                    bc = (T_out @ lp.bucket_T_output() @ np.array([*lp.bucket_points()["centre"], 1.0]))[:3]
                    log(f"t={state['t']:.1f}s seg {label}: target {{{', '.join(f'{k} {v:.1f}' for k, v in q.items())}}} "
                        f"actual {{{', '.join(f'{k} {v:.1f}' for k, v in qa.items())}}} bucket centre {np.round(bc, 2).tolist()}")
                    if writer is not None and label not in ("done", "hold"):
                        import cv2
                        img = spectator.rgb()
                        if img is not None and img.size:
                            cv2.imwrite(os.path.join(args.record, f"seg_{state['t']:06.1f}s_{label}.png"),
                                        cv2.cvtColor(np.ascontiguousarray(img), cv2.COLOR_RGB2BGR))
                if label_cb:
                    label_cb(label)
            done = label == "done"
        return t_traj

    def joints_now():
        qp = np.asarray(art.get_joint_positions()[0], float)
        return {k: math.degrees(qp[i]) for k, i in idx.items()}

    # ---------------------------------------------------------------- reference survey: one slow sweep
    log("reference sweep of the pile with the vertical LiDAR")
    sweep = lp.JointTrajectory(q_now).move(dict(swing=-35.0), "sweep_l", speed_scale=0.6)
    sweep.move(dict(swing=35.0), "sweep_r", speed_scale=0.6).move(dict(swing=0.0), "sweep_c", speed_scale=0.6)
    advance(sweep)
    cal = lidar.calibrate_azimuth(S["mound_centre"], S["mound_radius"] + 0.6, z_lo=S["bench_h"] + 0.1, z_hi=S["bench_h"] + 2.5)
    log(f"lidar azimuth calibration against the pile: {cal}")
    h_ref_lidar = terrain.heights(lidar.cloud(), pct=90)
    seen_ref = terrain.last_counts >= 2
    st0 = soil_stats()
    h_ref_gt = st0["terrain_gt"]
    log(f"soil after settle: highest particle z={st0['z_max']:.2f} (bench {S['bench_h']} + apex {S['mound_height']}), in mound {st0['in_mound']}/{soil.n}")
    report["reference"] = dict(lidar_frames=lidar.frames, lidar_points=lidar.points_total,
                               pile_volume_lidar_m3=terrain.volume(h_ref_lidar), pile_volume_gt_m3=terrain.volume(h_ref_gt),
                               pile_particles=st0["in_mound"], pile_volume_particles_m3=st0["in_mound"] * soil.particle_volume,
                               lidar_frame_of_reference=lidar.last_frame_of_reference)
    log(f"reference: lidar pile volume {terrain.volume(h_ref_lidar):.2f} m3, GT {terrain.volume(h_ref_gt):.2f} m3, "
        f"{lidar.points_total} lidar points in {lidar.frames} frames")
    report["reference"]["lidar_stats"] = lidar.stats()
    log(f"lidar cloud: {lidar.stats()}")
    if args.record:
        np.save(os.path.join(args.record, "lidar_cloud_reference.npy"), lidar.cloud()[::10].astype(np.float32))

    # ---------------------------------------------------------------- cycles
    for k in range(args.cycles):
        if state["t"] > args.max_sim_s:
            report["notes"].append(f"stopped at cycle {k}: sim time budget")
            break
        cyc = dict(cycle=k, t_start=state["t"])
        q_now = joints_now()

        cl = lidar.cloud(since=state["t"] - 8.0)
        h_l = terrain.heights(cl, pct=60)              # median-ish: a slope cell's 90th percentile overstates the surface
        pick = lp.choose_dig_point(terrain, h_l)
        source, h_used = "lidar", h_l
        log(f"cycle {k}: lidar map from {len(cl)} pts (last 8 s): cells seen {int((terrain.last_counts >= 2).sum())}, "
            f"cells > 0.15 m {int((h_l > 0.15).sum())}, max {h_l.max():.2f} m; pick {pick}")
        if pick is None:
            h_used = soil_stats()["terrain_gt"]
            pick = lp.choose_dig_point(terrain, h_used)
            source = "gt"
        if pick is None:
            report["notes"].append(f"cycle {k}: nothing left to dig in reach")
            break
        cyc["dig_point"] = dict(x=pick[0], y=pick[1], h=pick[2], source=source)
        log(f"cycle {k}: dig at r={math.hypot(pick[0], pick[1]):.2f} y={pick[1]:+.2f} h={pick[2]:.2f} ({source})")
        # re-run heights() on the map actually used so the sampler's coverage matches it
        terrain.heights(cl if source == "lidar" else soil.positions(), pct=60)
        tr = lp.plan_dig(model, q_now, pick, surface=terrain.sampler(h_used, fallback=pick[2]))
        cyc["dig_segments"] = [(lab, {k: round(v, 1) for k, v in qb.items()}) for _, _, _, qb, lab in tr.segments]
        st_before = soil_stats()
        advance(tr)
        st_lift = soil_stats()
        cyc["scooped_particles"] = st_lift["in_bucket"]
        cyc["scooped_m3"] = cyc["scooped_particles"] * soil.particle_volume
        cyc["particles_z_max_after_dig"] = st_lift["z_max"]
        log(f"cycle {k}: in bucket {st_lift['in_bucket']} particles (~{cyc['scooped_m3']:.3f} m3); "
            f"mound {st_before['in_mound']} -> {st_lift['in_mound']}, highest particle z={st_lift['z_max']:.2f}")

        # swing to the truck; the fan crosses the bed on the way
        q_now = joints_now()
        carry_truck = lp.carry_pose(model, S["truck_swing_deg"])[0]
        advance(lp.JointTrajectory(q_now).move(carry_truck, "swing_to_truck"))
        bed_l = bed.heights(lidar.cloud(since=state["t"] - 8.0), pct=90)
        bed_gt = np.array(soil_stats()["bed_heights_gt"])
        cell = bed.lowest_cell(bed_l) if bed.counts(lidar.cloud(since=state["t"] - 8.0)).sum() > 50 else bed.lowest_cell(bed_gt)
        cyc["bed_before"] = dict(lidar=bed_l.tolist(), gt=bed_gt.tolist(), cell=list(cell),
                                 evenness_gt=bed.evenness(bed_gt), evenness_lidar=bed.evenness(bed_l),
                                 lidar_points_in_bed=int(bed.counts(lidar.cloud(since=state["t"] - 8.0)).sum()))
        log(f"cycle {k}: bed (lidar) {np.round(bed_l, 2).tolist()} -> cell {cell}; GT {np.round(bed_gt, 2).tolist()}")

        q_now = joints_now()
        surface = float(bed_l[cell]) if cyc["bed_before"]["lidar_points_in_bed"] > 50 else float(bed_gt[cell])
        tr, pour = lp.plan_dump(model, q_now, bed, cell, surface)
        cyc["pour_pose_deg"] = pour
        cyc["pour_mouth_above_floor_m"] = surface + lp.DUMP_CLEARANCE
        advance(tr)
        st_after = soil_stats()
        bed_after = np.array(st_after["bed_heights_gt"])
        cyc["bed_after_gt"] = bed_after.tolist()
        cyc["evenness_after_gt"] = bed.evenness(bed_after)
        cyc["loaded_m3_total"] = st_after["loaded_m3"]
        cyc["spilled_near_truck"] = st_after["spilled_near_truck"]
        cyc["delivered_particles"] = st_after["in_bed"] - st_lift["in_bed"]
        log(f"cycle {k}: delivered {cyc['delivered_particles']} particles, bed GT {np.round(bed_after, 2).tolist()}, "
            f"range {cyc['evenness_after_gt']['range']:.2f} m, spilled {cyc['spilled_near_truck']}")

        q_now = joints_now()
        advance(lp.plan_return(model, q_now))
        h_now = terrain.heights(lidar.cloud(since=state["t"] - 8.0), pct=90)
        seen_now = terrain.last_counts >= 2
        cyc["pile_removed_lidar_m3"] = terrain.removed_volume(h_ref_lidar, h_now, seen_ref & seen_now)
        cyc["pile_cells_seen_both"] = int(np.sum(seen_ref & seen_now))
        cyc["pile_removed_gt_m3"] = (st0["in_mound"] - st_after["in_mound"]) * soil.particle_volume
        cyc["t_end"] = state["t"]
        report["cycles"].append(cyc)
        log(f"cycle {k} done at t={state['t']:.1f}s: pile removed lidar {cyc['pile_removed_lidar_m3']:.2f} / "
            f"GT {cyc['pile_removed_gt_m3']:.2f} m3")

    # ---------------------------------------------------------------- wrap up
    report["safety"] = dict(mode=args.safety, zone_events=safety.events, threat_events=threat.events)
    report["person_last"] = dict(cam=state["person_dist_cam"], gt=state["person_dist_gt"], px=state["person_px"])
    report["sim_time_s"] = state["t"]
    report["wall_time_s"] = time.time() - t_wall0
    report["lidar"] = dict(frames=lidar.frames, points=lidar.points_total, frame_of_reference=lidar.last_frame_of_reference)
    final = soil_stats()
    report["final"] = dict(loaded_m3=final["loaded_m3"], in_bed=final["in_bed"], spilled_near_truck=final["spilled_near_truck"],
                           bed_heights_gt=final["bed_heights_gt"], evenness_gt=bed.evenness(np.array(final["bed_heights_gt"])))
    os.makedirs(os.path.dirname(args.report), exist_ok=True)
    with open(args.report, "w") as f:
        json.dump(report, f, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    if writer is not None:
        writer.release()
        import cv2
        snap = spectator.rgb()
        if snap is not None and snap.size:
            cv2.imwrite(os.path.join(args.record, "final.png"), cv2.cvtColor(np.ascontiguousarray(snap), cv2.COLOR_RGB2BGR))
    log(f"report -> {args.report}; sim {state['t']:.1f} s in {report['wall_time_s']:.0f} s wall; "
        f"loaded {final['loaded_m3']:.2f} m3, spilled {final['spilled_near_truck']}, "
        f"threat events {len(threat.events)}, zone events {len(safety.events)}")

    if args.idle_after or args.stream:
        log("idling (Ctrl-C to quit)")
        try:
            while app.is_running():
                world.render()
        except KeyboardInterrupt:
            pass
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
