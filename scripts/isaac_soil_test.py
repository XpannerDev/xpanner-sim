#!/usr/bin/env python3
"""
isaac_soil_test.py -- does a PhysX PBD particle pile hold its shape? Spawns several cones side by side, each
with its own particle system + material, lets them settle, and prints apex height / 90 % radius over time.
Run 3 of the loading demo showed the 34-deg cone (friction 0.9, no adhesion) collapsing into a wide spread.

    ./scripts/run_isaac.sh python /work/xpanner-sim/scripts/isaac_soil_test.py
"""
import json
import math
import os
import sys
import time

sys.path.insert(0, "/work/xpanner-sim")
import numpy as np                                                       # noqa: E402

CASES = [
    dict(name="f0.9_a0",     friction=0.9, adhesion=0.0,  adh_scale=1.0, damping=0.0, slope_deg=34),
    dict(name="f1.0_a0.02",  friction=1.0, adhesion=0.02, adh_scale=2.0, damping=0.5, slope_deg=34),
    dict(name="f1.0_a0.05",  friction=1.0, adhesion=0.05, adh_scale=3.0, damping=0.5, slope_deg=34),
    dict(name="f1.0_a0.02_s25", friction=1.0, adhesion=0.02, adh_scale=2.0, damping=0.5, slope_deg=25),
]


def main():
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    import carb
    from pxr import Gf, Sdf, UsdGeom, Vt
    from omni.physx.scripts import particleUtils, physicsUtils
    from isaacsim.core.api import World
    import isaacsim.core.utils.stage as su
    from sim.isaac_site import mound_positions

    carb.settings.get_settings().set("/physics/updateParticlesToUsd", True)
    world = World(stage_units_in_meters=1.0, physics_dt=1 / 120, rendering_dt=1 / 20)
    pc = world.get_physics_context()
    pc.enable_gpu_dynamics(True)
    pc.set_broadphase_type("GPU")
    world.scene.add_default_ground_plane()
    stage = su.get_current_stage()
    spacing = float(os.environ.get("SOIL_SPACING", "0.08"))
    rest, pco = 0.45 * spacing, 0.5 * spacing + 0.005
    piles = []
    for i, c in enumerate(CASES):
        cx, cy = 0.0, i * 7.0
        radius = 2.0
        height = radius * math.tan(math.radians(c["slope_deg"]))
        base = f"/World/Pile{i}"
        UsdGeom.Scope.Define(stage, base)
        ps = Sdf.Path(f"{base}/particleSystem")
        particleUtils.add_physx_particle_system(stage=stage, particle_system_path=ps, contact_offset=pco, rest_offset=rest,
                                                particle_contact_offset=pco, solid_rest_offset=rest, fluid_rest_offset=rest * 0.6,
                                                solver_position_iterations=16, simulation_owner=Sdf.Path(pc.prim_path))
        mtl = Sdf.Path(f"{base}/mat")
        particleUtils.add_pbd_particle_material(stage, mtl, friction=c["friction"], damping=c["damping"], adhesion=c["adhesion"],
                                                particle_friction_scale=1.0, particle_adhesion_scale=c["adh_scale"])
        physicsUtils.add_physics_material_to_prim(stage, stage.GetPrimAtPath(ps), mtl)
        pos = mound_positions((cx, cy), radius, height, spacing, seed=i)
        mass = 1600.0 * spacing ** 3
        particleUtils.add_physx_particleset_pointinstancer(stage, Sdf.Path(f"{base}/particles"),
                                                           Vt.Vec3fArray([Gf.Vec3f(*map(float, p)) for p in pos]),
                                                           Vt.Vec3fArray([Gf.Vec3f(0, 0, 0)] * len(pos)),
                                                           ps, self_collision=True, fluid=False, particle_group=i,
                                                           particle_mass=mass, density=0.0)
        piles.append(dict(case=c, n=len(pos), height=height, centre=(cx, cy), prim=UsdGeom.PointInstancer(stage.GetPrimAtPath(f"{base}/particles"))))
    world.reset()

    def measure(p):
        q = np.asarray(p["prim"].GetPositionsAttr().Get(), float).reshape(-1, 3)
        r = np.linalg.norm(q[:, :2] - np.array(p["centre"]), axis=1)
        return dict(apex=float(np.percentile(q[:, 2], 99.5)), r90=float(np.percentile(r, 90)), r_max=float(r.max()),
                    beyond_3m=int((r > 3.0).sum()))

    out = {p["case"]["name"]: dict(n=p["n"], height0=p["height"], t=[]) for p in piles}
    t0 = time.time()
    for step in range(int(6.0 * 120) + 1):
        world.step(render=False)
        if step % 120 == 0:
            for p in piles:
                m = measure(p)
                m["t"] = step / 120
                out[p["case"]["name"]]["t"].append(m)
            print(f"[soil_test] t={step/120:.0f}s " + " | ".join(f"{p['case']['name']}: apex {out[p['case']['name']]['t'][-1]['apex']:.2f} "
                  f"r90 {out[p['case']['name']]['t'][-1]['r90']:.2f} far {out[p['case']['name']]['t'][-1]['beyond_3m']}" for p in piles), flush=True)
    print(f"[soil_test] wall {time.time() - t0:.0f} s for 6 s sim")
    with open("/work/xpanner-sim/build/isaac/soil_test.json", "w") as f:
        json.dump(out, f, indent=1)
    print("[soil_test] done", flush=True)
    os._exit(0)


if __name__ == "__main__":
    main()
