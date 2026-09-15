"""
isaac_scene.py -- shared setup for firmware-in-the-loop scenarios that run inside the Isaac container.

    scene = IsaacScene(usd)                       # starts SimulationApp, loads the machine, resets physics
    h, plant = scene.boot(q0=dict(...))           # IsaacPlant at q0, drives configured, firmware booted
    ...scenario...
    scene.static_check(h, plant)                  # pause, let it stop, compare firmware belief vs Isaac

Nothing here is imported outside the container: SimulationApp must exist before pxr/omni imports, so the
class creates it first and imports the rest lazily.
"""
import math

import numpy as np

REPO = "/work/xpanner-sim"


def quat_to_R(w, x, y, z):
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[w*w+x*x-y*y-z*z, 2*(x*y-w*z), 2*(w*y+x*z)], [2*(x*y+w*z), w*w-x*x+y*y-z*z, 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(w*x+y*z), w*w-x*x-y*y+z*z]])


class IsaacScene:
    def __init__(self, usd, physics_dt=0.005, base_z=1.445, prim="/World/ECR88"):
        from isaacsim import SimulationApp
        self.app = SimulationApp({"headless": True})
        from pxr import Gf, UsdGeom
        import isaacsim.core.utils.stage as su
        from isaacsim.core.api import World
        from isaacsim.core.prims import Articulation
        from isaacsim.core.utils.stage import add_reference_to_stage

        self.physics_dt = physics_dt
        self.world = World(stage_units_in_meters=1.0, physics_dt=physics_dt, rendering_dt=0.01)
        self.world.scene.add_default_ground_plane()
        add_reference_to_stage(usd, prim)
        UsdGeom.Xformable(su.get_current_stage().GetPrimAtPath(prim)).AddTranslateOp().Set(Gf.Vec3d(0, 0, base_z))
        self.world.reset()
        self.art = Articulation(prim)
        self.art.initialize()
        self.body_names = list(self.art.body_names)

    # -- Isaac truth --------------------------------------------------------------------------------
    def link(self, name):
        """(R, p) in world, from the articulation's own tensor view. (A RigidPrim view over articulation links
        created after world.reset() invalidates the simulation view -- found in step 28.)"""
        tf = self.art._physics_view.get_link_transforms()
        px, py, pz, qx, qy, qz, qw = np.asarray(tf.numpy() if hasattr(tf, "numpy") else tf, float)[0][self.body_names.index(name)]
        return quat_to_R(qw, qx, qy, qz), np.array([px, py, pz])

    def tool_in_chassis(self, fw):
        """Contact surface in the chassis frame: (firmware belief from y.links, Isaac truth in house_link)."""
        R_chs = np.array(fw["y.links.chs.R"], float).reshape(3, 3, order="F")
        cs_fw = R_chs.T @ (np.array(fw["y.links.contactSurface.p"], float) - np.array(fw["y.links.chs.p"], float))
        R_h, p_h = self.link("house_link")
        cs_is = R_h.T @ (self.link("contact_surface_link")[1] - p_h)
        return cs_fw, cs_is

    # -- firmware + plant ---------------------------------------------------------------------------
    def boot(self, q0=None, extra=(), settle_steps=40, **plant_kw):
        """IsaacPlant at q0 (degrees), velocity drives on, physics settled, firmware reset with nominal inputs and
        RTK fix, three ticks run. Same boot as sil/tests/test_valve_plant.booted()."""
        import sys
        sys.path.insert(0, REPO)
        from sil.harness import Harness
        from sil.isaac_plant import IsaacPlant
        from sil.plant import Hardware

        plant = IsaacPlant(self.world, self.art, physics_dt=self.physics_dt, q0=q0, degrees=True, **plant_kw)
        h = Harness(plant=[plant, *extra]).reset()
        plant.set_hardware(Hardware.compiled(h.fw))
        want = dict(plant.q)
        plant.push_pose()
        plant.configure_drives()
        for _ in range(settle_steps):
            self.world.step(render=False)
        plant.sync_from_isaac()
        self.settle_drift_deg = {k: math.degrees(plant.q[k] - want[k]) for k in want}
        h.nominal_inputs()
        h.gnss_rtk_fixed()
        h.tick(3)
        return h, plant

    @staticmethod
    def firmware_joints_deg(fw):
        """y.jnts.*.q in plant names and degrees (ChsToUc = -swing)."""
        from sil.plant import FW_JOINT
        return {k: math.degrees(-fw[f"y.jnts.{n}.q"] if k == "swing" else fw[f"y.jnts.{n}.q"]) for k, n in FW_JOINT.items()}

    def static_check(self, h, plant, seconds=2.0):
        """Pause the auto cycle, let the machine stop, compare the firmware's belief with Isaac (no estimator lag)."""
        h.pulse("u.jstAutoReq_StartPause")
        h.run_seconds(seconds)
        s_fw, s_is = self.tool_in_chassis(h.fw)
        fq = self.firmware_joints_deg(h.fw)
        return dict(state=h.main_state(), valves=h.valves(),
                    joint_rates_deg_s={k: math.degrees(v) for k, v in plant.qdot.items()},
                    joint_diff_firmware_minus_isaac_deg={k: fq[k] - math.degrees(plant.q[k]) for k in fq},
                    contact_surface_in_chassis_m=dict(firmware=s_fw.tolist(), isaac=s_is.tolist(),
                                                      diff_mm=(1000 * (s_fw - s_is)).tolist()))

    def close(self, report_line=None):
        import os
        import sys
        if report_line:
            print(report_line)
        sys.stdout.flush()
        os._exit(0)                 # SimulationApp.close() can hang in headless containers
