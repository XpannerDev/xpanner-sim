"""
isaac_soil.py -- the stockpile as PhysX position-based-dynamics particles (granular, not fluid).

Why PBD particles and not Newton MPM: Isaac Sim 6.0.1 ships `isaacsim.physics.newton`, but the
articulation, contacts and sensors of this scene all run on PhysX, and the two engines do not share a
stage in one simulation. PhysX particles give a granular material that the bucket can scoop and that
piles up in the truck bed with slopes, which is what the perception side needs to see. A Newton-MPM
soil with real material parameters (friction angle, cohesion, density) is the next step for digging
*forces*; see docs/LOADING_DEMO.md.

The particle spacing sets everything else (PhysX particle sample conventions):
    solid rest offset       = 0.45 * spacing      particle-particle resting distance / 2
    particle contact offset = 0.50 * spacing + 5 mm
    rest / contact offset   = same, against rigid bodies
"""
import numpy as np


class Soil:
    def __init__(self, stage, scene_path, positions, spacing=0.08, path="/World/Soil", density=1600.0,
                 friction=1.0, adhesion=0.02, adhesion_scale=2.0, damping=0.5, colour=(0.47, 0.33, 0.20)):
        # scripts/isaac_soil_test.py (2026-10-02): a 34-deg cone of 8 cm particles does not spread with any of
        # friction 0.9-1.0 / adhesion 0-0.05, but its apex creeps down ~0.4 m in 6 s; adhesion 0.05 + damping 0.5
        # crept least. GUESS-grade material either way: real soil shear strength is not in a PBD model.
        import carb
        from pxr import Gf, Sdf, UsdGeom, Vt
        from omni.physx.scripts import particleUtils, physicsUtils

        # PhysX only writes particle positions back to USD when asked; that is how we read them.
        carb.settings.get_settings().set("/physics/updateParticlesToUsd", True)

        self.stage, self.path, self.spacing = stage, path, float(spacing)
        UsdGeom.Scope.Define(stage, path)
        ps_path = Sdf.Path(f"{path}/particleSystem")
        rest = 0.45 * spacing
        pco = 0.5 * spacing + 0.005
        particleUtils.add_physx_particle_system(
            stage=stage, particle_system_path=ps_path,
            contact_offset=pco, rest_offset=rest,
            particle_contact_offset=pco, solid_rest_offset=rest, fluid_rest_offset=rest * 0.6,
            solver_position_iterations=16, simulation_owner=Sdf.Path(scene_path))
        mtl = f"{path}/pbdMaterial"
        particleUtils.add_pbd_particle_material(stage, Sdf.Path(mtl), friction=friction, damping=damping,
                                                adhesion=adhesion, particle_friction_scale=1.0,
                                                particle_adhesion_scale=adhesion_scale)
        physicsUtils.add_physics_material_to_prim(stage, stage.GetPrimAtPath(ps_path), Sdf.Path(mtl))

        positions = np.asarray(positions, float)
        self.n = len(positions)
        # a particle's share of the pile: spacing^3 of bulk material at the bulk density
        self.particle_volume = spacing ** 3
        mass = density * self.particle_volume
        self.set_path = Sdf.Path(f"{path}/particles")
        particleUtils.add_physx_particleset_pointinstancer(
            stage, self.set_path,
            Vt.Vec3fArray([Gf.Vec3f(*map(float, p)) for p in positions]),
            Vt.Vec3fArray([Gf.Vec3f(0.0, 0.0, 0.0)] * self.n),
            ps_path, self_collision=True, fluid=False, particle_group=0, particle_mass=mass, density=0.0)
        self.instancer = UsdGeom.PointInstancer(stage.GetPrimAtPath(self.set_path))
        # render the particles as spheres of the rest radius, soil coloured
        proto = self.instancer.GetPrototypesRel().GetTargets()
        if proto:
            sph = UsdGeom.Sphere(stage.GetPrimAtPath(proto[0]))
            if sph:
                sph.GetRadiusAttr().Set(float(rest))
                sph.CreateDisplayColorAttr([Gf.Vec3f(*colour)])
        self.mass = mass

    def positions(self):
        """Current particle positions (world), Nx3. PhysX writes them back to the PointInstancer."""
        p = self.instancer.GetPositionsAttr().Get()
        return np.asarray(p, float).reshape(-1, 3) if p is not None else np.zeros((0, 3))
