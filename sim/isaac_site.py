"""
isaac_site.py -- USD authoring for the dig-and-load demo scene (Isaac container only: pxr imports).

Everything is primitives, like scripts/build_site.py: no meshes exist for this machine (CLAUDE.md), and
the truck has no drawing at all (CLASS-TYPICAL numbers, 5-6 t class tipper, in TRUCK below).
"""
import math

import numpy as np

from sim import loading_planner as lp

TRUCK = dict(                                  # CLASS-TYPICAL 10 t tipper, nothing measured
    wheelbase=4.2, track=1.95, wheel_r=0.50, wheel_w=0.30,
    cab=(2.3, 2.3, 2.0), cab_z=1.0,              # cab box size and bottom height (roof 3.0 m)
    chassis=(7.0, 1.1, 0.30), chassis_z=0.85,
    bed_len=lp.SITE["bed_length"], bed_wid=lp.SITE["bed_width"],
    bed_floor_z=lp.SITE["bed_floor_above_grade"], rail_h=lp.SITE["bed_rail_h"], wall_t=0.06,
)

COL = dict(ground=(0.42, 0.38, 0.33), truck=(0.85, 0.55, 0.10), cab=(0.90, 0.90, 0.92), tyre=(0.08, 0.08, 0.08),
           bucket=(0.55, 0.52, 0.45), soil=(0.47, 0.33, 0.20), person=(0.95, 0.25, 0.20), lidar=(0.1, 0.2, 0.6))


def box(stage, path, size, xyz, colour, rot_xyz_deg=(0.0, 0.0, 0.0), collide=True):
    from pxr import Gf, UsdGeom, UsdPhysics
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.CreateDisplayColorAttr([Gf.Vec3f(*colour)])
    x = UsdGeom.Xformable(c)
    x.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in xyz]))
    if any(rot_xyz_deg):
        x.AddRotateXYZOp().Set(Gf.Vec3f(*[float(v) for v in rot_xyz_deg]))
    x.AddScaleOp().Set(Gf.Vec3f(*[float(v) for v in size]))
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    return c


def cylinder(stage, path, radius, length, xyz, colour, axis="Y", collide=True):
    from pxr import Gf, UsdGeom, UsdPhysics
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(float(radius))
    c.CreateHeightAttr(float(length))
    c.CreateAxisAttr(axis)
    c.CreateDisplayColorAttr([Gf.Vec3f(*colour)])
    UsdGeom.Xformable(c).AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in xyz]))
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    return c


def find_prim(stage, root, name):
    """First descendant of `root` whose name is `name` (the importer nests links under Geometry/...)."""
    from pxr import Usd
    root_prim = stage.GetPrimAtPath(root)
    for p in Usd.PrimRange(root_prim):
        if p.GetName() == name:
            return p
    raise KeyError(f"{name} not under {root}")


PANELLIFT_LINKS = ("tilt_mount_link", "tilt_link", "rotator_link", "attachment_link", "probe_link",
                   "contact_surface_link", "fork_back_link", "fork_link", "panel_stack_link",
                   "panel_top_link", "panel_pick_approach_link", "panel_lift_posn_link")


def hide_panellift(stage, root):
    """Make the PanelLift tool, fork and panel stack invisible and non-colliding. The links and joints stay
    (the articulation is unchanged), only their geometry is switched off."""
    from pxr import Usd, UsdGeom, UsdPhysics
    done = []
    for name in PANELLIFT_LINKS:
        try:
            link = find_prim(stage, root, name)
        except KeyError:
            continue
        for p in Usd.PrimRange(link):
            if p.IsA(UsdGeom.Gprim):
                UsdGeom.Imageable(p).MakeInvisible()
                if p.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Set(False)
                done.append(str(p.GetPath()))
    return done


def add_bucket(stage, root, b=lp.BUCKET):
    """Author the open-box bucket as static collision geometry under output_link. Children of a rigid-body
    link are part of that body, so the bucket moves with the firmware's output link and takes contacts."""
    from pxr import Gf, UsdGeom
    link = find_prim(stage, root, "output_link")
    grp = UsdGeom.Xform.Define(stage, link.GetPath().AppendChild("bucket"))
    T = lp.bucket_T_output(b)
    x = UsdGeom.Xformable(grp)
    x.AddTranslateOp().Set(Gf.Vec3d(*T[:3, 3].tolist()))
    x.AddRotateYOp().Set(float(b["tilt_deg"]))
    for name, (c, size) in lp.bucket_walls(b).items():
        box(stage, f"{grp.GetPath()}/{name}", size, c, COL["bucket"])
    return grp


def add_truck(stage, path, bed_centre_xy, yaw_deg, t=TRUCK):
    """A tipper whose bed interior is exactly lp.SITE's bed (length along the body x, floor at bed_floor_z).
    The body sits so the bed centre is at bed_centre_xy; cab toward +x."""
    from pxr import Gf, UsdGeom
    root = UsdGeom.Xform.Define(stage, path)
    xf = UsdGeom.Xformable(root)
    xf.AddTranslateOp().Set(Gf.Vec3d(float(bed_centre_xy[0]), float(bed_centre_xy[1]), 0.0))
    xf.AddRotateZOp().Set(float(yaw_deg))
    L, W, fz, rh, wt = t["bed_len"], t["bed_wid"], t["bed_floor_z"], t["rail_h"], t["wall_t"]
    # bed: floor + 4 walls (interior L x W, floor top at fz)
    box(stage, f"{path}/bed_floor", (L + 2 * wt, W + 2 * wt, wt), (0, 0, fz - wt / 2), COL["truck"])
    box(stage, f"{path}/bed_wall_front", (wt, W + 2 * wt, rh), (L / 2 + wt / 2, 0, fz + rh / 2), COL["truck"])
    box(stage, f"{path}/bed_wall_rear", (wt, W + 2 * wt, rh), (-L / 2 - wt / 2, 0, fz + rh / 2), COL["truck"])
    box(stage, f"{path}/bed_wall_left", (L + 2 * wt, wt, rh), (0, W / 2 + wt / 2, fz + rh / 2), COL["truck"])
    box(stage, f"{path}/bed_wall_right", (L + 2 * wt, wt, rh), (0, -W / 2 - wt / 2, fz + rh / 2), COL["truck"])
    # chassis + cab ahead of the bed
    cab_x = L / 2 + 0.35 + t["cab"][0] / 2
    box(stage, f"{path}/chassis", t["chassis"], (cab_x / 2 - 0.5, 0, t["chassis_z"]), COL["tyre"])
    box(stage, f"{path}/cab", t["cab"], (cab_x, 0, t["cab_z"] + t["cab"][2] / 2), COL["cab"])
    for i, x in enumerate((cab_x - 0.3, cab_x - 0.3 - t["wheelbase"])):
        for j, y in enumerate((t["track"] / 2, -t["track"] / 2)):
            cylinder(stage, f"{path}/wheel_{i}{j}", t["wheel_r"], t["wheel_w"], (x, y, t["wheel_r"]), COL["tyre"])
    return root


def add_bench(stage, path, centre_xy, half, height):
    """A flat bench (box) the stockpile sits on."""
    return box(stage, path, (2 * half, 2 * half, height), (centre_xy[0], centre_xy[1], height / 2), COL["ground"])


def add_person(stage, path, xyz, height=1.75, radius=0.22):
    """A capsule person. Visual + collision, labelled 'person' for the semantic camera."""
    from pxr import Gf, UsdGeom, UsdPhysics
    xf = UsdGeom.Xform.Define(stage, path)
    UsdGeom.Xformable(xf).AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in xyz]))
    cap = UsdGeom.Capsule.Define(stage, f"{path}/body")
    cap.CreateRadiusAttr(radius)
    cap.CreateHeightAttr(height - 2 * radius)
    cap.CreateAxisAttr("Z")
    cap.CreateDisplayColorAttr([Gf.Vec3f(*COL["person"])])
    UsdGeom.Xformable(cap).AddTranslateOp().Set(Gf.Vec3d(0, 0, height / 2))
    UsdPhysics.CollisionAPI.Apply(cap.GetPrim())
    return xf


def set_translate(prim, xyz):
    from pxr import Gf, UsdGeom
    xf = UsdGeom.Xformable(prim)
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(Gf.Vec3d(*[float(v) for v in xyz]))
            return
    xf.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in xyz]))


def mound_positions(centre_xy, radius, height, spacing, z0=0.0, seed=0):
    """Lattice points filling a cone (apex height `height`), jittered a little so layers do not lock."""
    rng = np.random.default_rng(seed)
    pts = []
    s = spacing
    nz = int(height / s)
    for k in range(nz):
        z = z0 + s / 2 + k * s
        r_here = radius * (1.0 - (z - z0) / height)
        if r_here <= s:
            break
        n = int(2 * r_here / s)
        for i in range(n):
            for j in range(n):
                x = -r_here + s / 2 + i * s + (0.0 if k % 2 == 0 else s / 2)
                y = -r_here + s / 2 + j * s
                if math.hypot(x, y) <= r_here:
                    pts.append((centre_xy[0] + x + rng.uniform(-0.1, 0.1) * s,
                                centre_xy[1] + y + rng.uniform(-0.1, 0.1) * s, z))
    return np.array(pts)


def add_lights(stage):
    from pxr import Gf, UsdGeom, UsdLux
    dome = UsdLux.DomeLight.Define(stage, "/World/Sky")
    dome.CreateIntensityAttr(800.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(2500.0)
    sun.CreateAngleAttr(0.53)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-50.0, 0.0, 20.0))
