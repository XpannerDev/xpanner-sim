"""
visuals.py -- LOOK ONLY. Replaces the grid floor, the lights and the flat colours of the loading-demo scene with
PBR materials, an HDRI dome and an oblique sun. It never touches collision, physics, joints or mass:

  * everything it creates lives under /World/Looks and /World/VisualGround (a cube WITHOUT a collision API);
  * on existing prims it only writes `material:binding` (a relationship) and, on the default ground plane,
    `visibility` (render-only; PhysX keeps colliding with an invisible prim -- that is why hide_panellift()
    in isaac_site.py has to switch collision off separately);
  * revert() removes exactly those opinions again, and the loading demo only calls apply() behind --visuals.

Assets come from the Isaac asset root (get_assets_root_path(), the 6.0 S3 bucket): Base/Natural/Dirt.mdl,
Base/Stone/Gravel.mdl, Skies/Cloudy/kloofendal_48d_partly_cloudy_4k.hdr. Everything else is OmniPBR with
constants. The look is a choice, not a measurement: no paint code or material sheet exists for this machine.
"""
LOOKS = "/World/Looks"
VISUAL_GROUND = "/World/VisualGround"
DEFAULT_GROUND = "/World/defaultGroundPlane"

HDRI = "/NVIDIA/Assets/Skies/Cloudy/kloofendal_48d_partly_cloudy_4k.hdr"
GROUND_MDL = ("/NVIDIA/Materials/Base/Natural/Dirt.mdl", "Dirt")
BENCH_MDL = ("/NVIDIA/Materials/Base/Natural/Soil_Rocky.mdl", "Soil_Rocky")   # Gravel.mdl rendered as a white slab

# name: (diffuse rgb, roughness, metallic, specular)
PBR = dict(
    HeavyYellow=((0.52, 0.38, 0.07), 0.50, 0.00, 0.40),   # dirty heavy-equipment yellow (0.66/0.50 read as pastel under the HDRI)
    SteelDark=((0.15, 0.15, 0.16), 0.80, 0.55, 0.40),     # tracks, undercarriage, chassis
    MetalBare=((0.56, 0.57, 0.58), 0.32, 1.00, 0.60),     # bucket edge, pins, bare steel
    CabDark=((0.11, 0.12, 0.13), 0.22, 0.10, 0.60),       # tinted cab glass / dark trim (was sky blue)
    Neutral=((0.46, 0.46, 0.46), 0.60, 0.00, 0.40),       # markers that used to be red/pink
    TruckGrey=((0.30, 0.31, 0.32), 0.55, 0.40, 0.45),
    TruckCab=((0.70, 0.71, 0.72), 0.40, 0.20, 0.45),
    Rubber=((0.04, 0.04, 0.04), 0.90, 0.00, 0.30),
    PersonGrey=((0.55, 0.55, 0.55), 0.80, 0.00, 0.30),
    SoilBrown=((0.27, 0.19, 0.12), 0.95, 0.00, 0.15),     # darker than the dirt floor so the pile reads as a pile
)

SUN = dict(intensity=1500.0, angle=0.53, rotate_xyz=(-38.0, 0.0, 35.0), color=(1.0, 0.96, 0.90))
DOME_INTENSITY = 400.0      # 800 + 2500 overexposed the spectator view: grey 0.30 rendered as 0.6 (check 10-02)

UNDERCARRIAGE = ("base_link", "track_left_link", "track_right_link", "dozer_link")
BY_BINDING = dict(xpanner_yellow="HeavyYellow", xpanner_dark="SteelDark", xpanner_steel="MetalBare",
                  xpanner_accent="Neutral", xpanner_frame="CabDark", xpanner_panel="Neutral")


def _pbr(stage, name):
    from pxr import Gf, Sdf, UsdShade
    path = f"{LOOKS}/{name}"
    if stage.GetPrimAtPath(path):
        return UsdShade.Material(stage.GetPrimAtPath(path))
    rgb, rough, metal, spec = PBR[name]
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/Shader")
    sh.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
    sh.SetSourceAsset("OmniPBR.mdl", "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(float(rough))
    sh.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(float(metal))
    sh.CreateInput("specular_level", Sdf.ValueTypeNames.Float).Set(float(spec))
    for out in ("surface", "displacement", "volume"):
        getattr(mat, f"Create{out.capitalize()}Output")("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    return mat


def _mdl(stage, name, asset, subid, texture_scale=1.0):
    from pxr import Gf, Sdf, UsdShade
    path = f"{LOOKS}/{name}"
    if stage.GetPrimAtPath(path):
        return UsdShade.Material(stage.GetPrimAtPath(path))
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/Shader")
    sh.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
    sh.SetSourceAsset(asset, "mdl")
    sh.SetSourceAssetSubIdentifier(subid, "mdl")
    # world-space planar projection so a 60 m slab tiles without UVs; inputs the MDL lacks are ignored
    sh.CreateInput("project_uvw", Sdf.ValueTypeNames.Bool).Set(True)
    sh.CreateInput("world_or_object", Sdf.ValueTypeNames.Bool).Set(True)
    sh.CreateInput("texture_scale", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(texture_scale, texture_scale))
    for out in ("surface", "displacement", "volume"):
        getattr(mat, f"Create{out.capitalize()}Output")("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    return mat


def _bind(prim, mat):
    from pxr import UsdShade
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)


def _binding_name(prim):
    rel = prim.GetRelationship("material:binding")
    if rel and rel.GetTargets():
        return rel.GetTargets()[0].name
    return None


def apply(stage, assets_root, machine_root="/World/ECR88", truck_root="/World/Truck", bench="/World/Bench",
          person="/World/Person", soil="/World/Soil", site_ground="/World/Ground", ground_size=80.0):
    """Apply the look. Returns a dict of what was touched (for the report and for revert())."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade
    touched = dict(bound=[], hidden=[], created=[], lights=[])
    UsdGeom.Scope.Define(stage, LOOKS)
    touched["created"].append(LOOKS)

    # -- ground: hide the grid asset's look (collision untouched), lay a visual-only dirt slab on it
    ground_mat = _mdl(stage, "GroundDirt", assets_root + GROUND_MDL[0], GROUND_MDL[1], texture_scale=0.25)
    gp = stage.GetPrimAtPath(DEFAULT_GROUND)
    if gp:
        UsdGeom.Imageable(gp).MakeInvisible()
        touched["hidden"].append(DEFAULT_GROUND)
        slab = UsdGeom.Cube.Define(stage, VISUAL_GROUND)
        slab.CreateSizeAttr(1.0)
        x = UsdGeom.Xformable(slab)
        x.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.011))
        x.AddScaleOp().Set(Gf.Vec3f(ground_size, ground_size, 0.02))
        _bind(slab.GetPrim(), ground_mat)
        touched["created"].append(VISUAL_GROUND)
    sg = stage.GetPrimAtPath(site_ground)
    if sg:                                       # solar_site.usd: its ground cube just gets the material
        _bind(sg, ground_mat)
        touched["bound"].append(site_ground)

    # -- lights
    sky = stage.GetPrimAtPath("/World/Sky")
    if not sky:
        sky = UsdLux.DomeLight.Define(stage, "/World/Sky").GetPrim()
        touched["created"].append("/World/Sky")
    dome = UsdLux.DomeLight(sky)
    dome.CreateTextureFileAttr().Set(assets_root + HDRI)
    dome.CreateIntensityAttr().Set(DOME_INTENSITY)
    dome.CreateTextureFormatAttr().Set("latlong")
    touched["lights"].append("/World/Sky")
    sun = stage.GetPrimAtPath("/World/Sun")
    if not sun:
        sun = UsdLux.DistantLight.Define(stage, "/World/Sun").GetPrim()
        touched["created"].append("/World/Sun")
    dl = UsdLux.DistantLight(sun)
    dl.CreateIntensityAttr().Set(SUN["intensity"])
    dl.CreateAngleAttr().Set(SUN["angle"])
    dl.CreateColorAttr().Set(Gf.Vec3f(*SUN["color"]))
    xf = UsdGeom.Xformable(sun)
    ops = [op for op in xf.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ]
    (ops[0] if ops else xf.AddRotateXYZOp()).Set(Gf.Vec3f(*SUN["rotate_xyz"]))
    touched["lights"].append("/World/Sun")

    # -- the machine: by undercarriage link, else by the URDF material it came with
    root = stage.GetPrimAtPath(machine_root)
    if root:
        for prim in Usd.PrimRange(root):
            if not prim.IsA(UsdGeom.Gprim):
                continue
            path = str(prim.GetPath())
            chain = []
            p = prim.GetParent()
            while p and p.GetPath() != root.GetPath():
                chain.append(p.GetName())
                p = p.GetParent()
            if "bucket" in chain:
                mat = _pbr(stage, "MetalBare")
            elif any(c in UNDERCARRIAGE for c in chain[:1]):
                mat = _pbr(stage, "SteelDark")
            else:
                mat = _pbr(stage, BY_BINDING.get(_binding_name(prim), "HeavyYellow"))
            _bind(prim, mat)
            touched["bound"].append(path)

    # -- truck, bench, person, soil
    tr = stage.GetPrimAtPath(truck_root)
    if tr:
        for prim in Usd.PrimRange(tr):
            if not prim.IsA(UsdGeom.Gprim):
                continue
            n = prim.GetName()
            mat = _pbr(stage, "Rubber" if n.startswith("wheel") else "TruckCab" if n == "cab" else
                       "SteelDark" if n == "chassis" else "TruckGrey")
            _bind(prim, mat)
            touched["bound"].append(str(prim.GetPath()))
    bp = stage.GetPrimAtPath(bench)
    if bp:
        _bind(bp, _mdl(stage, "BenchGravel", assets_root + BENCH_MDL[0], BENCH_MDL[1], texture_scale=0.5))
        touched["bound"].append(bench)
    pp = stage.GetPrimAtPath(person)
    if pp:
        for prim in Usd.PrimRange(pp):
            if prim.IsA(UsdGeom.Gprim):
                _bind(prim, _pbr(stage, "PersonGrey"))
                touched["bound"].append(str(prim.GetPath()))
    sp = stage.GetPrimAtPath(soil)
    if sp:
        for prim in Usd.PrimRange(sp):
            if prim.IsA(UsdGeom.Sphere):          # the PointInstancer prototypes
                _bind(prim, _pbr(stage, "SoilBrown"))
                touched["bound"].append(str(prim.GetPath()))
    return touched


def revert(stage, touched):
    """Undo apply(): drop the bindings and visibility opinions, delete what was created."""
    from pxr import UsdGeom
    for path in touched.get("bound", []):
        prim = stage.GetPrimAtPath(path)
        if prim and prim.HasProperty("material:binding"):
            prim.RemoveProperty("material:binding")
    for path in touched.get("hidden", []):
        prim = stage.GetPrimAtPath(path)
        if prim:
            UsdGeom.Imageable(prim).MakeVisible()
    for path in reversed(touched.get("created", [])):
        if stage.GetPrimAtPath(path):
            stage.RemovePrim(path)
