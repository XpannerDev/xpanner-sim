#!/usr/bin/env python3
"""
apply_visuals.py -- put the sim/visuals.py look on a USD scene file WITHOUT editing that file.

    ./scripts/run_isaac.sh python /work/xpanner-sim/scripts/apply_visuals.py \
        --stage /work/xpanner-sim/assets/site/solar_site.usd \
        --out   /work/xpanner-sim/assets/site/solar_site_visuals.usda [--screenshot /work/.../shot.png]

The output is a NEW layer whose only sublayer is the input stage; every material, light and binding opinion
lands in the output layer, so opening the original file gives the original look and deleting the output file
is the whole revert. Collision / physics / joints / mass are not touched (see sim/visuals.py).
For the loading demo (a scene built in memory) use `isaac_loading_demo.py --visuals` instead.
"""
import argparse
import os
import sys

sys.path.insert(0, "/work/xpanner-sim")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--screenshot", default="")
    ap.add_argument("--eye", default="-9,-13,8.5")
    ap.add_argument("--target", default="2.5,2.5,0.8")
    args = ap.parse_args()

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    from pxr import Sdf, Usd
    from isaacsim.storage.native import get_assets_root_path
    from sim import visuals

    out_layer = Sdf.Layer.CreateNew(args.out)
    out_layer.subLayerPaths.append(os.path.relpath(args.stage, os.path.dirname(os.path.abspath(args.out))))
    stage = Usd.Stage.Open(out_layer)
    stage.SetEditTarget(out_layer)
    touched = visuals.apply(stage, get_assets_root_path())
    out_layer.Save()
    print(f"[apply_visuals] {args.out}: {len(touched['bound'])} bindings, created {touched['created']}, "
          f"hidden {touched['hidden']}", flush=True)

    if args.screenshot:
        import cv2
        import numpy as np
        import omni.usd
        from sim.isaac_sensors import Spectator
        omni.usd.get_context().open_stage(args.out)
        import omni.timeline
        omni.timeline.get_timeline_interface().play()
        eye = [float(v) for v in args.eye.split(",")]
        tgt = [float(v) for v in args.target.split(",")]
        spec = Spectator(eye=eye, target=tgt, resolution=(1280, 720))
        spec.initialize()
        for _ in range(40):
            app.update()
        img = spec.rgb()
        cv2.imwrite(args.screenshot, cv2.cvtColor(np.ascontiguousarray(img), cv2.COLOR_RGB2BGR))
        print(f"[apply_visuals] screenshot -> {args.screenshot}", flush=True)
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
