#!/usr/bin/env python3
"""
isaac_stream_scene.py -- stream a USD scene over WebRTC from a python-driven Isaac Sim app.

WHY THIS EXISTS (2026-09-16). In the stock streaming app (`run_isaac.sh stream`) opening a scene from
the GUI kills the process: closing the default anonymous stage logs

    [omni.usd] Unexpected reference count of 2 for UsdStage 'anon:...:World0.usd' while being closed

and ~2 minutes later Python's garbage collector destroys that leaked stage and the process segfaults in
UsdStage::~UsdStage (Isaac Sim 6.0.1, exit 139, twice, backtrace through Tf_PyOwnershipHelper).
Suppressing the empty stage (`--/app/content/emptyStageOnStart=false`) deadlocks instead: with no stage
the viewport never renders a first frame, the app never reports ready, and `--exec` (which runs after
startup) never fires.

So this script owns the order: start the app, turn the livestream on, open the scene ONCE, then just
update. No stage is ever closed, so the crash path is never taken.

    ./scripts/run_isaac.sh python /work/xpanner-sim/scripts/isaac_stream_scene.py \
        [--scene /work/xpanner-sim/assets/site/solar_site.usd]

The livestream host/ports come from the environment (run_isaac.sh python exports them, same values as
its `stream` subcommand). Connect with the Isaac Sim WebRTC Streaming Client 2.0.0 to the host IP.
"""
import argparse
import os
import sys

DEFAULT_SCENE = "/work/xpanner-sim/assets/site/solar_site.usd"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default=DEFAULT_SCENE)
    ap.add_argument("--host", default=os.environ.get("ISAACSIM_HOST", ""))
    ap.add_argument("--signal-port", type=int, default=int(os.environ.get("ISAACSIM_SIGNAL_PORT", 49100)))
    ap.add_argument("--stream-port", type=int, default=int(os.environ.get("ISAACSIM_STREAM_PORT", 47998)))
    args = ap.parse_args()

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})

    import carb
    import omni.usd
    from isaacsim.core.utils.extensions import enable_extension

    # Settings first: the livestream extension reads them when it starts.
    s = carb.settings.get_settings()
    if args.host:
        s.set("/exts/omni.kit.livestream.app/primaryStream/publicIp", args.host)
    s.set("/exts/omni.kit.livestream.app/primaryStream/signalPort", args.signal_port)
    s.set("/exts/omni.kit.livestream.app/primaryStream/streamPort", args.stream_port)
    s.set("/app/window/drawMouse", True)
    enable_extension("omni.kit.livestream.app")
    app.update()

    res = omni.usd.get_context().open_stage(args.scene)   # bool here, (ok, err) in other versions
    ok = res[0] if isinstance(res, tuple) else bool(res)
    print(f"[stream_scene] open_stage({args.scene}) -> {res}", flush=True)
    if not ok:
        app.close()
        sys.exit(f"could not open {args.scene}")
    for _ in range(60):                      # let the stage and the renderer settle
        app.update()
    print(f"[stream_scene] streaming {args.scene} on {args.host or '<host ip>'} "
          f"tcp/{args.signal_port} udp/{args.stream_port} -- connect the WebRTC client now", flush=True)

    try:
        while app.is_running():
            app.update()
    except KeyboardInterrupt:
        pass
    finally:
        app.close()


if __name__ == "__main__":
    main()
