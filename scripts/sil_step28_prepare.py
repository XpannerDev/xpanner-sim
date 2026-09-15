#!/usr/bin/env python3
"""
sil_step28_prepare.py -- host-side prep for scripts/sil_isaac_step28.py, in one command.

    python3 scripts/sil_step28_prepare.py        # then run the docker exec lines it prints

1. expands the 1.7 m variant (the repo binary's unit) without cylinders and without the panel
   stack (step 28 is recorded against the bare backrest) into build/isaac/;
2. plans the pose with sil.ik: contact surface 0.5 m in front of fork_back_link, tool Z along the
   blade, swing held at 0 so it agrees with isSwingAligned = true;
3. writes build/isaac/step28_pose.json and prints the two container commands (USD conversion
   with that rest pose, then the run).
"""
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
from sil import ik  # noqa: E402
from sil.urdf_fk import UrdfModel  # noqa: E402

OUT = REPO / "build" / "isaac"
STANDOFF = 0.5


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    OUT.chmod(0o777)                          # the container runs as uid 1234
    urdf = OUT / "ecr88_kijang_step28.urdf"
    subprocess.run(["xacro", str(REPO / "assets/ecr88/urdf/ecr88.urdf.xacro"), "machine_variant:=ECR88_KIJANG",
                    "model_cylinders:=false", "model_panel_stack:=false", "-o", str(urdf)], check=True)
    m = UrdfModel(urdf)
    Tfb = m.fk("fork_back_link", {})
    c, s = math.cos(math.pi / 2), math.sin(math.pi / 2)
    z = (Tfb[:3, :3] @ np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]))[:, 2]
    target = Tfb[:3, 3] + STANDOFF * z
    target[1] = m.fk("contact_surface_link", ik.q_of(0, 0, 0, 0))[1, 3]   # the tool's own plane at swing 0
    res = ik.solve(m, target, z, swing_deg=0.0)
    if res is None or res[1] > 0.002:
        raise SystemExit(f"[step28] no pose within limits ({res})")
    pose = {k: round(float(v), 3) for k, v in res[0].items()}
    (OUT / "step28_pose.json").write_text(json.dumps(pose))
    print(f"[step28] pose {pose}  (position residual {res[1]*1000:.2f} mm)")
    W = "/work/xpanner-sim"
    rp = " ".join(f"--rest-pose {j}={pose[k]}" for j, k in
                  (("swing_joint", "swing"), ("boom_joint", "boom"), ("arm_joint", "arm"), ("bucket_joint", "bucket")))
    print(f"docker exec isaac-sim-jude /isaac-sim/python.sh {W}/scripts/urdf_to_usd.py "
          f"--urdf {W}/build/isaac/{urdf.name} --output {W}/build/isaac/ecr88_kijang_step28.usd {rp}")
    print(f"docker exec isaac-sim-jude /isaac-sim/python.sh {W}/scripts/sil_isaac_step28.py "
          f"--usd {W}/build/isaac/ecr88_kijang_step28.usd --pose {W}/build/isaac/step28_pose.json "
          f"--report {W}/build/isaac/step28_report.json")


if __name__ == "__main__":
    main()
