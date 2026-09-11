#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_stability.py -- step the ECR88 under physics and report what each joint does.

Written while chasing "cyl_arm_barrel is flying around". It exists because the
obvious reading of that symptom -- the solver is diverging -- was WRONG, and only
a per-joint trace showed why.

WHAT IT MEASURES, AND WHY TWO NUMBERS
-------------------------------------
  전체 p-p  peak-to-peak over the whole run. Includes the settling transient, so a
            large value here means the machine MOVED, not that it is unstable.
  말미 p-p  peak-to-peak over the last 80 steps. This is the one that separates a
            converged pose from a joint that is still ringing. Above ~0.02 rad the
            joint has not settled.

Judging on the first number alone is what makes a settling transient look like an
explosion. On this asset every joint converges; what looked violent was the
articulation STARTING somewhere other than its authored rest pose and then swinging
to correct -- bucket_joint opened at 103.8 deg against a requested 20 deg.

KNOWN, AND NOT A BUG: the settled pose sags below the commanded one. A pure
stiffness position drive has a steady-state error of torque/stiffness, and about
boom_joint that is 29 945 N.m / 174 533 N.m/rad = 0.17 rad ~ 10 deg, which is what
the trace shows. Raising stiffness or adding gravity compensation is the fix; more
damping is not, and was tried.

USAGE
    docker exec <container> /isaac-sim/python.sh \
        /work/xpanner-sim/scripts/check_stability.py <robot.usd>
"""
import sys
from isaacsim import SimulationApp
app=SimulationApp({"headless":True})
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.stage import open_stage, add_reference_to_stage
from pxr import UsdGeom, Gf
import isaacsim.core.utils.stage as su
w=World(stage_units_in_meters=1.0)
w.scene.add_default_ground_plane()
add_reference_to_stage(sys.argv[1], "/World/ECR88")
st=su.get_current_stage()
UsdGeom.Xformable(st.GetPrimAtPath("/World/ECR88")).AddTranslateOp().Set(Gf.Vec3d(0,0,1.445))
w.reset()
from isaacsim.core.prims import Articulation
art=Articulation("/World/ECR88"); art.initialize()
names=art.dof_names
# Watch every articulated joint, not a list somebody has to remember to extend.
# The hardcoded three silently skipped dozer_joint and boom_swing_joint the day
# they were added, which is exactly when a new joint most needs watching.
watch=[n for n in names]
idx={n:names.index(n) for n in watch if n in names}
hist={n:[] for n in idx}
for i in range(400):
    w.step(render=False)
    q=art.get_joint_positions()[0]
    for n,k in idx.items(): hist[n].append(float(q[k]))
print(f"STAB2 === {sys.argv[1]}")
bad=0
for n in idx:
    h=np.array(hist[n]); pp=h.max()-h.min(); tail=h[-80:]; tpp=tail.max()-tail.min()
    # 전체 p-p 는 정착 과도까지 포함한다. 마지막 80 스텝의 p-p 가 작으면 수렴한 것이고,
    # 크면 계속 떨고 있는 것이다. 둘을 구분해야 "새그"와 "발산"을 혼동하지 않는다.
    verdict = "발산/진동" if tpp>0.02 else "수렴"
    if tpp>0.02: bad+=1
    print(f"STAB2 {n:18s} start {h[0]:+8.4f} end {h[-1]:+8.4f}  전체p-p {pp:7.4f}  말미p-p {tpp:7.4f}  {verdict}")
print(f"STAB2 미수렴 {bad}개 / {len(idx)}")
app.close()
