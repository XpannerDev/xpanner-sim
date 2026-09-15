#!/bin/bash
# Rebuild every USD (docs/ISAAC_SIM_REMOTE.md 6.3 + step-28 USD) in the Isaac container. Run on the host after
# python3 scripts/sil_step28_prepare.py and the two xacro expansions (build/ecr88.urdf, build/ecr88_nocyl.urdf).
set -u
C=isaac-sim-jude; W=/work/xpanner-sim
RP="--rest-pose boom_joint=-30 --rest-pose arm_joint=110 --rest-pose bucket_joint=20"
cd /home/ubuntu/jude/xpanner-sim
run() { echo "=== $*"; docker exec $C /isaac-sim/python.sh "$@" 2>&1 | grep -a -E "^\[|Error|Traceback|error:|converged|미수렴|수렴|joint" | tail -25; echo "=== exit ${PIPESTATUS[0]}"; }
run $W/scripts/urdf_to_usd.py --urdf $W/build/isaac/ecr88_kijang_step28.urdf --output $W/build/isaac/ecr88_kijang_step28.usd --rest-pose swing_joint=0.0 --rest-pose boom_joint=-35.539 --rest-pose arm_joint=139.941 --rest-pose bucket_joint=-32.463
run $W/scripts/urdf_to_usd.py --urdf $W/build/ecr88.urdf --output $W/assets/ecr88/usd/ecr88.usd $RP
run $W/scripts/urdf_to_usd.py --urdf $W/build/ecr88_nocyl.urdf --output $W/assets/ecr88/usd/ecr88_physics.usd $RP
rm -f assets/site/solar_site.usd
run $W/scripts/build_site.py --robot $W/assets/ecr88/usd/ecr88.usd --output $W/assets/site/solar_site.usd
run $W/scripts/animate_cycle.py --stage $W/assets/site/solar_site.usd
run $W/scripts/add_cameras.py --stage $W/assets/site/solar_site.usd
run $W/scripts/check_stability.py $W/assets/ecr88/usd/ecr88_physics.usd
grep -rh "jointFriction" build/isaac/ecr88_kijang_step28_pkg assets/ecr88/usd/*_pkg --include=physx.usda | sort | uniq -c
echo REBUILD_DONE
