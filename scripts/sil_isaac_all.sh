#!/bin/bash
# Run the three Isaac firmware-in-the-loop scenarios (step 28, PreparePick, Positioning) on the step-28 USD.
# Reports land in build/isaac/*_report.json, logs in build/isaac/*.log. Run on the host.
C=isaac-sim-jude; W=/work/xpanner-sim; U=$W/build/isaac/ecr88_kijang_step28.usd
cd /home/ubuntu/jude/xpanner-sim
docker exec $C /isaac-sim/python.sh $W/scripts/sil_isaac_step28.py --usd $U --pose $W/build/isaac/step28_pose.json --report $W/build/isaac/step28_report.json > build/isaac/run28.log 2>&1; echo "STEP28 $?"
docker exec $C /isaac-sim/python.sh $W/scripts/sil_isaac_prepare_pick.py --usd $U --report $W/build/isaac/prepare_pick_report.json > build/isaac/prepare_pick.log 2>&1; echo "PREPARE_PICK $?"
docker exec $C /isaac-sim/python.sh $W/scripts/sil_isaac_positioning.py --usd $U --report $W/build/isaac/positioning_report.json > build/isaac/positioning.log 2>&1; echo "POSITIONING $?"
echo ALL_DONE
