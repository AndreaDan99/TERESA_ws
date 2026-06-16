#!/bin/bash
# Live perception capture sweep (run INSIDE teresa_gpu). Sequential (shared GPU).
# Each node self-terminates at --max-frames, writing mp4 + overlays + metrics.json.
# NOTE: no `set -u` — ROS setup.bash references unbound vars (COLCON_TRACE).
source /opt/ros/humble/install/setup.bash 2>/dev/null
P=/work/sensor_nodes
C=/work/cap
mkdir -p $C
NLF=/work/models/nlf_s_multi_v020.torchscript
YOLO=/work/models/yolo11x-pose.pt

run() { echo ">>> $1"; shift; "$@"; echo "<<< done ($?)"; }

echo "===== ORBBEC (Spot cam) ====="
run "orbbec pose"  python3 $P/pose_ros_node.py  --topic /orbbec/color/image_raw --model $YOLO \
    --save-dir $C/orbbec_pose --video $C/orbbec_pose.mp4 --out-fps 10 --max-frames 120 > $C/orbbec_pose.log 2>&1
run "orbbec nlf"   python3 $P/nlf_ros_node.py   --topic /orbbec/color/image_raw --model $NLF \
    --save-dir $C/orbbec_nlf --video $C/orbbec_nlf.mp4 --out-fps 6 --rate 8 --max-frames 60 > $C/orbbec_nlf.log 2>&1
run "orbbec wound" python3 $P/wound_depth_node.py --color /orbbec/color/image_raw --depth /orbbec/depth/image_raw \
    --info /orbbec/color/camera_info --method gdino --save-dir $C/orbbec_wound --video $C/orbbec_wound.mp4 \
    --out-fps 2 --period 1.2 --max-frames 12 > $C/orbbec_wound.log 2>&1

echo "===== REALSENSE (arm cam) ====="
run "rs pose"  python3 $P/pose_ros_node.py  --topic /camera/camera/color/image_raw --model $YOLO \
    --save-dir $C/rs_pose --video $C/rs_pose.mp4 --out-fps 10 --max-frames 120 > $C/rs_pose.log 2>&1
run "rs nlf"   python3 $P/nlf_ros_node.py   --topic /camera/camera/color/image_raw --model $NLF \
    --save-dir $C/rs_nlf --video $C/rs_nlf.mp4 --out-fps 6 --rate 8 --max-frames 50 > $C/rs_nlf.log 2>&1
run "rs wound" python3 $P/wound_depth_node.py --color /camera/camera/color/image_raw \
    --depth /camera/camera/aligned_depth_to_color/image_raw --info /camera/camera/color/camera_info \
    --method gdino --save-dir $C/rs_wound --video $C/rs_wound.mp4 --out-fps 2 --period 1.2 --max-frames 12 > $C/rs_wound.log 2>&1

echo "ALL_DONE" > $C/DONE
echo "===== CAPTURE SWEEP COMPLETE ====="
