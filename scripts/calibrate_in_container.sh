#!/bin/bash
# Run inside the koide3 container (see run_calibration_container.sh).
#   /tmp/scripts/calibrate_in_container.sh <intrinsics_json> [step]
# step: all (default) | setup | preprocess | matches | guess | calibrate | viewer
# <intrinsics_json> is a results/intrinsics/*.json file (fx fy cx cy xi k1 k2 p1 p2).
set -euo pipefail
INTR="${1:?intrinsics json}"; STEP="${2:-all}"
BAGS="${BAGS:-/tmp/input_bags}"; DST="${DST:-/tmp/preprocessed}"
IMAGE_TOPIC="${IMAGE_TOPIC:-/camera_front}"
INFO_TOPIC="${INFO_TOPIC:-/camera_info_omnidir}"
POINTS_TOPIC="${POINTS_TOPIC:-/ouster/points}"
VOXEL="${VOXEL:-0.008}"            # Ouster ROS 2 driver range quantisation (m)
SUPERGLUE_WEIGHTS="${SUPERGLUE_WEIGHTS:-indoor}"
read -r INTRINSICS DISTORTION < <(python3 - "$INTR" <<'PY'
import json, sys
j = json.load(open(sys.argv[1]))
print(",".join("%.6f" % j[k] for k in ("fx", "fy", "cx", "cy", "xi")),
      ",".join("%.8f" % j[k] for k in ("k1", "k2", "p1", "p2")))
PY
)
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
run() { echo "+ $*"; "$@"; }
if [[ "$STEP" == all || "$STEP" == setup ]]; then
  # SuperGlue is under a non-commercial licence: the user downloads it, it is not shipped here.
  apt-get update && apt-get install -y python3-pip
  pip3 install numpy opencv-python torch matplotlib
  [ -d ~/SuperGluePretrainedNetwork ] || git clone https://github.com/magicleap/SuperGluePretrainedNetwork.git ~/SuperGluePretrainedNetwork
fi
export PYTHONPATH="${PYTHONPATH:-}:$HOME/SuperGluePretrainedNetwork"
if [[ "$STEP" == all || "$STEP" == preprocess ]]; then
  run ros2 run direct_visual_lidar_calibration preprocess \
    --data_path "$BAGS" --dst_path "$DST" \
    --image_topic "$IMAGE_TOPIC" --points_topic "$POINTS_TOPIC" --camera_info_topic "$INFO_TOPIC" \
    --camera_model omnidir --camera_intrinsics "$INTRINSICS" --camera_distortion_coeffs "$DISTORTION" \
    --voxel_resolution "$VOXEL" --dynamic_lidar_integration --verbose --visualize
fi
if [[ "$STEP" == all || "$STEP" == matches ]]; then
  run ros2 run direct_visual_lidar_calibration find_matches_superglue.py "$DST" --superglue "$SUPERGLUE_WEIGHTS" --sinkhorn_iterations 25
fi
if [[ "$STEP" == all || "$STEP" == guess ]]; then
  run ros2 run direct_visual_lidar_calibration initial_guess_auto "$DST" --ransac_error_thresh 6 --robust_kernel_width 6
fi
if [[ "$STEP" == all || "$STEP" == calibrate ]]; then
  run ros2 run direct_visual_lidar_calibration calibrate "$DST"
fi
if [[ "$STEP" == all || "$STEP" == viewer ]]; then
  run ros2 run direct_visual_lidar_calibration viewer "$DST"
fi
echo "result: $DST/calib.json"
