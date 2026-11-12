#!/bin/bash
# Start the koide3/direct_visual_lidar_calibration container (not part of this repository).
#   ./run_calibration_container.sh <rig.env> <bag_dir> <result_dir> [humble|jazzy|noetic]
# Tag = ROS of the bags: ROS 2 bags -> humble / jazzy, ROS 1 bags -> noetic.
# Inside the container: /tmp/calib_repo/scripts/calibrate_in_container.sh [step]
set -euo pipefail
CFG="$(realpath "${1:?rig.env}")"; BAGS="$(realpath "${2:?bag dir}")"; RESULT="$(realpath -m "${3:?result dir}")"; TAG="${4:-humble}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$RESULT"
docker pull koide3/direct_visual_lidar_calibration:"$TAG"
xhost +local:root >/dev/null 2>&1 || true
docker run --rm -it --net host --gpus all \
  -e DISPLAY="${DISPLAY:-}" -v "$HOME/.Xauthority:/root/.Xauthority" \
  -v "$BAGS:/tmp/input_bags" -v "$RESULT:/tmp/preprocessed" \
  -v "$CFG:/tmp/rig.env:ro" -v "$REPO:/tmp/calib_repo:ro" \
  koide3/direct_visual_lidar_calibration:"$TAG" bash
