#!/bin/bash
# Start the koide3 direct_visual_lidar_calibration container with the bag and result
# directories mounted. The toolbox itself is NOT part of this repository.
#   ./run_calibration_container.sh <bag_dir> <result_dir> [image_tag]
# Inside the container run /tmp/calib_repo/scripts/calibrate_in_container.sh (repo mounted read-only).
set -euo pipefail
BAGS="$(realpath "${1:?bag dir}")"; RESULT="$(realpath -m "${2:?result dir}")"; TAG="${3:-humble}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$RESULT"
docker pull koide3/direct_visual_lidar_calibration:"$TAG"
xhost +local:root >/dev/null 2>&1 || true
docker run --rm -it --net host --gpus all \
  -e DISPLAY="$DISPLAY" -v "$HOME/.Xauthority:/root/.Xauthority" \
  -v "$BAGS:/tmp/input_bags" -v "$RESULT:/tmp/preprocessed" -v "$REPO:/tmp/calib_repo:ro" \
  koide3/direct_visual_lidar_calibration:"$TAG" bash
