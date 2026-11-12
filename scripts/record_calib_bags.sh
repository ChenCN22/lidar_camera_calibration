#!/bin/bash
# Record one ROS 2 calibration bag per viewpoint (camera image + camera info + lidar cloud).
#   ./record_calib_bags.sh <out_dir> <name> [duration_s]
# Keep the rig still (or move it very slowly) while recording; take 3-4 viewpoints of a
# textured, structured scene with objects 2-6 m away and no moving people.
set -euo pipefail
OUT="${1:?out dir}"; NAME="${2:?bag name}"; DUR="${3:-30}"
IMAGE_TOPIC="${IMAGE_TOPIC:-/camera_front}"
INFO_TOPIC="${INFO_TOPIC:-/camera_info_omnidir}"
POINTS_TOPIC="${POINTS_TOPIC:-/ouster/points}"
mkdir -p "$OUT"
echo "recording $DUR s -> $OUT/$NAME  ($IMAGE_TOPIC $INFO_TOPIC $POINTS_TOPIC)"
timeout --signal=INT "$DUR" ros2 bag record -o "$OUT/$NAME" "$IMAGE_TOPIC" "$INFO_TOPIC" "$POINTS_TOPIC" || true
ros2 bag info "$OUT/$NAME"
