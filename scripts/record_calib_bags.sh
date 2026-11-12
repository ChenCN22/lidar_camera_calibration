#!/bin/bash
# Record one calibration bag (image + point cloud [+ camera info]) with the ROS that is sourced (1 or 2).
#   ./record_calib_bags.sh <rig.env> <out_dir> <name> [duration_s]
# Spinning lidar: start at rest, then tilt the rig slowly up and down. Non-repetitive lidar: keep it still.
set -eo pipefail
CFG="${1:?rig.env}"; OUT="${2:?out dir}"; NAME="${3:?bag name}"; DUR="${4:-30}"
source "$CFG"
TOPICS=("${IMAGE_TOPIC:?}" "${POINTS_TOPIC:?}")
[ -n "${INFO_TOPIC:-}" ] && TOPICS+=("$INFO_TOPIC")
mkdir -p "$OUT"
echo "recording $DUR s -> $OUT/$NAME  (${TOPICS[*]})"
case "${ROS_VERSION:-}" in
  1) rosbag record --duration="$DUR" -O "$OUT/$NAME.bag" "${TOPICS[@]}"; rosbag info "$OUT/$NAME.bag" ;;
  2) timeout --signal=INT "$DUR" ros2 bag record -o "$OUT/$NAME" "${TOPICS[@]}" || true; ros2 bag info "$OUT/$NAME" ;;
  *) echo "source your ROS setup.bash first (ROS_VERSION is not set)"; exit 1 ;;
esac
