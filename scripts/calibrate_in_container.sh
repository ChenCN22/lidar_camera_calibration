#!/bin/bash
# Run inside the koide3 container (see run_calibration_container.sh).
#   calibrate_in_container.sh [all|setup|preprocess|guess|calibrate|viewer]
# Env: CFG (/tmp/rig.env), BAGS (/tmp/input_bags), DST (/tmp/preprocessed),
#      HEADLESS=1 (no GUI windows; calibrate still needs an X display, a virtual one such as Xvfb is enough).
set -eo pipefail
STEP="${1:-all}"
CFG="${CFG:-/tmp/rig.env}"; BAGS="${BAGS:-/tmp/input_bags}"; DST="${DST:-/tmp/preprocessed}"; HEADLESS="${HEADLESS:-0}"
source "$CFG"
source "/opt/ros/$ROS_DISTRO/setup.bash"
for ws in "$HOME/ros2_ws/install" "$HOME/catkin_ws/devel"; do
  [ -f "$ws/setup.bash" ] && source "$ws/setup.bash"
done
if [ "$ROS_VERSION" = 1 ]; then RUN=(rosrun direct_visual_lidar_calibration); else RUN=(ros2 run direct_visual_lidar_calibration); fi
run() { echo "+ $*"; "$@"; }
want() { [[ "$STEP" == all || "$STEP" == "$1" ]]; }

if want setup && [ "${INITIAL_GUESS:-superglue}" = superglue ]; then
  # SuperGlue (non-commercial licence) is downloaded by the user here; it is not shipped with this repository.
  apt-get update && apt-get install -y python3-pip git
  pip3 install numpy opencv-python torch matplotlib
  [ -d "$HOME/SuperGluePretrainedNetwork" ] || git clone https://github.com/magicleap/SuperGluePretrainedNetwork.git "$HOME/SuperGluePretrainedNetwork"
fi
export PYTHONPATH="${PYTHONPATH:-}:$HOME/SuperGluePretrainedNetwork"

if want preprocess; then
  ARGS=(--data_path "$BAGS" --dst_path "$DST" --image_topic "$IMAGE_TOPIC" --points_topic "$POINTS_TOPIC" --verbose)
  [ -n "${INFO_TOPIC:-}" ] && ARGS+=(--camera_info_topic "$INFO_TOPIC")
  if [ "${CAMERA_MODEL:-auto}" != auto ]; then
    ARGS+=(--camera_model "$CAMERA_MODEL" --camera_intrinsics "${CAMERA_INTRINSICS:?}" --camera_distortion_coeffs "${CAMERA_DISTORTION:?}")
  fi
  case "${LIDAR_TYPE:?}" in
    spinning) ARGS+=(--dynamic_lidar_integration) ;;
    non_repetitive) ;;
    *) echo "LIDAR_TYPE must be spinning or non_repetitive"; exit 1 ;;
  esac
  [ -n "${VOXEL_RESOLUTION:-}" ] && ARGS+=(--voxel_resolution "$VOXEL_RESOLUTION")
  [ -n "${MIN_DISTANCE:-}" ] && ARGS+=(--min_distance "$MIN_DISTANCE")
  [ -n "${INTENSITY_CHANNEL:-}" ] && ARGS+=(--intensity_channel "$INTENSITY_CHANNEL")
  [ "$HEADLESS" = 1 ] || ARGS+=(--visualize)
  run "${RUN[@]}" preprocess "${ARGS[@]}"
fi

if want guess; then
  if [ "${INITIAL_GUESS:-superglue}" = superglue ]; then
    SG=(); [ -n "${SUPERGLUE_WEIGHTS:-}" ] && SG+=(--superglue "$SUPERGLUE_WEIGHTS")
    # shellcheck disable=SC2086
    run "${RUN[@]}" find_matches_superglue.py "$DST" "${SG[@]}" ${SUPERGLUE_EXTRA_ARGS:-}
    IG=(); [ -n "${RANSAC_ERROR_THRESH:-}" ] && IG+=(--ransac_error_thresh "$RANSAC_ERROR_THRESH")
    [ -n "${ROBUST_KERNEL_WIDTH:-}" ] && IG+=(--robust_kernel_width "$ROBUST_KERNEL_WIDTH")
    run "${RUN[@]}" initial_guess_auto "$DST" "${IG[@]}"
  else
    run "${RUN[@]}" initial_guess_manual "$DST"
  fi
fi

if want calibrate; then
  CA=(); [ "$HEADLESS" = 1 ] && CA+=(--background --auto_quit)
  run "${RUN[@]}" calibrate "$DST" "${CA[@]}"
fi

if want viewer && [ "$HEADLESS" != 1 ]; then
  run "${RUN[@]}" viewer "$DST"
fi
echo "result: $DST/calib.json"
