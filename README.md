# lidar_camera_calibration

Extrinsic calibration of the Ouster OS-1 lidar and the Insta360 X4 dual-fisheye camera used by the POSE
real-robot stack (*5-DoF Legged Robot Semantic Exploration and Inspection in Crowded Unknown Environments*).
The result feeds `body_to_color_sensor_front / _back` and `omnidir_*` in the `map_manager` configuration
(`autonomous_robot/cfg/exploration_spot_real/multi_robo_occ_map_param.yaml`, branch `pose-real`).

The calibration itself is done with the target-less toolbox
[koide3/direct_visual_lidar_calibration](https://github.com/koide3/direct_visual_lidar_calibration)
(Koide et al., ICRA 2023) inside its published Docker image. **This repository contains no third-party code**:
only the recording / conversion scripts, the exact commands and parameters we used, and our results.
See [Third-party software](#third-party-software) for the licences involved.

![SuperGlue matches between the fisheye frame and the lidar intensity panorama](docs/img/superglue_matches.png)

## Hardware and frames

| item | value |
|---|---|
| lidar | Ouster OS-1, 1024x10, `/ouster/points` (reflectivity channel used as intensity) |
| camera | Insta360 X4, two 1440x1440 fisheye eyes (`/camera_front`, `/camera_back`), OpenCV `omnidir` (Mei) model |
| camera model | fx fy cx cy xi + k1 k2 p1 p2 (`results/intrinsics/*.json`, from a chessboard calibration of each eye) |
| lidar in body | `body_to_range_sensor` translation [0.45, 0, 0.43] m, body-aligned axes |
| calib output | `T_lidar_camera = [x y z qx qy qz qw]`, camera pose in the lidar frame of the recorded cloud, OpenCV camera axes |

## Pipeline

### 0. Camera intrinsics
One `omnidir` intrinsic set per eye (OpenCV `cv::omnidir::calibrate`, chessboard). Ours:
`results/intrinsics/front_omnidir_calibration.json` (109 images, rms 0.50 px). The back-eye set is in the
planner cfg (`omnidir_back_intrinsics / _distortion`). The intrinsics are passed to the toolbox explicitly, so the
`camera_info` topic does not need to be a `sensor_msgs/CameraInfo`.

### 1. Record calibration bags (ROS 2 host)
Drivers: the Insta360 ROS 2 driver (`/camera_front`, `/camera_back`, `/camera_info_omnidir`; hardware-timestamped
H.264 decode, see the robot repository) and `ouster-ros` (ROS 2 branch, `/ouster/points`).

```
scripts/record_calib_bags.sh <out_dir> calib_01 30      # repeat for 3-4 viewpoints
```
- rig standing still or moving very slowly; the preprocess step motion-compensates spinning lidars
  (`--dynamic_lidar_integration`), but a static 30 s bag gives the cleanest intensity image;
- textured, structured scene with surfaces 2-6 m away, no moving people;
- one bag per viewpoint, 3-4 viewpoints; the toolbox optimises over all bags in the directory.
Topic names are overridable with `IMAGE_TOPIC / INFO_TOPIC / POINTS_TOPIC`. Our 04/05 bags used `/image` and
`/camera_info` (older driver), the 06/03 bags `/camera_front` and `/camera_info_omnidir`.

### 2. Start the toolbox container
```
scripts/run_calibration_container.sh <bag_dir> <result_dir>        # pulls koide3/direct_visual_lidar_calibration:humble
```
Mounts: bags at `/tmp/input_bags`, results at `/tmp/preprocessed`, this repository at `/tmp/calib_repo`.
Needs an NVIDIA GPU for SuperGlue and an X display for `--visualize` / `viewer`.

### 3. Calibrate (inside the container)
```
/tmp/calib_repo/scripts/calibrate_in_container.sh /tmp/calib_repo/results/intrinsics/front_omnidir_calibration.json
```
`setup` installs torch / OpenCV and clones SuperGlue (the user accepts its licence, see below), then

| step | command | our parameters |
|---|---|---|
| preprocess | `preprocess --camera_model omnidir --camera_intrinsics fx,fy,cx,cy,xi --camera_distortion_coeffs k1,k2,p1,p2 --voxel_resolution 0.008 --dynamic_lidar_integration` | `voxel_resolution 0.008` = the range quantisation of the Ouster ROS 2 driver (19-bit range truncated to 16 bit); a wrong value scales the cloud |
| matches | `find_matches_superglue.py <dst> --superglue indoor --sinkhorn_iterations 25` | indoor weights for the lab / shop |
| initial guess | `initial_guess_auto <dst> --ransac_error_thresh 6 --robust_kernel_width 6` | looser RANSAC than default: fisheye keypoints outside the lidar band produce many bad matches |
| refine | `calibrate <dst>` | NID direct registration, lidar-driven |
| check | `viewer <dst>` | colour the cloud with the image, look at edges at 2-4 m |

Single steps: `calibrate_in_container.sh <intr.json> preprocess|matches|guess|calibrate|viewer`.
Output: `<result_dir>/calib.json` plus per-bag `*.ply`, intensity / index images and `*_superglue.png`.

### 4. Convert to the planner configuration
```
scripts/calib_to_body_extrinsic.py results/2026-06-03_mount_v2_front/calib.json --body_to_lidar 0.45 0 0.43 --lidar_yaw_deg 180
```
prints the 16 row-major floats for `body_to_color_sensor_front` (`body_to_color_sensor = T_body_lidar * R_z(yaw) * T_lidar_camera`).
`--lidar_yaw_deg 180` is needed when the bag cloud is in the Ouster `os_lidar` frame (x backwards) while the planner's
`body_to_range_sensor` describes a body-aligned lidar: check on the robot that an obstacle in front of Spot lands at
+x in the map, otherwise use 0. Repeat with the back-eye calib.json for `body_to_color_sensor_back`.
`--compare <16 floats>` diffs against a deployed value.

### 5. Verify on the robot
`map_manager` with `color_sensor_type: omnidir_dual` projects lidar points into the detector masks; a wrong extrinsic
shows up as object clouds offset from the detections (ghost objects next to walls). The quickest check is to colour
`/ouster/points` through the fisheye image with the fitted transform (the toolbox `viewer` does this offline).

## Results

| session | mount | bags | intrinsics used | T_lidar_camera t (m) / rpy (deg) | note |
|---|---|---|---|---|---|
| `results/2026-04-05_lab_front` | bench | 3 x 30 s, lab | 687.34 set | (-0.030, 0.018, 0.130) / (-90.0, 0.4, 97.1) | first run, FoV-overlap masking study (`session_report.txt`, `fov_params.json`) |
| `results/2026-04-08_mount_v1_front` | Spot mount v1 | 4 bags | 687.34 set | (-0.036, 0.008, 0.154) / (-90.2, 0.1, 89.2) | |
| `results/2026-06-03_mount_v2_front` | Spot mount v2 (TechSpark) | 3 bags | 622.77 set | (0.006, 0.070, 0.134) / (-91.4, 0.5, 89.2) | rotation deployed in the real cfg (max entry diff 0.008 after the 180 deg yaw) |

Deployed `body_to_color_sensor_front` (09/2026 TechSpark runs): rotation = the 06/03 result; translation
(0.474, -0.146, 0.591) differs from the converted value (0.444, -0.070, 0.564) by up to 7.6 cm (hand adjusted on the
robot). The 06/03 preprocess was run with the older 622.77 intrinsic set while the planner uses the 687.34 set; when
re-calibrating, pass the same intrinsics to `preprocess` that the planner will use.

## Known issues
- **Parallax ghosting**: with ~10 cm between the sensors, lidar points behind an object's silhouette take the object's
  colour at 1-3 m (70 px at 1 m). This is geometry, not a calibration error; `results/2026-04-05_lab_front/session_report.txt`
  documents the investigation (FoV masking of SuperGlue, grayscale fix for NID on BGR input).
- Indoor, short-range scenes are outside the toolbox's design envelope (outdoor, 10 m+); expect 0.5-1.5 deg spread
  between sessions and check the result on the robot.
- The viewer colours every point; occlusion culling exists only in the `calibrate` step.

## Third-party software
Nothing below is redistributed here; the scripts download or pull it on the user's machine.

| component | licence | role |
|---|---|---|
| koide3/direct_visual_lidar_calibration (+ Docker image) | MIT | calibration toolbox |
| magicleap/SuperGluePretrainedNetwork | academic / non-commercial research only | feature matching for the initial guess (`find_matches_superglue.py`). Commercial users must replace it, e.g. with the toolbox's manual initial guess |
| Insta360 CameraSDK | Insta360 SDK licence | camera driver dependency (robot side) |
| ouster-ros | BSD-3 | lidar driver |

## Files
```
scripts/record_calib_bags.sh          ros2 bag record wrapper (topics via env)
scripts/run_calibration_container.sh  docker pull + run with bag/result/scripts mounts
scripts/calibrate_in_container.sh     the five toolbox steps with our parameters (run inside the container)
scripts/calib_to_body_extrinsic.py    calib.json -> body_to_color_sensor 4x4 (no dependencies)
results/<date>_<mount>_front/         calib.json (+ fov_params.json, session_report.txt for 04/05)
results/intrinsics/                   front eye omnidir intrinsics
docs/img/                             fisheye frame, lidar intensity panorama, SuperGlue matches, FoV overlap (04/05 lab session)
```

## License
MIT, see `LICENSE` (our scripts and results only).
