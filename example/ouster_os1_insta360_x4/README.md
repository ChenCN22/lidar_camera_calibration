# Example: Ouster OS-1-128 + Insta360 X4 front fisheye

The commands below are what `scripts/` runs for [`rig.env`](rig.env), written out step by step with the
parameters of this rig.

| | |
|---|---|
| recording PC | Ubuntu 22.04, ROS 2 Humble |
| calibration PC | Ubuntu 24.04, Docker 29.9, RTX 4090, X11 display |
| lidar | Ouster OS-1-128, firmware 3.0.1, 1024x10, ROS 2 driver, `/ouster/points` |
| camera | Insta360 X4, front fisheye lens, 1440x1440, `/camera_front` (+ `/camera_info_omnidir`) |
| toolbox image | `koide3/direct_visual_lidar_calibration:humble` |

## 0. Camera intrinsics

Calibrated beforehand with a checkerboard and the OpenCV omnidir (Mei) model:
[`front_omnidir_intrinsics.json`](front_omnidir_intrinsics.json), 109 images, rms 0.50 px.

```
--camera_model omnidir
--camera_intrinsics 687.344052,687.263911,728.273899,722.792744,0.752012     # fx,fy,cx,cy,xi
--camera_distortion_coeffs -0.190037,0.020269,-0.000258,-0.001424             # k1,k2,p1,p2
```

## 1. Record bags (recording PC)

```
source /opt/ros/humble/setup.bash
scripts/record_calib_bags.sh example/ouster_os1_insta360_x4/rig.env ~/calib_bags calib_01 30
# = ros2 bag record -o ~/calib_bags/calib_01 /camera_front /ouster/points /camera_info_omnidir   (30 s)
```

Three or four bags per session (30 s each on 04/05), each from a different place. The OS-1 is a spinning lidar: start at
rest, then tilt the rig slowly up and down.

## 2. Start the container (calibration PC)

```
docker pull koide3/direct_visual_lidar_calibration:humble
xhost +local:root
docker run --rm -it --net host --gpus all \
  -e DISPLAY=$DISPLAY -v $HOME/.Xauthority:/root/.Xauthority \
  -v /path/to/input/bags:/tmp/input_bags \
  -v /path/to/save/result:/tmp/preprocessed \
  koide3/direct_visual_lidar_calibration:humble
```

`scripts/run_calibration_container.sh` runs the same command and also mounts the config and this repository.

## 3. Setup inside the container (once per container)

```
apt update
apt install -y pip git
pip install numpy opencv-python torch matplotlib
cd ~
git clone https://github.com/magicleap/SuperGluePretrainedNetwork.git
echo 'export PYTHONPATH=$PYTHONPATH:~/SuperGluePretrainedNetwork' >> ~/.bashrc
source ~/.bashrc
source /opt/ros/humble/setup.bash
cd ~/ros2_ws && source install/setup.bash
```

## 4. Preprocess

```
ros2 run direct_visual_lidar_calibration preprocess \
  --data_path /tmp/input_bags \
  --dst_path /tmp/preprocessed \
  --image_topic /camera_front \
  --points_topic /ouster/points \
  --camera_info_topic /camera_info_omnidir \
  --camera_model omnidir \
  --camera_intrinsics 687.344052,687.263911,728.273899,722.792744,0.752012 \
  --camera_distortion_coeffs -0.190037,0.020269,-0.000258,-0.001424 \
  --voxel_resolution 0.008 \
  --dynamic_lidar_integration \
  --verbose --visualize
```

| option | note |
|---|---|
| `--dynamic_lidar_integration` | only for mechanically spinning lidars (Ouster, Velodyne ...). Do not use it with a Livox. |
| `--voxel_resolution 0.008` | depends on the lidar and its driver, a wrong value gives scaling issues. The Ouster ROS 2 driver truncates the 19-bit range to 16 bit, so the cloud is quantised at 0.008 m per unit. Toolbox default: 0.002. |
| `--camera_info_topic` | has no effect once the intrinsics and distortion coefficients are given. |
| `--camera_model`, `--camera_intrinsics`, `--camera_distortion_coeffs` | from step 0. |
| intensity channel | left on `auto`, the toolbox picks `reflectivity` for the Ouster cloud. |

`--dst_path` is the directory used by all following commands.

## 5. Initial guess (SuperGlue)

```
ros2 run direct_visual_lidar_calibration find_matches_superglue.py /tmp/preprocessed --superglue indoor --sinkhorn_iterations 25
ros2 run direct_visual_lidar_calibration initial_guess_auto /tmp/preprocessed --ransac_error_thresh 6 --robust_kernel_width 6
```

| option | default | used here |
|---|---|---|
| `--superglue` | `outdoor` | `indoor` (indoor scenes) |
| `--sinkhorn_iterations` | 20 | 25 |
| `--keypoint_threshold` | 0.05 | default |
| `--nms_radius` | 4 | default |
| `--match_threshold` | 0.01 | default |
| `--max_keypoints`, `--show_keypoints`, `--force_cpu`, `--rotate_camera`, `--rotate_lidar` | | not used |
| `--ransac_error_thresh` (`initial_guess_auto`) | 10 | 6 |
| `--robust_kernel_width` (`initial_guess_auto`) | 10 | 6 |
| `--ransac_iterations` (`initial_guess_auto`) | 8192 | default |

## 6. Refine and inspect

```
ros2 run direct_visual_lidar_calibration calibrate /tmp/preprocessed
ros2 run direct_visual_lidar_calibration viewer /tmp/preprocessed
```

Result: `/tmp/preprocessed/calib.json`.

## Results

| session | bags | topics | intrinsics | `T_lidar_camera` translation (m) |
|---|---|---|---|---|
| [`2026-04-05_lab_front`](2026-04-05_lab_front/calib.json) | 3 x 30 s | `/image`, `/camera_info` | as above | -0.030, 0.018, 0.130 |
| [`2026-04-08_mount_v1_front`](2026-04-08_mount_v1_front/calib.json) | 4 | `/image`, `/camera_info` | as above | -0.036, 0.008, 0.154 |
| [`2026-06-03_mount_v2_front`](2026-06-03_mount_v2_front/calib.json) | 3 | `/camera_front`, `/camera_info_omnidir` | `622.768,619.839,723.514,726.569,0.5736` / `-0.2105,0.0327867,-0.0004916508,-0.0003278793` | 0.006, 0.070, 0.134 |

Re-running the 04/05 bags with `scripts/calibrate_in_container.sh` (2026-11-12): preprocess 2.5 min for the three
bags; SuperGlue + RANSAC kept 17 of 275 matches and the initial guess was 9.6 deg / 0.26 m from the stored result;
`calibrate` then converged to within 5 mm and 0.1 deg of it.
