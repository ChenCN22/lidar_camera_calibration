#!/usr/bin/env python3
"""Chain a calib.json with a body-to-lidar mounting to get the 4x4 camera pose in the robot body frame.

  calib_to_body_extrinsic.py calib.json --body_to_lidar X Y Z [--lidar_yaw_deg YAW] [--compare <16 floats>]

T_lidar_camera in calib.json is [x y z qx qy qz qw]: camera pose in the frame of the recorded cloud
(OpenCV camera axes, z forward). T_body_camera = T_body_lidar * R_z(yaw) * T_lidar_camera, where yaw is the
rotation of the recorded cloud frame against the body-aligned lidar frame (0 if they coincide).
"""
import argparse, json, math, sys


def quat_to_mat(qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return [[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]]


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def homog(r, t):
    return [r[0] + [t[0]], r[1] + [t[1]], r[2] + [t[2]], [0.0, 0.0, 0.0, 1.0]]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("calib_json")
    ap.add_argument("--body_to_lidar", nargs=3, type=float, default=[0.0, 0.0, 0.0], metavar=("X", "Y", "Z"),
                    help="lidar origin in the body frame (m)")
    ap.add_argument("--lidar_yaw_deg", type=float, default=0.0,
                    help="yaw of the recorded cloud frame relative to the body-aligned lidar frame")
    ap.add_argument("--compare", nargs=16, type=float, help="existing 16 row-major floats to diff against")
    a = ap.parse_args()
    v = json.load(open(a.calib_json))["results"]["T_lidar_camera"]
    t_lc = homog(quat_to_mat(*v[3:7]), v[0:3])
    c, s = math.cos(math.radians(a.lidar_yaw_deg)), math.sin(math.radians(a.lidar_yaw_deg))
    r_yaw = homog([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], [0.0, 0.0, 0.0])
    t_bl = homog([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], a.body_to_lidar)
    m = matmul(matmul(t_bl, r_yaw), t_lc)
    flat = [x for row in m for x in row]
    print("T_body_camera (row-major): [" + ", ".join("%.6f" % x for x in flat) + "]")
    print("camera position in body (m): %.3f %.3f %.3f" % (m[0][3], m[1][3], m[2][3]))
    print("camera optical axis in body: %.3f %.3f %.3f" % (m[0][2], m[1][2], m[2][2]))
    if a.compare:
        d_rot = max(abs(flat[i] - a.compare[i]) for i in range(16) if i % 4 != 3 and i < 12)
        d_t = [flat[i] - a.compare[i] for i in (3, 7, 11)]
        print("vs --compare: max rotation-entry diff %.4f, translation diff (m) %.3f %.3f %.3f" % (d_rot, *d_t))
    return 0


if __name__ == "__main__":
    sys.exit(main())
