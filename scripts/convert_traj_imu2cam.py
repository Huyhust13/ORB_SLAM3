#!/usr/bin/env python

import numpy as np
import os, sys, argparse
# import tf.transformations as tf
from scipy.spatial.transform import Rotation as R

parser = argparse.ArgumentParser(description='demo')
parser.add_argument('--input', type=str, default='', help='trajectory in imu frame', required=True)
parser.add_argument('--output', type=str, default='', help='trajectory in cam frame', required=True)
parser.add_argument('--dataset', type=str, default='', help='path to dataset folder', required=True)
args = parser.parse_args()

traj_input = args.input
traj_output = args.output
src_folder = args.dataset

traj_file = open(traj_input, 'r')
out_file = open(traj_output, 'w')

tf_velo_to_cam = np.identity(4)
with open(os.path.join(src_folder, '2011_09_30_calib/2011_09_30', 'calib_velo_to_cam.txt'), 'r') as f:
    lines = f.readlines()
    tf_velo_to_cam[0:3, 0:3] = np.fromstring(lines[1].split(':')[-1], dtype=float, sep=' ').reshape(3,3)
    tf_velo_to_cam[0:3, 3] = np.fromstring(lines[2].split(':')[-1], dtype=float, sep=' ').reshape(1,3)

tf_imu_to_velo = np.identity(4)
with open(os.path.join(src_folder, '2011_09_30_calib/2011_09_30', 'calib_imu_to_velo.txt'), 'r') as f:
    lines = f.readlines()
    tf_imu_to_velo[0:3, 0:3] = np.fromstring(lines[1].split(':')[-1], dtype=float, sep=' ').reshape(3,3)
    tf_imu_to_velo[0:3, 3] = np.fromstring(lines[2].split(':')[-1], dtype=float, sep=' ').reshape(1,3)

tf_cam_imu = np.linalg.inv(np.dot(tf_velo_to_cam, tf_imu_to_velo))
# tf_cam_imu = (np.dot(tf_velo_to_cam, tf_imu_to_velo))

lines = traj_file.readlines()

for idx, line in enumerate(lines):
    line_arr = line.split(' ')
    ori_pose = np.fromstring(line, dtype=float, sep=' ')
    # time = float(line_arr[0])
    # line_arr[0] = str(int(time*1e9))
    # out_file.write(" ".join(line_arr))
    pose = np.identity(4)
    pose[0:3, 0:3] = R.from_quat(ori_pose[4:]).as_matrix()
    pose[3, 0:3] = ori_pose[1:4]

    # pose = np.linalg.inv(np.dot(np.linalg.inv(pose), tf_cam_imu))
    # pose = (np.dot(pose, np.linalg.inv(tf_cam_imu)))
    pose = (np.dot(np.linalg.inv(tf_cam_imu), pose))
    # pose = np.linalg.inv(pose)

    new_quat = R.from_matrix(pose[0:3, 0:3]).as_quat()
    new_pose = [ori_pose[0], pose[3, 0], pose[3,1], pose[3,2], new_quat[0], new_quat[1], new_quat[2], new_quat[3]]
    out_file.write(" ".join(str(elem) for elem in new_pose) + '\n')

    # print(new_pose)
    # break
traj_file.close()
out_file.close()