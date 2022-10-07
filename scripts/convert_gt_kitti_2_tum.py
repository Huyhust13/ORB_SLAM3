#!/usr/bin/env python

import numpy as np
import os, sys, argparse
# import tf.transformations as tf
from scipy.spatial.transform import Rotation as R

parser = argparse.ArgumentParser(description='demo')
parser.add_argument('--input', type=str, default='', help='GT file input kitti format', required=True)
parser.add_argument('--output', type=str, default='', help='GT file output tum format', required=True)
parser.add_argument('--timestamps', type=str, default='')
args = parser.parse_args()

kitti_file = args.input
tum_filename = args.output

gt_kitti = np.loadtxt(kitti_file, dtype=float)
tum_file = open(tum_filename, 'w')

if args.timestamps != '':
    timestamps_line = open(args.timestamps, 'r').readlines()

fps = 10.0

for idx, line in enumerate(gt_kitti):
    pose_matrix = line.reshape(3,4)
    pose_matrix = np.vstack((pose_matrix, [0,0,0,1]))

    rotation = R.from_matrix(pose_matrix[0:3,0:3])
    quat = rotation.as_quat()
    if args.timestamps == '':
        time = int(idx*1e9/fps)
    else:
        time = float(timestamps_line[idx].split(',')[0])
    pose_tum = [time, pose_matrix[0,3], pose_matrix[1,3], pose_matrix[2,3], quat[0], quat[1], quat[2], quat[3]]


    # quat = tf.transformations.quaternion_from_matrix(pose_matrix)
    # pose_tum[4:] = [quat[3], quat[0], quat[1], quat[2]]
    tum_file.write(" ".join([str(elem) for elem in pose_tum]) + '\n')

tum_file.close()