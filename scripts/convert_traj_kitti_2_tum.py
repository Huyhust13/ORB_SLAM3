#!/usr/bin/env python

import numpy as np
import os, sys, argparse
# import tf.transformations as tf
from scipy.spatial.transform import Rotation as R

parser = argparse.ArgumentParser(description='demo')
parser.add_argument('--input', type=str, default='', help='GT file input kitti format', required=True)
parser.add_argument('--output', type=str, default='', help='GT file input kitti format', required=True)
args = parser.parse_args()

traj_input = args.input
traj_output = args.output

traj_file = open(traj_input, 'r')
out_file = open(traj_output, 'w')

lines = traj_file.readlines()

fps = 10.0

for idx, line in enumerate(lines):
    line_arr = line.split(' ')
    time = float(line_arr[0])
    line_arr[0] = str(int(time*1e9))
    out_file.write(" ".join(line_arr))

traj_file.close()
out_file.close()