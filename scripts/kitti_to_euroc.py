#!/usr/bin/env python3
import os
import argparse
import shutil, sys
from PIL import Image
import numpy as np
from scipy.spatial.transform import Rotation as R
from datetime import datetime

parser = argparse.ArgumentParser(description='demo')
parser.add_argument('--src', type=str, default='', help='dataset input', required=True)
parser.add_argument('--dest', type=str, default='', help='dataset output', required=True)
parser.add_argument('--orbslam3', type=str, default='', help='ORB_SLAM3 folder path', required=True)
parser.add_argument('--remove_dest', type=bool, default=False, help='remove dest_folder first')
args = parser.parse_args()
src_folder = args.src
dest_folder = args.dest
orb_folder = args.orbslam3

# src_folder = "/mnt/sda1/upwork_data/dataset/residential_seq_raw_unsync"
# dest_folder = "/mnt/sda1/upwork_data/dataset/residential_seq_raw_unsync/euroc_format_inv"

if os.path.exists(dest_folder) and args.remove_dest:
    shutil.rmtree(dest_folder)

### Walk through all files in the directory that contains the files to copy
### ---move images in jpg to png inside `image_0` foldef -------------------------:

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

tf_imu_cam = np.linalg.inv(np.dot(tf_velo_to_cam, tf_imu_to_velo))

rpy = R.from_matrix(tf_imu_cam[0:3,0:3]).as_euler('xyz')
rpy_degree = [e*180/3.14 for e in rpy]
print("tf_imu_cam", tf_imu_cam)
print("rpy", rpy)
print("rpy_degree", rpy_degree)

src_images = os.path.join(src_folder, "images_only_unrectified/2011_09_30/2011_09_30_drive_0027_extract/image_00/data")
dest_images = os.path.join(dest_folder, "image_0")

if not os.path.exists(dest_images):
    # os.mkdir(os.path.dirname(dest_images))
    os.makedirs(dest_images)

num_images = 0
width = 1440
height = 1080
fps = 10.0

def sort_key(e):
    base, extension = os.path.splitext(e)
    return int(base)

def convert_image_format():
    global num_images
    time_src = os.path.join(src_folder, "images_only_unrectified/2011_09_30/2011_09_30_drive_0027_extract/image_00/timestamps.txt")
    time_dest = os.path.join(dest_folder, 'times.txt')
    with open(time_src, 'r') as timestamp_in_f:
        timestamps = timestamp_in_f.readlines()

    time_out_f = open(time_dest, 'w')

    for root, dirs, files in os.walk(src_images):
        num_images = len(files)
        print("num_images", num_images)
        files.sort(key=sort_key)
        for (timestamp_str,filename) in zip(timestamps, files):

            # Timezone Name.
            dt_format = datetime.strptime(timestamp_str[:-4], "%Y-%m-%d %H:%M:%S.%f")

            # Timestamp
            timestamp = dt_format.timestamp()
            timestamp_out = str(int(timestamp*1e9))
            time_out_f.writelines(timestamp_out + '\n')
            # I use absolute path, case you want to move several dirs.
            old_name = os.path.join( os.path.abspath(root), filename )

            new_name = os.path.join(dest_images, timestamp_out + ".png")
            print(old_name + "-> " + new_name)
            shutil.copy(old_name, new_name)

    time_out_f.close()
###----------------------------------------------------------------------------#

###---- read calib result and put into *yaml file -----------------------------#
import re
import yaml
from pathlib import Path
def convert_config():
    calib_file_name = os.path.join(src_folder, '2011_09_30_calib/2011_09_30/calib_cam_to_cam.txt')
    calib_file = open(calib_file_name, 'r')
    cam_config_sample = os.path.join(orb_folder, "Examples/Monocular-Inertial/EuRoC.yaml")
    dest_cam_config_file = os.path.join(dest_folder, "cam_config.yaml")
    calib_lines = calib_file.readlines()
    width = int(float(calib_lines[2].split(' ')[1]))
    height = int(float(calib_lines[2].split(' ')[2]))
    K_arr = calib_lines[3].split(' ')
    fx = float(K_arr[1])
    cx = float(K_arr[3])
    fy = float(K_arr[5])
    cy = float(K_arr[6])
    print(fx, fy, cx, cy)

    D_arr = calib_lines[4].split(' ')

    k1 = float(D_arr[1])
    k2 = float(D_arr[2])
    p1 = float(D_arr[3])
    p2 = float(D_arr[4])
    k3 = float(D_arr[5])
    print(k1, k2, p1, p2)

    cam_config_f = open(cam_config_sample, 'r')
    dest_cam_config_f = open(dest_cam_config_file, 'w')
    cfg_lines = cam_config_f.readlines()
    for idx,line in enumerate(cfg_lines):
        line_arr = line.split(':')
        if line_arr[0] == 'Camera.type':
            line_arr[1] = ' "PinHole"\n'
        if line_arr[0] == 'Camera1.fx':
            line_arr[1] = ' ' + str(fx) + '\n'
        if line_arr[0] == 'Camera1.fy':
            line_arr[1] = ' ' + str(fy) + '\n'
        if line_arr[0] == 'Camera1.cx':
            line_arr[1] = ' ' + str(cx) + '\n'
        if line_arr[0] == 'Camera1.cy':
            line_arr[1] = ' ' + str(cy) + '\n'
        if line_arr[0] == 'Camera1.k1':
            line_arr[1] = ' ' + str(k1) + '\n'
        if line_arr[0] == 'Camera1.k2':
            line_arr[1] = ' ' + str(k2) + '\n'
        if line_arr[0] == 'Camera1.p1':
            line_arr[1] = ' ' + str(p1) + '\n'
        if line_arr[0] == 'Camera1.p2':
            line_arr[1] = ' ' + str(p2) + '\nCamera1.k3: ' + str(k3) + '\n'
        if line_arr[0] == 'Camera.width':
            line_arr[1] = ' ' + str(width) + '\n'
        if line_arr[0] == 'Camera.height':
            line_arr[1] = ' ' + str(height) + '\n'
        if line_arr[0] == 'Camera.newWidth':
            line_arr[1] = ' ' + str(width) + '\n'
        if line_arr[0] == 'Camera.newHeight':
            line_arr[1] = ' ' + str(height) + '\n'
        if line_arr[0] == 'Camera.fps':
            line_arr[1] = ' 10\n'
        if line_arr[0] == '   data':
            line_arr[1] = ' [' + str(tf_imu_cam[0,0]) + ', ' + str(tf_imu_cam[0,1]) + ', ' + str(tf_imu_cam[0,2]) + ', '+ str(tf_imu_cam[0,3]) + ',\n'
            cfg_lines[idx+1] = '          ' + str(tf_imu_cam[1,0]) + ', ' + str(tf_imu_cam[1,1]) + ', ' + str(tf_imu_cam[1,2]) + ', '+ str(tf_imu_cam[1,3])+ ',\n'
            cfg_lines[idx+2] = '          ' + str(tf_imu_cam[2,0]) + ', ' + str(tf_imu_cam[2,1]) + ', ' + str(tf_imu_cam[2,2]) + ', '+ str(tf_imu_cam[2,3])+ ',\n'
            cfg_lines[idx+3] = '          ' + '0.0, 0.0, 0.0, 1.0]\n'
        # IMU noise

        if line_arr[0] == 'IMU.NoiseGyro':
            line_arr[1] = ' 0.01\n'
        if line_arr[0] == 'IMU.NoiseAcc':
            line_arr[1] = ' 0.2\n'
        if line_arr[0] == 'IMU.GyroWalk':
            line_arr[1] = ' 0.002\n'
        if line_arr[0] == 'IMU.AccWalk':
            line_arr[1] = ' 0.3\n'
        if line_arr[0] == 'IMU.Frequency':
            line_arr[1] = ' 100.0\n'
        line = ':'.join(line_arr)
        dest_cam_config_f.writelines(line)
    cam_config_f.close()
    dest_cam_config_f.close()


import numpy as np
# convert imu data
def convert_imu_data():
    imu_src_folder = os.path.join(src_folder, 'images_only_unrectified/2011_09_30/2011_09_30_drive_0027_extract/oxts')
    imu_time_f = open(os.path.join(imu_src_folder, 'timestamps.txt'))
    imu_time_lines = imu_time_f.readlines()

    imu_out_f = open(os.path.join(dest_folder, 'imu_data.txt'), 'w')
    for root, dirs, files in os.walk(os.path.join(imu_src_folder, 'data')):
        files.sort(key=sort_key)
        for (timestamp_str,filename) in zip(imu_time_lines, files):
            dt_format = datetime.strptime(timestamp_str[:-4], "%Y-%m-%d %H:%M:%S.%f")
            # print("Date with Timezone Name::", dt_format)

            # Timestamp
            timestamp = dt_format.timestamp()
            timestamp_out = str(int(timestamp*1e9))

            imu_filename = os.path.join( os.path.abspath(root), filename)

            imu_src_arr = open(imu_filename, 'r').readlines()[0].split(' ')
            imu_data_arr_out = [timestamp_out, imu_src_arr[17], float(imu_src_arr[18]), imu_src_arr[19],  float(imu_src_arr[14]), float(imu_src_arr[15]), imu_src_arr[16]]
            imu_out_f.writelines(','.join(str(e) for e in imu_data_arr_out) + '\n')

    imu_out_f.close()
    imu_time_f.close()

if __name__ == '__main__':
    convert_image_format()
    convert_config()
    convert_imu_data()