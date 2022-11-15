#!/usr/bin/env python3
import os
import argparse
import shutil, sys
from PIL import Image
import numpy as np
from scipy.spatial.transform import Rotation as R

parser = argparse.ArgumentParser(description='demo')
parser.add_argument('--src', type=str, default='', help='dataset input', required=True)
parser.add_argument('--dest', type=str, default='', help='dataset output', required=True)
parser.add_argument('--orbslam3', type=str, default='', help='ORB_SLAM3 folder path', required=True)
parser.add_argument('--remove_dest', type=bool, default=False, help='remove dest_folder first')
args = parser.parse_args()

# src_folder = "/mnt/sda1/limo_data/custom_data"
# dest_folder = "/mnt/sda1/upwork_data/Nader-VO/vo_data_mimic_tum"
src_folder = args.src
dest_folder = args.dest
orb_folder = args.orbslam3


if os.path.exists(dest_folder) and args.remove_dest:
    shutil.rmtree(dest_folder)

### Walk through all files in the directory that contains the files to copy
### ---move images in jpg to png inside `image_0` foldef -------------------------:
src_images = os.path.join(src_folder, "flir_bfc_img_rectified")
dest_images = os.path.join(dest_folder, "image_0")
img_timestamp_filename = os.path.join(src_folder, "flir_bfc_img_timestamps/timestamps.csv")
time_filepath = os.path.join(dest_folder, "times.txt")

tf_imu_ouster = np.loadtxt(os.path.join(src_folder, 'extrinsics', 'sbgekinox_bcc-ouster64_bfc.txt'))
tf_ouster_cam = np.loadtxt(os.path.join(src_folder, 'extrinsics', 'ouster64_bfc-flir_bfc.txt'))

# tf_cam_imu = np.dot(tf_imu_ouster, tf_ouster_cam)
# tf_cam_imu = (np.dot(np.linalg.inv(tf_imu_ouster),np.linalg.inv(tf_ouster_cam)))
tf_cam_imu = np.linalg.inv(np.dot(tf_imu_ouster, tf_ouster_cam))
rpy = R.from_matrix(tf_cam_imu[0:3,0:3]).as_euler('xyz')
rpy_degree = [e*180/3.14 for e in rpy]
print("tf_cam_imu", tf_cam_imu)
print("rpy", rpy)
print("rpy_degree", rpy_degree)
# print(tf_imu_ouster)
# print(tf_ouster_cam)
# exit()

print(dest_images)
print(os.path.exists(dest_images))
if not os.path.exists(dest_images):
    # os.mkdir(os.path.dirname(dest_images))
    os.makedirs(dest_images)

num_images = 0
width = 1308 #1440
height = 913 #1080
fps = 10.0

def sort_key(e):
    base, extension = os.path.splitext(e)
    return int(base)

def convert_image_format():
    global num_images
    # Read images timestamps
    img_timestamps = []
    with open(img_timestamp_filename, 'r') as time_file:
        time_lines = time_file.readlines()
        for line in time_lines:
            img_timestamps.append(int(line.split(' ')[0]))

    f_time = open(time_filepath,'w')
    for root, dirs, files in os.walk(src_images):
        num_images = len(files)
        print("num_images", num_images)
        files.sort(key=sort_key)
        for (img_timestamp,filename) in zip(img_timestamps, files):
            # I use absolute path, case you want to move several dirs.
            old_name = os.path.join( os.path.abspath(root), filename )

            new_name = os.path.join(dest_images, str(img_timestamp) + ".jpg")
            f_time.writelines(str(img_timestamp) + '\n')
            # print(old_name + "-> " + new_name)
            # print(os.path.exists(new_name))
            # if not os.path.exists(new_name):  # folder exists, file does not
            #     im1 = Image.open(old_name)
            #     width, height = im1.size
            #     im1.save(new_name)
            shutil.copy(old_name, new_name)

    f_time.close()
###----------------------------------------------------------------------------#

###---- read calib result and put into *yaml file -----------------------------#
import re
import yaml
from pathlib import Path
def convert_config():
    calib_file_name = os.path.join(src_folder, 'intrinsics_rectified', "flir_bfc_img.txt")
    # calib_file_name = os.path.join(src_folder, 'intrinsics', "flir_bfc_calib-results.txt")
    cam_config_sample = os.path.join(orb_folder, "Examples/Monocular-Inertial/EuRoC.yaml")
    dest_cam_config_file = os.path.join(dest_folder, "cam_config.yaml")
    calib_file = open(calib_file_name, 'r')
    calib_lines = calib_file.readlines()
# For rectified images
    line1_arr = re.sub(' +', ' ', calib_lines[0][:-2]).split(' ')
    fx = float(line1_arr[0])
    cx = float(line1_arr[2])
    line2_arr = re.sub(' +', ' ', calib_lines[1][:-2]).split(' ')
    fy = float(line2_arr[1])
    cy = float(line2_arr[2])
    print("fx, fy, cx, cy: ", fx, fy, cx, cy)

# For unrectified image
    # line1_arr = re.sub(' +', ' ', calib_lines[0][9:-2]).split(' ')
    # fx = float(line1_arr[0])
    # cx = float(line1_arr[2])
    # line2_arr = re.sub(' +', ' ', calib_lines[1][4:-2]).split(' ')
    # fy = float(line2_arr[1])
    # cy = float(line2_arr[2])
    # print(fx, fy, cx, cy)

    # line4_arr = re.sub(' +', ' ', calib_lines[3][14:-3]).split(' ')
    # k1 = float(line4_arr[0])
    # k2 = float(line4_arr[1])
    # p1 = float(line4_arr[2])
    # p2 = float(line4_arr[3])
    # k3 = float(line4_arr[4])
    # print(k1, k2, k3, k4)

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
            # line_arr[1] = ' ' + str(k1) + '\n'
            line_arr[1] = ' 0.0\n'
        if line_arr[0] == 'Camera1.k2':
            # line_arr[1] = ' ' + str(k2) + '\n'
            line_arr[1] = ' 0.0\n'
        if line_arr[0] == 'Camera1.p1':
            # line_arr[1] = ' ' + str(p1) + '\n'
            line_arr[1] = ' 0.0\n'
        if line_arr[0] == 'Camera1.p2':
            # line_arr[1] = ' ' + str(p2) + '\n'
            line_arr[1] = ' 0.0\n'
        if line_arr[0] == 'Camera.width':
            line_arr[1] = ' ' + str(width) + '\n'
        if line_arr[0] == 'Camera.height':
            line_arr[1] = ' ' + str(height) + '\n'
        if line_arr[0] == 'Camera.newWidth':
            line_arr[1] = ' ' + str(int(width/2)) + '\n'
        if line_arr[0] == 'Camera.newHeight':
            line_arr[1] = ' ' + str(int(height/2)) + '\n'
        if line_arr[0] == 'Camera.fps':
            line_arr[1] = ' 10\n'
        if line_arr[0] == '   data':
            line_arr[1] = ' [' + str(tf_cam_imu[0,0]) + ', ' + str(tf_cam_imu[0,1]) + ', ' + str(tf_cam_imu[0,2]) + ', '+ str(tf_cam_imu[0,3]) + ',\n'
            cfg_lines[idx+1] = '          ' + str(tf_cam_imu[1,0]) + ', ' + str(tf_cam_imu[1,1]) + ', ' + str(tf_cam_imu[1,2]) + ', '+ str(tf_cam_imu[1,3])+ ',\n'
            cfg_lines[idx+2] = '          ' + str(tf_cam_imu[2,0]) + ', ' + str(tf_cam_imu[2,1]) + ', ' + str(tf_cam_imu[2,2]) + ', '+ str(tf_cam_imu[2,3])+ ',\n'
            cfg_lines[idx+3] = '          ' + '0.0, 0.0, 0.0, 1.0]\n'
        if line_arr[0] == 'IMU.NoiseGyro':
            line_arr[1] = ' 4.0\n'
        if line_arr[0] == 'IMU.NoiseAcc':
            line_arr[1] = ' 46.0\n'
        if line_arr[0] == 'IMU.GyroWalk':
            line_arr[1] = ' 1.5\n'
        if line_arr[0] == 'IMU.AccWalk':
            line_arr[1] = ' 1.5\n'
        if line_arr[0] == 'IMU.Frequency':
            line_arr[1] = ' 200.0\n'
        line = ':'.join(line_arr)
        dest_cam_config_f.writelines(line)
    cam_config_f.close()
    dest_cam_config_f.close()


import numpy as np
# convert imu data
def convert_imu_data():
    # Read imu_timestamp
    imu_timestamp_filename = os.path.join(src_folder, 'imudata_timestamps/timestamps.csv')
    imu_timestamps = []
    with open(imu_timestamp_filename, 'r') as imu_time_file:
        lines = imu_time_file.readlines()
        for line in lines:
            imu_timestamps.append(int(line.split(' ')[0]))

    imu_src_file_path = os.path.join(src_folder, 'sbgekinox_bcc_imudata_xf_yl_zu.txt')
    imu_dest_file_path = os.path.join(dest_folder, 'imu_data.txt')
    imu_dest_file = open(imu_dest_file_path, 'w')
    with open(imu_src_file_path) as f:
        lines = f.readlines()

        for (i,line) in enumerate(lines):
            arr = np.fromstring(line, dtype=float, sep=' ')
            #timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]
            new_arr = [imu_timestamps[i], arr[12], arr[13], arr[14], arr[2], arr[3], arr[4]]
            new_str = ','.join(str(x) for x in new_arr)
            imu_dest_file.write(new_str + '\n')

    imu_dest_file.close()
if __name__ == '__main__':
    convert_image_format()
    convert_config()
    convert_imu_data()