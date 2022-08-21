import numpy as np
import scipy
from scipy import io
from scipy.spatial.transform import Rotation as R
from math import sqrt

def dist(A, B):
    return sqrt((A[0]-B[0])**2 + (A[1]-B[1])**2 + (A[2]-B[2])**2)

est_trajs_file = open("FullTrajectory.txt", 'r')
est_poses = est_trajs_file.readlines()

gt_file = open("GT_kitti_pose_cam.txt", 'r')
gt_poses = gt_file.readlines()

if(len(est_poses) != len(gt_poses)):
    print("Size of estimate poses different with size of groundtruth")
    exit()

convention = 'Twv'
rel_dists = []
cum_dist = []
frame_idx = []
frame_ts = []
num_poses = len(est_poses)
pose_type = 'SE3'
poses_est = []
poses_gt = []

last_pose = [0, 0, 0]

for i in range(len(est_poses)):
    est_pose_arr = np.array(est_poses[i][:-1].split(" ")).astype(float)
    est_pose_tran = est_pose_arr[1:4]
    est_pose_quat = est_pose_arr[4:8]

    est_pose_rot = R.from_quat(est_pose_quat).as_dcm()
    est_pose = np.zeros(16).reshape(4,4)

    for ci in range(3):
        for cj in range(3):
            est_pose[ci][cj] = est_pose_rot[ci][cj]

    for ci in range(3):
        est_pose[ci][3] = est_pose_tran[ci]
    est_pose[3][3] = 1.0
    # print(est_pose)

    poses_est.append(est_pose)
    # print(poses_est)
    frame_idx.append(i)
    frame_ts.append(est_pose_arr[0])
    rel_dist = dist(est_pose_tran, last_pose)
    # print(rel_dist)
    rel_dists.append(rel_dist)
    if i == 0:
        cum_dist.append(rel_dist)
    else:
        cum_dist.append(rel_dist + cum_dist[i-1])

    gt_pose = np.array(gt_poses[i][:-1].split(" ")).astype(float).reshape(3,4)
    gt_pose = np.vstack([gt_pose, np.array([0,0,0,1])])
    # print(gt_pose)
    poses_gt.append(gt_pose)

# convert C-order array in numpy to F-order array used in mat file
poses_est = np.transpose(poses_est, axes=(1,2,0))
poses_gt = np.transpose(poses_gt, axes=(1,2,0))

scipy.io.savemat('fullTrajectory.mat', {
    'convention': convention,
    'cum_dist':cum_dist,
    'frame_idx': frame_idx,
    'frame_ts': frame_ts,
    'num_poses': num_poses,
    'pose_type': pose_type,
    'poses_est': poses_est,
    'poses_gt': poses_gt,
    'rel_dists': rel_dists})

