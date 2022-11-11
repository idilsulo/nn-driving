import numpy as np
import matplotlib.pyplot as plt
import math
from math import fabs, asin, cos, atan2, pi, copysign
import copy

def rotation_from_pyr(pitch, yaw, roll):

    """Generates a rotation matrix from the pitch,
    yaw and roll angles given in degrees"""
    yaw = float(yaw)
    pitch = float(pitch)
    roll = float(roll)
    yaw = np.radians(yaw)
    roll = np.radians(roll)
    pitch = np.radians(pitch)

    yaw_matrix = np.matrix([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
    ])

    pitch_matrix = np.matrix([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ])

    roll_matrix = np.matrix([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
    ])

    rotation_matrix = yaw_matrix * pitch_matrix * roll_matrix

    return rotation_matrix

def rotation_matrix_to_euler_zyx(R):
    EPSILON = 1e-6

    if fabs(fabs(R[2, 0]) - 1) < EPSILON:

        y = copysign(pi / 2, -R[2, 0])
        x = 0
        z = atan2(R[0, 1], R[0, 2])
        gimbal_lock = True

    else:
        y = -asin(R[2, 0])
        cy = cos(y)
        x = atan2(R[2, 1] / cy, R[2, 2] / cy)
        z = atan2(R[1, 0] / cy, R[0, 0] / cy)
        gimbal_lock = False

    return [np.degrees(z), np.degrees(y), np.degrees(x), gimbal_lock]

def rescale_pred_coord(gt_xyzs, pred_xyzs):

    # Optimize the scaling factor
    '''
    # for seq 01, only take the begining to compute the scale as it become unrealiable at later frames on the highway
    scale = np.sum(gt_xyzs[0:100,:] * pred_xyzs[0:100,:]) / np.sum(pred_xyzs[0:100,:] ** 2)
    '''
    scale = np.sum(gt_xyzs * pred_xyzs) / np.sum(pred_xyzs ** 2)
    pred_xyzs = pred_xyzs * scale
    return pred_xyzs


def plot_trajectory(gt_xyzs, pred_xyzs):
    # Plot the graph of the trajectory
    plt.plot(gt_xyzs[:, 0], gt_xyzs[:, 1], color='blue', label='Ground Truth')
    plt.plot(pred_xyzs[:, 0], pred_xyzs[:, 1], color='red', label='Visual Odometry')
    plt.xlabel('x [m]')
    plt.ylabel('z [m]')
    plt.legend()
    plt.show()


def evaluate(gt_poses_path, pred_poses_path, is_scale=False):
    print("-> Reading pose predictions by SLAM ", "pose.txt")

    # Load ground-truth and ORB-SLAM2 predicted poses
    gt_global_poses = np.loadtxt(gt_poses_path)
    pred_global_poses = np.loadtxt(pred_poses_path)

    # Rotate poses from ORB-SLAM2 to match the coordinate system of Carla
    pred_global_poses = pred_global_poses.reshape(-1, 3, 4)
    # Angle needs to be changed for different sequences
    angle = -90
    R1 = np.asarray(rotation_from_pyr(angle, 0, 0))
    for i in range(0, len(pred_global_poses)):
        R = pred_global_poses[i][:3, :3]
        arr = rotation_matrix_to_euler_zyx(R)
        R = rotation_from_pyr(arr[2], arr[1], arr[0])
        transR = R.reshape((1, 3,3))
        rotatedR = np.dot(transR, R1)[0]
        pred_global_poses[i][:3, :3] = rotatedR

        trans = pred_global_poses[i][:3, 3].reshape(1, 3)
        rotated = np.dot(trans, R1)
        pred_global_poses[i][0, 3] = rotated[0][2]
        pred_global_poses[i][1, 3] = rotated[0][0]
        pred_global_poses[i][2, 3] = rotated[0][1]

    # Get translation vector
    pred_global_poses = np.concatenate(
        (pred_global_poses, np.zeros((pred_global_poses.shape[0], 1, 4))), 1)
    pred_global_poses[:, 3, 3] = 1
    pred_xyzs = pred_global_poses[:, :3, 3]

    gt_global_poses = gt_global_poses[25:]
    gt_global_poses = gt_global_poses.reshape(-1, 3, 4)
    gt_global_poses = np.concatenate(
        (gt_global_poses, np.zeros((gt_global_poses.shape[0], 1, 4))), 1)
    gt_global_poses[:, 3, 3] = 1
    gt_xyzs = gt_global_poses[:, :3, 3]

    gt_trans_x = gt_xyzs[0, 0]
    gt_trans_y = gt_xyzs[0, 1]
    gt_trans_z = gt_xyzs[0, 2]

    xtmp = copy.deepcopy(gt_xyzs[:, 0])
    ytmp = copy.deepcopy(gt_xyzs[:, 1])
    ztmp = copy.deepcopy(gt_xyzs[:, 2])
    gt_xyzs[:, 0] = xtmp - gt_trans_x
    gt_xyzs[:, 1] = ytmp - gt_trans_y
    gt_xyzs[:, 2] = ztmp - gt_trans_z

    # Plot the trajectory
    if is_scale:
        pred_xyzs = rescale_pred_coord(gt_xyzs, pred_xyzs)
        print("plot_trajectory after rescale")

    plot_trajectory(gt_xyzs, pred_xyzs)

    return gt_xyzs, pred_xyzs


if __name__ == "__main__":
    gt_poses_path = "./pose_gt.txt"
    pred_poses_path = "./pose.txt"
    gt_00_xyzs, pred_stereo00_xyzs = evaluate(gt_poses_path, pred_poses_path,  True)
