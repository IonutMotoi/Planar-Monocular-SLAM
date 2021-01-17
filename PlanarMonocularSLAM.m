close all
clear
clc

source "./utils/data_utils.m"
source "./utils/plot_utils.m"

# Read the trajectory (measurements & ground truth)
[traj_meas,traj_gt] = readTrajectory("./Dataset/trajectory.dat");

# Read camera parameters
[cam_mat, cam_trans, z_near, z_far, width, height] = readCamera("./Dataset/camera.dat");

plotOdometryAndGT(traj_meas, traj_gt);