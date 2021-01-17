close all
clear
clc

source "./utils/data_utils.m"
source "./utils/plot_utils.m"

# Read the trajectory (measurements & ground truth)
[traj_meas,traj_gt] = readTrajectory("./Dataset/trajectory.dat");

# Read camera parameters
[cam_mat, cam_trans, z_near, z_far, width, height] = readCameraParams("./Dataset/camera.dat");

# Read the landmark true positions
# Note: id of the landmark lan_gt(i) is i-1
lan_gt = readLandmarksGT("./Dataset/world.dat");

plotOdometryAndGT(traj_meas, traj_gt);