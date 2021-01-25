close all
clear
clc

source "./utils/data_utils.m"
source "./utils/plot_utils.m"
source "./utils/geom_utils.m"

%%%%%%%%%%%%%%% LOAD DATA %%%%%%%%%%%%%%%

% Read the trajectory (measurements & ground truth)
% Matrices 3*num_poses
[traj_meas,traj_gt] = readTrajectory();
global num_poses = size(traj_meas,2)
global pose_dim = size(traj_meas,1)

% Convert the robot poses to an array of homogeneous matrices
% Matrices 3*3*num_poses
XR_guess = zeros(3,3,num_poses);
for i = 1:num_poses
  XR_guess(:,:,i) = v2t(traj_meas(:,i));
end

% Read camera parameters
[cam_mat, cam_trans, z_near, z_far, width, height] = readCameraParams();

% Read the landmark true positions
% Matrix 3*num_landmarks
lan_gt = readLandmarksGT();
global num_landmarks = size(lan_gt,2) % total number of landmarks assumed to be known
global landmark_dim = size(lan_gt,1)

% For now initialize landmark positions with ground truth
XL_guess = lan_gt;

%%%%%%%%%%%%%%% POSE MEASUREMENTS %%%%%%%%%%%%%%%
Zr=zeros(3,3,num_poses-1);

for measurement_num=1:num_poses-1
  Xi=XR_guess(:,:,measurement_num);
  Xj=XR_guess(:,:,measurement_num+1);
  Zr(:,:,measurement_num)=inv(Xi)*Xj;
end

%%%%%%%%%%%%%%% PROJECTION MEASUREMENTS %%%%%%%%%%%%%%%
num_projection_measurements=num_poses*num_landmarks;
Zp=zeros(2,num_projection_measurements);
projection_associations=zeros(2,num_projection_measurements);

% Read the projection measurements
measurement_num=0;
for pose_num = 1:num_poses
  [id_landmarks, measurements] = readMeasurements(pose_num);
  for meas = 1:size(measurements,1)
    measurement_num = measurement_num + 1;
    projection_associations(:,measurement_num) = [pose_num, id_landmarks(meas)]';
    Zp(:,measurement_num) = measurements(:,meas);
  end
end

# Crop the matrices
##num_projection_measurements=measurement_num-1;
##projection_associations=projection_associations(:,1:num_projection_measurements);
##Zp=Zp(:,1:num_projection_measurements);



%plotOdometryAndGT(traj_meas, traj_gt);