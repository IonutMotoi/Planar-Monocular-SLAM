close all
clear
clc

source "./utils/data_utils.m"
source "./utils/plot_utils.m"
source "./utils/geom_utils.m"
source "./least_squares.m"

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
global K;
global cam_pose;
global z_near;
global z_far;
global image_rows;
global image_cols;
[K, cam_pose, z_near, z_far, image_cols, image_rows] = readCameraParams();
tryPrintK()
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
Zp=zeros(2,num_poses*num_landmarks);
projection_associations=zeros(2,num_poses*num_landmarks);

% Read the projection measurements
measurement_num=0;
for pose_num = 1:num_poses
  [id_landmarks, measurements] = readMeasurements(pose_num);
  for meas = 1:size(measurements,2)
    measurement_num = measurement_num + 1;
    projection_associations(:,measurement_num) = [pose_num, id_landmarks(meas)]';
    Zp(:,measurement_num) = measurements(:,meas);
  end
end

% Crop the matrices
projection_associations=projection_associations(:,1:measurement_num);
Zp=Zp(:,1:measurement_num);

%%%%%%%%%%%%%%% GENERATION OF (WRONG) INITIAL GUESS %%%%%%%%%%%%%%%
pert_deviation=1;
pert_scale=eye(3)*pert_deviation;
for (pose_num=2:num_poses)
    xr=rand(3,1)-0.5;
    dXr=v2t(pert_scale*xr);
    XR_guess(:,:,pose_num)=dXr*XR_guess(:,:,pose_num);
endfor;

%%%%%%%%%%%%%%% LEAST SQUARES SOLVER %%%%%%%%%%%%%%%
damping=1;
kernel_threshold=1e3;
num_iterations=5;
[XR, XL, chi_stats_p, num_inliers_p, chi_stats_r, num_inliers_r, H, b]=doTotalLS(XR_guess, XL_guess,
                                                                                Zp, projection_associations, 
                                                                                Zr, 
                                                                                num_iterations,
                                                                                damping,
                                                                                kernel_threshold);


%%%%%%%%%%%%%%% PLOTS %%%%%%%%%%%%%%%

%plotOdometryAndGT(traj_meas, traj_gt);

figure(2);
hold on;
grid;
title("chi evolution");

subplot(1,2,1);
plot(chi_stats_r, 'r-', "linewidth", 2);
legend("Chi Poses"); grid; xlabel("iterations");
subplot(1,2,2);
plot(num_inliers_r, 'b-', "linewidth", 2);
legend("#inliers"); grid; xlabel("iterations");