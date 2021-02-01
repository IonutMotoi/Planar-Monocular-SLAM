source "./utils/geom_utils.m"
source "./least_squares_indices.m"

function [is_valid, e,Jr,Jl]=projectionErrorAndJacobian(Xr,Xl,z)
  global K;
  global cam_pose;
  global z_near;
  global z_far;
  global image_rows;
  global image_cols;

  is_valid=false;
  e = [0;0];
  Jr = zeros(2,3);
  Jl = zeros(2,3);

  % 2D pose -> 3D pose (z=0, rotation only around z axis)
  X_robot = eye(4);
  X_robot(1:2,1:2) = Xr(1:2,1:2);
  X_robot(1:2,4) = Xr(1:2,3);

  % Camera pose in world coordinates
  % X_cam = X_robot * cam_pose;
  iR_cam = cam_pose(1:3,1:3)';
  it_cam = -iR_cam * cam_pose(1:3,4);

  % inverse transform
  iR = X_robot(1:3,1:3)';
  it = -iR * X_robot(1:3,4);

  % point prediction
  % pw = iR*Xl + it;
  pw = iR_cam * (iR*Xl + it) + it_cam;
  if pw(3) < z_near || pw(3) > z_far
     return;
  end

  Jwr = zeros(3,3);
  Jwr(1:3,1:2) = -iR_cam * iR * [1,0; 0,1; 0,0];
  Jwr(1:3,3) = iR_cam * iR * [0,1,0; -1,0,0; 0,0,0] * Xl;
  Jwl = iR_cam * iR;

  p_cam = K * pw;
  iz = 1./p_cam(3);
  z_hat = p_cam(1:2)*iz;
  if (z_hat(1)<0 || 
      z_hat(1)>image_cols ||
      z_hat(2)<0 || 
      z_hat(2)>image_rows)
    return;
  end

  iz2 = iz*iz;
  Jp = [iz,  0, -p_cam(1)*iz2;
         0, iz, -p_cam(2)*iz2];
  
  e = z_hat - z;
  Jr = Jp * K * Jwr;
  Jl = Jp * K * Jwl;
  is_valid = true;
end


function [H,b, chi_tot, num_inliers]=buildLinearSystemProjections(XR, XL, Zl, associations, kernel_threshold)
  global num_poses;
  global num_landmarks;
  global pose_dim;
  global landmark_dim;

  system_size = pose_dim*num_poses + landmark_dim*num_landmarks; 
  H = zeros(system_size, system_size);
  b = zeros(system_size,1);
  chi_tot = 0;
  num_inliers = 0;

  for (measurement_num=1:size(Zl,2))
    pose_index=associations(1,measurement_num);
    landmark_index=associations(2,measurement_num);
    z=Zl(:,measurement_num);
    Xr=XR(:,:,pose_index);
    Xl=XL(:,landmark_index);

    [is_valid, e,Jr,Jl] = projectionErrorAndJacobian(Xr, Xl, z);
    if (! is_valid)
       continue;
    end

    chi=e'*e;
    if (chi>kernel_threshold)
      e*=sqrt(kernel_threshold/chi);
      chi=kernel_threshold;
    else
      num_inliers++;
    end
    chi_tot+=chi;

    pose_matrix_index=poseMatrixIndex(pose_index);
    landmark_matrix_index=landmarkMatrixIndex(landmark_index);

    H(pose_matrix_index:pose_matrix_index+pose_dim-1,
      pose_matrix_index:pose_matrix_index+pose_dim-1)+=Jr'*Jr;

    H(pose_matrix_index:pose_matrix_index+pose_dim-1,
      landmark_matrix_index:landmark_matrix_index+landmark_dim-1)+=Jr'*Jl;

    H(landmark_matrix_index:landmark_matrix_index+landmark_dim-1,
      landmark_matrix_index:landmark_matrix_index+landmark_dim-1)+=Jl'*Jl;

    H(landmark_matrix_index:landmark_matrix_index+landmark_dim-1,
      pose_matrix_index:pose_matrix_index+pose_dim-1)+=Jl'*Jr;

    b(pose_matrix_index:pose_matrix_index+pose_dim-1)+=Jr'*e;
    b(landmark_matrix_index:landmark_matrix_index+landmark_dim-1)+=Jl'*e;
  end
end