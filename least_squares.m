1;

function [XR, XL] = boxPlus(XR, XL, dx)
  global num_poses;
  global num_landmarks;
  global pose_dim;
  global landmark_dim;
  
  for pose_index = 1:num_poses
    pose_matrix_index = 1 + (pose_index-1)*pose_dim;
    dxr=dx(pose_matrix_index:pose_matrix_index+pose_dim-1);
    XR(:,:,pose_index)=v2t(dxr)*XR(:,:,pose_index);
  end
  for landmark_index = 1:num_landmarks
    landmark_matrix_index=1 + num_poses*pose_dim + (landmark_index-1)*landmark_dim;
    dxl=dx(landmark_matrix_index:landmark_matrix_index+landmark_dim-1,:);
    XL(:,landmark_index)+=dxl;
  end
end

function [XR, XL, chi_stats_p, num_inliers_p,chi_stats_r, num_inliers_r, H, b] = doTotalLS(XR, XL,
	     Zp, projection_associations,
	     Zr, pose_associations,
	     num_iterations,
	     damping,
	     kernel_threshold)
  
  global num_poses;
  global num_landmarks;
  global pose_dim;
  global landmark_dim;

  chi_stats_p=zeros(1,num_iterations);
  num_inliers_p=zeros(1,num_iterations);
  chi_stats_r=zeros(1,num_iterations);
  num_inliers_r=zeros(1,num_iterations);

  % size of the linear system
  system_size=pose_dim*num_poses+landmark_dim*num_landmarks; 
  for (iteration=1:num_iterations)
    H=zeros(system_size, system_size);
    b=zeros(system_size,1);
   
    [H_projections, b_projections, chi_, num_inliers_] = buildLinearSystemProjections(XR,XL,Zp,projection_associations, kernel_threshold); 
    chi_stats_p(iteration)+=chi_;
    num_inliers_p(iteration)=num_inliers_;

    [H_poses, b_poses, chi_, num_inliers_] = buildLinearSystemPoses(XR, XL, Zr, pose_associations, kernel_threshold);
    chi_stats_r(iteration)+=chi_;
    num_inliers_r(iteration)=num_inliers_;
    
    H = H_poses + H_projections;
    b = b_poses + b_projections;

    H+=eye(system_size)*damping;
    dx=zeros(system_size,1);

    % we solve the linear system, blocking the first pose
    % this corresponds to "remove" from H and b the locks
    % of the 1st pose, while solving the system
    dx(pose_dim+1:end)=-(H(pose_dim+1:end,pose_dim+1:end)\b(pose_dim+1:end,1));
    [XR, XL]=boxPlus(XR,XL,num_poses, num_landmarks, dx);
  end
end