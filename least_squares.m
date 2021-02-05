source "./least_squares_poses.m"
source "./least_squares_projections.m"

function [XR, XL] = boxPlus(XR, XL, dx)
  global num_poses;
  global num_landmarks;
  global pose_dim;
  global landmark_dim;
  
  for pose_index = 1:num_poses
    pose_matrix_index = poseMatrixIndex(pose_index);
    dxr=dx(pose_matrix_index:pose_matrix_index+pose_dim-1,:);
    XR(:,:,pose_index)=v2t(dxr)*XR(:,:,pose_index);
  end
  for landmark_index = 1:num_landmarks
    landmark_matrix_index = landmarkMatrixIndex(landmark_index);
    dxl=dx(landmark_matrix_index:landmark_matrix_index+landmark_dim-1,:);
    XL(:,landmark_index)+=dxl;
  end
end

function [XR, XL, chi_stats_p, num_inliers_p,chi_stats_r, num_inliers_r, H, b] = doTotalLS(XR, XL,
	     Zp, projection_associations,
	     Zr,
	     num_iterations,
	     damping,
       kernel_threshold,
       block_poses)
  
  global num_poses;
  global num_landmarks;
  global pose_dim;
  global landmark_dim;

  chi_stats_p=zeros(1,num_iterations);
  num_inliers_p=zeros(1,num_iterations);
  chi_stats_r=zeros(1,num_iterations);
  num_inliers_r=zeros(1,num_iterations);

  % Progress bar initialization
  msg = ["Iteration 0 out of ", num2str(num_iterations)];
  if(block_poses)
    wait_bar = waitbar(0, msg, "Name","Preliminary Landmarks Optimization");
  else
    wait_bar = waitbar(0, msg, "Name","Least Squares");
  endif

  % size of the linear system
  system_size=pose_dim*num_poses+landmark_dim*num_landmarks; 
  for (iteration=1:num_iterations)
    % Progress bar update
    msg = ["Iteration ", num2str(iteration), " out of ", num2str(num_iterations)];
    waitbar(iteration/num_iterations, wait_bar, msg);
    
    H=zeros(system_size, system_size);
    b=zeros(system_size,1);
   
    [H_projections, b_projections, chi_, num_inliers_] = buildLinearSystemProjections(XR,XL,Zp,projection_associations, kernel_threshold); 
    chi_stats_p(iteration)+=chi_;
    num_inliers_p(iteration)=num_inliers_;
    H += H_projections;
    b += b_projections;

    if(!block_poses)
      [H_poses, b_poses, chi_, num_inliers_] = buildLinearSystemPoses(XR, XL, Zr, kernel_threshold);
      chi_stats_r(iteration)+=chi_;
      num_inliers_r(iteration)=num_inliers_;
      H += H_poses;
      b += b_poses;
    endif

    H+=eye(system_size)*damping;
    dx=zeros(system_size,1);

    if(block_poses)
      % Solve the linear system while blocking all the poses
      dx(pose_dim*num_poses+1:end)=-(H(pose_dim*num_poses+1:end,pose_dim*num_poses+1:end)\b(pose_dim*num_poses+1:end,1));
    else
      % Solve the linear system while blocking the first pose
      dx(pose_dim+1:end)=-(H(pose_dim+1:end,pose_dim+1:end)\b(pose_dim+1:end,1));
    endif
    [XR, XL]=boxPlus(XR, XL, dx);
  endfor
  close(wait_bar);
end