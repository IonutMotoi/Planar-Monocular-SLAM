function [XL_guess, Zp, projection_associations, new_id_landmarks] = initializeLandmarks(XR_guess, Zp, projection_associations, id_landmarks)
  global num_poses;
  global num_landmarks;
  global pose_dim;
  global landmark_dim;
  global cam_pose;
  global z_far;
  global z_near;
  
  
  % Get all camera poses in world coordinates
  X_CAM = zeros(4,4,num_poses);
  for i = 1:num_poses
    X_CAM(:,:,i) = getCameraPose(XR_guess(:,:,i));
  endfor

  % Get the positions of the cameras
  CAM_POS = X_CAM(1:3,4,:);
  
  msg = ["Landmark 0 out of ", mat2str(num_landmarks)];
  wait_bar = waitbar(0, msg, "Name","Landmarks initialization");

  num_ignored_landmarks = 0;
  new_num_landmarks = 0;
  for current_landmark = 1:num_landmarks
    msg = ["Landmark ", mat2str(current_landmark), " out of ", mat2str(num_landmarks)];
    waitbar(current_landmark/num_landmarks, wait_bar, msg);

    idx = (projection_associations(2,:) == id_landmarks(current_landmark));
    poses = projection_associations(1,idx);
    projections = Zp(:,idx);

    % If the landmark is only seen once ignore it (cannot triangulate)
    % Also remove the projection measurement and the association
    if size(poses,2) < 2
      num_ignored_landmarks += 1;
      continue;
    endif
    
    % Get all the directions from the views pointing at the current landmark
    for current_pose = 1:size(poses,2)
      R = X_CAM(1:3,1:3,poses(current_pose));
      points(:,current_pose) = X_CAM(1:3,4,poses(current_pose));
      directions(:,current_pose) = R * directionFromImgCoordinates(projections(:,current_pose));
    endfor

    new_num_landmarks += 1; % to account for ignored landmarks
    % Triangulate landmark using all the available views
    XL_guess(:,new_num_landmarks) = triangulateMultipleViews(points, directions);
    new_id_landmarks(new_num_landmarks) = id_landmarks(current_landmark);
  endfor

  close(wait_bar);
  disp([mat2str(num_ignored_landmarks), " landmarks were ignored because they are only seen once"])
  num_landmarks = new_num_landmarks;
end

% Returns camera pose in world coordinates
function X_cam = getCameraPose(XR) 
  global cam_pose;
  X_cam = eye(4);
  X_cam(1:2,1:2) = XR(1:2,1:2);
  X_cam(1:2,4) = XR(1:2,3);
  X_cam = X_cam * cam_pose;
endfunction

% Returns a unit 3d vector given the image coordinates
function d = directionFromImgCoordinates(img_coord)
  global invK;
  img_coord(3,1) = 1;
  d = invK*img_coord;
  d *= 1/norm(d);
endfunction