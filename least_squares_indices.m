1;

function v_idx = poseMatrixIndex(pose_index)
  global pose_dim;
  v_idx = 1 + (pose_index-1)*pose_dim;
end

function v_idx = landmarkMatrixIndex(landmark_index)
  global pose_dim;
  global landmark_dim;
  global num_poses;
  v_idx = 1 + (num_poses)*pose_dim + (landmark_index-1)*landmark_dim;
end