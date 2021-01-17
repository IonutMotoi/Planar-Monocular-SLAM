# Functions used to read the .dat file in the Dataset
1;

function [traj_meas,traj_gt] = readTrajectory(file_path)
  fid = fopen(file_path, 'r');
  data = textscan(fid, "%*s %*d %f %f %f %f %f %f");
  traj_meas = cell2mat(data(1,1:3));
  traj_gt = cell2mat(data(1,4:6));
  fclose(fid);
endfunction

function [cam_mat, cam_trans, z_near, z_far, width, height] = readCamera(file_path)
  fid = fopen(file_path, 'r');
  data = textscan(fid, "%f %f %f", 3, "HeaderLines",1, "CollectOutput",1);
  cam_mat = cell2mat(data);
  data = textscan(fid, "%f %f %f %f", 4, "HeaderLines",1, "CollectOutput",1);
  cam_trans = cell2mat(data);
  data = textscan(fid, "%*s %f");
  data = cell2mat(data);
  z_near = data(1);
  z_far = data(2);
  width = data(3);
  height = data(4);
  fclose(fid);
endfunction