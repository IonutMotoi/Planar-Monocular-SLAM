% Functions used to read the .dat files in the Dataset
1;

function [traj_meas,traj_gt] = readTrajectory()
  fid = fopen("./Dataset/trajectory.dat", 'r');
  data = textscan(fid, "%*s %*d %f %f %f %f %f %f");
  traj_meas = cell2mat(data(1,1:3))';
  traj_gt = cell2mat(data(1,4:6))';
  fclose(fid);
end

function [cam_mat, cam_trans, z_near, z_far, width, height] = readCameraParams()
  fid = fopen("./Dataset/camera.dat", 'r');
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
end

function lan_gt = readLandmarksGT()
  fid = fopen("./Dataset/world.dat", 'r');
  data = textscan(fid, "%*d %f %f %f");
  lan_gt = cell2mat(data)';
  fclose(fid);
end

function [id_landmarks, measurements] = readMeasurements(i)
  i = num2str(i-1,'%05.f'); % i-1 since measurements start from 0
  fid = fopen(strcat("./Dataset/meas-",i,".dat"), 'r');
  data = textscan(fid, "%*s %*d %f %f %f", "HeaderLines",3);
  id_landmarks = cell2mat(data(1,1))';
  measurements = cell2mat(data(1,2:3))';
  fclose(fid);
end
