source "./utils/geom_utils.m"

function [e,Ji,Jj]=poseErrorAndJacobian(Xi,Xj,Z)
  global R0;
  Ri=Xi(1:2,1:2);
  Rj=Xj(1:2,1:2);
  ti=Xi(1:2,3);
  tj=Xj(1:2,3);
  tij=tj-ti;
  Ri_transposed=Ri';
  Ji=zeros(6,3);
  Jj=zeros(6,3);

  Jj(5:6,1:2)=Ri_transposed;
  Jj(1:4,3)=reshape(Ri_transposed*R0*Rj, 4, 1);
  Jj(5:6,3)=-Ri_transposed*R0*tj;
  Ji=-Jj;

  Z_hat=eye(3);
  Z_hat(1:2,1:2)=Ri_transposed*Rj;
  Z_hat(1:2,3)=Ri_transposed*tij;
  e=flattenMatrixByColumns(Z_hat-Z);
end


function [H,b, chi_tot, num_inliers]=buildLinearSystemPoses(XR, XL, Zr, kernel_threshold)
  global num_poses;
  global num_landmarks;
  global pose_dim;
  global landmark_dim;

  system_size = pose_dim*num_poses + landmark_dim*num_landmarks;
  H = zeros(system_size, system_size);
  b = zeros(system_size,1);
  chi_tot = 0;
  num_inliers = 0;
  for measurement_num = 1:size(Zr,3)
    Omega=eye(6);
    Omega(1:4,1:4)*=1e3; # we need to pimp the rotation  part a little
    Z=Zr(:,:,measurement_num);
    Xi=XR(:,:,measurement_num);
    Xj=XR(:,:,measurement_num+1);
    [e,Ji,Jj] = poseErrorAndJacobian(Xi, Xj, Z);
    chi=e'*Omega*e;
    if (chi>kernel_threshold)
      Omega*=sqrt(kernel_threshold/chi);
      chi=kernel_threshold;
    else
      num_inliers ++;
    end
    chi_tot+=chi;

    pose_i_matrix_index = 1 + (measurement_num-1)*pose_dim;
    pose_j_matrix_index= pose_i_matrix_index + pose_dim;
    
    H(pose_i_matrix_index:pose_i_matrix_index+pose_dim-1,
      pose_i_matrix_index:pose_i_matrix_index+pose_dim-1)+=Ji'*Omega*Ji;

    H(pose_i_matrix_index:pose_i_matrix_index+pose_dim-1,
      pose_j_matrix_index:pose_j_matrix_index+pose_dim-1)+=Ji'*Omega*Jj;

    H(pose_j_matrix_index:pose_j_matrix_index+pose_dim-1,
      pose_i_matrix_index:pose_i_matrix_index+pose_dim-1)+=Jj'*Omega*Ji;

    H(pose_j_matrix_index:pose_j_matrix_index+pose_dim-1,
      pose_j_matrix_index:pose_j_matrix_index+pose_dim-1)+=Jj'*Omega*Jj;

    b(pose_i_matrix_index:pose_i_matrix_index+pose_dim-1)+=Ji'*Omega*e;
    b(pose_j_matrix_index:pose_j_matrix_index+pose_dim-1)+=Jj'*Omega*e;
  end
end