1;

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
 endfunction;