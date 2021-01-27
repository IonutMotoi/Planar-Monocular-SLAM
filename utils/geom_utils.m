1;

% computes the homogeneous transform matrix A of the pose vector v
% A:[ R t ] 3x3 homogeneous transformation matrix
% v: [x,y,theta]  2D pose vector
function A=v2t(v)
  c=cos(v(3));
  s=sin(v(3));
	A=[c, -s, v(1);
	   s,  c, v(2);
	   0,  0,   1];
end

global R0 = [0 -1;
						 1  0];

function v = flattenMatrixByColumns(M)
	v = zeros(6,1);
	v(1:4) = reshape(M(1:2,1:2),4,1);
	v(5:6) = M(1:2,3);
end
