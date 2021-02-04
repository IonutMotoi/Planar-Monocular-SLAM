% Reference Paper: Optimal Ray Intersection For Computing 3D Points From N-View Correspondences

% points : camera centers (x,y,z)'
% directions : rays passing through the projections of the landmarks (a,b,c)'
function P = triangulateMultipleViews(points, directions)
  A = zeros(3,3);
  B = zeros(3,1);
  P = zeros(3,1);

  for i = 1:size(points,2)
    a = directions(1,i);
    b = directions(2,i);
    c = directions(3,i);

    x = points(1,i);
    y = points(2,i);
    z = points(3,i);

    A(1,1) += 1 - a*a;
    A(1,2) += -a*b;
    A(1,3) += -a*c;
    A(2,2) += 1 - b*b;
    A(2,3) += -b*c;
    A(3,3) += 1 - c*c;

    B(1,1) += (1-a*a)*x - a*b*y - a*c*z;
    B(2,1) += -a*b*x + (1-b*b)*y - b*c*z;
    B(3,1) += -a*c*x - b*c*y + (1-c*c)*z;
  endfor
  
  % A is symmetric
  A(2,1) = A(1,2);
  A(3,1) = A(1,3);
  A(3,2) = A(2,3);

  % Solve linear system
  P = A\B;
endfunction