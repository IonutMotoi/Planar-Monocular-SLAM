function XL_guess = initializeLandmarks(XR_guess, Zp, projection_associations, id_landmarks)
  global num_poses;
  global num_landmarks;
  global pose_dim;
  global landmark_dim;
  global cam_pose;
  global z_far;
  global z_near;
  
  XL_guess = zeros(landmark_dim, num_landmarks);
  defaultZ = (z_far - z_near) / 2;
  
  for i = 1:num_landmarks
    idx = (projection_associations(2,:) == id_landmarks(i));
    associations = projection_associations(1,idx);
    % if the landmark is only seen once remove it from the list (cannot triangulate)
    if size(associations,2) < 2
      XL_guess(:,i) = [NaN, NaN, NaN]';
      id_landmarks(i) = NaN;
      continue;
    endif
    projections = Zp(:,idx);
    

    X1 = getCameraPose(XR_guess(:,:,associations(1)));
    R1 = X1(1:3,1:3);
    t1 = X1(1:3,4);
    d1 = R1*directionFromImgCoordinates( projections(:,1) );

    X2 = getCameraPose(XR_guess(:,:,associations(2)));
    R2 = X2(1:3,1:3);
    d2 = R2*directionFromImgCoordinates( projections(:,2) );

    second_cam_pose = 2;
    min_dot_product = abs(dot(d1,d2));

    if size(associations,2) > 2
      for j = 3:size(associations,2);
        Xtemp = getCameraPose( XR_guess(:,:,associations(j)) );
        Rtemp = Xtemp(1:3,1:3);
        dtemp = Rtemp * directionFromImgCoordinates( projections(:,j) );
        dot_product = abs(dot(d1,dtemp));

        if dot_product < min_dot_product
          min_dot_product = dot_product;
          second_cam_pose = j;
          X2 = Xtemp;
          d2 = dtemp;
        endif
      endfor
    endif

    t2 = X2(1:3,4);
    p2 = t2 -t1;
    
    [success, l, e] = triangulatePoint(p2,d1,d2);
    XL_guess(:,i) = l + t1;
  endfor
  % update the number of landmarks
  num_landmarks = size(id_landmarks,2);
end

% Returns camera pose in world coordinates
function X_cam = getCameraPose(XR) 
  global cam_pose;
  X_cam = eye(4);
  X_cam(1:2,1:2) = XR(1:2,1:2);
  X_cam(1:2,4) = XR(1:2,3);
  X_cam = X_cam * cam_pose;
endfunction

function d = directionFromImgCoordinates(img_coord)
  global invK;
  img_coord(3,1) = 1;
  d = invK*img_coord;
  d *= 1/norm(d);
endfunction

function [success, p, e]=triangulatePoint(p2, d1, d2)
  p=zeros(3,1);
  success=false;
  e=-1;
                      
  D=[-d1, d2];         #assemble system matrix to find ascissa 
  s=-(D'*D)\(D'*p2);  
  #s: ascissa of closest point on p1 and p2
  if (s(1)<0 || s(2)<0)
    disp("ERRROROORORORORO")
    return;
  endif;
  success=true;
  p1_triangulated=d1*s(1);   # point on 1st line
  p2_triangulated=d2*s(2)+p2; # point on 2nd line
  e=norm(p1_triangulated-p2_triangulated); #difference between the points
  p=0.5*(p1_triangulated+p2_triangulated);               #midpoint
endfunction;