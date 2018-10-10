function [xyzPoints, view] = optim_point(view, tracks, step, startNum, endNum)
% p is point 1xN , Rt estimated rotation and translation mat4x3xM
% v calucated bearing vector
tracks_cell = struct2cell(tracks);
nxyzPoints = reshape(cell2mat(tracks_cell(3,:,:)), 3, []); % get pointcloud
p = nxyzPoints'; % all points now
p_select = p(startNum:step: endNum, :);
tracks_select = tracks(1, startNum:step: endNum);
numViews = numel(view);
numPoints = size(p_select,1);
p_one_row = reshape(p_select, 1, []);
R_one_row = reshape([view.rot], 1, []);
t_one_row = reshape([view.trans], 1, []);
v = reshape([view.bearing_vector], [], 6, numViews);
x0 = [p_one_row R_one_row(10:end) t_one_row(4:end)];
p_lb = -Inf(1, size(p_one_row,2));
p_ub = Inf(1, size(p_one_row,2));
R_lb = -ones(1, size(R_one_row(10:end),2));
R_ub = ones(1, size(R_one_row(10:end),2));
t_lb = -Inf(1, size(t_one_row(4:end),2));
t_ub = Inf(1, size(t_one_row(4:end),2));
lb = [p_lb R_lb t_lb];
ub = [p_ub R_ub t_ub];
%opts = optimset('Display', 'iter');
%opts = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','MaxFunEvals',3e4,'TolFun',1e-3);
opts = optimoptions('lsqnonlin','Display','iter','Algorithm','levenberg-marquardt','MaxFunEvals',3e4,'TolFun',1e-3);
out = lsqnonlin(@(x)fun(x,v,numViews,numPoints,tracks_select), x0, [],[], opts);
xyzPoints = reshape(out(1:3*numPoints),[numPoints, 3]);
R_opm = reshape(out(3*numPoints+1:3*numPoints + 9*(numViews-1)),[3,3, numViews-1]);
t_opm = reshape(out(3*numPoints + 9*(numViews-1)+1:end), [3 1 numViews-1]);
 for  k = 2: numViews
    view(k).rot = R_opm(:,:,k-1);
    view(k).trans = t_opm(:,:,k-1);
 end
end
% startNum the starting num of 3D points for optim
% endNum the ending num of 3D points for optim
% the step for 3D points for optim
function fval = fun(x, v, numViews,numPoints, tracks)
 m = numViews; % how many views
 n = numPoints; % how many points
 points = reshape(x(1:3*n), [],3);
 Rot = reshape(x(3*n+1:3*n+9*(m-1)), [3 3 m-1] );
 trans = reshape(x(3*n+9*(m-1)+1:end),[1 3 m-1]);
fval = zeros(1, n+6*(m-1));
for i = 1:n
    e_total = 0;
   for j = 1 : size(tracks(i).views, 2)
       view = tracks(i).views(j);
       if(view == 1)
           rotate = eye(3);
           translation = zeros(1,3);
       else
           rotate = Rot(:,:,view-1);
           translation = trans(:,:,view-1);
       end
       
       xc = rotate'*(points(i,:) - translation)'; 
       
       xc = xc';
       ro = v(tracks(i).points, 1:3, view);
       xs = v(tracks(i).points, 4:6, view);
       ro_est = xc - xs;
       ro_est = ro_est/norm(ro_est);
      
       e1 =norm(cross(ro, ro_est));
       tmp = [0 1 1];
       N1 = xs.*tmp;
       N1_norm = N1/norm(N1);
       xw_normal = cross(ro, N1_norm);
       e2 = dot(ro_est, xw_normal);
       e_total = e_total + e1 + e2;
   end
   fval(i) = e_total/size(tracks(i).views, 2);
end
 for j = 1:numViews-1
     rotate = Rot(:,:,j);
     fval(n + 6*(j-1)+1) = norm(rotate(1,:))-1;
     fval(n + 6*(j-1)+2) = norm(rotate(2,:))-1;
     fval(n + 6*(j-1)+3) = dot(rotate(1,:), rotate(2,:));
     r3 = cross(rotate(1,:), rotate(2,:));
     fval(n + 6*(j-1)+4) = r3(1)- rotate(3,1);
     fval(n + 6*(j-1)+5) = r3(2)- rotate(3,2);
     fval(n + 6*(j-1)+6) = r3(3)- rotate(3,3);
     
 end
% (reshape(x(3*n+9*view -17:3*n+9*view-8),[3 3])'*...
%            (x(3*i-2:3*i)'-x(3*n+9*(m-1)+3*view-5:3*n+9*(m-1)+3*view-3)'))'-xs
  %fval(end+1: end + n)= xs_est(:,1)-xs(:,1);
  % fval(end+1: end + n)= xs_est(:,2)-xs(:,2);
   %fval(end+1: end + n)= xs_est(:,3)-xs(:,3);
  %xw_normal = cross(ro, N1_norm, 2);
  %fval(end+1: end+ n) = dot(ro_est, xw_normal, 2);
%   fval(end+1)= norm(Rot(1,:,i-1)) -1;
%   fval(end+1) = norm(Rot(2,:,i-1)) - 1;
%   fval(end+1) = dot(Rot(1,:,i-1),Rot(2,:,i-1));
%   r3 = cross(Rot(1,:,i-1), Rot(2,:,i-1)); 
%   fval(end+1) = r3(1) - Rot(3,1,i-1);
%   fval(end+1) = r3(2) - Rot(3,2,i-1);
%   fval(end+1) = r3(3) - Rot(3,3,i-1);  
end
