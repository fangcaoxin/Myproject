function [xyzPoints, view] = optim_point(view, tracks, K, step, startNum, endNum)
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
<<<<<<< HEAD
v = reshape([view.bearing_vector], [], 6, numViews);
points_2d = reshape([view.points], [], 2, numViews);
x0 = [p_one_row R_one_row(10:end) t_one_row(4:end)];
=======
v = reshape([view.bearing_vector], [], 9, numViews);
x0 = [p_one_row R_one_row t_one_row];
>>>>>>> 77e8b05ee755f8118a972afe53521fb931f38ddf
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
<<<<<<< HEAD
%opts = optimoptions('lsqnonlin','Display','iter','Algorithm','levenberg-marquardt','MaxFunEvals',3e10,'TolFun',1e-10);
out = lsqnonlin(@(x)fun(x,v,points_2d,K, numViews,numPoints,tracks_select), x0, [],[], opts);
=======
opts = optimoptions('lsqnonlin','Display','iter','Algorithm','levenberg-marquardt','MaxFunEvals',3e10,'TolFun',1e-5);
out = lsqnonlin(@(x)fun(x,v,numViews,numPoints,tracks_select), x0, [],[], opts);
>>>>>>> 77e8b05ee755f8118a972afe53521fb931f38ddf
xyzPoints = reshape(out(1:3*numPoints),[numPoints, 3]);
R_opm = reshape(out(3*numPoints+1:3*numPoints + 9*(numViews)),[3,3, numViews]);
t_opm = reshape(out(3*numPoints + 9*(numViews)+1:end), [3 1 numViews]);
 for  k = 1: numViews
    view(k).rot = R_opm(:,:,k);
    view(k).trans = t_opm(:,:,k)';
 end
end
% startNum the starting num of 3D points for optim
% endNum the ending num of 3D points for optim
% the step for 3D points for optim
function fval = fun(x, v, points_2d,K, numViews,numPoints, tracks)
 m = numViews; % how many views
 n = numPoints; % how many points
 points = reshape(x(1:3*n), [],3);
 Rot = reshape(x(3*n+1:3*n+9*(m)), [3 3 m] );
 trans = reshape(x(3*n+9*(m)+1:end),[1 3 m]);
fval = zeros(1, n+6*(m));
for i = 1:n
    e_total = 0;
   for j = 1 : size(tracks(i).views, 2)
       view = tracks(i).views(j);
      
        rotate = Rot(:,:,view);
        translation = trans(:,:,view);
       
       
       xc = rotate'*(points(i,:) - translation)'; 
       
       xc = xc';
       ro = v(tracks(i).points, 1:3, view);
       xs = v(tracks(i).points, 4:6, view);
       ro_est = xc - xs;
       ro_est = ro_est/norm(ro_est);
      
       e1 =norm(cross(ro, ro_est));
<<<<<<< HEAD
=======
       ri = v(tracks(i).points, 7:9, view);
>>>>>>> 77e8b05ee755f8118a972afe53521fb931f38ddf
       tmp = [0 0 1];
       N1 = xs.*tmp;
       N1_norm = N1/norm(N1);
       xw_normal = cross(ro, N1_norm);
       e2 = dot(ro_est, xw_normal);
       xc_2d = K'*xc';
       xc_2d_norm = [xc_2d(1,1) xc_2d(2,1)]./xc_2d(3,1);
       e3 = norm(xc_2d_norm - points_2d(tracks(i).points, :, view));
       e_total = e_total + e1 + e2;
   end
   fval(i) = e_total/size(tracks(i).views, 2);
end
 for j = 1:numViews
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
