function [xyzPoints, view] = optim_point(view, tracks)
% p is point 1xN , Rt estimated rotation and translation mat4x3xM
% v calucated bearing vector
step = 10;
tracks_cell = struct2cell(tracks);
nxyzPoints = reshape(cell2mat(tracks_cell(3,:,:)), 3, []);
p = nxyzPoints';
p = p(1:step:end, :);
numViews = numel(view);
numPoints = size(p,1);
p_one_row = reshape(p, 1, []);
R_one_row = reshape([view.rot], 1, []);
t_one_row = reshape([view.trans], 1, []);
v = reshape([view.bearing_vector], [], 6, numViews);
x0 = [p_one_row R_one_row(10:end) t_one_row(4:end)];

%opts = optimset('Display', 'iter');
%opts = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','MaxFunEvals',3e4,'TolFun',1e-3);
opts = optimoptions('lsqnonlin','Display','iter','Algorithm','levenberg-marquardt');
out = lsqnonlin(@(x)fun(x,v,numViews,numPoints,tracks, step), x0, [],[], opts);
xyzPoints = reshape(out(1:3*numPoints),[numPoints, 3]);
R_opm = reshape(out(3*numPoints+1:3*numPoints + 9*(numViews-1)),[3,3, numViews-1]);
t_opm = reshape(out(3*numPoints + 9*(numViews-1)+1:end), [3 1 numViews-1]);
 for  k = 2: numViews
    view(k).rot = R_opm(:,:,k-1);
    view(k).trans = t_opm(:,:,k-1);
 end
end

function fval = fun(x, v, numViews,numPoints, tracks,step)
 m = numViews; % how many views
 n = numPoints; % how many points
 points = reshape(x(1:3*n), [],3);
 Rot = reshape(x(3*n+1:3*n+9*(m-1)), [3 3 m-1] );
 trans = reshape(x(3*n+9*(m-1)+1:end),[1 3 m-1]);
fval = 0;
for i = 1:n
    rn = 1 + step*(i-1);
   for j = 1 : size(tracks(rn).views, 2)
       view = tracks(rn).views(j);
       if(view == 1)
           continue;
       end
       rotate = Rot(:,:,view-1);
       translation = trans(:,:,view-1);
       xc = rotate'*(points(i,:) - translation)';
       xc = xc';
       ro = v(tracks(rn).points, 1:3, view);
       xs = v(tracks(rn).points, 4:6, view);
       ro_est = xc - xs;
       ro_est = ro_est/norm(ro_est);
       ro_proj = ro(:,1:2)./ro(:,3);
       ro_est_proj = ro_est(:, 1:2)./ro_est(:,3);
       %fval(end + 1,1) = norm(ro_proj - ro_est_proj);
       fval(end + 1, 1) = norm(ro-ro_est);
%       tmp = [0 1 1];
%       N1 = xs.*tmp;
%       N1_norm = N1/norm(N1);
%       xw_normal = cross(ro, N1_norm);
%       fval(end+1)= dot(ro_est, xw_normal);
   end
end

 for j = 1 : size(tracks(rn).views, 2)
       view = tracks(rn).views(j);
       if(view == 1)
           continue;
       end
       rotate = Rot(:,:,view-1);
         fval(end+1,1) = norm(rotate(1,:))-1;
        fval(end+1,1) = norm(rotate(2,:))-1;
        fval(end+1,1) = dot(rotate(1,:),rotate(2,:));
         r3 = cross(rotate(1,:), rotate(2,:));
       fval(end+1,1) = r3(1) - rotate(3,1);
       fval(end+1,1) = r3(2) - rotate(3,2);
       fval(end+1,1) = r3(3) - rotate(3,3);  
 end

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