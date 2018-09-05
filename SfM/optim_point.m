function out = optim_point(p, view, m, n)
% p is point 1xN , Rt estimated rotation and translation mat4x3xM
% v calucated bearing vector
p_one_row = reshape(p, 1, []);
R_one_row = reshape([view.rot], 1, []);
t_one_row = reshape([view.trans], 1, []);
v = reshape([view.bearing_vector], [], 6, m);
x0 = [p_one_row R_one_row(10:end) t_one_row(4:end)];

opts = optimset('Display', 'iter');
%opts = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','MaxFunEvals',3e4,'TolFun',1e-3);
out = lsqnonlin(@(x)fun(x,v,m,n), x0, [],[], opts);


end

function fval = fun(x, v, m, n)
 points = reshape(x(1: 3*n),[],3);
 Rot = reshape(x(3*n+1:3*n+9*(m-1)), [3 3 m-1] );
 trans = reshape(x(3*n+9*(m-1)+1:end),[1 3 m-1]);
 fval = 0;
for i = 2:m
    % m view m-1 rot needed to optim
  vec = v;
  %% the 3D points are alrealy in i view coordinate system
  xc = (points -trans(:,:,i-1))*Rot(:,:,i-1); % transfer to base view
  ro = vec(:,1:3, 1);
  xs = vec(:,4:6, 1);
  tmp = [0 1 1];
  coeff = repmat(tmp, size(xs,1), 1);
  N1 = xs.* coeff; % normal between  glass and water
  N1_norm = N1./sqrt(sum(N1.*N1,2));
  %t = (ro(:,3).*xc(:,3) + ro(:,2).*xc(:,2) + sqrt((ro(:,3).*xc(:,3) + ro(:,2).*xc(:,2)).^2 -(ro(:,2).*ro(:,2)+ro(:,3).*...
   % ro(:,3)).*(xc(:,2).*xc(:,2)+ xc(:,3).*xc(:,3)-50*50)))./(ro(:,2).*ro(:,2)+ro(:,3).*ro(:,3)); 
  % xs_est = xc - t.*ro;
  ro_est = xc - xs;
  ro_proj = ro(:,1:2)./ro(:,3);
  ro_est_proj = ro_est(:, 1:2)./ro_est(:,3);
  fval(end+1: end+n) = ro_proj(:,1)- ro_est_proj(:,1);
  fval(end+1: end + n)= ro_proj(:,2)- ro_est_proj(:,2);
  %fval(end+1: end + n)= xs_est(:,1)-xs(:,1);
  % fval(end+1: end + n)= xs_est(:,2)-xs(:,2);
   %fval(end+1: end + n)= xs_est(:,3)-xs(:,3);
  %xw_normal = cross(ro, N1_norm, 2);
  %fval(end+1: end+ n) = dot(ro_est, xw_normal, 2);
  fval(end+1)= norm(Rot(1,:,i-1)) -1;
  fval(end+1) = norm(Rot(2,:,i-1)) - 1;
  fval(end+1) = dot(Rot(1,:,i-1),Rot(2,:,i-1));
  r3 = cross(Rot(1,:,i-1), Rot(2,:,i-1)); 
  fval(end+1) = r3(1) - Rot(3,1,i-1);
  fval(end+1) = r3(2) - Rot(3,2,i-1);
  fval(end+1) = r3(3) - Rot(3,3,i-1);  
end

end