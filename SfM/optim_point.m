function out = optim_point(p, Rt, v, m, n, count_of_each_point)
% p is point 1xN , Rt estimated rotation and translation mat4x3xM
% v calucated bearing vector
p_one_row = reshape(p, 1, []);
Rt_one_row = reshape(Rt, 1, []);
x0 = [p_one_row Rt_one_row];
 opts = optimset('Display', 'off');
%opts = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','MaxFunEvals',3e4,'TolFun',1e-3);
out = lsqnonlin(@(x)fun(x,v,m,n, count_of_each_point), x0, [],[], opts);

end

function fval = fun(x, v, m, n, count_of_each_point)
fval = zeros(3*n+6,1,m);
p = reshape (x(1:3*n), [n 3]);
Rt = reshape(x(3*n+1:end), [4 3 m]);

for i = 1:m
  num = sum(count_of_each_point(:,i)==1);
  xc = zeros(num, 3);
  ro = zeros(num,3);
  xs = zeros(num,3);
  count = 1;
  for k = 1: n
      if(count_of_each_point(k,i)==1)
      xc(count,:)= (p(k,:) - Rt(4,:,i))*Rt(1:3,:,i);
      ro(count,:) = v(k, 1:3,i);
      xs(count,:) = v(k, 4:6,i);
      count = count + 1;
      end
  end
  tmp = [0 1 1];
  coeff = repmat(tmp, size(xs,1), 1);
  N1 = xs.* coeff; % normal between  glass and water
  N1_norm = N1./sqrt(sum(N1.*N1,2));
  ro_est = xc - xs;
  ro_proj = ro(:,1:2)./ro(:,3);
  ro_est_proj = ro_est(:, 1:2)./ro_est(:,3);
  fval(1:num,1,i) = ro_proj(:,1)- ro_est_proj(:,1);
  fval(num+1:2*num,:,i) = ro_proj(:,2)- ro_est_proj(:,2);
  xw_normal = cross(ro, N1_norm, 2);
  fval(2*num+1:3*num,:,i) = dot(ro_est, xw_normal, 2);
  fval(3*num+1,:, i) = norm(Rt(1,:,i)) - 1;
  fval(3*num+2,:, i) = norm(Rt(2,:,i)) - 1;
  fval(3*num+3,:, i) = dot(Rt(1,:,i),Rt(2,:,i));
  r3 = cross(Rt(1,:,i), Rt(2,:,i));
  fval(3*num+4,:, i) = r3(1) - Rt(3,1,i);
  fval(3*num+5,:, i) = r3(2) - Rt(3,2,i);
  fval(3*num+6,:, i) = r3(3) - Rt(3,3,i);  
end
end