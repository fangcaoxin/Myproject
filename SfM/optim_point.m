function out = optim_point(p, view, m, n, matchedPairs)
% p is point 1xN , Rt estimated rotation and translation mat4x3xM
% v calucated bearing vector
[r c] = find(matchedPairs(:,1)>0);
p_one_row = reshape(p(r,:), 1, []);
R_one_row = reshape([view.rot], 1, []);
t_one_row = reshape([view.trans], 1, []);
v = reshape([view.bearing_vector], [], 6, m);
x0 = [p_one_row Rt_one_row t_one_row];
 opts = optimset('Display', 'off');
%opts = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','MaxFunEvals',3e4,'TolFun',1e-3);
out = lsqnonlin(@(x)fun(x,v,m,n, matchedPairs), x0, [],[], opts);

end

function fval = fun(x, v, m, n, matchedPairs)
   [baseRow baseCol] = find(matchedPairs(:,1)>0);
   pointsNum = zeros(1, m-1);
   points = struct('p',{});
 for j = 2:m
   [srcRows srccols] = find(matchedPairs(:,j)>0);
    pointsNum(1,j-1) = size(srcRows, 1);
    sRows = findRows(baseRow, srcRows);
    points(j-1).p = x(1: size(baseRow,1));
 end
 
fval = zeros(6*sum(pointsNum)+(m-1)*6,1,m);
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
% to get 
function sRows = findRows(x1, x2)
  sRows = zeros(size(x2,1),1);
  for i = 1: size(x2,1)
     [sRows(i,1), sCols]=find(x1 == x2(i,1));
  end
end
end