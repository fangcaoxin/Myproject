function out = optim_point(p, view, m, n, matchedPairs)
% p is point 1xN , Rt estimated rotation and translation mat4x3xM
% v calucated bearing vector
[r c] = find(matchedPairs(:,1)>0);
p_one_row = reshape(p(r,:), 1, []);
R_one_row = reshape([view.rot], 1, []);
t_one_row = reshape([view.trans], 1, []);
v = reshape([view.bearing_vector], [], 6, m);
x0 = [p_one_row R_one_row(10:9*m) t_one_row(4:3*m)];
  opts = optimset('Display', 'off');
%opts = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','MaxFunEvals',3e4,'TolFun',1e-3);
out = lsqnonlin(@(x)fun(x,v,m,n, matchedPairs), x0, [],[], opts);

end

function fval = fun(x, v, m, n, matchedPairs)
   [baseRow baseCol] = find(matchedPairs(:,1)>0);
   pointsNumOfEachView = zeros(1, m);
   cpn = zeros(1,m); % calcPointNum
   inputPointNum = size(baseRow,1);
 %fval = zeros(2*sum(pointsNumOfEachView)+(m-1)*6,1);
 Rot = reshape(x(3*inputPointNum+1:3*inputPointNum+9*(m-1)), [3 3 m-1]);
 trans = reshape(x(3*inputPointNum+1+9*(m-1):end),[1 3 m-1]);
 fval = 0;
for i = 2:m
    % m view m-1 rot needed to optim
    [srcRows srccols] = find(matchedPairs(:,i)>0);
    pointsNumOfEachView(1,i) = size(srcRows, 1);
    sRows = findRows(baseRow, srcRows);
    points = reshape(x(1: 3*size(srcRows,1)),[],3);
    vec = v(srcRows,:,i);
  cpn(i) = 3*sum(pointsNumOfEachView(1:i)+(i-1)*6);
  xc = (points -trans(:,:,i-1))*Rot(:,:,i-1);
  ro = vec(:,1:3);
  xs = vec(:,4:6);
  tmp = [0 1 1];
  coeff = repmat(tmp, size(xs,1), 1);
  N1 = xs.* coeff; % normal between  glass and water
  N1_norm = N1./sqrt(sum(N1.*N1,2));
  ro_est = xc - xs;
  ro_proj = ro(:,1:2)./ro(:,3);
  ro_est_proj = ro_est(:, 1:2)./ro_est(:,3);
  %fval(end+1: end+pointsNumOfEachView(i)) = ro_proj(:,1)- ro_est_proj(:,1);
  %fval(end+1: end + pointsNumOfEachView(i))= ro_proj(:,2)- ro_est_proj(:,2);
  xw_normal = cross(ro, N1_norm, 2);
  fval(end+1: end+ pointsNumOfEachView(i)) = dot(ro_est, xw_normal, 2);
  fval(end+1)= norm(Rot(1,:,i-1)) -1;
  fval(end+1) = norm(Rot(2,:,i-1)) - 1;
  fval(end+1) = dot(Rot(1,:,i-1),Rot(2,:,i-1));
  r3 = cross(Rot(1,:,i-1), Rot(2,:,i-1)); 
  fval(end+1) = r3(1) - Rot(3,1,i-1);
  fval(end+1) = r3(2) - Rot(3,2,i-1);
  fval(end+1) = r3(3) - Rot(3,3,i-1);  
end
% to get 
function sRows = findRows(x1, x2)
  sRows = zeros(size(x2,1),1);
  for k = 1: size(x2,1)
     [sRows(k,1), sCols]=find(x1 == x2(k,1));
  end
end
end