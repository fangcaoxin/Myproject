function [xyzPoints, camPoses] = refractiveBA(p, bearingVec, camPoses)
% p is point 1xN , Rt estimated rotation and translation mat4x3xM
% v calucated bearing vector
p_one_row = reshape(p, 1, []);
R_one_row = reshape(cell2mat(camPoses.Orientation), 1, []);
t_one_row = reshape(cell2mat(camPoses.Location), 1, []);
x0 = [p_one_row R_one_row t_one_row];
numPoints = numel(v);
numViews = size(camPoses,1);
%   opts = optimset('Display', 'off');
opts = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','MaxFunEvals',3e4,'TolFun',1e-3);
out = lsqnonlin(@(x)fun(x,bearingVec,numPoints, numViews), x0, [],[], opts);
xyzPoints = reshape(out(1:3*numPoints),[numPoints, 3]);
R_opm = reshape(out(3*numPoints+1:3*numPoints + 9*(numViews-1)),[3,3, numViews]);
t_opm = reshape(out(3*numPoints + 9*(numViews-1)+1:end), [1 3 numViews]);
for k = 1: numViews
    camPoses.Orientation(k) = R_opm(:,:,k);
    camPoses.Location(k)= t_opm(:,:,k);
end

function fval = fun(x, v, numPoints, numViews)

 points = reshape(x(1:3*numPoints), [],3);
 Rot = reshape(x(3*numPoints+1:3*numPoints+9*(numViews-1)), 3,3,[]);
 trans = reshape(x(3*numPoints+1+9*(numViews-1):end),1,3,[]);
 fval = 0;
 count = 1;
 for i = 1: numPoints
     for j = 1: size(v(i).ViewIds,2)
         view = v(i).ViewIds(j);
         xc = (points(i,:) -trans(:,:,view))*Rot(:,:,view);
         xs = v(i).BearingVector(j,1:3);
         ro = v(i).BearingVector(j,4:6);
         tmp = [0 1 1];
        N1 = xs.* tmp; % normal between  glass and water
        N1_norm = N1/norm(N1);
        ro_est = xc - xs;
        xw_normal = cross(ro, N1_norm, 2);
        fval(count) = dot(ro_est, xw_normal, 2);
        count = count + 1;
     end
 end

%   t = (ro(:,3).*xc(:,3) + ro(:,2).*xc(:,2) + sqrt((ro(:,3).*xc(:,3) + ro(:,2).*xc(:,2)).^2 -(ro(:,2).*ro(:,2)+ro(:,3).*...
%     ro(:,3)).*(xc(:,2).*xc(:,2)+ xc(:,3).*xc(:,3)-50*50)))./(ro(:,2).*ro(:,2)+ro(:,3).*ro(:,3)); 
%  xs_est = xc - t.*ro;
%   ro_est = xc - xs;
%   %ro_proj = ro(:,1:2)./ro(:,3);
%   %ro_est_proj = ro_est(:, 1:2)./ro_est(:,3);
%   %fval(end+1: end+pointsNumOfEachView(i)) = ro_proj(:,1)- ro_est_proj(:,1);
%   %fval(end+1: end + pointsNumOfEachView(i))= ro_proj(:,2)- ro_est_proj(:,2);
% %   fval(end+1: end + pointsNumOfEachView(i))= xs_est(:,1)-xs(:,1);
% %   fval(end+1: end + pointsNumOfEachView(i))= xs_est(:,2)-xs(:,2);
% %   fval(end+1: end + pointsNumOfEachView(i))= xs_est(:,3)-xs(:,3);
%   xw_normal = cross(ro, N1_norm, 2);
%   fval(end+1: end+ pointsNumOfEachView(i)) = dot(ro_est, xw_normal, 2);
%   fval(end+1)= norm(Rot(1,:,i-1)) -1;
%   fval(end+1) = norm(Rot(2,:,i-1)) - 1;
%   fval(end+1) = dot(Rot(1,:,i-1),Rot(2,:,i-1));
%   r3 = cross(Rot(1,:,i-1), Rot(2,:,i-1)); 
%   fval(end+1) = r3(1) - Rot(3,1,i-1);
%   fval(end+1) = r3(2) - Rot(3,2,i-1);
%   fval(end+1) = r3(3) - Rot(3,3,i-1);  


end
end