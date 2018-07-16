%  load imagePoints.mat
load cube_points.mat
% cube_points = imagePoints;
views = [1 3 4 7 8];
m = size(views, 2);
n = size(cube_points, 1);
[xw_average, v, Rt, count_of_each_point] = sfm_multi_view_Rt(cube_points, views);
out = optim_point(xw_average, Rt, v, m, n, count_of_each_point);
xw_est = reshape(out(1:3*n),[n, 3]);
scatter3(xw_est(:,1), xw_est(:,2), xw_est(:,3));