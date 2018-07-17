%  load imagePoints.mat
load cube1_points.mat
%cube_points = imagePoints;
cube_points = cube1_points;
views = [1 3 5 7 9];
m = size(views, 2);
n = size(cube_points, 1);
[xw_average, v, Rt, count_of_each_point] = sfm_multi_view_Rt(cube_points, views);
out = optim_point(xw_average, Rt, v, m, n, count_of_each_point);
xw_est = reshape(out(1:3*n),[n, 3]);
scatter3(xw_est(:,1), xw_est(:,2), xw_est(:,3));