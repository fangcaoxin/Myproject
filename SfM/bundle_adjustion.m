%  load imagePoints.mat
load cube_points.mat
%cube_points = imagePoints;
% cube_points = cube1_points;
views = [1 4 5 7 9];
m = size(views, 2);
n = size(cube_points, 1);
[xw_est, Rot, trans] = sfm_multi_view_Rt(cube_points, views);
scatter3(xw_est(:,1), xw_est(:,2), xw_est(:,3));