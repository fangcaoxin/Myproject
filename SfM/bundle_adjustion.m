%load imagePoints.mat
% load cube_points.mat
load imagePointMatrix.mat
views = [1 3 6 8];

[xw_est, Rot, trans] = sfm_multi_view_simulation(imagePointMatrix, views);
scatter3(xw_est(:,1), xw_est(:,2), xw_est(:,3));