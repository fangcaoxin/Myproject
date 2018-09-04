%load imagePoints.mat
% load cube_points.mat
load imagePointMatrix.mat
load basePoint.mat
views = [1 2 3 4 5];
imagepoints = zeros(size(basePoint,1), size(basePoint,2), size(imagePointMatrix,3)+1);
imagepoints(:,:,1) = basePoint;
imagepoints(:,:,2:end) = imagePointMatrix;
[xw_est, Rot, trans] = sfm_multi_view_simulation(imagepoints, views);
scatter3(xw_est(:,1), xw_est(:,2), xw_est(:,3));