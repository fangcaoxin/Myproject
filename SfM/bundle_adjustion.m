%load imagePoints.mat
% load cube_points.mat
load imagePointMatrix1.mat % using R' to project imagePointMatrix using R to project
load basePoint.mat

imagePoints = zeros(size(basePoint,1), size(basePoint,2),size(imagePointMatrix, 3) + 1);
imagePoints(:,:,1) = basePoint;
imagePoints(:,:,2:end) = imagePointMatrix;

views = [1 2 3 7 9];
test_imagePoints = imagePoints(1:10:end, :,:);
[xw_est, view] = sfm_multi_view_simulation(imagePoints, views);
scatter3(xw_est(:,1), xw_est(:,2), xw_est(:,3));