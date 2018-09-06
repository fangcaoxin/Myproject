%load imagePoints.mat
% load cube_points.mat
load imagePointMatrix.mat
load basePoint.mat

imagePoints = zeros(size(basePoint,1), size(basePoint,2),size(imagePointMatrix, 3) + 1);
imagePoints(:,:,1) = basePoint;
imagePoints(:,:,2:end) = imagePointMatrix;

views = [1 2];
test_imagePoints = imagePoints(1:10:end, :,:);
[xw_est, Rot, trans] = sfm_multi_view_simulation(imagePoints, views);
scatter3(xw_est(:,1), xw_est(:,2), xw_est(:,3));