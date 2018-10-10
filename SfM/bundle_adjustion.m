%load imagePoints.mat
% load cube_points.mat
addpath('cylindrical')
load imagePointMatrix1008.mat % using R' to project imagePointMatrix using R to project
load basePoint1008.mat

imagePoints = zeros(size(basePoint,1), size(basePoint,2),size(imagePointMatrix, 3) + 1);
imagePoints(:,:,1) = basePoint;
imagePoints(:,:,2:end) = imagePointMatrix;
imagePoints_noise_4 = imagePoints + 0.01*randn(size(imagePoints));
views = [1 2 3 4 5];
test_imagePoints = imagePoints(1:10:end, :,:);
[xw_est, view] = sfm_multi_view_simulation(imagePoints_noise_4, views);
%  scatter3(xw_est(:,1), xw_est(:,2), xw_est(:,3), 5, 'MarkerFaceColor',[0 0 1], 'MarkerEdgeColor', [0 0 0.5]);
%  grid on
%  axis equal