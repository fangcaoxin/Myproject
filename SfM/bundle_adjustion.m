load imagePoints.mat
views = [1 2 3 4 5];
m = size(views, 2);
n = size(imagePoints, 1);
[xw_average, v, Rt] = sfm_multi_view_Rt(imagePoints, views);
out = optim_point(xw_average, Rt, v, m, n);
xw_est = reshape(out(1:3*n),[n, 3]);
scatter3(xw_est(:,1), xw_est(:,2), xw_est(:,3));