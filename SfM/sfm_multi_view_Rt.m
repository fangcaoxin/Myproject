function [xw_average, v, Rt] = sfm_multi_view_Rt(imagePoints, views)
% calibration result
gg = [ -0.72005  2.06590  42.66089  -0.28110  -1.39643 -1.96133 ];
K =[590.2313 0 0; 0 559.4365 0; 369.2098 272.4348 1];
c = [1 1.49 1];
Ra = 50;
ra = 46; 
n = size(imagePoints, 1);
m =  size(views, 2); % the number of view
Rt = zeros(4,3,m);
xw_average = zeros(n, 3);
xw = zeros(n, 3, m);
v = zeros(n, 6, m);
base_view = views(1);
 x1 = imagePoints(:,:, base_view);
[xs1, ro1] = sfm_one_view_Rt(gg, x1, K, c, Ra, ra);
Rt(:,:,1) = [1 0 0; 0 1 0; 0 0 1; 0 0 0];
bearing_vec1 = [ro1 xs1];
v(:,:,1) = bearing_vec1;
for i = 2:m
    x = imagePoints(:,:, views(i));
    [xs, ro] = sfm_one_view_Rt(gg, x, K, c, Ra, ra);
    bearing_vec = [ro xs];
    U=umatrix_generator_general(bearing_vec1, bearing_vec);
    [R_est,t_est]=R_t_estimator_pixel(U);
    Rt(1:3,:, i) = R_est;
    Rt(4,:,i) = t_est';
    v(:,:, i) = bearing_vec;
    xw(:,:,i-1)= triangulate(bearing_vec1, bearing_vec, R_est, t_est);
end
xw_average = mean(xw, 3);
end

function xw = triangulate(vec1, vec2, R_est, t_est)
    r_out_w1 = vec1(:, 1:3);
    xs_w1 = vec1(:, 4:6);
    ro2 = vec2(:, 1:3);
    xs2 = vec2(:, 4:6);
    r_out_w2 = ro2*R_est';
    xs_w2 = xs2*R_est';
    xs_w2 = xs_w2 + t_est';
    v1 = sum(r_out_w1.*r_out_w1,2);
    v2 = sum(r_out_w2.*r_out_w2,2);
    v3 = sum(r_out_w1.*r_out_w2,2);
    w1 = sum((xs_w2-xs_w1).*r_out_w1,2);
    w2 = sum((xs_w2-xs_w1).*r_out_w2,2);
    s1 = (w1.*v2 - w2.*v3)./(v1.*v2-v3.*v3);
    s2 = (s1.*v3 -w2)./v2;
    xw = (xs_w1 + s1.*r_out_w1 + xs_w2 + s2.*r_out_w2)/2;
end

