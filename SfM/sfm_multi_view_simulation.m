function [xw_est, R_opm, t_opm] = sfm_multi_view_simulation(imagePoints, views)
% calibration result
load parameter.mat
load rotateMatrix.mat
load TransMatrix.mat
base_view = views(1);
m = size(views, 2);
prevPoints = imagePoints(:,:, base_view);
[ro, xs] = ray_in_out_pixel(prevPoints,d, 0);
prevBearing = [ro xs];
point_num = size(imagePoints, 1);
view = struct('points',{}, 'label',{},'rot',{},'trans',{}, 'bearing_vector',{});
view = addview(view, prevPoints, [], rotateMatrix(:,:,base_view), ...
    TransMatrix(base_view, :), prevBearing,base_view);
 xw_total = zeros(point_num, 3);
for i = 2:m
    currPoints = imagePoints(:,:, views(i));
    [ro1, xs1] = ray_in_out_pixel(currPoints,d, 0);
    currBearing = [ro1 xs1];
     U=umatrix_generator_general(prevBearing, currBearing);
     [R_est,t_est]=R_t_estimator_pixel(U);
     prevRot = view(i-1).rot;
     prevTrans = view(i-1).trans;
     rot = R_est* prevRot
     trans = prevTrans + t_est'*prevRot 
     view = addview(view, currPoints, [], rot, trans, currBearing,i);
    for j = 2:i
     xyzPoints = triangulate(view(j-1).bearing_vector, view(j).bearing_vector,...
          view(j).rot, view(j).trans);
     xw_total = xw_total + xyzPoints;
     scatter3(xyzPoints(:,1), xyzPoints(:,2), xyzPoints(:,3));
    end
    xw_average = sum(xw_total, 3)./point_num;
   %scatter3(xw_average(:,1), xw_average(:,2), xw_average(:,3));
   matchedPairs = ones(point_num, m);
    out = optim_point(xw_average, view, i, point_num, matchedPairs);
    xw_est = reshape(out(1:3*point_num),[point_num, 3]);
    R_opm = reshape(out(3*point_num+1:3*point_num + 9*(i-1)),[3,3, i-1]);
    t_opm = reshape(out(3*point_num + 9*(i-1)+1:end), [1 3 i-1]);
    view = updateView(view, R_opm, t_opm, i);
    prevBearing = currBearing;
end

end

function xw = triangulate(vec1Full, vec2Full, R1_est, t1_est)
    vec1 =  vec1Full;
    vec2  = vec2Full;
    r_out_w1 = vec1(:, 1:3)*R1_est';
    xs_w1 = vec1(:, 4:6)*R1_est' + t1_est;
    ro2 = vec2(:, 1:3);
    xs2 = vec2(:, 4:6);
    r_out_w2 = ro2;
    xs_w2 = xs2;
    v1 = sum(r_out_w1.*r_out_w1,2);
    v2 = sum(r_out_w2.*r_out_w2,2);
    v3 = sum(r_out_w1.*r_out_w2,2);
    w1 = sum((xs_w2-xs_w1).*r_out_w1,2);
    w2 = sum((xs_w2-xs_w1).*r_out_w2,2);
    s1 = (w1.*v2 - w2.*v3)./(v1.*v2-v3.*v3);
    s2 = (s1.*v3 -w2)./v2;
    xw = (xs_w1 + s1.*r_out_w1 + xs_w2 + s2.*r_out_w2)/2;
end

