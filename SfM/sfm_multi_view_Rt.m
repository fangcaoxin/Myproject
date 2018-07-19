function [xw_est, R_opm, t_opm] = sfm_multi_view_Rt(imagePoints, views)
% calibration result
gg = [ -0.72005  2.06590  42.66089  -0.28110  -1.39643 -1.96133 ];
K =[590.2313 0 0; 0 559.4365 0; 369.2098 272.4348 1];
c = [1 1.49 1];
Ra = 50;
ra = 46; 
n = size(imagePoints, 1);
m =  size(views, 2); % the number of view
Rt = zeros(4,3,m);
% xw_average = zeros(n, 3);
xw = zeros(n, 3, m);
v = zeros(n, 6, m);
base_view = views(1);
 prevPoints = imagePoints(:,:, base_view);
 prevLabels = set_label(prevPoints);
[xs, ro] = sfm_one_view_Rt(gg, prevPoints, K, c, Ra, ra);
Rt(:,:,1) = [1 0 0; 0 1 0; 0 0 1; 0 0 0];
prevBearing = [ro xs];

view = struct('points',{}, 'label',{},'rot',{},'trans',{}, 'bearing_vector',{});
view = addview(view, prevPoints, prevLabels, eye(3), zeros(1,3), prevBearing,1);
matchedPairs= zeros(n,m);
for i = 2:m
    currPoints = imagePoints(:,:, views(i));
    currLabels = set_label(currPoints);
    [xs1, ro1] = sfm_one_view_Rt(gg, currPoints, K, c, Ra, ra);
    currBearing = [xs1 ro1];
    bearing_vec = [ro xs];
    [matchedVector1, matchedVector2, indexPairs] = matchVectors...
        (prevBearing,currBearing,prevLabels,currLabels);
     U=umatrix_generator_general(matchedVector1, matchedVector2);
     [R_est,t_est]=R_t_estimator_pixel(U);
     prevRot = view(i-1).rot;
     prevTrans = view(i-1).trans;
     rot = R_est* prevRot;
     trans = prevTrans + t_est'*prevRot; 
     view = addview(view, currPoints, currLabels, rot, trans, currBearing,i);
 %Out = opengv('seventeenpt',[1:1:size(bearing_vec_select, 1)],bearing_vec1_select', bearing_vec_select');
 %R_est = Out(:,1:3);
 %t_est = Out(:,4);
    matchedPairs(:,1) = matchedPairs(:,1) + indexPairs;
    matchedPairs(:,i) = indexPairs;
    points_num = sum(matchedPairs(:,1)>0);
    xw_total = zeros(n, 3);
    xw = zeros(n, 3);
    for k = 1: i-1
      xyzPoints = triangulate(matchedVector1, matchedVector2,rot, trans);
      [rows cols] = find(indexPairs > 0);
      xw(rows,:) = xyzPoints;
      xw_total = xw_total + xw;
    end
    xw_average = sum(xw_total, 3)./sum(matchedPairs,2);
    out = optim_point(xw_average, view, i, n, matchedPairs);
    xw_est = reshape(out(1:3*points_num),[points_num, 3]);
    R_opm = reshape(out(3*points_num+1:3*points_num + 9*(i-1)),[3,3, i-1]);
    t_opm = reshape(out(3*points_num + 9*(i-1)+1:end), [1 3 i-1]);
    view = updateView(view, R_opm, t_opm, i);
    prevBearing = currBearing;
    prevLabels = currLabels;
end

end

function xw = triangulate(vec1, vec2, R_est, t_est)
    r_out_w1 = vec1(:, 1:3);
    xs_w1 = vec1(:, 4:6);
    ro2 = vec2(:, 1:3);
    xs2 = vec2(:, 4:6);
    r_out_w2 = ro2*R_est';
    xs_w2 = xs2*R_est';
    xs_w2 = xs_w2 + t_est;
    v1 = sum(r_out_w1.*r_out_w1,2);
    v2 = sum(r_out_w2.*r_out_w2,2);
    v3 = sum(r_out_w1.*r_out_w2,2);
    w1 = sum((xs_w2-xs_w1).*r_out_w1,2);
    w2 = sum((xs_w2-xs_w1).*r_out_w2,2);
    s1 = (w1.*v2 - w2.*v3)./(v1.*v2-v3.*v3);
    s2 = (s1.*v3 -w2)./v2;
    xw = (xs_w1 + s1.*r_out_w1 + xs_w2 + s2.*r_out_w2)/2;
end

