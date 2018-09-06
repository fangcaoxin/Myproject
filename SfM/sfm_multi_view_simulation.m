function [xw_est, R_opm, t_opm] = sfm_multi_view_simulation(imagePoints, views)
% calibration result
load parameter.mat
load rotateMatrix.mat
load TransMatrix.mat
addpath('helpers');
base_view = views(1);
m = size(views, 2);
prevPoints = imagePoints(:,:, base_view);
% plot(prevPoints(:,1), prevPoints(:,2),'r.');
% hold on
[ro, xs] = ray_in_out_pixel(prevPoints,d, 0);
 prevBearing = [ro xs];
point_num = size(imagePoints, 1);
view = struct('points',{}, 'label',{},'rot',{},'trans',{}, 'bearing_vector',{});
tracks = struct ('points',{}, 'views', {}, 'pointcloud',{});
view = addview(view, prevPoints, [], eye(3), ...
    zeros(3,1), prevBearing,base_view);

for i = 2:m 
    currPoints = imagePoints(:,:, views(i));
    matchPairs = matching_points(prevPoints, currPoints);
    plot(currPoints(matchPairs,1),currPoints(matchPairs,2),'g.');
    axis([0 1280 0 960]);
    [ro1, xs1] = ray_in_out_pixel(currPoints,d, 0);
    currBearing = [ro1 xs1];
    matchVector1 = prevBearing(matchPairs, :);
    matchVector2 = currBearing(matchPairs, :);
    testVector(:,:,1) = prevBearing(1:2,:);
    testVector(:,:,2) = currBearing(1:2,:);
   
     U=umatrix_generator_general(matchVector1, matchVector2);
     [R_est,t_est]=R_t_estimator_pixel(U, testVector);
 
     prevRot = view(i-1).rot;
     prevTrans = view(i-1).trans;
     rot = R_est* prevRot
    % trans = prevTrans + t_est'*prevRot 
    trans = R_est*prevTrans + t_est;
     view = addview(view, currPoints, [], rot, trans, currBearing,i);    
    
     %xyzPoints = triangulate(view(j-1).bearing_vector, view(j).bearing_vector,...
      %    rotateMatrix(:,:,j-1), TransMatrix(j-1,:)'); test
     
     xyzPoints = triangulate(matchVector1, matchVector2,...
          R_est, t_est); 
      scatter3(xyzPoints(:,1), xyzPoints(:,2), xyzPoints(:,3));
    tracks = update_tracks(tracks, matchPairs, i, xyzPoints);
    tracks_cell = struct2cell(tracks);
    nxyzPoints = reshape(cell2mat(tracks_cell(3,:,:)), 3, []);
    nxyzPoints = nxyzPoints';
     scatter3(nxyzPoints(:,1), nxyzPoints(:,2), nxyzPoints(:,3));
    valid_num = size(tracks, 1);
    out = optim_point(xyzPoints, view, i, tracks);
    xw_est = reshape(out(1:3*point_num),[point_num, 3]);
    R_opm = reshape(out(3*point_num+1:3*point_num + 9*(i-1)),[3,3, i-1]);
    t_opm = reshape(out(3*point_num +9*(i-1)+ 1:end), [1 3 i-1]);
    view = updateView(view, rot, trans, i);
    prevBearing = currBearing;
    % scatter3(xw_est(:,1), xw_est(:,2), xw_est(:,3));
end

end

function xw = triangulate(vec1, vec2, R_est, t_est)
    r_out_w1 = vec1(:, 1:3)*R_est';
    xs_w1 = vec1(:, 4:6)*R_est' + t_est';
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

