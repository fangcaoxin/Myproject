function [xw_est, view] = sfm_multi_view_simulation(imagePoints, views)
% calibration result
addpath('helpers');
addpath('common');
addpath('cylindrical');
load parameter.mat
load rotateMatrix1008.mat
load TransMatrix.mat
load teapot.mat
teapot_ground = teapot(1:10:end,:) + [0 0 600];
base_view = views(1);
m = size(views, 2);
rotateArray = zeros(3,3,11);
rotateArray(:,:,1)= eye(3);
rotateArray(:,:,2:end) = rotateMatrix;
transArray= zeros(11,3);
transArray(1,:) = zeros(1,3);
transArray(2:end,:)= TransMatrix;
prevPoints = imagePoints(:,:, base_view);
% plot(prevPoints(:,1), prevPoints(:,2),'r.');
% hold on
[ro, xs] = ray_in_out_pixel(prevPoints,d, 0);
 prevBearing = [ro xs];
view = struct('points',{}, 'label',{},'rot',{},'trans',{}, 'bearing_vector',{});
tracks = struct ('points',{}, 'views', {}, 'pointcloud',{});

view = addview(view, prevPoints, [], rotateArray(:,:,base_view), ...
    transArray(base_view, :)', prevBearing,1);

for i = 2:m 
    currPoints = imagePoints(:,:, views(i));
    matchPairs = matching_points(prevPoints, currPoints);
%     plot(currPoints(matchPairs,1),currPoints(matchPairs,2),'g.');
%     axis([0 1280 0 960]);
    [ro1, xs1] = ray_in_out_pixel(currPoints,d, 0);
    currBearing = [ro1 xs1];
    matchVector1 = prevBearing(matchPairs, :);
    matchVector2 = currBearing(matchPairs, :);
    testVector(:,:,1) = prevBearing(1:2,:);
    testVector(:,:,2) = currBearing(1:2,:);
     matchedNums = size(matchVector1, 1);
     matchedForRt = round(matchedNums/2);
     U=umatrix_generator_general(matchVector1, matchVector2);
     [R_est,t_est]=R_t_estimator_pixel(U, testVector,0);
 
%      prevRot = view(i-1).rot;
%      prevTrans = view(i-1).trans;
%      rot =  prevRot* R_est 
%      trans = prevTrans + prevRot*t_est;
rot = R_est;
trans = t_est;
     view = addview(view, currPoints, matchPairs, rot, trans, currBearing,i);    
    
     xyzPoints = triangulateR(matchVector1, matchVector2, rot, trans); 
     outlier = find(xyzPoints(:,3)>2000 );
     test_outlier = triangulateR(matchVector1(outlier,:), matchVector2(outlier, :), rot, trans); 
      dir = xyzPoints - matchVector1(:, 4:6);
      dir1 = dir./(sqrt(sum(dir.*dir, 2)));
      angle = cross(dir1, matchVector1(:,1:3), 2);
      angle_norm = sqrt(sum(angle.*angle, 2));
      small_angle = find(angle_norm < 1e-3);
      scatter3(xyzPoints(small_angle,1), xyzPoints(small_angle,2), xyzPoints(small_angle,3));
      hold on
      axis equal
     tracks = update_tracks(tracks, matchPairs, i, xyzPoints);
     num_tracks = numel(tracks);
[xw1_est, view] = optim_point(view, tracks,10, 1, num_tracks);

 
%    prevBearing = currBearing;
    
end
num_tracks = numel(tracks);
[xw1_est, view] = optim_point(view, tracks,10, 1, num_tracks);
%  scatter3(xw1_est(:,1), xw1_est(:,2), xw1_est(:,3), 5, 'MarkerFaceColor',[0 0 1], 'MarkerEdgeColor', [0 0 0.5]);
baseVector = view(1).bearing_vector;
tracksR = struct ('points',{}, 'views', {}, 'pointcloud',{});
for i = 2 :m
     currVector = view(i).bearing_vector;
     matched = view(i).label;
     xyzPointsR = triangulateR(baseVector(matched, :), currVector(matched, :), view(i).rot, view(i).trans);
     tracksR = update_tracks(tracksR, matched, i, xyzPointsR);    
end
tracksR_cell = struct2cell(tracksR);
xw_est = reshape(cell2mat(tracksR_cell(3,:,:)), 3, []); % get pointcloud
xw_est = xw_est';
xw_true = teapot_ground(matched, :);
error = norm(xw_est -xw_true)/size(matched, 1);
%    scatter3(nxyzPoints(:,1), nxyzPoints(:,2), nxyzPoints(:,3));
end



