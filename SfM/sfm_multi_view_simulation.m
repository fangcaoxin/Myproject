function [xw_est, view] = sfm_multi_view_simulation(imagePoints, views)
% calibration result
addpath('helpers');
addpath('common');
addpath('cylindrical');
load parameter.mat
load rotateMatrix.mat
load TransMatrix.mat

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
     rot =  prevRot* R_est 
     trans = prevTrans + prevRot*t_est;
     view = addview(view, currPoints, [], rot, trans, currBearing,i);    
    
     xyzPoints = triangulate(matchVector1, matchVector2,...
          prevRot, prevTrans, rot, trans); 
      scatter3(xyzPoints(:,1), xyzPoints(:,2), xyzPoints(:,3));
    tracks = update_tracks(tracks, matchPairs, i, xyzPoints);
    [xw_est, view] = optim_point(view, tracks);
   %tracks_cell = struct2cell(tracks);
   %xw_est = reshape(cell2mat(tracks_cell(3,:,:)), 3, []);
   %view = updateView(view, rot, trans, i);
   xw = xw_est;
    prevBearing = currBearing;
     scatter3(xw(:,1), xw(:,2), xw(:,3));
end

end



