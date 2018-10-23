% Use |imageDatastore| to get a list of all image file names in a
% directory.
clear;clc;
addpath('../common');
load corres.mat
IntrinsicMatrix = [2881.84239103060,0,0;0,2890.20944782907,0;2073.63152572517,1398.01946105023,1];
radialDistortion = [0.0424498292149281,-0.0489981810340664];
%cameraParams = cameraParameters('IntrinsicMatrix',IntrinsicMatrix, 'RadialDistortion',radialDistortion);
bearingVec = struct('ViewIds',{},'BearingVector',{});

order = [9 8 7 6 5];

% fileID_corres = fopen('res.txt', 'r');
% formatSpec = '%f %f';
% sizeA = [2 Inf];
% corresMat = fscanf(fileID_corres, formatSpec,sizeA);
% fclose(fileID_corres);
% corres = reshape(corresMat,2,60,9);
% save corres.mat corres
% Estimate the camera pose
%gg = [89.28,-2.18342172302219,2.76380543162081,-1.05];
gg = [89.28, 0, 0, 0];
c = [1.0 1.49 1.333];
w = 5.11;
K = IntrinsicMatrix;
order = [9 8 7 6 5];
order1 = [ 1 4 6 8];
%K = cameraParams.IntrinsicMatrix;
view = struct('points',{}, 'label',{},'rot',{},'trans',{}, 'bearing_vector',{});
tracks = struct ('points',{}, 'views', {}, 'pointcloud',{});
basePoints = corres(:,:,order(1))';

[xs_t1, ro_t1, ri_t1] = sfm_one_view(gg, basePoints, K, c,w);
prevBearing =  [ro_t1 xs_t1 ri_t1];
[temp1,temp] = find(basePoints(:,1)>0);
prevlabel = temp1;
view = addview(view, basePoints,prevlabel, eye(3), zeros(1,3),prevBearing, 1 );
for i = 2:size(order,2)
    currPoints = corres(:,:,order(i))';
    [currlabel,temp] = find(currPoints(:,1)>0);
    matchedPairs = intersect(prevlabel, currlabel);
    [xs_t2, ro_t2, ri_t2] = sfm_one_view(gg, currPoints, K, c, w);
    currBearing = [ro_t2 xs_t2 ri_t2];
     matchVector1 = prevBearing(matchedPairs, 1:6);
    matchVector2 = currBearing(matchedPairs, 1:6);
   testVector1 = matchVector1(1:2,1:6);
    testVector2 = matchVector2(1:2,1:6);

   U=umatrix_generator_general(matchVector1, matchVector2);
   [relativeOrient,relativeLoc]=R_t_estimator_pixel(U, 1, 1);
    relativeOrient
    relativeLoc

    % triangulation
      prevRot = view(i-1).rot;
      prevTrans = view(i-1).trans;
      rot =  relativeOrient* prevRot 
      trans = prevTrans + relativeLoc*prevRot
      xw_test = triangulateR(testVector1, testVector2, prevRot, prevTrans, rot, trans);

      if(xw_test(:,3) < 0)
      [relativeOrient,relativeLoc]=R_t_estimator_pixel(U, 0, 1);
      end
      rot =  relativeOrient* prevRot 
      trans = prevTrans + relativeLoc*prevRot
% if(i >= 3)
%      tracks_cell = struct2cell(tracks);
%      xyzPoints = reshape(cell2mat(tracks_cell(3,:,:)), 3, []);
%      label = reshape(cell2mat(tracks_cell(1,:,:)), 1, []);
%      [exist_label, ia, ib] = intersect(label, currlabel);
%      d3_points = xyzPoints';
%      [rot, trans] = R_t_estimator_3d(currBearing(exist_label, :), d3_points(ia, :), 0);
%       rot
%       trans
% end
    points3D = triangulateR(matchVector1, matchVector2, prevRot,prevTrans, rot, trans); % in C1
scatter3(points3D(:,1), points3D(:,2), points3D(:,3), 10, 'MarkerFaceColor',[1 0 0],...
    'MarkerEdgeColor',[0.9,0.5,0.3]);

      view = addview(view, currPoints, currlabel, rot, ...
      trans, currBearing, i);
     basePoints = currPoints;
     prevlabel = currlabel;
     prevBearing = currBearing;
tracks = update_tracks(tracks, matchedPairs, i, points3D);
tracks_cell = struct2cell(tracks);
nxyzPoints = reshape(cell2mat(tracks_cell(3,:,:)), 3, []);
nxyzPoints = nxyzPoints'; % get pointcloud

 [xw_est, view] = optim_point(view, tracks, 1, 1, numel(tracks));
 scatter3(xw_est(:,1), xw_est(:,2), xw_est(:,3)); 
% Get the color of each reconstructed point
axis equal
%ptCloud = pointCloud(xw_est);

%ax= pcshow(ptCloud, 'MarkerSize', 50);
%axis(ax, 'equal');
%   axis([-1 -0.5 2.02 2.06 41.6 42]);
% pcwrite(ptCloud, 'structure_out', 'PLYFormat', 'binary');
% Rotate and zoom the plot
% camorbit(0, -30);
% camzoom(1.5);

% Label the axes


xlabel('x-axis');
ylabel('y-axis');
zlabel('z-axis')
end
[xw_est, view] = optim_point(view, tracks, K, 1, 1, numel(tracks));
scatter3(xw_est(:,1), xw_est(:,2), xw_est(:,3));
function vector_no_scale = vectorNoScale(x)
hcx = K(3,1);
hcy = K(3,2);
fx = K(1,1);
fy = K(2,2);

r_in = u_v./[fx fy 1];
r_in = r_in./sqrt(sum(r_in.*r_in,2)); % normalize
x_s = zeros(size(x, 1),3);
vector_no_scale = [r_in, x_s];
end
