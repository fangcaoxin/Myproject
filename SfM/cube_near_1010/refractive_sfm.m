% Use |imageDatastore| to get a list of all image file names in a
% directory.

addpath('../common');
load corres.mat
IntrinsicMatrix = [2881.84239103060,0,0;0,2890.20944782907,0;2073.63152572517,1398.01946105023,1];
radialDistortion = [0.0424498292149281,-0.0489981810340664];
cameraParams = cameraParameters('IntrinsicMatrix',IntrinsicMatrix, 'RadialDistortion',radialDistortion);
bearingVec = struct('ViewIds',{},'BearingVector',{});

% fileID_corres = fopen('res.txt', 'r');
% formatSpec = '%f %f';
% sizeA = [2 Inf];
% corresMat = fscanf(fileID_corres, formatSpec,sizeA);
% fclose(fileID_corres);
% corres = reshape(corresMat,2,60,9);
% save corres.mat corres
% Estimate the camera pose
gg = [56.9670253618861,-2.18342172302219,2.76380543162081,-0.862991761238783];
c = [1.0 1.49 1.333];
w = 5.10;
K = cameraParams.IntrinsicMatrix;
view = struct('points',{}, 'label',{},'rot',{},'trans',{}, 'bearing_vector',{});
tracks = struct ('points',{}, 'views', {}, 'pointcloud',{});
basePoints = corres(:,:,1)';
[xs_t1, ro_t1] = sfm_one_view(gg, basePoints, K, c,w);
prevBearing =  [ro_t1 xs_t1];
[temp1,temp] = find(basePoints(:,1)>0);
prevlabel = temp1;
view = addview(view, basePoints,prevlabel, eye(3), zeros(1,3),prevBearing, 1 );
for i = 2:9
    currPoints = corres(:,:,i)';
    [currlabel,temp] = find(currPoints(:,1)>0);
    matchedPairs = intersect(prevlabel, currlabel);
    [xs_t2, ro_t2] = sfm_one_view(gg, currPoints, K, c, w);
    currBearing = [ro_t2 xs_t2];
     matchVector1 = prevBearing(matchedPairs, :);
    matchVector2 = currBearing(matchedPairs, :);
   testVector(:,:,1) = matchVector1(1:2,:);
    testVector(:,:,2) = matchVector2(1:2,:);
   U=umatrix_generator_general(matchVector1, matchVector2);
   [relativeOrient,relativeLoc]=R_t_estimator_pixel(U, testVector, 1);
    relativeOrient
    relativeLoc
   
    % triangulation
      prevRot = view(i-1).rot;
      prevTrans = view(i-1).trans;
      rot =  relativeOrient* prevRot; 
      trans = prevTrans + relativeLoc'*prevRot;

      view = addview(view, currPoints, currlabel, rot, ...
      trans, currBearing, i);
     basePoints = currPoints;
     prevlabel = currlabel;
     prevBearing = currBearing;
points3D = triangulateR(matchVector1, matchVector2, rot, trans');
tracks = update_tracks(tracks, matchedPairs, 2, points3D);
 [xw_est, view] = optim_point(view, tracks, 1, 1, numel(tracks));
% Get the color of each reconstructed point

ptCloud = pointCloud(xw_est);

pcshow(ptCloud, 'MarkerSize', 50);
%   axis([-1 -0.5 2.02 2.06 41.6 42]);
% pcwrite(ptCloud, 'structure_out', 'PLYFormat', 'binary');
% Rotate and zoom the plot
% camorbit(0, -30);
% camzoom(1.5);

% Label the axes
axis equal
xlabel('x-axis');
ylabel('y-axis');
zlabel('z-axis')
end

function vector_no_scale = vectorNoScale(x)
load parameter.mat
u_v = x - [hcx hcy];
u_v(:,3) = 1;
r_in = u_v./[fx fy 1];
r_in = r_in./sqrt(sum(r_in.*r_in,2)); % normalize
x_s = zeros(size(x, 1),3);
vector_no_scale = [r_in, x_s];
end
