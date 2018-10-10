% Use |imageDatastore| to get a list of all image file names in a
% directory.
refractive = 0;
imageDir = fullfile('real_images_2/dehazing');
imds = imageDatastore(imageDir);
IntrinsicMatrix = [590.2313 0 0; 0 559.4365 0; 369.2098 272.4348 1];
radialDistortion = [-0.4084 0.1707];
cameraParams = cameraParameters('IntrinsicMatrix',IntrinsicMatrix, 'RadialDistortion',radialDistortion);
bearingVec = struct('ViewIds',{},'BearingVector',{});
% Display the images.
% figure
% montage(imds.Files, 'Size', [3, 2]);

% Convert the images to grayscale.
images = cell(1, numel(imds.Files));
for i = 1:numel(imds.Files)
    I = readimage(imds, i);
    images{i} = rgb2gray(I);
end
I = undistortImage(images{1}, cameraParams);
border = 5;
roi = [border, border, size(I, 2)- 2*border, size(I, 1)- 2*border];
prevPoints   = detectSURFFeatures(I,  'ROI', roi);
prevFeatures = extractFeatures(I, prevPoints);
vSet = viewSet;
viewId = 1;
vSet = addView(vSet, viewId, 'Points', prevPoints, 'Orientation', ...
    eye(3, 'like', prevPoints.Location), 'Location', ...
    zeros(1, 3, 'like', prevPoints.Location));
%for i = 2:numel(images)
for i = 2:numel(imds.Files)
    % Undistort the current image.
     I = undistortImage(images{i}, cameraParams);
%    I = images{i};
    % Detect, extract and match features.
    currPoints   = detectSURFFeatures(I, 'ROI', roi);
    currFeatures = extractFeatures(I, currPoints);
    indexPairs = matchFeatures(prevFeatures, currFeatures, ...
        'MaxRatio', .7, 'Unique',  true);

    % Select matched points.
    matchedPoints1 = prevPoints(indexPairs(:, 1));
    matchedPoints2 = currPoints(indexPairs(:, 2));
    figure; ax = axes;
    showMatchedFeatures(images{i-1}, images{i}, matchedPoints1, matchedPoints2,...
        'montage', 'Parent', ax);
    % Estimate the camera pose of current view relative to the previous view.
    % The pose is computed up to scale, meaning that the distance between
    % the cameras in the previous view and the current view is set to 1.
    % This will be corrected by the bundle adjustment.
    if(refractive==0)
     [relativeOrient, relativeLoc, inlierIdx] = helperEstimateRelativePose(...
         matchedPoints1, matchedPoints2, cameraParams);
    else
    [relativeOrient, relativeLoc, inlierIdx] = refractiveEstimateRelativePose(...
      matchedPoints1, matchedPoints2, cameraParams);
    end
    % Add the current view to the view set.
    vSet = addView(vSet, i, 'Points', currPoints);

    % Store the point matches between the previous and the current views.
    vSet = addConnection(vSet, i-1, i, 'Matches', indexPairs(inlierIdx,:));

    % Get the table containing the previous camera pose.
    prevPose = poses(vSet, i-1);
    prevOrientation = prevPose.Orientation{1};
    prevLocation    = prevPose.Location{1};

    % Compute the current camera pose in the global coordinate system
    % relative to the first view.
    orientation = relativeOrient * prevOrientation;
    location    = prevLocation + relativeLoc * prevOrientation;
    vSet = updateView(vSet, i, 'Orientation', orientation, ...
        'Location', location);

    % Find point tracks across all views.
    tracks = findTracks(vSet);
    if(refractive~=0)
      bearingVec = addBearingVec(tracks);
    end
    % Get the table containing camera poses for all views.
    camPoses = poses(vSet);

    % Triangulate initial locations for the 3-D world points.
    if(refractive == 0)
         xyzPoints = triangulateMultiview(tracks, camPoses, cameraParams);
    else
         xyzPoints = refractivetriangulateMultiview(bearingVec, camPoses, cameraParams);
    end

    % Refine the 3-D world points and camera poses.
    if(refractive == 0)
       [xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints, ...
        tracks, camPoses, cameraParams, 'FixedViewId', 1, ...
        'PointsUndistorted', true);
    else
        [xyzPoints, camPoses] = refractiveBA(xyzPoints, bearingVec);
    end

    % Store the refined camera poses.
    vSet = updateView(vSet, camPoses);

    prevFeatures = currFeatures;
    prevPoints   = currPoints;
end

% Display camera poses.
camPoses = poses(vSet);
figure;
plotCamera(camPoses, 'Size', 0.2);
hold on

% Exclude noisy 3-D points.
% goodIdx = (reprojectionErrors < 5);
% xyzPoints = xyzPoints(goodIdx, :);

% Display the 3-D points.
pcshow(xyzPoints, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 45);
grid on
hold off

% Specify the viewing volume.
loc1 = camPoses.Location{1};
xlim([loc1(1)-5, loc1(1)+4]);
ylim([loc1(2)-5, loc1(2)+4]);
zlim([loc1(3)-1, loc1(3)+20]);
camorbit(0, -30);

title('Refined Camera Poses');