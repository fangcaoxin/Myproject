function [points3d, errors] = refractivetriangulateMultiview(bearingVec, ...
    camPoses, cameraParams)
%outputType = validateInputs(pointTracks, camPoses, cameraParams);

numTracks = numel(bearingVec);
points3d = zeros(numTracks, 3);

numCameras = size(camPoses, 1);
cameraMatrices = containers.Map('KeyType', 'uint32', 'ValueType', 'any');

for i = 1:numCameras
    id = camPoses{i, 'ViewId'};
    R  = camPoses{i, 'Orientation'}{1};
    t  = camPoses{i, 'Location'}{1};
    cameraMatrices(id) = cameraMatrix(cameraParams, R', -t*R');
end
for i = 1:numTracks
    track = bearingVec(i);
    points3d(i, :) = triangulateOnePoint(track, cameraMatrices);    
end
%points3d = cast(points3d, outputType);
end

function point3d = triangulateOnePoint(track, cameraMatrices)

viewIds = track.ViewIds;
%points  = track.Points';
points = track.BearingVector(:,1:3)';
numViews = numel(viewIds);
A = zeros(numViews * 2, 4);

for i = 1:numViews
    % Check if the viewId exists
    if ~isKey(cameraMatrices, viewIds(i))
        error(message('vision:absolutePoses:missingViewId', viewIds(i)));
    end
   
    P = cameraMatrices(viewIds(i))';
    idx = 2 * i;
    A(idx-1:idx,:) = points(1:2, i) * P(3,:) -points(3,i)* P(1:2,:);
end

[~,~,V] = svd(A);
X = V(:, end);
X = X/X(end);

point3d = X(1:3)';   
end