function [xyzPoints, camPoses, reprojectionErrors] = refractivebundleAdjustment(...
    xyzPoints, i, tracks, camPoses, cameraParams)
    out = optim_point(xyzPoints, view, i, n, matchedPairs);
end
