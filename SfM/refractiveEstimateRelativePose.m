function [relativeOrient, relativeLoc,inlierIdx] = refractiveEstimateRelativePose(matchedPoints1, matchedPoints2, cameraParams)
if ~isnumeric(matchedPoints1)
    matchedPoints1 = matchedPoints1.Location;
end

if ~isnumeric(matchedPoints2)
    matchedPoints2 = matchedPoints2.Location;
end
gg = [ -0.72005  2.06590  42.66089  -0.28110  -1.39643 -1.96133 ];
c = [1 1.49 1];
Ra = 50;
ra = 46;
K = cameraParams.IntrinsicMatrix;
[xs, ro] = sfm_one_view_Rt(gg, matchedPoints1, K, c, Ra, ra);
[xs1, ro1] = sfm_one_view_Rt(gg, matchedPoints2, K, c, Ra, ra);
prevBearing = [ro xs];
currBearing = [xs1 ro1];
U=umatrix_generator_general(prevBearing, currBearing);
[relativeOrient,relativeLoc]=R_t_estimator_pixel(U);
inlierIdx = ones(size(matchedPoints1,1),1);
end