function bearingVec = addBearingVec(tracks)
numTracks = numel(pointTracks);
gg = [ -0.72005  2.06590  42.66089  -0.28110  -1.39643 -1.96133 ];
K =[590.2313 0 0; 0 559.4365 0; 369.2098 272.4348 1];
c = [1 1.49 1];
Ra = 50;
ra = 46;
bearingVec = cell(1,numTracks);
for i = 1:numTracks
    bearingVec(1,i).ViewIds = tracks(1,i).ViewIds;
    points = tracks(1,i).Points;
    [xs1, ro1] = sfm_one_view_Rt(gg, points, K, c, Ra, ra);
    bearingVec(1,i).BearingVector = [xs1 ro1];
end