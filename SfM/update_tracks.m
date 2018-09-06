function tracks = update_tracks(tracks, matchedPairs, view, xyzPoints)
  new_num = size(matchedPairs, 1); % new num
 if(exist('tracks.points'))
    exist_num = size(tracks, 2); % exist num
    tracks_cell = struct2cell(tracks);
    tracks_point_mat = reshape(cell2mat(tracks_cell(1,:,:)), 1, []);
    curr_num = exist_num;
    for i = 1: new_num
      res = find(tracks_point_mat == matchedPairs(i,1));
      if(res)
          tracks(res).views = [tracks(i).views view];
          tracks(res).pointcloud = tracks(res).pointcloud...
              /(size(tracks(res).views, 2) -1);
      else
          curr_num = curr_num + 1;
          tracks(curr_num).points = matchedPairs(i,1);
          tracks(curr_num).views = view;
          tracks(curr_num).pointcloud = xyzPoints(i, :);
      end
    end
 else
    for i = 1:new_num
      tracks(i).points = matchedPairs(i,1);
      tracks(i).views = [1 2];
      tracks(i).pointcloud = xyzPoints(i,:);
 end
end