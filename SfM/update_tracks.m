function tracks = update_tracks(tracks, matchedPairs, view)
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
      else
          curr_num = curr_num + 1;
          tracks(curr_num).points = matchedPairs(i,1);
          tracks(curr_num).views = view;
      end
    end
 else
    for i = 1:new_num
      tracks(i).points = matchedPairs(i,1);
      tracks(i).views = [1 2];
 end
end