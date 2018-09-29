function matchedPairs = matching_points(prevPoints, currPoints)
   preValidx = find(prevPoints(:,1)>=0&prevPoints(:,1)<=1280);
   preValidy = find(prevPoints(:,2)>=0&prevPoints(:,2)<=960);
   preValid = intersect(preValidx, preValidy);
   
   curValidx = find(currPoints(:,1)>0&currPoints(:,1)<=1280);
   curValidy = find(currPoints(:,2)>0&currPoints(:,2)<=960);
   curValid = intersect(curValidx, curValidy);
   matchedPairs = intersect(preValid, curValid);   
end