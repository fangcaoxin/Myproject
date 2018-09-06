function matchedPairs = matching_points(prevPoints, currPoints)
   preValidx = find(prevPoints(:,1));
   preValidy = find(prevPoints(:,2));
   preValid = intersect(preValidx, preValidy);
   
   curValidx = find(currPoints(:,1));
   curValidy = find(currPoints(:,2));
   curValid = intersect(curValidx, curValidy);
   matchedPairs = intersect(preValid, curValid);   
end