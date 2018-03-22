function [distance,max_error,heikin]=error_calculator

	load co_points.mat;
	load points_est.mat;

	error=co_points-points_est;	
	distance=[];
	for i=1:size(co_points,1)
		distance=[distance;norm(error(i,:))];
    end
[max_error,max_error_index]=max(distance)
	distance
	heikin=0;
	for i=1:size(co_points,1)
		heikin=heikin+distance(i);
    end
heikin=heikin/size(co_points,1)
	
end
    


	