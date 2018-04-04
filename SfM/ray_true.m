% caculate the ray from 3D point to 2D point
% this 3D point is in camera coordinate
function [r_in_true,image_point,r_out_true] = ray_true(point,type)
	load parameter.mat
  if(type == 1)
    c = fermat_thin(point,n1,n3,R,d);
    point_at_glass_water = [c(1) c(2) sqrt(R*R-c(2)*c(2))-d];
    r_in_true = [point_at_glass_water(1) point_at_glass_water(2) point_at_glass_water(3)];
    r_in_true = r_in_true/norm(r_in_true);
    r_out_true = [point(1)-point_at_glass_water(1) point(2)-point_at_glass_water(2) point(3)-point_at_glass_water(3)];
    r_out_true = r_out_true/norm(r_out_true);
    t = (point_at_glass_water(3)-focal)/(r_in_true(3));
    point_2d = [point_at_glass_water(1)-r_in_true(1)*t point_at_glass_water(2)-r_in_true(2)*t];
    image_point = [point_2d(1)/sx+hcx hcy-point_2d(2)/sx];
  else
	  c=fermat(point,n1,n2,n3,R,r,d);
    point_at_glass_water = [c(1) c(2) sqrt(R*R-c(2)*c(2))];
    point_at_glass_air =[c(3) c(4) sqrt(r*r -c(4)*c(4))];
    r_in_true = [point_at_glass_air(1) point_at_glass_air(2) point_at_glass_air(3)-d];
    r_in_true = r_in_true/norm(r_in_true);
    r_out_true = [point_at_glass_water(1) - point(1) point_at_glass_water(2)-point(2) point_at_glass_water(3) - point(3)];
    r_out_true=-r_out_true/norm(r_out_true);
    t = (point_at_glass_air(3) - (focal + d))/r_in_true(3);
    point_2d = [point_at_glass_air(1)-r_in_true(1)*t point_at_glass_air(2)-r_in_true(2)*t];
    image_point = [point_2d(1)/sx+hcx hcy-point_2d(2)/sx];
    end
	end