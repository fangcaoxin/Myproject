function image_point = ray_projection(point3d)
load parameter.mat
c=fermat(point3d,n1,n2,n3,R,r,d,0);
point_at_glass_water = [c(1) c(2) sqrt(R*R-c(2)*c(2))];
point_at_glass_air =[c(3) c(4) sqrt(r*r -c(4)*c(4))];
r_in_true = [point_at_glass_air(1) point_at_glass_air(2) point_at_glass_air(3)-d];
r_in_true = r_in_true/norm(r_in_true);
r_out_true = point3d - point_at_glass_water;
r_out_true= r_out_true/norm(r_out_true);
t = (point_at_glass_air(3) - (focal + d))/r_in_true(3);
point_2d = [point_at_glass_air(1)-r_in_true(1)*t point_at_glass_air(2)-r_in_true(2)*t];
image_point = [point_2d(1)/sx+hcx hcy+point_2d(2)/sx];
end