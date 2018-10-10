function image_point = projection_flat(point3d)
load parameter.mat
c=fermat_flat(point3d,n1,n2,n3,w,d_flat,0);
point_at_glass_water = [c(1) c(2) d_flat+w];
point_at_glass_air =[c(3) c(4) d_flat];
r_in_true = [point_at_glass_air(1) point_at_glass_air(2) point_at_glass_air(3)];
r_in_true = r_in_true/norm(r_in_true);
t = (point_at_glass_air(3) -focal)/r_in_true(3);
point_2d = [point_at_glass_air(1)-r_in_true(1)*t point_at_glass_air(2)-r_in_true(2)*t];
image_point = [point_2d(1)/sx+hcx hcy+point_2d(2)/sx];
end