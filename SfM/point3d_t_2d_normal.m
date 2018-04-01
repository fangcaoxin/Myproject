function image_points = point3d_t_2d_normal(points, R, t)
load parameter.mat
points_num = size(points, 1);
image_points = zeros(points_num, 2);
for i = 1 : points_num
    points1 = R*(points(i,:)'- t);
    point_u = focal*points1(1)/points(3);
    point_v = focal*points1(2)/points(3);
    image_points(i,1) = point_u + hcx;
    image_points(i,2) = hcy - point_v;
end