function image_points = point3d_t_2d(points, R_input, t_input)
load parameter.mat
point_num = size(points,1);
image_points = zeros(point_num, 2);
r_in_true = zeros(point_num,3);
r_out_true = zeros(point_num,3);
for i = 1: point_num
    point_camera = R_input*(points(i,:)'-t_input);
    [r_in_true(i,:),image_point,r_out_true(i,:)] = ray_true(point_camera);
    image_points(i,:) = round(image_point);
end
save ray_true.mat r_in_true r_out_true;

