function [image_points1, image_points2] = point3d_t_2d(points, R_1, t_1,R_2,t_2,type)
load parameter.mat
point_num = size(points,1);
image_points1 = zeros(point_num, 2);
image_points2 = zeros(point_num, 2);
r1_in_true = zeros(point_num,3);
r1_out_true = zeros(point_num,3);
r2_in_true = zeros(point_num,3);
r2_out_true = zeros(point_num,3);
cross_point_1 = zeros(point_num,3);
cross_point_2 = zeros(point_num, 3);
for i = 1: point_num
    point_camera_1 = R_1*points(i,:)'+t_1;
    point_camera_2 = R_2*points(i,:)'+t_2;
   
    [r1_in_true(i,:),image_point_1,cross_point_1(i,:),r1_out_true(i,:)] = ray_true(point_camera_1,type);
    image_points1(i,:) = image_point_1;
    [r2_in_true(i,:),image_point_2,cross_point_2(i,:), r2_out_true(i,:)] = ray_true(point_camera_2,type);
    image_points2(i,:) = image_point_2;
   
end
save ray1_true.mat r1_in_true r1_out_true;
save ray2_true.mat r2_in_true r2_out_true;
save cross_point.mat cross_point_1 cross_point_2;
