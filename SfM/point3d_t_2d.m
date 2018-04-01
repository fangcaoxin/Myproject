function [image_points1, image_points2] = point3d_t_2d(points, R_1, t_1,R_2,t_2)
load parameter.mat
point_num = size(points,1);
image_points1 = zeros(point_num, 2);
image_points2 = zeros(point_num, 2);
r1_in_true = zeros(point_num,3);
r1_out_true = zeros(point_num,3);
r2_in_true = zeros(point_num,3);
r2_out_true = zeros(point_num,3);
for i = 1: point_num
    point_camera_1 = R_1*(points(i,:)'-t_1);
    point_camera_2 = R_2*(points(i,:)'-t_2);
%     point_2d =[focal*point_camera(1)/point_camera(3) focal*point_camera(2)/point_camera(3)];
   % image_p_no_refractive = [point_2d(1)/sx+hcx hcy-point_2d(2)/sx];
    %image_points(i,:) = round(image_p_no_refractive);
    [r1_in_true(i,:),image_point_1,r1_out_true(i,:)] = ray_true(point_camera_1);
    image_points1(i,:) = round(image_point_1);
    [r2_in_true(i,:),image_point_2,r2_out_true(i,:)] = ray_true(point_camera_2);
    image_points2(i,:) = round(image_point_2);
end
save ray1_true.mat r1_in_true r1_out_true;
save ray2_true.mat r2_in_true r2_out_true;

