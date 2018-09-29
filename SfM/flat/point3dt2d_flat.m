function [image_points] = point3dt2d_flat(points)
load parameter.mat
point_num = size(points,1);
image_points = zeros(point_num, 2);

for i = 1: point_num
    i
    image_point = projection_flat(points(i,:));
    image_points(i,:) = image_point;   
end
end