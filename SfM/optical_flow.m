load points.mat
R = [1 0 0; 0 1 0; 0 0 1];
t = [0; 0;0];
R1 = [ 0.99940   0.03453   0.00119;
  -0.03455   0.99881   0.03453;
   0.00000  -0.03455   0.99940]; % 2 0 2 degree
t1 = [5; 3;4];
image_points1 = point3d_t_2d_normal(points, R, t);
image_points2 = point3d_t_2d_normal(points, R1, t1);
optical_flow = image_points2 - image_points1;
%flow_choose = choose_flow(optical_flow);
[v, omega] = velocity_calc(optical_flow, image_points1);
