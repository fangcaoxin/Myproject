function [v, omega] = velocity_calc(optical_flow, image_points)
load points.mat
load parameter.mat
flow_num = size(optical_flow,1);
C = zeros(2*flow_num, 6);
d = zeros(2*flow_num, 1);
for i = 1 : flow_num
   x = image_points(i,1);
   y = image_points(i,2);
   Z = points(i,3);
   ux = optical_flow(i,1);
   uy = optical_flow(i,2);
   C(2*i -1, :) = [-focal/Z, 0,x/Z, x*y/focal, -(focal+x*x/focal),y];
   C(2*i,:) = [0, -focal/Z, y/Z, (focal+y*y/focal),-x*y, -x];
   d(2*i -1) = ux;
   d(2*i) = uy;
end
 A = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0; 0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1];
 b = [5 5 5 0.1 0.1 0.1];
 x = lsqlin(C,d,A,b);
 v = x(1:3);
 omega = x(4:6);