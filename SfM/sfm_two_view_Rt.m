function xw_est = sfm_two_view_Rt()
load imagePoints.mat
load worldPoints.mat
load historyn5.mat
in = [1 2 3 5 9];
view = [3 4]; % good result
x_best = historyn5.x(end,:);
gg = [x_best(end-5:end) ];
x1 = imagePoints(:,:,in(view(1)));
x2 = imagePoints(:,:,in(view(2)));
K =[590.2313 0 0; 0 559.4365 0; 369.2098 272.4348 1];
c = [1 1.49 1];
Ra = 50;
ra = 46;
[xs1, ro1] = sfm_one_view_Rt(gg, x1, K, c, Ra, ra);
[xs2, ro2] = sfm_one_view_Rt(gg, x2, K, c, Ra, ra);
vec1 = [ro1 xs1];
vec2 = [ro2 xs2];
U=umatrix_generator_general(vec1, vec2);
[R_est,t_est]=R_t_estimator_pixel(U);
R_est
t_est
end
function [loc, ori] = camera_pose(gg)
r1 = [gg(1) gg(2) gg(3)];
r2 = [gg(4) gg(5) gg(6)];
r3 = cross(r1,r2);
Rot = [r1;r2;r3];
ts = [gg(7); gg(8);gg(9)];
ori = Rot';
loc = -Rot'*ts;
end
