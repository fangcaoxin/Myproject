%% error caculate
load imagePoints.mat
load worldPoints.mat
load parameter.mat
load Rotation_Matrix.mat
load Translation.mat
K =[590.2313 0 0; 0 559.4365 0; 369.2098 272.4348 1];
c = [1 1.49 1];
Ra = 50;
ra = 46;
 Rot = Rotation_Matrix(:,:,1);
 trans = Translation(:,:,1);
%  Rot = [0.99 0.1 0; 0 0.98 0.2; 0 0.05 0.99];
%  trans = [5; 25; 483];
% Rt_c = [-3+rand*6 -3+rand*6 25 -2+rand*4 -2+rand*4 -2+rand*4];
Rt_c = [0 0 25 0 0 0];
%  Rt_c = 25;
init = [Rot(1,1) Rot(1,2) Rot(1,3) ...
        Rot(2,1) Rot(2,2) Rot(2,3) ...
        trans(1) trans(2) trans(3) Rt_c];
x = imagePoints(:,:,1);
x_w = worldPoints;
% res =  error_min_1(init, x,x_w, K, c, Ra, ra);
 res = error_min_2(init, x, x_w, K, c, Ra, ra);
save res.mat res