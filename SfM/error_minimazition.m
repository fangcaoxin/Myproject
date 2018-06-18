%% error caculate
load imagePoints.mat
load worldPoints.mat
load parameter.mat
%load Rotation_Matrix.mat
%load Translation.mat
K =[590.2313 0 0; 0 559.4365 0; 369.2098 272.4348 1];
c = [1 1.49 1];
Ra = 50;
ra = 46;
%Rot = Rotation_Matrix(:,:,1);
%trans = Translation(:,:,1);
Rot = [1 0 0; 0 1 0; 0 0 1];
trans = [0; 0; 500];
init = [Rot(1,1) Rot(1,2) Rot(1,3) ...
        Rot(2,1) Rot(2,2) Rot(2,3) ...
        trans(1) trans(2) trans(3) 25];
x = imagePoints(:,:,1);
x_w = worldPoints(:,:,1);
res =  error_min(init, x,x_w, K, c, Ra, ra);