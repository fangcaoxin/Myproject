%% error caculate
load imagePoints.mat
load worldPoints.mat
load parameter.mat
load Rotation_Matrix.mat
load Translation.mat
N = 5; % image num
in = [1 2 3 5 9]; % image no.
K =[590.2313 0 0; 0 559.4365 0; 369.2098 272.4348 1];
c = [1 1.49 1];
Ra = 50;
ra = 46;
Rot = zeros(3,3,N);
trans = zeros(1,3,N);
init = zeros(1,6*N+3*N+6);
lb = zeros(1,6*N+3*N+6);
ub = zeros(1,6*N+3*N+6);
x = zeros(size(imagePoints,1),size(imagePoints,2),N);
for i = 1:N
    Rot(:,:,i) = Rotation_Matrix(:,:,in(i));
    trans(:,:,i) = Translation(:,:,in(i));
    x(:,:,i) =imagePoints(:,:,in(i));
end
hp = [0 0 30 0 0 0];
% Rt_c = [-3+rand*6 -3+rand*6 25 -2+rand*4 -2+rand*4 -2+rand*4];
% Rt_c = [0 0 41 0 0 0];
 Rt_c = 41;
 for i = 1:N
     init(9*i-8:9*i) = [Rot(1,1,i) Rot(1,2,i) Rot(1,3,i) ...
        Rot(2,1,i) Rot(2,2,i) Rot(2,3,i) ...
        trans(1,1,i) trans(1,2,i) trans(1,3,i)];
    lb(9*i-8:9*i) = [-1 -1 -1 -1 -1 -1 -500 -500 0 ];
    ub(9*i-8:9*i) = [1 1 1 1 1 1 500 500 800 ];
 end
lb(9*N+1:9*N+6) = [-3 -3 0 -2 -2 -2];
ub(9*N+1:9*N+6) = [3 3 43 2 2 2];
init(9*N+1:9*N+6) =  hp;
x_w = worldPoints;
% res =  error_min_1(init, x,x_w, K, c, Ra, ra);
res = error_min_2(init, x, x_w, K, c, Ra, ra, lb, ub, N);
% save history.mat history