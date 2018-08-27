%%simulate SfM
%% load 3D points
load('teapot.mat');
teapot(:,3) = teapot(:,3) + 300; % Z>300
%% transform 3D points to another view(R,t) external matrix
load camera_motion.mat
teapot2 = teapot*Rotate' + translation';
%% intrinsic matrix
K = [1000 0 640; 0 1000 480; 0 0 1];
teapot_c1 = teapot*K';
teapot_c1_norm = teapot_c1(:,1:2)./teapot_c1(:,3);
plot(teapot_c1_norm(:,1), teapot_c1_norm(:,2),'.');
