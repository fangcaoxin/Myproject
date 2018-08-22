function [Rotate,translation] = camera_motion_generator
alpha = 0.056*pi; %0.075
beta = 0.056*pi; %0.075
gama = 0.056*pi; % -0.06
R1 = [cos(gama) sin(gama) 0; -sin(gama) cos(gama) 0; 0 0 1];
R2 = [cos(beta) 0 -sin(beta);0 1 0; sin(beta) 0 cos(beta)];
R3 = [1 0 0; 0 cos(alpha) sin(alpha); 0 -sin(alpha) cos(alpha)];
Rotate = R1*R2*R3;
translation = [100;-100;100];
save camera_motion.mat Rotate translation;