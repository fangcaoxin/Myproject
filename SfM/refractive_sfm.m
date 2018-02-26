% parameters %
alpha = pi/4;
beta = pi/4;
gama = pi/4;
Rot_gama = [cos(gama), sin(gama),0; -sin(gama), cos(gama),0; 0,0,1];
Rot_beta = [cos(beta),0, -sin(beta);0,1,0;sin(beta),0,cos(beta)];
Rot_alpha = [1,0,0; 0,cos(alpha),sin(alpha);0,-sin(alpha),cos(alpha)];
Rot = Rot_gama*Rot_beta*Rot_alpha;

image_width = 1280;
image_height = 960;
point_num = 100;
range = 1000;
% X(1,:) = 2*range*rand([1,point_num]) - range;
% X(2,:) = 1.5*range*rand([1,point_num]) -0.75*range;
% X(3,:) = range*rand([1,point_num])+500;
X(1,:) = 100;
X(2,:) = 50;
X(3,:) = 1000;
 phi = atan2(X(3,:),X(2,:)); % angle with y axis
 y_phi_array = sqrt(X(3,:).*X(3,:) + X(2,:).*X(2,:));
plot3(X(1,:),X(2,:),X(3,:),'o');
global n1 n2 n3 R r;
n1 = 1.000; % air
n2 = 1.49; % glass
n3 = 1.333; % water

pixel_length = 0.0035;
R = 50; % R 50mm
R_pixel= 50/pixel_length;
r = 46; % r 46mm
r_pixel= 46/pixel_length;
d = 21.4; % distance between optical center and axis 21.4mm
d_pixel = 40/pixel_length;
f= 3.7; % pixel 3.7mm
f_pixel = f/pixel_length;
axis_center = [0,0,0];
focal_center = [0,0,d-f];
d1 = d -f;
[c s_d]= solve_c(R,r,X,d1,f);

c_pixel = c/pixel_length;
% v = 30;
% u = 40;


u = f*c(:,1)/(r-d1)/pixel_length + image_width/2;
v = image_height/2 - y_phi_array.*tan(phi)/pixel_length; 
s = (sqrt(f*f*d1*d1-(v*v + f*f)*(d1*d1-r*r))-f*d1)/(v*v+f*f);
r_in = [s*u,s*v,s*f];
r_in_norm = r_in/norm(r_in);
N = [0,s*v,s*f+d1];
N_norm = N/norm(N);
theta_1 = atan(sqrt(u*u+v*v)/f);
r_glass = n1/n2*r_in_norm - (n1/n2*cos(theta_1)-sqrt(1- n1*n1/(n2*n2)*sin(theta_1)*sin(theta_1)))*N_norm;
r_glass_norm = norm(r_glass);
theta_2 = asin(n1*sin(theta_1)/n2);
r_out = n2/n3*r_glass_norm - (n2/n3*cos(theta_2)-sqrt(1- n2*n2/(n3*n3)*sin(theta_2)*sin(theta_2)))*N_norm;
theta_3 = asin(n2*sin(theta_2)/n3);
r_out_mov = [0,0, R- (R-r)*tan(theta_2)*tan(theta_3)-(r-d1)*tan(theta_1)*tan(theta_3)];



 

