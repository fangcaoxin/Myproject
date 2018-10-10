function make_matfile_parameter()
n1 = 1.0; %air
n2 = 1.49; % glass
n3 = 1.333; % water
focal = 3.7; % 3.7mm
sx = 0.0035;
sy = 0.0035;
fx = focal/sx;
fy = focal/sy;
hcx = 640;
hcy = 480;
R = 50; %
r = 46; % radius 50mm
d = 25; % distance between camera center and axis
r_pixel = r/sx;
d_pixel = d/sx;
w = 4;
d_flat = 25;


save parameter.mat n1 n2 n3 focal sx sy fx fy hcx hcy R r d r_pixel d_pixel w d_flat;