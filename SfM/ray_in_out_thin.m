function [r_in,r_out, dis] = ray_in_out_thin(uv)
load parameter.mat;
u = uv(1) - hcx;
v = hcy - uv(2);
r_in=[u, v, fx];

r_in = r_in/norm(r_in);
t_0 = (-r_in(3)*d + sqrt(r_in(3)*r_in(3)*d*d-(r_in(2)*r_in(2)+r_in(3)*r_in(3))*(d*d-R*R)))/(r_in(2)*r_in(2)+r_in(3)*r_in(3));
if(d+r_in(3)*t_0 < 0)
    t_0 = (-r_in(3)*d - sqrt(r_in(3)*r_in(3)*d*d-(r_in(2)*r_in(2)+r_in(3)*r_in(3))*(d*d-R*R)))/(r_in(2)*r_in(2)+r_in(3)*r_in(3));
end
p1 = [r_in(1)*t_0 r_in(2)*t_0 r_in(3)*t_0]; % point at glass and air
N = [0 p1(2) p1(3)]; % normal between air and glass
N_norm = N/norm(N);
s1 = norm(cross(r_in, N_norm)); % sin(theta_1)
c1 =norm( r_in.*N_norm); % cos(theta_1)

r_out = n1*r_in/n3 - (n1*c1/n3-sqrt(1- n1*n1/(n3*n3)*s1*s1))*N_norm;
r_out = r_out/norm(r_out);

cross_point = p1;
%dis = p2;
dis = cross_point;

end