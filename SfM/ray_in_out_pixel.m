function [r_in,r_out, dis] = ray_in_out_pixel(uv)
load parameter.mat;
u = uv(1) - hcx;
v = uv(2) - hcy;
s = (sqrt(fx*fx*d_pixel*d_pixel-(v*v + fx*fx)*(d_pixel*d_pixel-r_pixel*r_pixel))-fx*d_pixel)/(v*v+fx*fx);
p1_pixel = [s*u, s*v, s*fx+d_pixel]; % point of air and  glass
p1 = p1_pixel*sx;
r_in=[u, v*fx/fy, fx];
ux=fx*r_in(1)/r_in(3)+hcx;
uy=fy*r_in(2)/r_in(3)+hcy;
r_in = r_in/norm(r_in);
n = [0 p1(2) p1(3)]; % normal between air and glass
n_norm = n/norm(n);
s1 = norm(cross(r_in, n_norm)); % sin(theta_1)
c1 =norm( r_in.*n_norm); % cos(theta_1)
% theta_1 =asin(cross(r_in, n_norm));
% theta_2 =asin(n1/n2*sin(theta_1));

r_glass = n1*r_in/n2 - (n1*c1/n2-sqrt(1- n1*n1/(n2*n2)*s1*s1))*n_norm;
r_glass_norm = r_glass/norm(r_glass);
t =( -(p1(2)*r_glass_norm(2) +p1(3)*r_glass_norm(3)) + sqrt((p1(2)*r_glass_norm(2) +p1(3)*r_glass_norm(3))*(p1(2)*r_glass_norm(2) +p1(3)*r_glass_norm(3))...
      -(r_glass_norm(2)*r_glass_norm(2)+r_glass_norm(3)*r_glass_norm(3))*(p1(2)*p1(2) + p1(3)*p1(3)-R*R)))/(r_glass_norm(2)*r_glass_norm(2)+r_glass_norm(3)*r_glass_norm(3));
p2 = [p1(1) + t*r_glass_norm(1) p1(2) + t*r_glass_norm(2) p1(3)+ t*r_glass_norm(3)]; % point at glass and water
n1 = [0 p2(2) p2(3)]; % normal between glass and water
n1_norm = n1/norm(n1);
s2 = norm(cross(r_glass_norm,n1_norm));
c2 = norm(r_glass_norm.*n1_norm);
r_out = n2/n3*r_glass_norm - (n2/n3*c2-sqrt(1- n2*n2/(n3*n3)*s2*s2))*n1_norm;

%r_out = n1/n2*r_in - (n1/n2*cos(theta_1)-sqrt(1- n1*n1/(n2*n2)*sin(theta_1)*sin(theta_1)))*n_norm;
r_out = r_out/norm(r_out);


r_in_project_point = [- d_pixel*u/fx, -d_pixel*v/fx, 0]; % on base plane
n_norm_project_point = [s*u, 0, 0];
project_line = n_norm_project_point - r_in_project_point;
project_line = project_line/norm(project_line);
t2 = cross((n_norm_project_point - p1),r_in)/cross(r_in, project_line);
cross_point = n_norm_project_point + t2*project_line;
dis = cross_point*sx;


end