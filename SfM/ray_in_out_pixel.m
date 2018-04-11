function [r_in,r_out, dis] = ray_in_out_pixel(uv)
load parameter.mat;
u = uv(1) - hcx;
v = hcy - uv(2);
%s = (sqrt(fx*fx*d_pixel*d_pixel-(v*v + fx*fx)*(d_pixel*d_pixel-r_pixel*r_pixel))-fx*d_pixel)/(v*v+fx*fx);
%p1_pixel = [s*u, s*v, s*fx+d_pixel]; % point of air and  glass
%p1 = p1_pixel*sx;
r_in=[u, v*fx/fy, fx];
% ux=fx*r_in(1)/r_in(3)+hcx;
% uy=fy*r_in(2)/r_in(3)+hcy;
r_in = r_in/norm(r_in);
t_0 = (-r_in(3)*d + sqrt(r_in(3)*r_in(3)*d*d-(r_in(2)*r_in(2)+r_in(3)*r_in(3))*(d*d-r*r)))/(r_in(2)*r_in(2)+r_in(3)*r_in(3));
if(d+r_in(3)*t_0 < 0)
    t_0 = (-r_in(3)*d - sqrt(r_in(3)*r_in(3)*d*d-(r_in(2)*r_in(2)+r_in(3)*r_in(3))*(d*d-r*r)))/(r_in(2)*r_in(2)+r_in(3)*r_in(3));
end
p1 = [r_in(1)*t_0 r_in(2)*t_0 r_in(3)*t_0+d]; % point at glass and air
N = [0 p1(2) p1(3)]; % normal between air and glass
N_norm = N/norm(N);
s1 = norm(cross(r_in, N_norm)); % sin(theta_1)
c1 =norm( r_in.*N_norm); % cos(theta_1)
% theta_1 =asin(cross(r_in, n_norm));
% theta_2 =asin(n1/n2*sin(theta_1));

r_glass = n1*r_in/n2 - (n1*c1/n2-sqrt(1- n1*n1/(n2*n2)*s1*s1))*N_norm;
r_glass_norm = r_glass/norm(r_glass);
t_1 =( -(p1(2)*r_glass_norm(2) +p1(3)*r_glass_norm(3)) + sqrt((p1(2)*r_glass_norm(2) +p1(3)*r_glass_norm(3))*(p1(2)*r_glass_norm(2) +p1(3)*r_glass_norm(3))...
      -(r_glass_norm(2)*r_glass_norm(2)+r_glass_norm(3)*r_glass_norm(3))*(p1(2)*p1(2) + p1(3)*p1(3)-R*R)))/(r_glass_norm(2)*r_glass_norm(2)+r_glass_norm(3)*r_glass_norm(3));
if(p1(3)+ t_1*r_glass_norm(3)<0)
     t_1 =( -(p1(2)*r_glass_norm(2) +p1(3)*r_glass_norm(3)) - sqrt((p1(2)*r_glass_norm(2) +p1(3)*r_glass_norm(3))*(p1(2)*r_glass_norm(2) +p1(3)*r_glass_norm(3))...
      -(r_glass_norm(2)*r_glass_norm(2)+r_glass_norm(3)*r_glass_norm(3))*(p1(2)*p1(2) + p1(3)*p1(3)-R*R)))/(r_glass_norm(2)*r_glass_norm(2)+r_glass_norm(3)*r_glass_norm(3));
end
p2 = [p1(1)+t_1*r_glass_norm(1) p1(2)+t_1*r_glass_norm(2) p1(3)+t_1*r_glass_norm(3)]; % point at glass and water
N1 = [0 p2(2) p2(3)]; % normal between glass and water
N1_norm = N1/norm(N1);
s2 = norm(cross(r_glass_norm,N1_norm));
c2 = norm(r_glass_norm.*N1_norm);
r_out = n2/n3*r_glass_norm - (n2/n3*c2-sqrt(1- n2*n2/(n3*n3)*s2*s2))*N1_norm;

%r_out = n1/n2*r_in - (n1/n2*cos(theta_1)-sqrt(1- n1*n1/(n2*n2)*sin(theta_1)*sin(theta_1)))*n_norm;
r_out = r_out/norm(r_out);

% t = d_pixel/r_in(3);
% r_in_project_point = [-t*r_in(1), -t*r_in(1), 0]; % on base plane
% n_norm_project_point = [s*u, 0, 0];
% project_line = n_norm_project_point - r_in_project_point;
% project_line = project_line/norm(project_line);
%t2 = cross((n_norm_project_point - p1),r_in)/cross(r_in, project_line);
t2 = p2(1)/r_out(1);

%cross_point = n_norm_project_point + t2*project_line;
%cross_point = [0 p2(2)-t2*r_out(2) p2(3)-t2*r_out(3)];
dis = p2;
%dis = cross_point;

end