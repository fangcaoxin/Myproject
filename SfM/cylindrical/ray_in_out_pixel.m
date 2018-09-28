function [r_out_norm, x_s] = ray_in_out_pixel(x,d, cali)
load parameter.mat;
if cali==1
    n3 = n1;
end
tc = [0; 0; d];
u_v = x - [hcx hcy];
u_v(:,3) = 1;
r_in = u_v./[fx fy 1];
r_in = r_in./sqrt(sum(r_in.*r_in,2)); % normalize
t_0 = (-(r_in(:,3)*tc(3) + r_in(:,2)*tc(2)) + sqrt((r_in(:,3).*tc(3) + r_in(:,2).*tc(2)).^2 -(r_in(:,2).*r_in(:,2)+r_in(:,3).*...
    r_in(:,3))*(tc(2)*tc(2)+ tc(3)*tc(3)-r*r)))./(r_in(:,2).*r_in(:,2)+r_in(:,3).*r_in(:,3));
p1 = r_in.* t_0 + tc'; % point at glass and air
tmp = [0 1 1];
coeff = repmat(tmp, size(p1,1), 1);
N = p1.* coeff; % normal between air and glass
N_norm = N./sqrt(sum(N.*N, 2));
s1 = cross(r_in, N_norm, 2); % sin(theta_1)
s1_norm = sqrt(sum(s1.*s1,2));
c1_norm = dot(r_in, N_norm,2); % cos(theta_1)
r_glass = n1*r_in/n2 - (n1*c1_norm./n2-sqrt(1- n1*n1/(n2*n2)*s1_norm.*s1_norm)).*N_norm;
r_glass_norm = r_glass./sqrt(sum(r_glass.*r_glass, 2));
t_1 =( -(p1(:,2).*r_glass_norm(:,2) +p1(:,3).*r_glass_norm(:,3)) + sqrt((p1(:,2).*r_glass_norm(:,2) +p1(:,3).*r_glass_norm(:,3)).*(p1(:,2).*r_glass_norm(:,2) +p1(:,3).*r_glass_norm(:,3))...
      -(r_glass_norm(:,2).*r_glass_norm(:,2)+r_glass_norm(:,3).*r_glass_norm(:,3)).*(p1(:,2).*p1(:,2) + p1(:,3).*p1(:,3)-R*R)))./(r_glass_norm(:,2).*r_glass_norm(:,2)+r_glass_norm(:,3).*r_glass_norm(:,3));
p2 = p1 + t_1.*r_glass_norm; % point at glass and water
tmp = [0 1 1];
coeff = repmat(tmp, size(p2,1), 1);
N1 = p2.* coeff; % normal between  glass and water
N1_norm = N1./sqrt(sum(N1.*N1,2));
s2 = cross(r_glass_norm,N1_norm, 2);
s2_norm = sqrt(sum(s2.*s2, 2));
c2_norm = dot(r_glass_norm, N1_norm, 2);
r_out = n2/n3*r_glass_norm - (n2/n3*c2_norm - sqrt(1- n2*n2/(n3*n3)*s2_norm.*s2_norm)).*N1_norm;
r_out_norm = r_out./sqrt(sum(r_out.*r_out, 2));
x_s = p2;
end