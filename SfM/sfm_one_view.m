function [xs_w, r_out_w] = sfm_one_view(gg, x, K, c, Ra, ra)
r1 = [gg(1) gg(2) gg(3)];
r2 = [gg(4) gg(5) gg(6)];
r3 = cross(r1,r2);
Rot = [r1;r2;r3];
ts = [gg(7); gg(8);gg(9)];
tc = [gg(10); gg(11); gg(12)];
ac = [gg(13); gg(14); gg(15)]; %rotate x, y, z
Rc = angle2Rot(gg(13), gg(14), gg(15));
hcx = K(3,1);
hcy = K(3,2);
fx = K(1,1);
fy = K(2,2);
n1 = c(1);
n2 = c(2);
n3 = c(3);
R = Ra;
r = ra;
u_v = x - [hcx hcy];
u_v(:,3) = 1;
r_in = u_v./[fx fy 1];
r_in = r_in*Rc';
r_in = r_in./sqrt(sum(r_in.*r_in,2)); % normalize
t_0 = (-(r_in(:,3)*tc(3) + r_in(:,2)*tc(2)) + sqrt((r_in(:,3).*tc(3) + r_in(:,2).*tc(2)).^2 -(r_in(:,2).*r_in(:,2)+r_in(:,3).*...
    r_in(:,3))*(tc(2)*tc(2)+ tc(3)*tc(3)-r*r)))./(r_in(:,2).*r_in(:,2)+r_in(:,3).*r_in(:,3));
 %if(d+r_in(3)*t_0 < 0)
 %     t_0 = (-r_in(3)*d - sqrt(r_in(3)*r_in(3)*d*d-(r_in(2)*r_in(2)+r_in(3)*r_in(3))*(d*d-r*r)))/(r_in(2)*r_in(2)+r_in(3)*r_in(3));
 %end
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
xs_w =(x_s - ts')*Rot;
r_out_w = r_out_norm * Rot;
end

function rotate = angle2Rot(alpha, beta, gamma)
rotate = zeros(3,3);
r1 = [cosd(gamma) sind(gamma) 0;
      -sind(gamma) cosd(gamma) 0;
      0 0 1 ];
r2 = [cosd(beta) 0 -sind(beta);
      0 1 0;
      sind(beta) 0 cosd(beta)];
r3 = [1 0 0;
      0 cosd(alpha) sind(alpha);
      0 -sind(alpha) cosd(alpha)];
rotate = r1*r2*r3;
end