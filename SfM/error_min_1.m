function res =  error_min_1(init, x,x_w, K, c, Ra, ra)
fun = @(gg)fun1(gg, x, x_w, K, c, Ra, ra);
lb = [-1 -1 -1 -1 -1 -1 -500 -500 0 0];
ub = [1 1 1 1 1 1 500 500 800 45];
%options=optimoptions('lsqnonlin', 'Display','iter','FunctionTolerance',1e-10);
opts = optimset("MaxIter", 1e5, "Display", "on");
nonlcon = @cameraRot;
A = [];
b = [];
Aeq = [];
beq = [];
res = fmincon(fun, init, A, b, Aeq, beq, lb, ub, nonlcon);
problem  =createOptimProblem('fmincon', 'x0', init, 'objective', fun, ...
'A', A, 'b', b, 'Aeq', Aeq, 'beq', beq, 'lb', lb, 'ub', ub, 'nonlcon', nonlcon);
gs = GlobalSearch;
[xg, fg, flg, og] = run(gs, problem);
fun
end

function [c, ceq] = cameraRot(x, x_w, uv, K , c, Ra, ra)
c = [];
ceq = zeros(3+size(x,2),1);
ceq(1) = x(1)^2 + x(2)^2 + x(3)^2 -1;
ceq(2) = x(4)^2 + x(5)^2 + x(6)^2 -1;
ceq(3) = x(1)*x(4) + x(2)*x(5) + x(3)*x(6);
r1 = [x(1) x(2) x(3)];
r2 = [x(4) x(5) x(6)];
r1 = r1/norm(r1);
r2 = r2/norm(r2);
r3 = cross(r1,r2);
Rot = [r1;r2;r3];
ts = [x(7); x(8);x(9)];
d = x(10);
hcx = K(3,1);
hcy = K(3,2);
fx = K(1,1);
fy = K(2,2);
n1 = c(1);
n2 = c(2);
n3 = c(3);
R = Ra;
r = ra;
u = uv(:,1) - hcx; % coord on image sensor
v = uv(:,2) - hcy;
r_in=[u, v*fx/fy, fx]; % ray in air
r_in = r_in/sqrt(sum(r_in.*r_in,2)); % normalize
t_0 = (-r_in(:,3)*d + sqrt(r_in(:,3).*r_in(:,3)*d*d-(r_in(:,2).*r_in(:,2)+r_in(:,3).*...
    r_in(:,3))*(d*d-r*r)))/(r_in(:,2).*r_in(:,2)+r_in(:,3).*r_in(:,3));
 %if(d+r_in(3)*t_0 < 0)
 %     t_0 = (-r_in(3)*d - sqrt(r_in(3)*r_in(3)*d*d-(r_in(2)*r_in(2)+r_in(3)*r_in(3))*(d*d-r*r)))/(r_in(2)*r_in(2)+r_in(3)*r_in(3));
 %end
p1 = r_in.* t_0 + [0 0 d]; % point at glass and air
tmp = [0 1 1];
coeff = repmat(tmp, size(p1,1), 1);
N = p1.* coeff; % normal between air and glass
N_norm = N./sqrt(sum(N.*N, 2));
s1 = cross(r_in, N_norm); % sin(theta_1)
s1_norm = s1./sqrt(sum(s1.*s1,2));
c1 = r_in.*N_norm; % cos(theta_1)
c1 _norm = c1./sqrt(sum(c1.*c1,2));
r_glass = n1*r_in/n2 - (n1*c1_norm./n2-sqrt(1- n1*n1/(n2*n2)*s1_norm.*s1_norm)).*N_norm;
r_glass_norm = r_glass/sqrt(sum(r_glass.*r_glass, 2));
t_1 =( -(p1(:,2).*r_glass_norm(:,2) +p1(:,3).*r_glass_norm(:,3)) + sqrt((p1(:,2).*r_glass_norm(:,2) +p1(:,3).*r_glass_norm(:,3)).*(p1(:,2)*r_glass_norm(:,2) +p1(:,3)*r_glass_norm(:,3))...
      -(r_glass_norm(:,2).*r_glass_norm(:,2)+r_glass_norm(:,3).*r_glass_norm(:,3)).*(p1(:,2).*p1(:,2) + p1(:,3).*p1(:,3)-R*R)))/(r_glass_norm(:,2).*r_glass_norm(:,2)+r_glass_norm(:,3).*r_glass_norm(:,3));
p2 = p1 + t_1.*r_glass_norm; % point at glass and water
tmp = [0 1 1];
coeff = repmat(tmp, size(p2,1), 1);
N1 = p2.* coeff; % normal between  glass and water
N1_norm = N1/sqrt(sum(N1.*N1,2));
s2 = cross(r_glass_norm,N1_norm);
s2_norm = s2./sqrt(sum(s2.*s2, 2));
c2 = r_glass_norm.*N1_norm;
c2_norm = c2./sqrt(sum(c2.*c2, 2));
r_out = n2/n3*r_glass_norm - (n2/n3*c2_norm - sqrt(1- n2*n2/(n3*n3)*s2_norm.*s2_norm)).*N1_norm;
r_out_norm = r_out/sqrt(sum(r_out.*r_out, 2));
x_s = p2;
x_w(:, 3) = 0;
x_wc = x_w * Rot' + ts;
x_out = x_wc - xs;
angle = cross(x_out, r_out_norm);
angle_norm = angle./sqrt(sum(angle.*angle, 2));
ceq(4: end) = angle_norm;

end

function val = fun1(x, uv, x_w, K, c, Ra, ra)
r1 = [x(1) x(2) x(3)];
r2 = [x(4) x(5) x(6)];
r1 = r1/norm(r1);
r2 = r2/norm(r2);
r3 = cross(r1,r2);
Rot = [r1;r2;r3];
ts = [x(7); x(8);x(9)];
d = x(10);
hcx = K(3,1);
hcy = K(3,2);
fx = K(1,1);
fy = K(2,2);
n1 = c(1);
n2 = c(2);
n3 = c(3);
R = Ra;
r = ra;
u = uv(:,1) - hcx; % coord on image sensor
v = uv(:,2) - hcy;
r_in=[u, v*fx/fy, fx]; % ray in air
r_in = r_in/sqrt(sum(r_in.*r_in,2)); % normalize
t_0 = (-r_in(:,3)*d + sqrt(r_in(:,3).*r_in(:,3)*d*d-(r_in(:,2).*r_in(:,2)+r_in(:,3).*...
    r_in(:,3))*(d*d-r*r)))/(r_in(:,2).*r_in(:,2)+r_in(:,3).*r_in(:,3));
 %if(d+r_in(3)*t_0 < 0)
 %     t_0 = (-r_in(3)*d - sqrt(r_in(3)*r_in(3)*d*d-(r_in(2)*r_in(2)+r_in(3)*r_in(3))*(d*d-r*r)))/(r_in(2)*r_in(2)+r_in(3)*r_in(3));
 %end
p1 = r_in.* t_0 + [0 0 d]; % point at glass and air
tmp = [0 1 1];
coeff = repmat(tmp, size(p1,1), 1);
N = p1.* coeff; % normal between air and glass
N_norm = N./sqrt(sum(N.*N, 2));
s1 = cross(r_in, N_norm); % sin(theta_1)
s1_norm = s1./sqrt(sum(s1.*s1,2));
c1 = r_in.*N_norm; % cos(theta_1)
c1 _norm = c1./sqrt(sum(c1.*c1,2));
r_glass = n1*r_in/n2 - (n1*c1_norm./n2-sqrt(1- n1*n1/(n2*n2)*s1_norm.*s1_norm)).*N_norm;
r_glass_norm = r_glass/sqrt(sum(r_glass.*r_glass, 2));
t_1 =( -(p1(:,2).*r_glass_norm(:,2) +p1(:,3).*r_glass_norm(:,3)) + sqrt((p1(:,2).*r_glass_norm(:,2) +p1(:,3).*r_glass_norm(:,3)).*(p1(:,2)*r_glass_norm(:,2) +p1(:,3)*r_glass_norm(:,3))...
      -(r_glass_norm(:,2).*r_glass_norm(:,2)+r_glass_norm(:,3).*r_glass_norm(:,3)).*(p1(:,2).*p1(:,2) + p1(:,3).*p1(:,3)-R*R)))/(r_glass_norm(:,2).*r_glass_norm(:,2)+r_glass_norm(:,3).*r_glass_norm(:,3));
p2 = p1 + t_1.*r_glass_norm; % point at glass and water
tmp = [0 1 1];
coeff = repmat(tmp, size(p2,1), 1);
N1 = p2.* coeff; % normal between  glass and water
N1_norm = N1/sqrt(sum(N1.*N1,2));
s2 = cross(r_glass_norm,N1_norm);
s2_norm = s2./sqrt(sum(s2.*s2, 2));
c2 = r_glass_norm.*N1_norm;
c2_norm = c2./sqrt(sum(c2.*c2, 2));
r_out = n2/n3*r_glass_norm - (n2/n3*c2_norm - sqrt(1- n2*n2/(n3*n3)*s2_norm.*s2_norm)).*N1_norm;
r_out_norm = r_out/sqrt(sum(r_out.*r_out, 2));
x_s = p2;
xs_w =(x_s - ts')*Rot;
r_out_w = r_out_norm * Rot;
lamda = -xs_w(:,3)./r_out_w(:,3);
x_chess(:,1) = xs_w(:,1) + lamda.*r_out_w(:,1);
x_chess(:,2) = xs_w(:,2) + lamda.*r_out_w(:,2);
val = norm(x_chess - x_w);
end