function res =  error_min(init, x,x_w, K, c, Ra, ra)
fun = @(gg)fun1(gg, x, K, c, Ra, ra) - x_w;
lb = [-1 -1 -1 -1 -1 -1 -500 -500 0 0];
ub = [1 1 1 1 1 1 500 500 800 45];
%options=optimoptions('lsqnonlin', 'Display','iter','FunctionTolerance',1e-10);
opts = optimset("MaxIter", 1e5, "Display", "on");
res = lsqnonlin(fun, init,lb, ub, opts);
%problem = createOptimProblem('lsqnonlin','x0',init,'objective',fun,...
 %   'lb',lb,'ub',ub);
%ms = MultiStart;
%[xx,f] = run(ms, problem, 30);
%f
%res = xx;
fun
end

function val = fun1(gg, x, K, c, Ra, ra)
r1 = [gg(1) gg(2) gg(3)];
r2 = [gg(4) gg(5) gg(6)];
r1 = r1/norm(r1);
r2 = r2/norm(r2);
r3 = cross(r1,r2);
Rot = [r1;r2;r3];
ts = [gg(7); gg(8);gg(9)];
d = gg(10);
hcx = K(3,1);
hcy = K(3,2);
fx = K(1,1);
fy = K(2,2);
n1 = c(1);
n2 = c(2);
n3 = c(3);
R = Ra;
r = ra;
u = x(1) - hcx; % coord on image sensor
v = x(2) - hcy;
r_in=[u, v*fx/fy, fx]; % ray in air
r_in = r_in/norm(r_in); % normalize
t_0 = (-r_in(3)*d + sqrt(r_in(3)*r_in(3)*d*d-(r_in(2)*r_in(2)+r_in(3)*...
    r_in(3))*(d*d-r*r)))/(r_in(2)*r_in(2)+r_in(3)*r_in(3));
 if(d+r_in(3)*t_0 < 0)
      t_0 = (-r_in(3)*d - sqrt(r_in(3)*r_in(3)*d*d-(r_in(2)*r_in(2)+r_in(3)*r_in(3))*(d*d-r*r)))/(r_in(2)*r_in(2)+r_in(3)*r_in(3));
 end
p1 = [r_in(1)*t_0 r_in(2)*t_0 r_in(3)*t_0+d]; % point at glass and air
N = [0 p1(2) p1(3)]; % normal between air and glass
N_norm = N/norm(N);
s1 = norm(cross(r_in, N_norm)); % sin(theta_1)
c1 =norm( r_in.*N_norm); % cos(theta_1)
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
r_out = r_out/norm(r_out);
x_s = p2;
xs_w = Rot'*(x_s' - ts);
r_out_w = Rot'*r_out';
lamda = -xs_w(3)/r_out_w(3);
val(1) = xs_w(1) + lamda*r_out_w(1);
val(2) = xs_w(2) + lamda*r_out_w(2);
end