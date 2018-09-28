function res =  error_min_2(init, x,x_w, K, c, Ra, ra,lb,ub, N)
historyn5.x = [];
historyn5.fval = [];
fun = @(ga)fun_total(ga, x, x_w, K, c, Ra, ra, N);
% options=optimoptions(@fmincon, 'Display','iter', 'Algorithm','sqp',...
%     'MaxIterations',3000, 'MaxFunctionEvaluations', 1e4, 'ConstraintTolerance', 1e-1);
options=optimoptions(@fmincon,  'OutputFcn', @outfun,'Display','iter', 'Algorithm','sqp',...
    'MaxIterations',3000, 'MaxFunctionEvaluations', 1e4, 'ConstraintTolerance', 1e-1);
% opts = optimset("MaxIter", 1e5, "Display", "on");
% nonlcon1 = @(gg)cameraRot1(gg);

nonlcon1 = @(ga)cameraRot_total(ga, x, x_w, K , c, Ra, ra, N);

A = [];
b = [];
Aeq = [];
beq = [];
res = fmincon(fun, init, A, b, Aeq, beq, lb, ub, nonlcon1,options);
%  problem  =createOptimProblem('fmincon', 'objective', fun,'x0', init, 'lb', lb, 'ub', ub, 'nonlcon', nonlcon1,'options',options);
%   gs = GlobalSearch;
%   [res, val]= run(gs, problem);
%[gg, val] = sqp(init, fun, nonlcon1, [], lb, ub);
% [res, val, info, iter, nf, lambda] = sqp(init, fun, nonlcon1, [], lb, ub);
res
%  val



function[c, ceq] = cameraRot1(gg)
c =[];
ceq(1,1) = gg(1)^2 + gg(2)^2 + gg(3)^2 -1;
ceq(2,1) = gg(4)^2 + gg(5)^2 + gg(6)^2 -1;
ceq(3,1) = gg(1)*gg(4) + gg(2)*gg(5) + gg(3)*gg(6);

end

function [c1, ceq] = cameraRot_total(ga, uv, x_w, K , c, Ra, ra, N)
   ceq = zeros(15*N+3,1);
   for i = 1:N
       gg = [ga(9*i-8:9*i) ga(9*N+1: end)];
       [c1, ceq(15*i-14:15*i,1)] = cameraRot(gg, uv(:,:,i), x_w, K , c, Ra, ra);
   end
end
function[c1, ceq] = cameraRot(gg, uv, x_w, K , c, Ra, ra)
c1=[];
% ceq = zeros(3+size(uv,1),1);
ceq(1,1) = gg(1)^2 + gg(2)^2 + gg(3)^2 -1;
ceq(2,1) = gg(4)^2 + gg(5)^2 + gg(6)^2 -1;
ceq(3,1) = gg(1)*gg(4) + gg(2)*gg(5) + gg(3)*gg(6);
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
u_v = uv - [hcx hcy];
u_v(:,3) = 1;
r_in = u_v./[fx fy 1];
r_in = r_in*Rc';
r_in = r_in./sqrt(sum(r_in.*r_in,2)); % normalize
t_0 = (-(r_in(:,3)*tc(3) + r_in(:,2)*tc(2)) + sqrt((r_in(:,3).*tc(3) + r_in(:,2).*tc(2)).^2 -(r_in(:,2).*r_in(:,2)+r_in(:,3).*...
    r_in(:,3))*(tc(2)*tc(2)+ tc(3)*tc(3)-r*r)))./(r_in(:,2).*r_in(:,2)+r_in(:,3).*r_in(:,3));
p1 = r_in.* t_0 + tc'; % point at glass and air
tmp = [0 1 1];
coeff = repmat(tmp, size(p1,1), 1);
N = p1.* coeff; % normal between air and glass
N_norm = N./sqrt(sum(N.*N, 2));
s1 = cross(r_in, N_norm,2); % sin(theta_1)
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
s2 = cross(r_glass_norm,N1_norm,2);
s2_norm = sqrt(sum(s2.*s2, 2));
c2_norm = dot(r_glass_norm, N1_norm, 2);
r_out = n2/n3*r_glass_norm - (n2/n3*c2_norm - sqrt(1- n2*n2/(n3*n3)*s2_norm.*s2_norm)).*N1_norm;
r_out_norm = r_out./sqrt(sum(r_out.*r_out, 2));
x_s = p2;
x_w(:, 3) = 0;
x_wc = x_w * Rot' + ts';
x_out = x_wc - x_s;
x_out_norm = x_out./sqrt(sum(x_out.*x_out, 2));
angle = cross(x_out_norm, r_out_norm);
angle_norm = sqrt(sum(angle.*angle, 2));
ceq(4: 6,1) = angle_norm(1:3);
ceq(7: 9,1) = angle_norm(24:26);
ceq(10: 12,1) = angle_norm(50:52);
ceq(13: 15,1) = angle_norm(68:70);
end
function val_total = fun_total(ga, x, x_w, K, c, Ra, ra, N)
    gg = [ga(1:9) ga(9*N+1:end)];
     val_total = fun1(gg, x(:,:,1), x_w, K, c, Ra, ra);
   for i = 2:N
       gg = [ga(i*9-8:i*9) ga(9*N+1:end)];
      val_total= val_total + fun1(gg, x(:,:,i), x_w, K, c, Ra, ra);
   end
end
function val = fun1(gg, x, x_w, K, c, Ra, ra)
r1 = [gg(1) gg(2) gg(3)];
r2 = [gg(4) gg(5) gg(6)];
r1 = r1/norm(r1);
r2 = r2/norm(r2);
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
lamda = -xs_w(:,3)./r_out_w(:,3);
x_chess(:,1) = xs_w(:,1) + lamda.*r_out_w(:,1);
x_chess(:,2) = xs_w(:,2) + lamda.*r_out_w(:,2);
error = x_chess - x_w;
val = error.*error;
val = sum(val, 2);
val = sum(val, 1);
% val = norm(error, 'fro');
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

function stop = outfun(x,optimValues,state)
    stop = false;
  
 
    switch state
        case 'init'
           hold on
         case 'iter'
         % Concatenate current point and objective function
         % value with history. x must be a row vector.
          historyn5.fval = [historyn5.fval; optimValues.fval];
           historyn5.x = [historyn5.x; x];
%            plot(ga(48),optimValues.fval, 'o');
%            optimValues.fval
         % Concatenate current search direction with 
         % searchdir.
%            searchdir = [searchdir;... 
%                         optimValues.searchdirection'];
%            plot(x(1),x(2),'o');
         % Label points with iteration number and add title.
         % Add .15 to x(1) to separate label from plotted 'o'
%            text(x(1)+.15,x(2),... 
%                 num2str(optimValues.iteration));
%            title('Sequence of Points Computed by fmincon');
        case 'done'
            save historyn5.mat historyn5;
         
             hold off
         otherwise
     end
end
end