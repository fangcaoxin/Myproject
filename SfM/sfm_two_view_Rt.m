function xw_est = sfm_two_view_Rt()
load imagePoints.mat
load worldPoints.mat
load historyn5.mat
in = [1 2 3 5 9];
view = [3 4]; % good result
x_best = historyn5.x(end,:);
gg = [x_best(end-5:end) ];
x1 = imagePoints(:,:,in(view(1)));
x2 = imagePoints(:,:,in(view(2)));
K =[590.2313 0 0; 0 559.4365 0; 369.2098 272.4348 1];
c = [1 1.49 1];
Ra = 50;
ra = 46;
[xs1, ro1] = sfm_one_view_Rt(gg, x1, K, c, Ra, ra);
[xs2, ro2] = sfm_one_view_Rt(gg, x2, K, c, Ra, ra);
vec1 = [ro1 xs1];
vec2 = [ro2 xs2];
U=umatrix_generator_general(vec1, vec2);
[R_est,t_est]=R_t_estimator_pixel(U);
R_est
t_est
 r_out_w1 = ro1;
 xs_w1 = xs1;
r_out_w2 = ro2*R_est';
xs_w2 = xs2*R_est';
xs_w2 = xs_w2 + t_est';
v1 = sum(r_out_w1.*r_out_w1,2);
v2 = sum(r_out_w2.*r_out_w2,2);
v3 = sum(r_out_w1.*r_out_w2,2);
w1 = sum((xs_w2-xs_w1).*r_out_w1,2);
w2 = sum((xs_w2-xs_w1).*r_out_w2,2);
s1 = (w1.*v2 - w2.*v3)./(v1.*v2-v3.*v3);
s2 = (s1.*v3 -w2)./v2;
xw_est = (xs_w1 + s1.*r_out_w1 + xs_w2 + s2.*r_out_w2)/2;
%% draw 
loc1 = [0;0;0];
loc2 = t_est;
ori1 = [1 0 0; 0 1 0; 0 0 1];
ori2 = R_est;

cam1 = plotCamera('Location',loc1, 'Orientation', ori1,'Size', 1.5);
hold on
cam2 = plotCamera('Location',loc2, 'Orientation', ori2,'Size', 1.5);
hold on
% scatter3(worldPoints(:,1), worldPoints(:,2), Z,'MarkerFaceColor',[0 0 1]);
% hold on
scatter3(xw_est(:,1), xw_est(:,2), xw_est(:,3),'MarkerFaceColor',[1 0 0]);
grid on
xlabel('X(mm)');
ylabel('Y(mm)');
zlabel('Z(mm)');

 axis([-10 10 -0 10 -100 50]);
end

