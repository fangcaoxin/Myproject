function xw_est = sfm_two_view()
load imagePoints.mat
load worldPoints.mat
load historyn5.mat
in = [1 2 3 5 9];
view = [3 4]; % good result
x_best = historyn5.x(end,:);
gg1 = [x_best(view(1)*9-8:view(1)*9) x_best(end-5:end) ];
gg2 = [x_best(view(2)*9-8:view(2)*9) x_best(end-5:end) ];
x1 = imagePoints(:,:,in(view(1)));
x2 = imagePoints(:,:,in(view(2)));
K =[590.2313 0 0; 0 559.4365 0; 369.2098 272.4348 1];
c = [1 1.49 1];
Ra = 50;
ra = 46;
[xs_w1, r_out_w1] = sfm_one_view(gg1, x1, K, c, Ra, ra);
[xs_w2, r_out_w2] = sfm_one_view(gg2, x2, K, c, Ra, ra);
v1 = sum(r_out_w1.*r_out_w1,2);
v2 = sum(r_out_w2.*r_out_w2,2);
v3 = sum(r_out_w1.*r_out_w2,2);
w1 = sum((xs_w2-xs_w1).*r_out_w1,2)
w2 = sum((xs_w2-xs_w1).*r_out_w2,2);
s1 = (w1.*v2 - w2.*v3)./(v1.*v2-v3.*v3);
s2 = (s1.*v3 -w2)./v2;
xw_est = (xs_w1 + s1.*r_out_w1 + xs_w2 + s2.*r_out_w2)/2;
error_x = xw_est(:,1)- worldPoints(:,1);
error_y = xw_est(:,2)- worldPoints(:,2);
error_z = xw_est(:,3);
error = error_x.*error_x + error_y.*error_y + error_z.*error_z; 
error_sum = sqrt(sum(error, 1)/70);

% save xw_est.mat xw_est
%% draw 
Z = zeros(70,1);
[loc1, ori1] = camera_pose(gg1)
[loc2, ori2] = camera_pose(gg2);
cam1 = plotCamera('Location',loc1, 'Orientation', ori1,'Size', 15);
hold on
cam2 = plotCamera('Location',loc2, 'Orientation', ori2,'Size', 15);
hold on
scatter3(worldPoints(:,1), worldPoints(:,2), Z,'MarkerFaceColor',[0 0 1]);
hold on
scatter3(xw_est(:,1), xw_est(:,2), xw_est(:,3),'MarkerFaceColor',[1 0 0]);
grid on
xlabel('X(mm)');
ylabel('Y(mm)');
zlabel('Z(mm)');
legend('groud truth', 'constructed points', 'Location', 'northeast');
axis([-150 350 0 380 -600 20]);
end
function [loc, ori] = camera_pose(gg)
r1 = [gg(1) gg(2) gg(3)];
r2 = [gg(4) gg(5) gg(6)];
r3 = cross(r1,r2);
Rot = [r1;r2;r3];
ts = [gg(7); gg(8);gg(9)];
ori = Rot';
loc = -Rot'*ts;
end
