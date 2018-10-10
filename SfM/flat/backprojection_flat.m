clc;
R = 50;
r = 46;
d = 25;
load image_points_1.mat
load p1.mat
load parameter.mat
load('teapot.mat');
teapot1 = teapot(1:10:end,:) + [0 0 600]; % Z>600
label = [747   306    92   648   323   200   335    80   109   782];
imagepoints = image_points_1(label,:);
u_v = imagepoints - [hcx hcy];
u_v(:,3) = 1;
r_in = u_v./[fx fy 1];
focal = repmat([0 0 0],10,1);
[r_out_norm, x_s] = ray_out_flat(imagepoints,d_flat, w,0);
% quiver3(focal(:,1),focal(:,2),focal(:,3), r_in(:,1),r_in(:,2),r_in(:,3),3);
l1 = quiver3(x_s(:,1),x_s(:,2),x_s(:,3), r_out_norm(:,1),r_out_norm(:,2),r_out_norm(:,3),150, 'color','r');
object_points = teapot1(label, :);
hold on
for i = 1:10
plot3([focal(i,1), p1(i,1)],[focal(i,2), p1(i,2)],[focal(i,3), p1(i,3)], 'b','linewidth', 2 );
plot3([p1(i,1), x_s(i,1)],[p1(i,2), x_s(i,2)],[p1(i,3),x_s(i,3)], 'g','linewidth', 2 );
end
l2 = plot3(focal(1,1),focal(1,2),focal(1,3),'m.','Markersize',20); %plot focal
hold on
l3 = plot3(object_points(:,1),object_points(:,2),object_points(:,3), 'g.','Markersize',20); % plot 3D point


% axis_point_x = [-25:25];
% axis_point_y = zeros(1,51);
% axis_point_z= zeros(1,51);
% plot3([-25 25], [0,0],[0,0], 'b-.');
% hold on
% [Z,Y,X]=cylinder(R,50);
% X_n = [-25+ X(1,:);25+X(2,:)];
% hold on
% [Z1,Y1,X1] = cylinder(r,50);
% X1_n =[-25+X1(1,:);25+X1(2,:)];
% hold on
% [x y]= meshgrid(-5:5);
% z= 0*x + 25;
% surf(x,y,z,'facecolor',[0 1 0]);
% h1 = mesh(X_n,Y,Z,'facecolor',[1 0 0]);
% h2 = mesh(X1_n,Y1,Z1,'facecolor',[1 0 0]);
% alpha(h1,0.5);
% alpha(h2,0.5);

axis on
xlabel('X[mm]');
ylabel('Y[mm]');
zlabel('Z[mm]');
legend([l1,l2,l3],{'Outer ray', 'Camera center','3D object point'});
grid on
%shading interp
