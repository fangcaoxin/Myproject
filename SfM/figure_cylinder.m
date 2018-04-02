clc;
R = 50;
r = 46;
point = [-10,10,100]; % 3d point
gw_point = [-1.7513 2.3869 49.94]; % intersect at glass and water
ga_point =[-1.672 1.8270 45.967]; % interset at glass and air
focal = [0 0 41]; % focal point
plot3(focal(1),focal(2),focal(3),'y.','Markersize',20); %plot focal
hold on
plot3(point(1),point(2),point(3), 'r.','Markersize',20); % plot 3D point
hold on
plot3([point(1), gw_point(1)],[point(2),gw_point(2)],[point(3),gw_point(3)],'g','linewidth',2); % ray_out
hold on
plot3([ga_point(1), gw_point(1)],[ga_point(2),gw_point(2)],[ga_point(3),gw_point(3)],'r','linewidth',2); % ray_mid
hold on
plot3([ga_point(1), 0],[ga_point(2),0],[ga_point(3),41],'b','linewidth',2); % ray_in
hold on
plot3([gw_point(1), gw_point(1)],[0,gw_point(2)],[0,gw_point(3)],'g--','linewidth',1); % normal_1
hold on
plot3([ga_point(1), ga_point(1)],[ga_point(2),0],[ga_point(3),0],'r--','linewidth',2);

axis_point_x = [-25:25];
axis_point_y = zeros(1,51);
axis_point_z= zeros(1,51);
plot3([-25 25], [0,0],[0,0], 'b-.');
hold on
[Z,Y,X]=cylinder(R,50);
X_n = [-25+ X(1,:);25+X(2,:)];
hold on
[Z1,Y1,X1] = cylinder(r,50);
X1_n =[-25+X1(1,:);25+X1(2,:)];
hold on
[x y]= meshgrid(-5:5);
z= 0*x + 44.7;
surf(x,y,z,'facecolor',[0 1 0]);
h1 = mesh(X_n,Y,Z,'facecolor',[1 0 0]);
h2 = mesh(X1_n,Y1,Z1,'facecolor',[1 0 0]);
alpha(h1,0.5);
alpha(h2,0.5);

axis on
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on
%shading interp
