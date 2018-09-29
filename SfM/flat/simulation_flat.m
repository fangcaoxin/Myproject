%%simulate SfM
%% load 3D points
addpath('../common');
load('teapot.mat');
teapot1 = teapot(1:10:end,:) + [0 0 600]; % Z>600
%% transform 3D points to another view(R,t) external matrix
load camera_motion.mat
load parameter.mat
teapot2 = teapot1*Rotate' + translation';
teapot1 = cast(teapot1,'double');
teapot2 = cast(teapot2,'double');
%% intrinsic matrix
K = [fx 0 640; 0 fy 480; 0 0 1];
%% no refraction, directly projection
teapot_c1 = teapot1*K';
teapot_c2 = teapot2*K';
teapot_c1_norm = teapot_c1(:,1:2)./teapot_c1(:,3);
teapot_c2_norm = teapot_c2(:,1:2)./teapot_c2(:,3);
   plot(teapot_c2_norm(:,1), teapot_c2_norm(:,2),'r.');
   axis([0 1280 0 960]);
   hold on
%% considering refraction 3d->2d
 image_points_1 = point3dt2d_flat(teapot1);
 image_points_2 = point3dt2d_flat(teapot2);
 plot(image_points_2(:,1), image_points_2(:,2), '.');
%% considering refraction 2d->3d
load image_points_1.mat
load image_points_2.mat
[r_out_norm1, x_s1] = ray_out_flat(image_points_1,d_flat, w 0);
[r_out_norm2, x_s2] = ray_in_out_pixel(image_points_2,d_flat, w, 0);
%% estimate R and t
matchedVector1 = [r_out_norm1 x_s1];
matchedVector2 = [r_out_norm2 x_s2];
 testVector(:,:,1) = matchedVector1(1:2,:);
 testVector(:,:,2) = matchedVector2(1:2,:);
U=umatrix_generator_general(matchedVector1, matchedVector2);
[R_est,t_est]=R_t_estimator_pixel(U, testVector);
Rotate
R_est
translation
t_est
%% reconstruction
xw = triangulate(matchedVector1, matchedVector2, R_est, t_est);
t_no_scale = translation/norm(translation);
vec1_no_scale = vectorNoScale(image_points_1);
vec2_no_scale = vectorNoScale(image_points_2);
xw_no_scale = triangulate(vec1_no_scale, vec2_no_scale, Rotate, t_no_scale);
%% draw
color1 = [1 0 0];
draw(R_est, t_est, xw, color1);
 hold on 
 scatter3(teapot2(:,1), teapot2(:,2), teapot2(:,3), 5, 'MarkerFaceColor',[0 0 1]);
% hold on
% draw(Rotate, t_no_scale, 50*xw_no_scale, [0 1 0]);


function draw(R_est, t_est, xw_est, color)
loc1 = [0;0;0];
loc2 = t_est;
ori1 = [1 0 0; 0 1 0; 0 0 1];
ori2 = R_est;

% cam1 = plotCamera('Location',loc1, 'Orientation', ori1,'Size', 20,...
%     'label', '1', 'AxesVisible', false);
% hold on
% cam2 = plotCamera('Location',loc2, 'Orientation', ori2,'Size', 20,...
%     'Color',[0 0 1], 'label', '2', 'AxesVisible', false);
% hold on
% scatter3(worldPoints(:,1), worldPoints(:,2), Z,'MarkerFaceColor',[0 0 1]);
% hold on
scatter3(xw_est(:,1), xw_est(:,2), xw_est(:,3), 5, 'MarkerFaceColor',color, 'MarkerEdgeColor', 0.5*color);

grid on
xlabel('X[mm]');
ylabel('Y[mm]');
zlabel('Z[mm]');
% axis_x_min = min([xw_est(:,1); t_est(1,1)]);
% axis_x_max = max([xw_est(:,1); t_est(1,1)]);
% axis_y_min = min([xw_est(:,2); t_est(2,1)]);
% axis_y_max = max([xw_est(:,2); t_est(2,1)]);
% axis_z_min = min([xw_est(:,3); t_est(3,1)]);
% axis_z_max = max([xw_est(:,3); t_est(3,1)]);
%  axis([axis_x_min axis_x_max axis_y_min axis_y_max axis_z_min axis_z_max]); 
end

function vector_no_scale = vectorNoScale(x)
load parameter.mat
u_v = x - [hcx hcy];
u_v(:,3) = 1;
r_in = u_v./[fx fy 1];
r_in = r_in./sqrt(sum(r_in.*r_in,2)); % normalize
x_s = zeros(size(x, 1),3);
vector_no_scale = [r_in, x_s];
end



