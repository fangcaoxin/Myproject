
%% error caculate
load imagePoints.mat
load worldPoints.mat
load R_est.mat
load t_est.mat
load parameter.mat
load Rotation_Matrix.mat
load Translation.mat
d = 0:1:45;
d_range = size(d, 2);
point_num = size(imagePoints,1);
error_sum = zeros(d_range,1);
error = zeros(point_num, 1);
i_mc =[590.2313 0 0; 0 559.4365 0; 369.2098 272.4348 1];
ra_c = [-0.4084 0.1707];
k1 = ra_c(1);
k2 = ra_c(2);
cali = 1;
for i = 1: d_range
  for k = 1: point_num
       x_w = [worldPoints(k,:),0];
        uv = imagePoints(k,:,1);
%        x_wc = Rotation_Matrix(:,:,1)*x_w' + Translation(:,:,1)';
       [r_out, dis] = ray_in_out_pixel(uv,d(i),cali);
       xs_w = R_est(:,:,i)'*(dis' - t_est(:,i));
       r_out_w = R_est(:,:,i)'*r_out';
       lamda = -xs_w(3)/r_out_w(3);
       x_wb(1) = xs_w(1) + lamda*r_out_w(1);
       x_wb(2) = xs_w(2) + lamda*r_out_w(2);
       
%         x_wc = R_est(:,:,i)*x_w' + t_est(:,i);
%          c=fermat(x_wc,n1,n2,n3,R,r,d(i),cali);
%         point_at_glass_water = [c(1) c(2) sqrt(R*R-c(2)*c(2))];
%         point_at_glass_air =[c(3) c(4) sqrt(r*r -c(4)*c(4))];
%         x_c = [point_at_glass_air(1); point_at_glass_air(2); point_at_glass_air(3) - d(i)];
%        r_in_true = [point_at_glass_air(1) point_at_glass_air(2) point_at_glass_air(3)-d(i)];
%        r_in_true = r_in_true/norm(r_in_true);
%       t = (point_at_glass_air(3) - (fx*sx + d(i)))/r_in_true(3);
%       point_2d = [point_at_glass_air(1)-r_in_true(1)*t point_at_glass_air(2)-r_in_true(2)*t];
%       uv_1 = [point_2d(1)/sx+hcx hcy-point_2d(2)/sx];
%         x_uv = i_mc'*x_c;
%         uv_1 = [x_uv(1)/x_uv(3) x_uv(2)/x_uv(3)];
%          x = (uv_1(1)-hcx)/fx;
%          y = (uv_1(2)-hcy)/fy;
%          r_2 = x*x + y*y;
%          r_4 = r_2*r_2;
%         uv_dis = [(uv_1(1)-hcx)*(1+ k1*r_2 + k2*r_4)+hcx ...
%                   (uv_1(2)-hcy)*(1+ k1*r_2 + k2*r_4)+hcy];
%       if(uv_dis(1)<0|| uv_dis(1)>w||uv_dis(2)<0||uv_dis(2)>h)
%           error(k,1) = 400;
%       else
%       error(k,1)= norm(uv_dis-imagePoints(k,:,1));
%       end
error(k,1) = norm(worldPoints(k,:) - [x_wb(1) x_wb(2)]);
  end
  error_sum(i,:) = sum(error)/point_num;
 end 
