function points_3d = points_reconstruction
load ray1_select.mat
load ray2_select.mat
load R_t_est.mat
load effctive_num.mat 
load camera_motion.mat
load cross_point_effctive.mat

%R_est = Rotate;
%t_est = translation;
point_num = effctive_point;
points_3d = zeros(point_num,3);
for i = 1: point_num
    r1= ray1_out_select(i,:)';
    r2 =ray2_out_select(i,:)';
    d2 = cross_point2_effctive(i,:);
    d1 = cross_point1_effctive(i,:);
    r2w = R_est'*r2;
    d2w = R_est'*d2';
    dd =t_est + d2w;
    A1=r1(1)^2+r1(2)^2+r1(3)^2;
    A2=r2w(1)^2+r2w(2)^2+r2w(3)^2;
    B=r1(1)*r2w(1)+r1(2)*r2w(2)+r1(3)*r2w(3);
    C1=(dd(1)-d1(1))*r1(1)+(dd(2)-d1(2))*r1(2)+(dd(3)-d1(3))*r1(3);
    C2=(dd(1)-d1(1))*r2w(1)+(dd(2)-d1(2))*r2w(2)+(dd(3)-d1(3))*r2w(3);
    s=(C1*A2-B*C2)/(A1*A2-B^2);
	t=(B*s-C2)/A2;		
	point_est=(d1'+s*r1+dd+t*r2w)/2;
	points_3d(i,:)=point_est';
end
save points_3d_est.mat points_3d;