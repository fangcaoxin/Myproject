function [R_est,t_est]=R_t_estimator_pixel(U, test_points)
load parameter.mat
[R_est, t_est] = Rt_estimate(U, 0);
xw = triangulate(test_points(:,:,1), test_points(:,:,2), R_est, t_est);
if(xw(:,3) < R)
    [R_est, t_est] = Rt_estimate(U, 1);
end

end

function [R_est, t_est] = Rt_estimate(U, mark)
   U = cast(U, 'double');
    [v,lambda]=eig(U'*U);
    g=v(:,1);
    k=sqrt(g(10)^2+g(11)^2+g(12)^2);
    g0 = g/k;
    if mark == 1
    g0 = -g/k;
    end
    %g = lagrange(U,g0); 
    g = g0;
 
  g(10)^2+g(11)^2+g(12)^2
  g(13)^2+g(14)^2+g(15)^2
  g(16)^2+g(17)^2+g(18)^2
  R1=[g(10) g(11) g(12)]; 
	R2=[g(13) g(14) g(15)]; 
	R3=[g(16) g(17) g(18)];
	R_est=[R1;R2;R3];
  E=[g(1) g(2) g(3);
	   g(4) g(5) g(6);
	   g(7) g(8) g(9)];
	T=E*R_est';
   
	t_3=(T(2,1)-T(1,2))/2;
	t_2=(T(1,3)-T(3,1))/2;
	t_1=(T(3,2)-T(2,3))/2;
	
	t_est=[t_1;t_2;t_3];
end

function xw = triangulate(vec1Full, vec2Full, R1_est, t1_est)
    vec1 =  vec1Full;
    vec2  = vec2Full;
    r_out_w1 = vec1(:, 1:3)*R1_est';
    xs_w1 = vec1(:, 4:6)*R1_est' + t1_est';
    ro2 = vec2(:, 1:3);
    xs2 = vec2(:, 4:6);
    r_out_w2 = ro2;
    xs_w2 = xs2;
    v1 = sum(r_out_w1.*r_out_w1,2);
    v2 = sum(r_out_w2.*r_out_w2,2);
    v3 = sum(r_out_w1.*r_out_w2,2);
    w1 = sum((xs_w2-xs_w1).*r_out_w1,2);
    w2 = sum((xs_w2-xs_w1).*r_out_w2,2);
    s1 = (w1.*v2 - w2.*v3)./(v1.*v2-v3.*v3);
    s2 = (s1.*v3 -w2)./v2;
    xw = (xs_w1 + s1.*r_out_w1 + xs_w2 + s2.*r_out_w2)/2;
end