function [R_est,t_est]=R_t_estimator_pixel(U, mark, vertical)
%load parameter.mat
addpath('common');

[R_est, t_est] = Rt_estimate(U, mark, vertical);

end

function [R_est, t_est] = Rt_estimate(U, mark, vertical)
   U = cast(U, 'double');
    [v,lambda]=eig(U'*U);
    if (vertical)
       g=v(:,2);
    else
       g =v(:,1);
    end
    k=sqrt(g(10)^2+g(11)^2+g(12)^2);
    g0 = g/k;
    if mark == 1
    g0 = -g/k;
    end
    g = lagrange(U,g0); 
   % g = g0;
 
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
	[U1, S1, V1] = svd(E);
    W = [ 0 -1 0; 1 0 0; 0 0 1];
    R_est_1(:,:,1) = U1*W*V1';
    R_est_1(:,:,2) = U1*W*V1';
    R_est_1(:,:,3) = U1*W'*V1';
    R_est_1(:,:,4) = U1*W'*V1';
    t_est_1(:,:,1) = U1(:,3);
    t_est_1(:,:,2) = -U1(:,3);
    t_est_1(:,:,3) = U1(:,3);
    t_est_1(:,:,4) = -U1(:,3);
  %U1
  %S1
  %V1
   T  = E*R_est';
   t_err= 0.5*[T(3,2)-T(2,3); T(1,3)-T(3,1);T(2,1)-T(1,2)];
   sign_t_err = sign(t_err);
   scale = (S1(1,1) + S1(2,2))/2;
   t_est = scale * U1(:,3);
   sign_t_est = sign(t_est);
   if(sign_t_err ~= sign_t_est)
      t_est = -t_est;
   end
 t_est = t_err';
end

