function [R_est,t_est]=R_t_estimator_pixel(U)

    U = cast(U, 'double');
    [v,lambda]=eig(U'*U);
    g=v(:,1);
    k=sqrt(g(10)^2+g(11)^2+g(12)^2);
    g0=g/k;
    g = lagrange(U,g0); 
 
  g(10)^2+g(11)^2+g(12)^2
  g(13)^2+g(14)^2+g(15)^2
  g(16)^2+g(17)^2+g(18)^2
  R1=[g(10) g(11) g(12)]; 
	R2=[g(13) g(14) g(15)]; 
	R3=[g(16) g(17) g(18)];
	R_est=[R1;R2;R3]
  E=[g(1) g(2) g(3);
	   g(4) g(5) g(6);
	   g(7) g(8) g(9)];
	
	T=E*R_est'
   
	t_3=(T(2,1)-T(1,2))/2;
	t_2=(T(1,3)-T(3,1))/2;
	t_1=(T(3,2)-T(2,3))/2;
	
	t_est=[t_1;t_2;t_3] 
end