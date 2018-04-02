function [R_est,t_est,ray1_vector,ray2_vector]=R_t_estimator_pixel(imgp1,imgp2,check)
load parameter.mat
	[U,ray1_vector,ray2_vector]=umatrix_generator_pixel(imgp1,imgp2);		

	[v,lambda]=eig(U'*U);
% 	save v.mat v
	g=v(:,1);
    
    gf=[0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]';
    if(norm(g-gf)<1e-6)
        g=v(:,2)
    end
    
	k=sqrt(g(10)^2+g(11)^2+g(12)^2);
	g0=g/k;
    R1=[g0(10) g0(11) g0(12)]; %the first row of R
	R2=[g0(13) g0(14) g0(15)];  %the second row of R
	R3=cross(R1,R2);
	%	if (abs(g(16)-R3(1))<1)&&(abs(g(17)-R3(2))<1) %?½?½?½Ì•ï¿½?½?½?½?½è‚·?½?½?½B
%	if (g0(16)*R3(1)>1e-16)&&(g0(17)*R3(2)>1e-16) %?½?½?½Ì•ï¿½?½?½?½?½è‚·?½?½?½B
%	else
%		g0=-g0;
%    end
%?½?½?½?½?½?½?½?½?½?½É‚ï¿½?½?½?½D
%	g0=[-1.0897e+002 9.4870e+001 -1.6155e+002 -1.0691e+002 1.6910e+001 1.4618e+002 1.3855e+002 -5.6690e+001 -5.0347e+001 8.4740e-001 7.9316e-002 -5.2500e-001 -2.7534e-001 9.1109e-001 -3.0677e-001 4.5399e-001 4.0451e-001]
%	g0=g0';
%	g0
    if (check==1)
        g0=-g0;
    end
if(d == 0)
    g = lagrange(U,g0);
else
	g = lagrange(U,g0);
end
g(10)^2+g(11)^2+g(12)^2
g(13)^2+g(14)^2+g(15)^2
g(16)^2+g(17)^2+g(18)^2

%U*g

%load g_true.matrix
%U*g_true

%%%%%%%%%%?½?½?½?½?½ê‚½?½?½?½?½?½?½R?½Æ‚ï¿½?½ğ??½%%%%%%%%%%%%%%%%%%%%%%%	
	R1=[g(10) g(11) g(12)]; %R?½Ì‚P?½s?½Ú‚Ìs?½x?½N?½g?½?½?½B
	R2=[g(13) g(14) g(15)]; %R?½?½2?½s?½Ú‚Ìs?½x?½N?½g?½?½?½B
	R3=[g(16) g(17) g(18)];
%	R3=cross(R1,R2);
	R_est=[R1;R2;R3] %R?½Ìï¿½?½?½?½l?½B

%	t_3=(R_est(2,3)*g(1)-R_est(1,3)*g(4))/(R_est(2,3)*R_est(1,2)-R_est(1,3)*R_est(2,2))			%6/5
%	t_2=(R_est(2,2)*g(1)-R_est(1,2)*g(4))/(R_est(1,2)*R_est(2,3)-R_est(2,2)*R_est(1,3))			%6/5
%	t_1=(g(10)*t_2-g(3))/g(11)																	%6/5


	t_3=(R_est(3,3)*g(4)-R_est(2,3)*g(7))/(R_est(3,3)*R_est(2,2)-R_est(2,3)*R_est(3,2));
	t_2=(R_est(3,2)*g(4)-R_est(2,2)*g(7))/(R_est(2,2)*R_est(3,3)-R_est(3,2)*R_est(2,3));
	t_1=(R_est(2,1)*g(3)-R_est(1,1)*g(6))/(R_est(1,1)*R_est(2,2)-R_est(2,1)*R_est(1,2));

	
	E=[g(1) g(2) g(3);
	   g(4) g(5) g(6);
	   g(7) g(8) g(9)];
	
	T=R_est'*E

	t_3=(T(2,1)-T(1,2))/2;
	t_2=(T(1,3)-T(3,1))/2;
	t_1=(T(3,2)-T(2,3))/2;
	
	t_est=[t_1;t_2;t_3]
    
    save R_t_est.mat R_est t_est
	
%	save g.matrix g

end