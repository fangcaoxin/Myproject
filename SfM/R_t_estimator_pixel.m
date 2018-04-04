
function [R_est,t_est,ray1_vector,ray2_vector]=R_t_estimator_pixel(imgp1,imgp2,check,type)
load parameter.mat
	[U,ray1_vector,ray2_vector]=umatrix_generator_pixel(imgp1,imgp2,type);		
	[v,lambda]=eig(U'*U);
	g=v(:,1);
  gf=[0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]';
  gf_1=[0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0]';
 if(norm(g-gf)<1e-6 || norm(g-gf_1)<1e-6)
    g=v(:,2);
    k=sqrt(g(13)^2+g(14)^2+g(15)^2);
	  g0=g/k;
    g = lagrange_special(U,g0);
 else 
    k=sqrt(g(10)^2+g(11)^2+g(12)^2);
	  g0=g/k;
    g = lagrange(U,g0); 
 end

g(10)^2+g(11)^2+g(12)^2
g(13)^2+g(14)^2+g(15)^2
g(16)^2+g(17)^2+g(18)^2

%U*g

%load g_true.matrix
%U*g_true

%%%%%%%%%%?�?�?�?�?�ꂽ?�?�?�?�?�?�R?�Ƃ�?���??�%%%%%%%%%%%%%%%%%%%%%%%	
	R1=[g(10) g(11) g(12)]; %R?�̂P?�s?�ڂ̍s?�x?�N?�g?�?�?�B
	R2=[g(13) g(14) g(15)]; %R?�?�2?�s?�ڂ̍s?�x?�N?�g?�?�?�B
	R3=[g(16) g(17) g(18)];
%	R3=cross(R1,R2);
	R_est=[R1;R2;R3] %R?�̐�?�?�?�l?�B

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