function [R_est,t_est,ray1_vector,ray2_vector]=R_t_estimator_pixel(imgp1,imgp2,scale)
load parameter.mat
	[U,ray1_vector,ray2_vector]=umatrix_generator_pixel(imgp1,imgp2,scale);		

	[v,lambda]=eig(U'*U);
	g=v(:,1);
if(scale == 1)
	k=sqrt(g(10)^2+g(11)^2+g(12)^2);
	g0=g/k;
    R1=[g0(10) g0(11) g0(12)]; %the first row of R
	R2=[g0(13) g0(14) g0(15)];  %the second row of R
	R3=cross(R1,R2);
else
    k=sqrt(g(1)^2+g(2)^2+g(3)^2);
	g0=g/k;
 end
	
 
if(scale == 0)
    g = lagrange_no_scale(U,g0);
    E=[g(1) g(2) g(3);
	   g(4) g(5) g(6);
	   g(7) g(8) g(9)];
   [U_,S_,V_] =svd(E);
   if(det(U_)<0)
       U_(:,3)= U_(:,3)*(-1);
   end
   if(det(V_)<0)
       V_(3,:)=-V_(3,:);
   end
   W = [0 -1 0; 1 0 0; 0 0 1];
   R_est = U_*W*V_;
   t_est = U_(:,3);
else
	g = lagrange(U,g0);
    g(10)^2+g(11)^2+g(12)^2
    g(13)^2+g(14)^2+g(15)^2
    g(16)^2+g(17)^2+g(18)^2
    R1=[g(10) g(11) g(12)];
	R2=[g(13) g(14) g(15)]; 
	R_est=[R1;R2;R3];
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
end


    
    save R_t_est.mat R_est t_est
	
%	save g.matrix g

end