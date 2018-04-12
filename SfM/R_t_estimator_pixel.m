function [R_est,t_est,ray1_vector,ray2_vector]=R_t_estimator_pixel(imgp1,imgp2,scale,type)
load parameter.mat
load camera_motion.mat
	%[U,ray1_vector,ray2_vector]=umatrix_generator_pixel(imgp1,imgp2,type,scale);		
	[U,ray1_vector,ray2_vector]=umatrix_generator_general(imgp1,imgp2,type,scale);
 if(scale == 1)
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
  R1=[g(10) g(11) g(12)]; 
	R2=[g(13) g(14) g(15)]; 
	R3=[g(16) g(17) g(18)];
	R_est=[R1;R2;R3];
  t_3=(R_est(3,3)*g(4)-R_est(2,3)*g(7))/(R_est(3,3)*R_est(2,2)-R_est(2,3)*R_est(3,2));
	t_2=(R_est(3,2)*g(4)-R_est(2,2)*g(7))/(R_est(2,2)*R_est(3,3)-R_est(3,2)*R_est(2,3));
	t_1=(R_est(2,1)*g(3)-R_est(1,1)*g(6))/(R_est(1,1)*R_est(2,2)-R_est(2,1)*R_est(1,2));
  E=[g(1) g(2) g(3);
	   g(4) g(5) g(6);
	   g(7) g(8) g(9)];
	
	%T=R_est'*E
  T = Rotate'*E

	t_3=(T(2,1)-T(1,2))/2;
	t_2=(T(1,3)-T(3,1))/2;
	t_1=(T(3,2)-T(2,3))/2;
	
	t_est=[t_1;t_2;t_3] 
 else
    %k=sqrt(2);
	  g0=g/k;
    g = lagrange_no_scale(U,g0);
     E=[g(1) g(2) g(3);
	   g(4) g(5) g(6);
	   g(7) g(8) g(9)];
    [U_,S_,V_] = svd(E);
     Vt = V_';
     if(det(U_)<0)
     U_(:,3)= -U_(:,3);
     end
     if(det(Vt)<0)
     Vt(3,:)= -Vt(3,:);
     end
     W = [0 -1 0; 1 0 0; 0 0 1];
     R_p(1:3,:) = U_*W*Vt;
     R_p(4:6,:) = U_*W*Vt;
     R_p(7:9,:) = U_*W'*Vt;
     R_p(10:12,:) = U_*W'*Vt;
     ts_p(1:3,:)=U_(:,3);
     ts_p(4:6,:)=-U_(:,3);
     ts_p(7:9,:)=U_(:,3);
     ts_p(10:12,:)=-U_(:,3);
     
 end									    
 save R_t_est.mat R_est t_est

end