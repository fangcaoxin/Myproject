% use fermat to determine the intersect point at glass
function [c]=fermat_thin(point,n1,n3,R,d)
	load parameter.mat
  v = [point(1), point(2), point(3)];
  v = v/norm(v);
 t =( -(point(2)*v(2) +(point(3)+d)*v(3)) + sqrt((point(2)*v(2) +(point(3)+d)*v(3))*(point(2)*v(2) +(point(3)+d)*v(3))...
      -(v(2)*v(2)+v(3)*v(3))*(point(2)*point(2) + point(3)*point(3)+d*d -R*R+2*point(3)*d)))/(v(2)*v(2)+v(3)*v(3));
 z0 = point(3) + v(3)*t;
  if(z0 < 0)
     t =( -(point(2)*v(2) +(point(3)+d)*v(3)) - sqrt((point(2)*v(2) +(point(3)+d)*v(3))*(point(2)*v(2) +(point(3)+d)*v(3))...
      -(v(2)*v(2)+v(3)*v(3))*(point(2)*point(2) + point(3)*point(3)+d*d -R*R+2*point(3)*d)))/(v(2)*v(2)+v(3)*v(3));
   end
  
  x0 = point(1) + v(1)*t;
  y0 = point(2) + v(2)*t;
  z0 = point(3) + v(3)*t;
  c0 = [x0;y0];
  x = point(1);
  y = point(2);
  z = point(3);
	f=@(c)L_thin(c,x,y,z,n1,n3,R,d);

  %options=optimoptions('fsolve','Display','off','TolFun',3e-16,'TolX',3e-16); %only matlab
  % [c,fval,info]=fsolve(f,c0,options);
  [c, fval, info] = fsolve(f, c0);

%fval
%info
end