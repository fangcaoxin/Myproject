% use fermat to determine the intersect point at glass
function [c]=fermat(point,n1,n2,n3,R,r,d, cali)
	load parameter.mat
  if(cali > 0)
      n3 = n1;
  end
%   point = double(point);
  v = [point(1), point(2), point(3)-d];
 % v =[0 point(2)-camera_center(2) point(3)-camera_center(3)];
  v = v/norm(v);
  t =( -(point(2)*v(2) +point(3)*v(3)) + sqrt((point(2)*v(2) +point(3)*v(3))*(point(2)*v(2) +point(3)*v(3))...
      -(v(2)*v(2)+v(3)*v(3))*(point(2)*point(2) + point(3)*point(3)-R*R)))/(v(2)*v(2)+v(3)*v(3));
  t1 = ( -(point(2)*v(2) +point(3)*v(3)) + sqrt((point(2)*v(2) +point(3)*v(3))*(point(2)*v(2) +point(3)*v(3))...
      -(v(2)*v(2)+v(3)*v(3))*(point(2)*point(2) + point(3)*point(3)-r*r)))/(v(2)*v(2)+v(3)*v(3));
  z0 = point(3) + v(3)*t;
  z1 = point(3) + v(3)*t1;
  if(z0 < 0)
     t =( -(point(2)*v(2) +point(3)*v(3)) - sqrt((point(2)*v(2) +point(3)*v(3))*(point(2)*v(2) +point(3)*v(3))-(v(2)*v(2)+v(3)*v(3))*(point(2)*point(2) + point(3)*point(3)-R*R)))/(v(2)*v(2)+v(3)*v(3)); 
   end
  
  if(z1 < 0)
     t1 =( -(point(2)*v(2) +point(3)*v(3)) - sqrt((point(2)*v(2) +point(3)*v(3))*(point(2)*v(2) +point(3)*v(3))-(v(2)*v(2)+v(3)*v(3))*(point(2)*point(2) + point(3)*point(3)-r*r)))/(v(2)*v(2)+v(3)*v(3));
  end
  x0 = point(1) + v(1)*t ;
  y0 = point(2) + v(2)*t;
  x1 = point(1) + v(1)*t1;
  y1 = point(2) + v(2)*t1;

    c0 = [x0;y0;x1;y1];
    x = point(1);
    y = point(2);
    z = point(3);
	f=@(c)L(c,x,y,z,n1,n2,n3,R,r,d);

   options=optimoptions('fmincon','Display','off'); %only matlab
%    [c,fval,info]=fsolve(f,c0,options);
c = fmincon(f, c0,[],[],[],[],[-25 -25 -25 -23], [25 25 25 23],[], options);
  

%fval
%info
end