% use fermat to determine the intersect point at glass
function [c]=fermat_flat(point,n1,n2,n3,w,d,cali)
	load parameter.mat
  if(cali > 0)
      n3 = n1;
  end

  v = [-point(1), -point(2), -point(3)];
  v = v/norm(v);
  t =  (d+w-point(3))/v(3);
  t1 = (d-point(3))/v(3);
  
  x0 = point(1) + v(1)*t ; % outer
  y0 = point(2) + v(2)*t;
  x1 = point(1) + v(1)*t1; %inner
  y1 = point(2) + v(2)*t1;

    c0 = [x0;y0;x1;y1];
    x = point(1);
    y = point(2);
    z = point(3);
	f=@(c)L_flat(c,x,y,z,n1,n2,n3,R,r,d);

   options=optimoptions('fmincon','Display','off'); %only matlab
%    [c,fval,info]=fsolve(f,c0,options);
c = fmincon(f, c0,[],[],[],[],[-25 -25 -25 -23], [25 25 25 23],[], options);
  

%fval
%info
end