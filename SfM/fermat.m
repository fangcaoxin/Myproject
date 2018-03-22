function [c]=fermat(point,n1,n2,n3,R,r,d)
%光路長を最小にする非線形最小化問題をとく 
%-------------カメラパラメータの取得.検証するときだけこの方法で取得する-------------%
	load parameter.mat
%----------------------------------------%
  v = [point(1), point(2), point(3)-d];
  v = v/norm(v);
  t =( -(point(2)*v(2) +point(3)*v(3)) + sqrt((point(2)*v(2) +point(3)*v(3))*(point(2)*v(2) +point(3)*v(3))...
      -(v(2)*v(2)+v(3)*v(3))*(point(2)*point(2) + point(3)*point(3)-R*R)))/(v(2)*v(2)+v(3)*v(3));
  t1 = ( -(point(2)*v(2) +point(3)*v(3)) + sqrt((point(2)*v(2) +point(3)*v(3))*(point(2)*v(2) +point(3)*v(3))...
      -(v(2)*v(2)+v(3)*v(3))*(point(2)*point(2) + point(3)*point(3)-r*r)))/(v(2)*v(2)+v(3)*v(3));
  x0 = x + v(1)*t;
  y0 = y + v(2)*t;
  x1 = x+v(1)*t1;
  y1 = x + v(2)*t1;
%	c0 =[z/x*rho1;z/x*rho2];%c1,c2の初期解		5/22
	%c0 =[x/z*rho1;x/z*rho2];%c1,c2の初期解
    c0 = [x0;y0;x1;y1];
    x = point(1);
    y = point(2);
    z = point(3);
	f=@(c)L(c,x,y,z,n1,n2,n3,R, r,d);
%	[c,fval,info]=fsolve(f,c0,optimset("TolFun",3e-16,"TolX",3e-16));
    options=optimoptions('fsolve','Display','off','TolFun',3e-16,'TolX',3e-16);
	[c,fval,info]=fsolve(f,c0,options);

%fval
%info
end