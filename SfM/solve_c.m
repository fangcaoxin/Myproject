
function [c,s]= solve_c(R,r,X,d,f)
  fi = atan2(X(3,:),X(2,:)); % angle with y axis
  y_fi_array = sqrt(X(3,:).*X(3,:) + X(2,:).*X(2,:));
  c1_init = (r-d)*X(1,:)./y_fi_array;
  c2_init = (R-d)*X(1,:)./y_fi_array;
  s_init = X(2,:)*r./sqrt(X(2,:).*X(2,:)+ X(3,:).*X(3,:));
  global d_c_a;
  global x y z;
  global y_fi;
  d_c_a =d;
  c = zeros(size(X,2),2);
  s = zeros(size(X,2),2);
 for i = 1:size(X,2)
    x = X(1,i);
    y = X(2,i);
    z = X(3,i);
    y_fi = y_fi_array(1,i);
    c0 = [c1_init(1,i),c2_init(1,i)];
    s0= [s_init(1,i),s_init(1,i)];
   
    fun= @myfun;
    c(i,:) = fsolve(fun,c0);
%     fun1 =@myfun1;
%     s(i,:)= fsolve(fun1,s0);
 end
end

function q = myfun(p)
c1 = p(1);
c2 = p(2);
global n1 n2 n3 R r x y_fi d_c_a;
q(1) = -n2*(c2-c1)/sqrt((R-r)*(R-r) + (c2 -c1)*(c2-c1)) + n1*c1/sqrt((r-d_c_a)*(r-d_c_a) + c1*c1);
q(2) = -n3*(x - c2)/sqrt((y_fi-R)*(y_fi-R)+(x -c2)*(x-c2))+ ...,
       n2*(c2-c1)/sqrt((R-r)*(R-r)+(c2-c1)*(c2-c1));
end

function q1 = myfun1(p1)
 syms s1 s2;

global n1 n2 n3 R r y z d_c_a;

% L = n3*sqrt((y-s2)*(y-s2)+(z-sqrt(R*R-s2*s2))) +...,
%     n2*sqrt((s2-s1)*(s2-s1)+(sqrt(R*R-s2*s2)-sqrt(r*r-s1*s1))*(sqrt(R*R-s2*s2)-sqrt(r*r-s1*s1)))+...,
%     n1*sqrt((sqrt(r*r-s1*s1)-d_c_a)*(sqrt(r*r-s1*s1)-d_c_a)+s1*s1);
% L_diff_s1 =diff(L,s1,1);
% L_diff_s2 =diff(L,s2,1);
s1 = p1(1);
s2 = p1(2);
q1(1) = (n1*(2*s1 + (2*s1*(d_c_a - (r^2 - s1^2)^(1/2)))/(r^2 - s1^2)^(1/2)))/(2*((d_c_a - (r^2 - s1^2)^(1/2))^2 + s1^2)^(1/2)) + (n2*(2*s1 - 2*s2 + (2*s1*((R^2 - s2^2)^(1/2) - (r^2 - s1^2)^(1/2)))/(r^2 - s1^2)^(1/2)))/(2*((s1 - s2)^2 + ((R^2 - s2^2)^(1/2) - (r^2 - s1^2)^(1/2))^2)^(1/2));
q1(2) = (n3*(2*s2 - 2*y + s2/(R^2 - s2^2)^(1/2)))/(2*(z + (s2 - y)^2 - (R^2 - s2^2)^(1/2))^(1/2)) - (n2*(2*s1 - 2*s2 + (2*s2*((R^2 - s2^2)^(1/2) - (r^2 - s1^2)^(1/2)))/(R^2 - s2^2)^(1/2)))/(2*((s1 - s2)^2 + ((R^2 - s2^2)^(1/2) - (r^2 - s1^2)^(1/2))^2)^(1/2));
end

% for test
function F = root2d(x)
a = 1;
F(1) = a*exp(-exp(-(x(1)+x(2)))) - x(2)*(1+x(1)^2);
F(2) = x(1)*cos(x(2)) + x(2)*sin(x(1)) - 0.5;
end

function q2 =  slove_touch_point(p1)
L = n3*sqrt((x2-x)*(x2-x)+(y2-y)*(y2-y)+(sqrt(R*R-y2*y2)-z)*(sqrt(R*R-y2*y2)-z))+ ...,
n2*sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)+(sqrt(R*R-y2*y2)-sqrt(R*R-y1*y1))*(sqrt(R*R-y2*y2)-sqrt(R*R-y1*y1)))+ ...,
n1*sqrt(x1*x1 + y1*y1 + (sqrt(r*r-y1*y1)-d_c_a)*(sqrt(r*r-y1*y1)-d_c_a));
                   n1*x1                                                    n2
  -------------------------------------------- + -------------------------------
       _______________________________________        __________________________
      /                                     2        /
     /              /            __________\        /
    /     2     2   |           /  2     2 |       /             2             2
  \/    x1  + y1  + \-d_c_a + \/  r  - y1  /     \/    (-x1 + x2)  + (-y1 + y2)

  *(x1 - x2)
  --------------------------------------
  ______________________________________
                                      2
     /     __________      __________\
     |    /  2     2      /  2     2 |
   + \- \/  R  - y1   + \/  R  - y2  /
x1 = p1(1);
q2(1) = n1*x1/sqrt(x1*x1 + y1*y1 + (-d_c_a+ sqrt(r*r - y1*y1)))+ ...,
n2*(x1-x2)/sqrt((-x1+x2)*(-x1+x2) + (-y1+ y2)*(-y1 + y2)+ (-sqrt(R*R -y1*y1)+sqrt(R*R - y2*y2))* ...,
(-sqrt(R*R -y1*y1)+sqrt(R*R - y2*y2)));

                            n2*(-x1 + x2)
  --------------------------------------------------------------------- + ------
       ________________________________________________________________        _
      /                                                              2        /
     /                              /     __________      __________\        /
    /             2             2   |    /  2     2      /  2     2 |       /
  \/    (-x1 + x2)  + (-y1 + y2)  + \- \/  R  - y1   + \/  R  - y2  /     \/

                 n3*(-x + x2)
  ------------------------------------------------
  ________________________________________________
                                                2
                            /        __________\
           2            2   |       /  2     2 |
  (-x + x2)  + (-y + y2)  + \-z + \/  R  - y2  /
  
  x2 = p1(2);
  q2(2) = n2*(-x1+x2) /sqrt((-x1 + x2)*(-x1 + x2) + (-y1 + y2)*(-y1 + y2) + ...,
 (-sqrt(R*R-y1*y1)+sqrt(R*R-y2*y2))*(-sqrt(R*R-y1*y1)+sqrt(R*R-y2*y2)) )+ ...,
 n3*(-x + x2)/sqrt((-x + x2)*(-x + x2) + (-y + y2)*(-y + y2) + ...,
 (-z+sqrt(R*R-y2*y2))*(-z+sqrt(R*R-y2*y2)));

end

