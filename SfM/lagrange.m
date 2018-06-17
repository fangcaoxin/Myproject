function g = lagrange(U,C,g0)
gg0 = [g0; 0; 0;0];
f=@(gg)Ug(gg,U,C);
options=optimoptions('fsolve','Display','off','TolFun',1e-10,'TolX',1e-10,'MaxIter',1e20);
[gg,fval,info]=fsolve(f,gg0,options);
g =gg;
g(10,:)=[];
g(10,:)=[];
g(10,:)=[];
end

function Ug_val = Ug(gg, U,C)
 g = gg;
 g(10,:)=[];
 g(10,:)=[];
 g(10,:)=[];
 UU = U'*U;
 A = U';
 C1=-A(1,:)*C;
 C2 =-C'*U;
 Ug_val(1) = UU(1,:)*g -A(1,:)*C + C2(1)+ gg(10)*gg(1) + gg(12)*gg(4);
 Ug_val(2) = UU(2,:)*g -A(2,:)*C + C2(2)+ gg(10)*gg(2) + gg(12)*gg(5);
 Ug_val(3) = UU(3,:)*g -A(3,:)*C + C2(3)+ gg(10)*gg(3) + gg(12)*gg(6);
 Ug_val(4) = UU(4,:)*g- A(4,:)*C + C2(4)+ gg(11)*gg(4) + gg(12)*gg(1);
 Ug_val(5) = UU(5,:)*g -A(5,:)*C + C2(5)+ gg(11)*gg(5) + gg(12)*gg(2);
 Ug_val(6) = UU(6,:)*g -A(6,:)*C + C2(6)+ gg(11)*gg(6) + gg(12)*gg(3);
 Ug_val(7) = UU(7,:)*g -A(7,:)*C + C2(7);
 Ug_val(8) = UU(8,:)*g -A(8,:)*C + C2(8);
 Ug_val(9) = UU(9,:)*g -A(9,:)*C + C2(9);
 Ug_val(10) = (gg(1)^2 + gg(2)^2 + gg(3)^2 -1)/2;
 Ug_val(11) = (gg(4)^2 + gg(5)^2 + gg(6)^2 -1)/2;
 Ug_val(12) = gg(1)*gg(4)+gg(2)*gg(5)+gg(3)*gg(6);
end