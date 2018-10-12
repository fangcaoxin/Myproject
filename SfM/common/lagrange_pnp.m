function g=lagrange_pnp(U,g0)

    l = zeros(6,1);
   %l = [0.5;0.5;0.5;0.5;0.5;0.5];
	gg0=[g0;l];%init
	f=@(gg)Ug(gg,U);%

  	[gg,fval,info]=fsolve(f,gg0,optimset("TolFun",3e-16,"TolX",3e-16,"MaxIter",1e20));
   %options=optimoptions('fsolve','Algorithm', 'levenberg-marquardt',...
   %'Display','iter',...
  %    'FunctionTolerance',1e-6,'MaxFunctionEvaluations', 1e6, ...
  %    'StepTolerance', 1e-6,'MaxIterations',3000);
	%[gg,fval,info]=fsolve(f,gg0,options);

	g=gg;

    g(13,:)=[];
    g(13,:)=[];
	g(13,:)=[];
    g(13,:)=[];
    g(13,:)=[];
	g(13,:)=[];
   
end


function Ug_val=Ug(gg,U)

	g=gg;
  g(13,:)=[];
  g(13,:)=[];
	g(13,:)=[];
	g(13,:)=[];
  g(13,:)=[];
  g(13,:)=[];
  
  UU=U'*U;

	Ug_val(1)= UU(1,:)*g+gg(13)*gg(1)-gg(17)*gg(6)+gg(15)*gg(4)+ gg(18)*gg(5);
	Ug_val(2)= UU(2,:)*g+gg(13)*gg(2)-gg(18)*gg(4)+gg(15)*gg(5)+gg(16)*gg(6);
	Ug_val(3)= UU(3,:)*g+gg(13)*gg(3)-gg(16)*gg(5)+gg(15)*gg(6)+gg(17)*gg(4);
	Ug_val(4)= UU(4,:)*g+gg(14)*gg(4)-gg(18)*gg(2)+gg(15)*gg(1)+gg(17)*gg(3);
	Ug_val(5)= UU(5,:)*g+gg(14)*gg(5)-gg(16)*gg(3)+gg(15)*gg(2)+gg(18)*gg(1);
	Ug_val(6)= UU(6,:)*g+gg(14)*gg(6)-gg(17)*gg(1)+gg(15)*gg(3)+gg(16)*gg(2);
  Ug_val(7)= UU(7,:)*g-gg(16);
	Ug_val(8)= UU(8,:)*g-gg(17);
	Ug_val(9)= UU(9,:)*g-gg(18);
  Ug_val(10) = UU(10, :)*g;
  Ug_val(11) = UU(11, :)*g;
  Ug_val(12) = UU(12, :)*g;
	Ug_val(13)= (gg(1)^2+gg(2)^2+gg(3)^2-1)/2;
	Ug_val(14)= (gg(4)^2+gg(5)^2+gg(6)^2-1)/2;
	Ug_val(15)= gg(1)*gg(4)+gg(2)*gg(5)+gg(3)*gg(6);
  Ug_val(16)= gg(2)*gg(6)-gg(3)*gg(5)-gg(7);
  Ug_val(17)= gg(3)*gg(4)-gg(1)*gg(6)-gg(8);
  Ug_val(18)= gg(1)*gg(5)-gg(2)*gg(4)-gg(9);
	
end