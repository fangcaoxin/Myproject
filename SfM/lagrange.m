function g=lagrange(U,g0)

    l = zeros(6,1);
   %l = [0.5;0.5;0.5;0.5;0.5;0.5];
	gg0=[g0;l];%init
	f=@(gg)Ug(gg,U);%
  options = optimset("TolFun",1e-10,"TolX",1e-10,"MaxIter",1e3);
 	[gg,fval,info]=fsolve(f,gg0);
   %options=optimoptions('fsolve','Algorithm', 'levenberg-marquardt',...
   %'Display','iter',...
    %   'FunctionTolerance',1e-6,'MaxFunctionEvaluations', 1e6, ...
    %   'StepTolerance', 1e-6,'MaxIterations',3000);
	%[gg,fval,info]=fsolve(f,gg0,options);

	g=gg;

    g(19,:)=[];
    g(19,:)=[];
	g(19,:)=[];
    g(19,:)=[];
    g(19,:)=[];
	g(19,:)=[];
   
end


function Ug_val=Ug(gg,U)

	g=gg;
  g(19,:)=[];
  g(19,:)=[];
	g(19,:)=[];
	g(19,:)=[];
  g(19,:)=[];
  g(19,:)=[];
  
  UU=U'*U;
	Ug_val(1)= UU(1,:)*g; %+ gg(25)*(gg(5)*gg(9)- gg(6)*gg(8));
	Ug_val(2)= UU(2,:)*g; % + gg(25)*(gg(6)*gg(7)-gg(4)*gg(9)); 
	Ug_val(3)= UU(3,:)*g; % + gg(25)*(gg(4)*gg(8)-gg(5)*gg(7));
	Ug_val(4)= UU(4,:)*g; % + gg(25)*(gg(3)*gg(8)-gg(2)*gg(9));
	Ug_val(5)= UU(5,:)*g; % + gg(25)*(gg(1)*gg(9)-gg(3)*gg(7));
	Ug_val(6)= UU(6,:)*g; % + gg(25)*(gg(2)*gg(7)-gg(1)*gg(8));
	Ug_val(7)= UU(7,:)*g; % + gg(25)*(gg(2)*gg(6)-gg(3)*gg(5));
	Ug_val(8)= UU(8,:)*g; % + gg(25)*(gg(3)*gg(4)-gg(1)*gg(6));
	Ug_val(9)= UU(9,:)*g; % + gg(25)*(gg(1)*gg(5)-gg(2)*gg(4));
	Ug_val(10)= UU(10,:)*g+gg(19)*gg(10)-gg(15)*gg(23)+gg(14)*gg(24)+gg(13)*gg(21);
	Ug_val(11)= UU(11,:)*g+gg(19)*gg(11)-gg(13)*gg(24)+gg(15)*gg(22)+gg(14)*gg(21);
	Ug_val(12)= UU(12,:)*g+gg(19)*gg(12)-gg(14)*gg(22)+gg(13)*gg(23)+gg(15)*gg(21);
	Ug_val(13)= UU(13,:)*g+gg(20)*gg(13)-gg(11)*gg(24)+gg(12)*gg(23)+gg(10)*gg(21);
	Ug_val(14)= UU(14,:)*g+gg(20)*gg(14)-gg(12)*gg(22)+gg(10)*gg(24)+gg(11)*gg(21);
	Ug_val(15)= UU(15,:)*g+gg(20)*gg(15)-gg(10)*gg(23)+gg(11)*gg(22)+gg(12)*gg(21);
  Ug_val(16)= UU(16,:)*g-gg(22);
	Ug_val(17)= UU(17,:)*g-gg(23);
	Ug_val(18)= UU(18,:)*g-gg(24);
	Ug_val(19)= (gg(10)^2+gg(11)^2+gg(12)^2-1)/2;
	Ug_val(20)= (gg(13)^2+gg(14)^2+gg(15)^2-1)/2;
	Ug_val(21)= gg(10)*gg(13)+gg(11)*gg(14)+gg(12)*gg(15);
  Ug_val(22)= gg(11)*gg(15)-gg(12)*gg(14)-gg(16);
  Ug_val(23)= gg(12)*gg(13)-gg(10)*gg(15)-gg(17);
  Ug_val(24)= gg(10)*gg(14)-gg(11)*gg(13)-gg(18);
%   Ug_val(25) = gg(1)*gg(5)*gg(9)+gg(2)*gg(6)*gg(7) + gg(3)*gg(4)*gg(8)...
%                -gg(3)*gg(5)*gg(7) - gg(1)*gg(6)*gg(8) - gg(2)*gg(4)*gg(9);
	
end