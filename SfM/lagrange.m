function g=lagrange(U,g0)
	gg0=[g0;0;0;0];%gg�̏������D
	f=@(gg)Ug(gg,U);%gg�́Cg�ɁCalpha�Cbeta���ǉ�������́D
	%options=optimoptions('fsolve','Display','off','TolFun',3e-16,'TolX',3e-16,'MaxIter',1e20);
	%[gg,fval,info]=fsolve(f,gg0,options);
  [gg, fval, info] = fsolve(f, gg0);
%info
	g=gg;
%fval
	g(19,:)=[];
	g(19,:)=[];
	g(19,:)=[];%�����ł������Ƃ��ɂȂ�
end


function Ug_val=Ug(gg,U)
%gg�́Cg�ɁCalpha�Cbeta���ǉ�������́D
	g=gg;
    g(19,:)=[];
	g(19,:)=[];
	g(19,:)=[];
    UU=U'*U;
	Ug_val(1)= UU(1,:)*g;
	Ug_val(2)= UU(2,:)*g;
	Ug_val(3)= UU(3,:)*g;
	Ug_val(4)= UU(4,:)*g;
	Ug_val(5)= UU(5,:)*g;
	Ug_val(6)= UU(6,:)*g;
	Ug_val(7)= UU(7,:)*g;
	Ug_val(8)= UU(8,:)*g;
	Ug_val(9)= UU(9,:)*g;
	Ug_val(10)= UU(10,:)*g+gg(19)*gg(10);
	Ug_val(11)= UU(11,:)*g+gg(19)*gg(11);
	Ug_val(12)= UU(12,:)*g+gg(19)*gg(12);
	Ug_val(13)= UU(13,:)*g+gg(20)*gg(13);
	Ug_val(14)= UU(14,:)*g+gg(20)*gg(14);
	Ug_val(15)= UU(15,:)*g+gg(20)*gg(15);
	Ug_val(16)= UU(16,:)*g+gg(21)*gg(16);
	Ug_val(17)= UU(17,:)*g+gg(21)*gg(17);
	Ug_val(18)= UU(18,:)*g+gg(21)*gg(18);
	Ug_val(19)= gg(10)^2+gg(11)^2+gg(12)^2-1;
	Ug_val(20)= gg(13)^2+gg(14)^2+gg(15)^2-1;
	Ug_val(21)= gg(16)^2+gg(17)^2+gg(18)^2-1;
	
end 