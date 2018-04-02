function g=lagrange_no_scale(U,g0)
	gg0=g0;%ggの初期解．
	f=@(gg)Ug(gg,U);%ggは，gに，alpha，betaを追加したもの．
%	[gg,fval,info]=fsolve(f,gg0,optimset("TolFun",3e-16,"TolX",3e-16,"MaxIter",1e20));
    options=optimoptions('fsolve','Display','off','TolFun',1e-10,'TolX',1e-10,'MaxIter',1e20);
	[gg,fval,info]=fsolve(f,gg0,options);
    g = gg;
end


function Ug_val=Ug(gg,U)
%ggは，gに，alpha，betaを追加したもの．
	g=gg;
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
end