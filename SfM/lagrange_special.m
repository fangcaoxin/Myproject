function g=lagrange_special(U,g0)
	gg0=[g0;0;0;0;0;0];%ggの初期解．
	f=@(gg)Ug(gg,U);%ggは，gに，alpha，betaを追加したもの．
%	[gg,fval,info]=fsolve(f,gg0,optimset("TolFun",3e-16,"TolX",3e-16,"MaxIter",1e20));
    options=optimoptions('lsqnonlin','Display','off','TolFun',1e-10,'TolX',1e-10,'MaxIter',1e20);
	gg=lsqnonlin(f,gg0,options);
   
%info
	g=gg;
%fval
	g(18,:)=[];
	g(18,:)=[];
    g(18,:)=[];
	g(18,:)=[];
	g(18,:)=[];%これでちゃんとｇになる
end


function Ug_val=Ug(gg,U)
%ggは，gに，未定乗数を追加したもの．
    g=gg;
	g(18,:)=[];
	g(18,:)=[];
    g(18,:)=[];
	g(18,:)=[];
    g(18,:)=[];
    
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
    Ug_val(10)= UU(11,:)*g-gg(21);
	Ug_val(11)= UU(12,:)*g-gg(22) ;
	Ug_val(12)= UU(13,:)*g+gg(18)*gg(13)+gg(20)*gg(16)-gg(21)*gg(18);
	Ug_val(13)= UU(14,:)*g+gg(18)*gg(14)+gg(20)*gg(17)-gg(22)*gg(16);
	Ug_val(14)= UU(15,:)*g+gg(18)*gg(15)+gg(20)*gg(12)+gg(21)*gg(16);
	Ug_val(15)= UU(16,:)*g+gg(19)*gg(16)+gg(21)*gg(15)-gg(22)*gg(14);
	Ug_val(16)= UU(17,:)*g+gg(19)*gg(17)+gg(20)*gg(14)+gg(22)*gg(13);
	Ug_val(17)= UU(18,:)*g+gg(19)*gg(18)-gg(21)*gg(13);
    
	Ug_val(18)= (gg(13)^2+gg(14)^2+gg(15)^2-1)/2;
	Ug_val(19)= (gg(16)^2+gg(17)^2+gg(18)^2-1)/2;
    Ug_val(20) = gg(13)*gg(16)+gg(14)*gg(17)+gg(15)*gg(18);
    Ug_val(21) = gg(15)*gg(16)-gg(13)*gg(18)-gg(11);
    Ug_val(22) = gg(13)*gg(17)-gg(14)*gg(16)-gg(12);
 	
end