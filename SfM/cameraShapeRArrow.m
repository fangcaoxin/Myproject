function cameraShapeRArrow(size,R,t)
    hold on;
    x=[1.5,1.5,-1.5,-1.5,1.5,0,1.5,-1.5,0,-1.5];
    y=[1,-1,-1,1,1,0,-1,-1,0,1];
    z=[1,1,1,1,1,0,1,1,0,1];
    
    x=size*x;
    y=size*y;
    z=size*z*2;
    
    for i=1:10
       p=[x(i);y(i);z(i)];
       q=R'*p+t;
       x(i)=q(1);
       y(i)=q(2);
       z(i)=q(3);       
    end
    
    line(x,y,z,'Color','black');
    
    a=size*2;
    
    xa=[a;0;0];
    xa=R'*xa;
    ya=[0;a;0];
    ya=R'*ya;
    za=[0;0;a];
    za=R'*za;
    
    hold on;
    quiver3(t(1),t(2),t(3),xa(1),xa(2),xa(3),'color',[1,0,0],'linewidth',2);
    hold on;
    quiver3(t(1),t(2),t(3),ya(1),ya(2),ya(3),'color',[0,1,0],'linewidth',2);
    hold on;
    quiver3(t(1),t(2),t(3),za(1),za(2),za(3),'color',[0,0,1],'linewidth',2);
end