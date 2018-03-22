function cameraShapeR(size,R,t,color)
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
    
    line(x,y,z,'Color',color);
end