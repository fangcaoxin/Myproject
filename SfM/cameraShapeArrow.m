function cameraShapeArrow(size)
    hold on;
    x=[1.5,1.5,-1.5,-1.5,1.5,0,1.5,-1.5,0,-1.5];
    y=[1,-1,-1,1,1,0,-1,-1,0,1];
    z=[1,1,1,1,1,0,1,1,0,1];
    
    x=size*x;
    y=size*y;
    z=size*z*2;
        
    line(x,y,z,'Color','black');
    
    a=size*2;
    hold on;
    quiver3(0,0,0,a,0,0,'color',[1,0,0],'linewidth',2);
    hold on;
    quiver3(0,0,0,0,a,0,'color',[0,1,0],'linewidth',2);
    hold on;
    quiver3(0,0,0,0,0,a,'color',[0,0,1],'linewidth',2);
end