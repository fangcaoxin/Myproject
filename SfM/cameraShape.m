function cameraShape(size)
    hold on;
    x=[1.5,1.5,-1.5,-1.5,1.5,0,1.5,-1.5,0,-1.5];
    y=[1,-1,-1,1,1,0,-1,-1,0,1];
    z=[1,1,1,1,1,0,1,1,0,1];
    
    x=size*x;
    y=size*y;
    z=size*z*2;
        
    line(x,y,z,'Color','black');
end