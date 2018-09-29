function sumL=L_flat(c,x,y,z,n1,n2,n3,w,d)
    point_c = [x,y,z];
    point_outer = [c(1), c(2), d+w];
    point_inner = [c(3), c(4), d];
    point_camera = [0 0 0];
    sumL = n3*norm(point_c - point_outer) + n2*norm(point_outer - point_inner)+...
        n1*norm(point_inner - point_camera);

end