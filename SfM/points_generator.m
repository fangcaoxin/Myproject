function points_generator()

	n_p=100;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	xmin=-300; 
	xmax=300;
	ymin=-200;
	ymax=200;
	zmin=600;
	zmax=1000;
	points = zeros(n_p, 3);
	for i=1:n_p
		k=rand(1);
		l=rand(1);
		m=rand(1);
		p=[xmin+(xmax-xmin)*k ymin+(ymax-ymin)*l zmin+(zmax-zmin)*m]
		%fid=fopen('points.matrix','a');
		%fprintf(fid,'%d %d %d\n',p);
		%fclose(fid);
    points(i,:) = p;
  end
save points.mat points
end

