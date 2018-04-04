function points_generator()

	n_p=100; % number of points
	xmin=-1000; 
	xmax=1000;
	ymin=-750;
	ymax=750;
	zmin=1500;
	zmax=2000;
	points = zeros(n_p, 3);
	for i=1:n_p
		k=rand(1);
		l=rand(1);
		m=rand(1);
		p=[xmin+(xmax-xmin)*k ymin+(ymax-ymin)*l zmin+(zmax-zmin)*m]
       points(i,:) = p;
  end
save points.mat points
end

