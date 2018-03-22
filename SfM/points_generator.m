%3次元空間内の対応点を、好きな個数ランダムに生成してpoints.datに書き込む。

function points_generator()
%%%%%%%%%%%生成する点の個数の設定%%%%%%%%%%%
	n_p=100;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%x,y,zの範囲を指定して、それぞれの範囲内の一様乱数を発生する。
	xmin=-300; %生成する点のｘ座標の範囲を、(xmin,xmax)とする
	xmax=300;
	ymin=-300;
	ymax=300;
	zmin=500;
	zmax=1000;
	
	for i=1:n_p
		k=rand(1);
		l=rand(1);
		m=rand(1);
		p=[xmin+(xmax-xmin)*k ymin+(ymax-ymin)*l zmin+(zmax-zmin)*m]
		fid=fopen('points.matrix','a');
		fprintf(fid,'%d %d %d\n',p);
		fclose(fid);
    end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	xrange=5000; %点のｘ座標の範囲を、-xrangeから＋xrangeとする
%	yrange=5000;
%	zrange=5000;
%	for i=1:300;
%		k=rand(1)
%		l=rand(1)
%		m=rand(1)
%		p=[(2*k-1)*xrange (2*l-1)*yrange (2*m-1)*zrange]
%		fid=fopen('points.matrix','a');
%		fprintf(fid,'%d %d %d\n',p);
%		fclose(fid);
%	endfor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end

