function get_pixel
%�V�~�����[�V�����p�D����\���̉摜��W���C�s�N�Z���\���ɕϊ�����
	load imgs.mat;
	load parameter.mat;
  load n_cp.matrix;
	
	imgp1=[];
	imgp2=[];
	
    add_Gaussian=0;
    
    sigma=0.1;
    ave=0.0;
    stats1_log=[];
    stats2_log=[];
       
 	pa = 1;
    
hcx
hcy
	for i=1:size(img1,1)
%		X1=img1(i,1)/sx+hcx;
%		Y1=img1(i,2)/sy+hcy;
		X1=img1(i,1)+hcx;
		Y1=img1(i,2)+hcy;
		
		%--------------------�l�̌ܓ�---------------------
		X1=X1*pa;
		Y1=Y1*pa;
		
		X1=round(X1);
		Y1=round(Y1);
		
		X1=X1/pa;
		Y1=Y1/pa;
		%--------------------�l�̌ܓ�---------------------
		
		imgp1=[imgp1;X1 Y1];
	
%		X2=img2(i,1)/sx+hcx;
%		Y2=img2(i,2)/sy+hcy;
		X2=img2(i,1)+hcx;
		Y2=img2(i,2)+hcy;
		
		%--------------------�l�̌ܓ�---------------------
		X2=X2*pa;
		Y2=Y2*pa;
		
		X2=round(X2);
		Y2=round(Y2);
		
		X2=X2/pa;
		Y2=Y2/pa;
		%--------------------�l�̌ܓ�---------------------
        
		imgp2=[imgp2;X2 Y2];
    end
    
    %�K�E�V�A���m�C�Y�̕t��
    if (add_Gaussian==1)

        %�摜1�@x�����p�덷
        randn_error1_x=randn(1,n_cp);
        G_error1_x=sigma*randn_error1_x+ave;
        %�摜1�@y�����p�덷
        randn_error1_y=randn(1,n_cp);
        G_error1_y=sigma*randn_error1_y+ave;
        %���ρC�W���΍��̊m�F�@
        stats1 = [mean(G_error1_x) std(G_error1_x) mean(G_error1_y) std(G_error1_y)];

        imgp1(:,1)=imgp1(:,1)+G_error1_x';
        imgp1(:,2)=imgp1(:,2)+G_error1_y';
        
        stats1_log(i,:)=stats1;

        %�摜2�@x�����p�덷
        randn_error2_x=randn(1,n_cp);
        G_error2_x=sigma*randn_error2_x+ave;
        %�摜2�@y�����p�덷
        randn_error2_y=randn(1,n_cp);
        G_error2_y=sigma*randn_error2_y+ave;
        %���ρC�W���΍��̊m�F�@
        stats2 = [mean(G_error2_x) std(G_error2_x) mean(G_error2_y) std(G_error2_y)];
        
        imgp2(:,1)=imgp2(:,1)+G_error2_x';
        imgp2(:,2)=imgp2(:,2)+G_error2_y';

        stats2_log(i,:)=stats2;

    end
	
	save imgs_p.mat imgp1 imgp2
    save stats_error.mat stats1_log stats2_log 

end