
    I = im2double(imread('01.png'));
    [H W D] = size(I);
    [LB LR] = sepGlow(I, 500, zeros(H,W,D), I);
    
    imwrite(2*LB,  'J.png');
    imwrite(2*LR,  'G.png');
    patch_size = 15;
    
    [out_Im J T_est T A] = rmvHaze(2*LB, patch_size, 1 );

    imwrite(A, 'A.png');
    imwrite(T, 'T.png');
    imwrite(out_Im,  'final.png');
