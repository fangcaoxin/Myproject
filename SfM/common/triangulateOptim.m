function xw = triangulateOptim(vec1, vec2, R_2, t_2)
% R_2, t_2 the camera pose relative world coordinate system
    r_out_w1 = vec1(:, 1:3);
    xs_w1 = vec1(:, 4:6);
    ro2 = vec2(:, 1:3);
    xs2 = vec2(:, 4:6);
    r_out_w2 = ro2*R_2';
    xs_w2 = xs2*R_2'+t_2';
    num = size(r_out_w1, 1);
k0 =50* ones(num, 2);
 opts = optimoptions('lsqnonlin','Display','iter','Algorithm','levenberg-marquardt','MaxFunEvals',3e10,'TolFun',1e-10);
out = lsqnonlin(@(k)fun(k,r_out_w1, xs_w1, r_out_w2, xs_w2),k0, [],[], opts);
xw = xs_w1 + out(:,1).*r_out_w1;
end
function fval = fun(k, r_out_w1, xs_w1, r_out_w2, xs_w2)
num = size(r_out_w1, 1);
fval = xs_w1 + k(:,1).*r_out_w1 - xs_w2 - k(:, 2).*r_out_w2;
end