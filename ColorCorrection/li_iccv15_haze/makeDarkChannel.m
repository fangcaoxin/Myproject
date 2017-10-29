function J = makeDarkChannel( I, patch_size )
    patch_side = floor(patch_size/2);
    I = padarray(I, [patch_side patch_side], 'symmetric');
    I_min=min(I,[],3);
    J = colfilt(I_min, [patch_size patch_size], 'sliding', @min);
    J = J(patch_side +1:end-patch_side ,patch_side+1:end-patch_side);
end

