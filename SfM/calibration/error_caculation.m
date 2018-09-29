load imagePoints.mat
load worldPoints.mat
x = imagePoints(:,:,1);
x_w = worldPoints;
val = backProjectionError(x, x_w);
val