 
%  drawmultiview();

 
function generate2D()

 %drawmultiview();
  generate_2DPoints();
end
function generate_2DPoints()
addpath('cylindrical')
 load teapotMatrix1008.mat
imagePointMatrix = zeros(size(teapotMatrix,1), 2, 10);
for i = 1:10
    imagePointMatrix(:,:,i) = point3d_t_2d(teapotMatrix(:,:,i));
%    
plot(imagePointMatrix(:,1,i), imagePointMatrix(:,2,i),'r.');
end
save imagePointMatrix1008.mat imagePointMatrix
load('teapot.mat');
 teapot1 = teapot(1:10:end,:) + [0 0 600]; % Z>600
 basePoint = point3d_t_2d(double(teapot1));
 save basePoint1008.mat basePoint
end
 function drawmultiview()
 load rotateMatrix1008.mat
 load TransMatrix.mat
 load('teapot.mat');
 teapot1 = teapot(1:10:end,:) + [0 0 600]; % Z>600
 teapotMatrix = zeros(size(teapot1, 1), 3, 10);
for i = 1:10
    loc = TransMatrix(i,:);
    ori = rotateMatrix(:,:,i)';
    teapotMatrix(:,:,i) = (teapot1- loc)*ori';
    color = i/10*[1 0 0];
    plotCamera('Location',loc, 'Orientation', ori,'Size', 20,...
     'label', int2str(i), 'color', color, 'AxesVisible', false);
    hold on
end
save teapotMatrix1008.mat teapotMatrix
scatter3(teapot1(:,1), teapot1(:,2), teapot1(:,3), 5, 'MarkerFaceColor',[0 0 1],...
    'MarkerEdgeColor',[0.3,0.5,0.9]);
grid on
axis equal
xlabel('X[mm]');
ylabel('Y[mm]');
zlabel('Z[mm]');
 end
 
 function generateR()
 addpath('helpers');
 rotateMatrix = zeros(3,3,10);
 for i = 1: 10
   rotateMatrix(:,:,i) = generateBoundedR(pi/180*10);
 end
save ratateMatrix1008.mat rotateMatrix
 end
 