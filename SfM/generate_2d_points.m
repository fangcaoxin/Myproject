 
%  drawmultiview();

 
function generate2D()
load teapotMatrix.mat
imagePointMatrix = zeros(size(teapotMatrix,1), 2, 10);
for i = 1:10
    imagePointMatrix(:,:,i) = point3d_t_2d(teapotMatrix(:,:,i));
end
save imagePointMatrix.mat imagePointMatrix
end
 function drawmultiview()
  load RotateMatrix.mat
 load TransMatrix.mat
 load('teapot.mat');
 teapot1 = teapot(1:10:end,:) + [0 0 600]; % Z>600
 teapotMatrix = zeros(size(teapot1, 1), 3, 10);
for i = 1:10
    loc = TransMatrix(i,:);
    ori = rotateMatrix(:,:,i);
    teapotMatrix(:,:,i) = teapot1*ori' + loc;
    color = i/10*[1 0 0];
    plotCamera('Location',loc, 'Orientation', ori,'Size', 20,...
     'label', int2str(i), 'color', color, 'AxesVisible', false);
    hold on
end
% save teapotMatrix.mat teapotMatrix
scatter3(teapot1(:,1), teapot1(:,2), teapot1(:,3), 5, 'MarkerFaceColor',[0 0 1],...
    'MarkerEdgeColor',[0.3,0.5,0.9]);
grid on 
xlabel('X[mm]');
ylabel('Y[mm]');
zlabel('Z[mm]');
 end
 