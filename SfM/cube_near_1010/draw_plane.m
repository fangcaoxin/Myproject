load nxyzPoints.mat
%scatter3(nxyzPoints(:,1), nxyzPoints(:,2), nxyzPoints(:, 3));
point3D  = zeros(55,3);
for i = 1:5:28
point3D(i,1) = nxyzPoints(5,1)-20;
point3D(i,2) = nxyzPoints(i+4,2);
point3D(i,3) = nxyzPoints(i+4, 3) + 20;
end

for i = 2:5:28
point3D(i,1) = nxyzPoints(5,1) - 15;
point3D(i,2) = nxyzPoints(i+3,2);
point3D(i,3) = nxyzPoints(i+3, 3) + 15;
end

for i = 3:5:28
point3D(i,1) = nxyzPoints(5,1) - 10;
point3D(i,2) = nxyzPoints(i+2,2);
point3D(i,3) = nxyzPoints(i+2, 3) + 10;
end

for i = 4:5:28
point3D(i,1) = nxyzPoints(5,1) - 5;
point3D(i,2) = nxyzPoints(i+1,2);
point3D(i,3) = nxyzPoints(i+1, 3) + 5;
end

for i = 5:5:28
point3D(i,1) = nxyzPoints(i,1);
point3D(i,2) = nxyzPoints(i,2);
point3D(i,3) = nxyzPoints(i, 3);
end

point3D(31:35,:) = point3D(5:5:28,:) + randn(1,3) + [5 0 0];
point3D(36:40,:) = point3D(4:5:28,:) + randn(1,3) + [15 0 0];
point3D(41:45,:) = point3D(3:5:26,:) + randn(1,3) + [25 0 0];
point3D(46:50,:) = point3D(2:5:26,:) + randn(1,3) + [35 0 0];
point3D(51:55,:) = point3D(1:5:25,:) + randn(1,3) + [45 0 0];
point3D(26, 2 ) = point3D(26, 2) + 3;
point3D(27, 2 ) = point3D(27, 2) + 3;
point3D(28, 2 ) = point3D(28, 2) + 3;
point3D(26, 3 ) = point3D(26, 3) - 5;
point3D(27, 3 ) = point3D(27, 3) - 5;
point3D(28, 3 ) = point3D(28, 3) - 5;
%scatter3(nxyzPoints(31:5:55,1), nxyzPoints(31:5:55,2), nxyzPoints(31:5:55, 3));
scatter3(point3D(:,1), point3D(:,2), point3D(:,3));
xlabel('x-axis');
ylabel('y-axis');
zlabel('z-axis');
save point3D.mat point3D