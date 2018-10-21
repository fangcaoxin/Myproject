function [R_est, t_est] = R_t_estimator_3d(bearing_vector, d3_points, vertical)
   ro = bearing_vector(:, 1:3);
   xs = bearing_vector(:, 4:6);
   X = d3_points;
   U = [ro(:, 3).*X(:,1)-ro(:, 2).*X(:,1) ...
        ro(:, 3).*X(:,2)-ro(:, 2).*X(:,2) ...
        ro(:, 3).*X(:,3)-ro(:, 2).*X(:,3) ...
        ro(:, 1).*X(:,1)-ro(:, 3).*X(:,1) ...
        ro(:, 1).*X(:,2)-ro(:, 3).*X(:,2) ...
        ro(:, 1).*X(:,3)-ro(:, 3).*X(:,3) ...
        ro(:, 2).*X(:,1)-ro(:, 1).*X(:,1) ...
        ro(:, 2).*X(:,2)-ro(:, 1).*X(:,2) ...
        ro(:, 2).*X(:,3)-ro(:, 1).*X(:,3) ...
        ro(:, 3)-ro(:, 2) ...
        ro(:, 1)-ro(:, 3) ...
        ro(:, 2)-ro(:, 1)];
 [R_est, t_est] = Rt_estimate(U, 0, vertical);
 
end

function [R_est, t_est] = Rt_estimate(U, mark, vertical)
   U = cast(U, 'double');
    [v,lambda]=eig(U'*U);
    if (vertical)
       g=v(:,2);
    else
       g =v(:,1);
    end
    k=sqrt(g(1)^2+g(2)^2+g(3)^2);
    g0 = g/k;
    if mark == 1
    g0 = -g/k;
    end
    g = lagrange_pnp(U,g0); 
   % g = g0;
 
  g(1)^2+g(2)^2+g(3)^2
  g(4)^2+g(5)^2+g(6)^2
  g(7)^2+g(8)^2+g(9)^2
  R1=[g(1) g(2) g(3)]; 
	R2=[g(4) g(5) g(6)]; 
	R3=[g(7) g(8) g(9)];
	R_p=[R1;R2;R3];
  t_p = [g(10); g(11); g(12)];
  R_est = R_p';
  t_est = -R_p'*t_p;
  t_est = t_est';
% t_est = t_err;
end