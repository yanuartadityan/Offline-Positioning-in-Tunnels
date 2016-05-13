%% init
A = [0 0 0; 1 1 0];
P = [2 3 0];

%% dot product
% A + dot(AP,AB) / dot(AB,AB) * AB
xyz  = [0 0 0] + (dot (P, A(2,:))/ dot (A(2,:), A(2,:))) * A(2,:);
Pxyz = [P; xyz]; 

%% plot
plot3 (A(:,1), A(:,2), A(:,3)); hold on;
line (Pxyz(:,1), Pxyz(:,2), Pxyz(:,3));

grid on;