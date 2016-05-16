function drawImagePlane( K, inverse_T )
%DRAWIMAGEPLANE Summary of this function goes here
%   Detailed explanation goes here
% draw image plane
corner0 = K \ [0,0,1].';
corner1 = K \ [1280,0,1].';
corner2 = K \ [1280,960,1].';
corner3 = K \ [0,960,1].';

projectedcorner0 = inverse_T * [corner0; 1];
projectedcorner1 = inverse_T * [corner1; 1];
projectedcorner2 = inverse_T * [corner2; 1];
projectedcorner3 = inverse_T * [corner3; 1];

cornerpoints = [projectedcorner0 projectedcorner1 projectedcorner2 projectedcorner3];
x = cornerpoints(1, 1:4);
y = cornerpoints(2, 1:4);
z = cornerpoints(3, 1:4);
fill3 (x,y,z,'g');
labels = {'0,0','1280,960','0,960','1280,0'}; 
text (x,y,z, labels);
alpha(0.3);

end

