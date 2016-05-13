function [ w_tracedPoints, w_tracedDists ] = computeReprojection2Dto3D( inputXYZ, inputPC, rays, numNN)
%COMPUT Summary of this function goes here
%
%   First thing to do is build the ray, origin from the camera and traverse
%   through space from 1 to 50 meters ahead
%   
w_tracedPoints = zeros(3, size(rays,3));
w_tracedDists  = zeros(1,size(rays,3));
w_raysLong     = zeros(3,size(rays,3));

for i=1:size(rays,3)
    w_rayVector = rays(1:3,2,i)-rays(1:3,1,i);
    for j=1:1:50
        w_raysLong(1:3,j,i) = rays(1:3,1,i) +  w_rayVector * j;
    end
    
    %   once we got all the long projection rays then, we're ready
    [w_tracedPoints(:,i), w_tracedDists(i)] = computeIntersection (inputXYZ, inputPC, w_raysLong(:,:,i), numNN);
end

end

