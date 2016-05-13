function [ w_points, inverse_T] = computeTransformCamera2World( r, t, c_points )
%COMPUTETRANSFORMCAMERA2WORLD Summary of this function goes here
%   Detailed explanation goes here
    inverse_T = [inv(r) -inv(r)*t; 0 0 0 1];
    w_points = zeros(4, size(c_points, 2));
    for n = 1:size(c_points,2)
        w_points(:,n) = inverse_T * [c_points(:,n); 1];
    end
end

