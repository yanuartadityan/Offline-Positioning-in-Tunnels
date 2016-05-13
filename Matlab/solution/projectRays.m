function [ rays] = projectRays( origin, points )
%PROJECTRAY Summary of this function goes here
%   Detailed explanation goes here
    rays = zeros(size(points,1), 2, size(points,2));
    for n = 1:size(points,2)
        rays(:,:,n) = [[origin; 1] points(:,n)];
    end
end

