function [ w_nearestNeighbour, w_distance ] = computeIntersection( inputXYZ, inputPC, rays, numOfNN)
%COMPUTEINTERSECTION Summary of this function goes here
%   Detailed explanation goes here
    w_distance = 0;
    for n=1:size(rays, 2)
        [ w_nearestNeighbors, w_nearestDistances] = findNearestNeighbors (inputPC, rays(1:3,n).', numOfNN);
        
        temp = sort(w_nearestDistances);
        
        if n == 1
            w_distance = temp(1);
        else
            if temp < w_distance
                w_nearestNeighbour = inputXYZ(w_nearestNeighbors(1),1:3);
                w_distance         = temp(1);
            end
        end
    end
end

