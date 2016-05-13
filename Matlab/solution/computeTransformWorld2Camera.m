function [ camerapoints ] = computeTransformWorld2Camera( r, t, worldpoints )
%COMPUTETRANSFORMWORLD2CAMERA Summary of this function goes here
%   Detailed explanation goes here
    camerapoints = zeros(3,10);
    for n = 1:size(worldpoints,1)
       camerapoints(:,n) = r * worldpoints(n,:).' + t;
    end
end

