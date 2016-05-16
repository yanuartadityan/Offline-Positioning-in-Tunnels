function [ camerapoints ] = computeTransformPixel2Camera( K, pixelpoints )
%COMPUTETRANSFORMPIXEL2CAMERA Summary of this function goes here
%   Detailed explanation goes here
    camerapoints = zeros(3,10);
    for n = 1:size (pixelpoints,1)
        camerapoints(:,n) = K \ [pixelpoints(n,:) 1].';
    end
end

