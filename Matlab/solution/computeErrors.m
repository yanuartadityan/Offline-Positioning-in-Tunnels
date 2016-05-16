function [ errors ] = computeErrors( w_lut, w_tracedPoints )
%COMPUTEERRORS Summary of this function goes here
%   Detailed explanation goes here
    errors = zeros(1,10);
    for n=1:size(w_tracedPoints,2)
        errors(:,n) = sqrt(abs(w_tracedPoints(1,n)-w_lut(1,n))^2 + abs(w_tracedPoints(2,n)-w_lut(2,n))^2 + abs(w_tracedPoints(3,n)-w_lut(3,n))^2);
    end
end

