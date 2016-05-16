function drawRays( rays, argText )
%DRAWRAYS Summary of this function goes here
%   Detailed explanation goes here
    for n = 1:size(rays,3)
        plot3 (rays(1,:,n), rays(2,:,n), rays(3,:,n)); hold on;
        text  (rays(1,2,n), rays(2,2,n), rays(3,2,n), sprintf('%s-%d', argText, n));
    end
end

