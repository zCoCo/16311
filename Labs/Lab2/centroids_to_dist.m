% @file centroids_to_dist.m
% @brief INSERT YOUR DESCRIPTION HERE
%
% @author YOUR NAME (ANDREWID)
%--------------------------------------------------------------------------
function [dist] = centroids_to_dist(centroids, ball_separation, isTA)

% Set the focal length parameters found from calibrations
focalLength = 26/25.4;
if isTA
    % DO NOT change the TA focal length
    focalLength = 920;
end

% If distance between first two is << 0.5*perimeter, use just first two.

% Your code here.
dist = 100;
end