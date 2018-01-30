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

% Your code here.
dist = 100;
end