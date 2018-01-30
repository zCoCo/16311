% @file image_to_dist.m
% @brief Runs the Entire Pipeline to Determine the Distance to a Jig of 4
% Tennis-Balls in a Given input_image.
%
% @author Connor W. Colombo
%--------------------------------------------------------------------------
function [dist] = image_to_dist(input_image, threshold, ball_separation, isTA)
    thresh = threshold_image(input_image, threshold);
    segments = segment_image(thresh);
    centroids = calculate_centroids(segments);
    dist = centroids_to_dist(centroids, ball_separation, isTA);
end
