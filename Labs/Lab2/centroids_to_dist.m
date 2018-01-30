% @file centroids_to_dist.m
% @brief Given a List of Centroids Positions for the (likely) Tennis Balls
% in an Image, this will Verify that all Candidates are Balls, Compute
% their Separation, and Use that Data to Compute the Distance to the Jig.
%
% @author Connor W. Colombo (cwcolomb)
%--------------------------------------------------------------------------
function [dist] = centroids_to_dist(centroids, ball_separation, isTA)

% Set the focal length parameters found from calibrations
focalLength = 912.801; % Default Value
if isTA
    % DO NOT change the TA focal length
    focalLength = 920;
end

    %% Organize All Centroids Counter-Clockwise:
    center = [mean(centroids(:,1)), mean(centroids(:,2))]; % Center Point of the Point Cloud
    angs = []; % Vector of Angles to Each Centroid from the Center
    i = 1;
    while i<=length(centroids)
        Delta = (centroids(i,:) - center);
        angs = [angs; atan2(Delta(2), Delta(1))];
     i = i+1;
    end % i<length
    
    angs = angs - pi/2; % Rotate so 0rad is pointing down.
    angs = wrapToPi(angs); % Re-Wrap to Allow for Consistent Sorting
    
    % Append Angle Vector to Centroid Vector and Sort by Angle:
    centroids_sorter = [centroids, angs];
    centroids_sorter = sortrows(centroids_sorter, 3, 'descend');
    centroids = centroids_sorter(:,1:2);
    
    %% Get Length of Every Side:
    
    lengths = [];
    i = 1;
    while i<length(centroids)
        lengths(i) = norm( centroids(i,:) - centroids(i+1,:) );
     i = i+1;
    end % i<length
    lengths(i) = norm( centroids(end, :) - centroids(1, :) ); % Edge Case.
    
    %% Toss Out Outliers and Get Mean Side-Length:
    u_raw = mean(lengths); % Raw Mean Length (before tossing outliers)
    side_length = u_raw; % Default Value
    
    sum_len = 0; % Sum of All Valid Lengths
    N_len = 0; % Number of Valid Lengths
    
    i = 1;
    while i<=(length(lengths))
        %Toss Out Points Further than .25 L Longer than the mean
        if abs(lengths(i) - u_raw) <= 0.25*u_raw && lengths(i) > 0
            sum_len = sum_len + lengths(i);
            N_len = N_len + 1;
        end
     i = i+1;
    end % i<=length
    if N_len > 0 % Should always be true (assert?)
        side_length = sum_len / N_len;
    end
   
    %% Compute Distance!
    dist = ball_separation * focalLength / side_length;
end