% Wavefronts

% Resolution (grid cells per inch)
res = 3;
% Initialize Matrix Representing the Grid of the World:
width = 84; %inches
height = 48;
mat = zeros(height * res, width * res);

%Obstacles Coordinates:
P0s = [1, 13.5; ]
P1s = [22.5, 19.5; ]; 
obstacles; % Vector of Rect_Obj representing obstacles

%% PROCEDURE:
f
% Loop through all P0s, P1s and Create a Rect_Obj entry in Obstacle vector for each

% Initialize World Matrix
% Loop Through Matrix and Test Each Coordinate (use res to convert i,j to
% inches) against each entry in obstacle. If any of them return 1, set
% pixel to 1.

% Identify target in Matrix (with a 2, I think)

% Perform Wavefront Variant of Flood-Fill on Matrix (as given in slides)
% 8-point connectivity?

% Search through Filled-Matrix to find path, create waypoints. Store result
% in:
waypoint_xs;
waypoint_ys; % vectors


% xzc .