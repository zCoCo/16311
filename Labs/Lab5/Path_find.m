% Wavefronts

% Resolution (grid cells per inch)
res = 3;
% Initialize Matrix Representing the Grid of the World:
width = 84; %inches
height = 48;
mat = zeros(height * res, width * res);

%Obstacles Coordinates:
P1x = [1];
P1y = [31.5];
P2x = [5.25];
P2y = [36];
P3x = [22.5];
P3y = [19.5];
P4x = [18.25];
P4y = [15.25];



obstacles; % Vector of Rect_Obj representing obstacles

%% PROCEDURE:

% Loop through all P0s, P1s and Create a Rect_Obj entry in Obstacle vector for each
rows = length(P1x);
% magic number for amount of objects
objectList = [object1,object2,object3,object4,object5,object6];

for row = 1:rows
    objectList(row) = Rect_obj;
    objectList(row).X1 =  P1x(row);
    objectList(row).Y1 =  P1y(row);
    objectList(row).X2 =  P2x(row);
    objectList(row).Y2 = P2y(row);
    objectList(row).X3 = P3x(row);
    objectList(row).Y3 = P3y(row);
    objectList(row).X4 = P4x(row);
    objectList(row).Y4 = P4y(row);
    
end

% Initialize World Matrix
[wrows,wcols] = size(mat);

% Loop Through Matrix and Test Each Coordinate (use res to convert i,j to
% inches) against each entry in obstacle. If any of them return 1, set
% pixel to 1.

for row = 1:wrows
    for col = 1:wcols
        
        
    end
end

% Identify target in Matrix (with a 2, I think)

% Perform Wavefront Variant of Flood-Fill on Matrix (as given in slides)
% 8-point connectivity?

% Search through Filled-Matrix to find path, create waypoints. Store result
% in:
waypoint_xs;
waypoint_ys; % vectors


% xzc .