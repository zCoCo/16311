% Wavefronts

% Resolution (grid cells per inch)
res = 4; %Pick 4 Because quarter inches are easier

% Initialize Matrix Representing the Grid of the World:
width  =  84; %inches
height =  48;
mat = zeros(height * res, width * res);

%Obstacles Coordinates from left to right objects and coords CCW from top
P1x = [01.00,  22.75,  47.50,  52.25,  68.00,  72.00];
P1y = [31.50,  30.25,  35.25,  18.75,  26.00,  42.50];
P2x = [05.25,  26.25,  51.50,  56.50,  72.75,  76.25];
P2y = [36.00,  34.50,  30.75,  14.75,  23.50,  38.25];
P3x = [22.50,  30.50,  34.00,  52.50,  69.00,  72.00];
P3y = [19.50,  30.50,  14.75,  10.50,  17.75,  34.00];
P4x = [18.25,  26.25,  29.75,  48.00,  64.25,  67.50];
P4y = [15.25,  26.25,  19.25,  14.25,  21.50,  38.25];



%obstacles; % Vector of Rect_Obj representing obstacles

%% PROCEDURE:

% Loop through all P0s, P1s and Create a Rect_Obj entry in Obstacle vector for each
rows = length(P1x);
objectList = Rect_Obj(0,0,0,0,0,0,0,0);

for row = 1:rows
    objectList(row) = Rect_Obj(P1x(row),P1y(row),P2x(row),P2y(row),P3x(row),P3y(row),P4x(row),P4y(row));
    
end

% Initialize World Matrix
%Creating the map of the world
% Loop Through Matrix and Test Each Coordinate (use res to convert i,j to
% inches) against each entry in obstacle. If any of them return 1, set
% pixel to 1.
[wrows,wcols] = size(mat);


for wrow = 1:wrows
    for wcol = 1:wcols
        x =  wcol/res;
        y =  wrow/res;
        for row = 1:rows
            % Check my positioning 
            if mat(wrow,wcol) ~= 1
                mat(wrow,wcol) = isIn(objectList(row),x,y);
            end
        end
    end
end


% Identify target in Matrix (with a 2, I think)
%Set target coordinates
targetx = 84 * res;
targety = 48 * res;
 
 mat(targetx,targety) = 2;
 
% Perform Wavefront Variant of Flood-Fill on Matrix (as given in slides)
% 8-point connectivity?

%{
while mat(1,1) ~= 0
    
    curr_x = targetx;
    curr_y = targety;
    
        % 8-point connectivity
        
        if mat(curr_x, curr_y + 1) == 0 && curr_y + 1 < wcols %North
            mat(curr_x, curr_y + 1) = mat(curr_x,curr_y) + 1;
        end 
        if mat(curr_x + 1, curr_y) == 0 && curr_x + 1 < wrows %East
            mat(curr_x + 1, curr_y) = mat(curr_x,curr_y) + 1;
        end
        if mat(curr_x, curr_y - 1) == 0 && curr_y - 1 > wcols %South
            mat(curr_x, curr_y - 1) = mat(curr_x,curr_y) + 1;
        end 
        if mat(curr_x - 1, curr_y) == 0 && curr_x - 1 > wrows %West
            mat(curr_x - 1, curr_y) = mat(curr_x,curr_y) + 1;
        end
        if mat(curr_x +1, curr_y + 1) == 0 && curr_y + 1 < wcols && curr_x + 1 < wrows %North-East
            mat(curr_x + 1, curr_y + 1) = mat(curr_x,curr_y) + 1;
        end
        if mat(curr_x + 1, curr_y - 1) == 0 && curr_y - 1 > wcols && curr_x + 1 < wrows%South-East
            mat(curr_x + 1, curr_y - 1) = mat(curr_x,curr_y) + 1;
        end
        if mat(curr_x - 1, curr_y - 1) == 0 && curr_y - 1 > wcols && curr_x - 1 > wrows%South-West
            mat(curr_x - 1, curr_y - 1) = mat(curr_x,curr_y) + 1;
        end
        if mat(curr_x - 1, curr_y + 1) == 0 && curr_y + 1 < wcols && curr_x - 1 > wrows%North-West
            mat(curr_x - 1, curr_y + 1) = mat(curr_x,curr_y) + 1;
        end

    mat(1,1) = 3;
    
end
%}

% Search through Filled-Matrix to find path, create waypoints. Store result
% in:
imagesc(mat)
set(gca, 'YDir', 'Reverse')
waypoint_xs = [];
waypoint_ys = []; % vectors


% xzc .