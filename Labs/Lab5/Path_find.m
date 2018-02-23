function Path_find(tX, tY)
% Keepout Radius:
keepout = 6 / 2.54; % inches

% Resolution (grid cells per inch)
res = 1; %Pick 4 Because quarter inches are easier

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


%% PROCEDURE:

%% Create Obstacles:
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
            % Doesn't double fill a point
            if mat(wrow,wcol) ~= 1
                mat(wrow,wcol) = isIn(objectList(row),x,y);
            end
        end
    end
end

% Add Edge-Boundary to Map
mat(:,1) = 1;
mat(1,:) = 1;
mat(:,wcols) = 1;
mat(wrows,:) = 1;

%% Add Keepout Buffer around All Obstacles:
buff = ceil(keepout * res);
matref = mat;
    % Bound: Ensures value v is inclusively within lower bounds vl and upper bounds vu.
    function vo = bound(v, vl,vu)
        if (v>vl && v<vu)
            vo = v;
        elseif (v<vl)
            vo = vl;
        else
            vo = vu;
        end
    end % #bound
for wrow = 1:wrows
    for wcol = 1:wcols
        if(matref(wrow,wcol) == 1)
            minX = bound(wcol - buff, 1, wcols);
            minY = bound(wrow - buff, 1, wrows);
            idxs = (1:(2 * buff)); % identify max bounds for masking region
            for i = idxs
                for j = idxs
                    dist = sqrt( (i-buff)^2 + (j-buff)^2 );
                    if dist <= buff
                        matref(minY + i, minX + j) = 1;
                    end
                end
            end
        end
    end
end

%% Create Wavefronts:

% Identify target in Matrix
%Set target coordinates
targetx = tX * res;
targety = tY * res;
 
mat(targety,targetx) = 2;

start_x = 12 * res;
start_y = 42 * res;

figure
imagesc(mat)

% Perform Wavefront Variant of Flood-Fill on Matrix (as given in slides)
% 8-point connectivity
    %% Fill From
    % Recursive Fill of Region Connected (8-point) to the Seed Point (x,y)
    % with the Value, val
    function fill_from(x,y, v)
        val = v;
        if(mat(y,x) == 0 || mat(y,x) == 2)
            mat(y,x) = val; % Fill
            val = v + 1;
            % Infect:
            if(x+1 <= wcols)
                fill_from(x+1,y, val); %Right
                if(y-1>0)
                    fill_from(x+1,y-1, val); %Upper Right
                end
                if(y+1<=wrows)
                    fill_from(x+1,y+1, val); %Lower Right
                end
            end
            if(x-1 > 0)
                fill_from(x-1,y, val); %Left
                if(y-1>0)
                    fill_from(x-1,y-1, val); %Upper Left
                end
                if(y+1<=wrows)
                    fill_from(x-1,y+1, val); %Lower Left
                end
            end
            if(y-1 > 0)
                fill_from(x,y-1, val); %Down
            end
            if(y+1 <= wrows)
                fill_from(x,y+1, val); %Up
            end
        end
    end % #fill_from

fill_from(targetx, targety, 2);

% Search through Filled-Matrix to find path, create waypoints. Store result
% in:
pause
figure
imagesc(mat)
%set(gca, 'YDir', 'Reverse')
%set(gca, 'XDir', 'Reverse')

[waypoint_xs, waypoint_ys] = Waypoints(mat, 10,10);

hold on
    plot(waypoint_xs, waypoint_ys)
hold off

% xzc .
end