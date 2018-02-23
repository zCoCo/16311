function Path_find(tX, tY)
% Keepout Radius:
keepout = 7 / 2.54; % inches

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
buff = ceil(keepout*res)+0.5; % Ensure Twice the value is Odd (width of mask)
%Create Circular Keepout Mask:
[mxs, mys] = meshgrid(1:2*buff, 1:2*buff);
mask = (mxs-buff-0.5).^2 + (mys-buff-0.5).^2;
mask = (mask<=buff^2);
%Pad Matrix:
pad = ceil(buff + 1);
padmat = zeros(wrows+2*pad, wcols+2*pad);
padmat(pad+1:end-pad, pad+1:end-pad) = mat;
%Move Mask over Obstacles:
for wrow = ((pad+1):(wrows+pad))
    for wcol = ((pad+1):(wcols+pad))
        if(mat(wrow-pad,wcol-pad) == 1)
            minX = wcol - buff + 0.5;
            maxX = wcol + buff - 0.5;
            minY = wrow - buff + 0.5;
            maxY = wrow + buff - 0.5;
            
            padmat(minY:maxY, minX:maxX) = padmat(minY:maxY, minX:maxX) | mask;
        end
    end
end
%Recover Matrix:
mat = padmat(pad+1:end-pad, pad+1:end-pad);


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
    
    % Propage a Wave from the Given Cell with initial value v
    function wave_from(x,y, v)
        val = v;
        Sx = x; Sy = y;
        
        ripple_from(Sx,Sy, val); % Only One of These
        val = val + 1;
        
        running = 1;
        while(running)
            ripple_from(Sx,Sy, val);
            [Sx, Sy] = find_seed_near(Sx,Sy, val);
            if(Sx == -1) % No Seeds Found
                val = val +  1;
                Sx = x; Sy = y;
                [Sx, Sy] = find_seed_near(Sx,Sy, val);
                if(Sx == -1) % Still No Seeds
                    running = 0; % Then we done.
                end
            end
        end
    end % #wave_from

    % Find the Seed for the Next Wavefront near the Given Cell with Value v
    function [Wx,Wy] = find_seed_near(x,y, v)
        % Valid Seed has Value v and at Least One Vacant Neighbor
        Wx = x; Wy = y;
        xdir = 1; ydir = 1; % X and Y Search Directions
        searching = 1;
        
        while(searching)
            Wx = Wx + xdir;
            if(Wx > wcols) %Search right until a wall is hit
                Wx = x-1; % Start Over
                xdir = -1; % And Search the Other Direction
            elseif(Wx < 1)
             % Start Over and Search the Next Row
                Wx = x;
                xdir = 1;
                Wy = Wy + ydir; % Down First
                if(Wy > wrows)
                    Wy = y-1; % Start Over
                    ydir = -1; % Search the Other Direction
                elseif(Wy < 1)
                    Wx = -1; Wy = -1; % No Seeds Found
                    searching = 0;
                end
            end
            if(searching)
                if(mat(Wy,Wx) == v)
                    N = neighbors(Wx,Wy);
                    num_zeros = sum(sum(~(N|0)));
                    if(num_zeros > 0)
                        searching = 0; % Search Done. Match Found.
                    end
                end
            end
        end
    end % #find_seed_near

    % Infect all Adjacent Vacant Cells (a ripple of the wave)
    function ripple_from(x,y, v)
        val = v + 1;
        % Infect:
        if(x+1 <= wcols)
            fill_cell(x+1,y, val); %Right
            if(y-1>0)
                fill_cell(x+1,y-1, val); %Upper Right
            end
            if(y+1<=wrows)
                fill_cell(x+1,y+1, val); %Lower Right
            end
        end
        if(x-1 > 0)
            fill_cell(x-1,y, val); %Left
            if(y-1>0)
                fill_cell(x-1,y-1, val); %Upper Left
            end
            if(y+1<=wrows)
                fill_cell(x-1,y+1, val); %Lower Left
            end
        end
        if(y-1 > 0)
            fill_cell(x,y-1, val); %Up
        end
        if(y+1 <= wrows)
            fill_cell(x,y+1, val); %Down
        end
    end% #ripple_from
    % Fill a Cell if Currently Vacant:
    function fill_cell(x,y, v)
        if( mat(y,x) == 0 )
            mat(y,x) = v;
        end
    end % #fill_cell
    
    % Returns all the (8-Point) Neighbors of the Given Cell
    function N = neighbors(x,y)
        N = mat(y-1:y+1,x-1:x+1);
    end
    
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

wave_from(targetx, targety, 2);

% Search through Filled-Matrix to find path, create waypoints. Store result
% in:
pause
figure
imagesc(mat)
%set(gca, 'YDir', 'Reverse')
%set(gca, 'XDir', 'Reverse')

[waypoint_xs, waypoint_ys] = Waypoints(mat, 10,10);

hold on
    plot(waypoint_xs, waypoint_ys, 'r')
hold off

% xzc .
end