% Wavefronts

clear all
close all
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
            % Doesn't double fill a point
            if mat(wrow,wcol) ~= 1
                mat(wrow,wcol) = isIn(objectList(row),x,y);
            end
        end
    end
end

% Identify target in Matrix (with a 2, I think)
%Set target coordinates
targetx = 83 * res;
targety = 47 * res;
 
mat(targety,targetx) = 2;

curr_x = targetx;
curr_y = targety;

start_x = 12 * res;
start_y = 42 * res;
% Perform Wavefront Variant of Flood-Fill on Matrix (as given in slides)
% 8-point connectivity?

%{
 initialize:
    graph := {nodes}, {edges}
    The graph is array of zeros and ones
    fringe := {root}
    The fringe is the list, initially empty, of all nodes queued for expanding or examining.
    visited := empty
    The visited list is the list, initially empty, of all nodes already visited, which prevents the search from going in circles.

%}
graph = mat;
fringe = [targety, targetx;];
visited = [targety, targetx;];

figure
imagesc(mat)

while isempty(fringe) ~= 1
       [fringe_rows,fring_cols] = size(fringe);
       node_y = fringe(1,1);
       node_x = fringe(1,2);
       curr_y = node_y;
       curr_x = node_x;
       children = [];
       
       node_array = [node_y,node_x];
       if node_y == start_y && node_x == start_x
          break
       end
       children = [;];
       %do whatever you need to do to node here
       %children := find children of node in graph
       %NORTH
        if curr_y + 1 < wrows
            if mat(curr_y+1,curr_x) == 0
                mat(curr_y+1,curr_x) = mat(curr_y,curr_x) + 1;
                children = [children; [curr_y+1,curr_x]];
            end
        end
        %North East
        if curr_y + 1 < wrows && curr_x + 1 < wcols
            if mat(curr_y+1,curr_x+1) == 0
                mat(curr_y+1,curr_x+1) = mat(curr_y,curr_x) + 1;
                children = [children; [curr_y+1,curr_x+1]];
            end
        end
        %EAST
        if curr_x + 1 < wcols
            if mat(curr_y,curr_x+1) == 0
                mat(curr_y,curr_x+1) = mat(curr_y,curr_x) + 1;
                children = [children; [curr_y,curr_x+1]];
            end
        end
        % SOUTHEAST
        if curr_y - 1 > 0 && curr_x + 1 < wcols
            if mat(curr_y-1,curr_x+1) == 0
                mat(curr_y-1,curr_x+1) = mat(curr_y,curr_x) + 1;
                children = [children; [curr_y-1,curr_x+1]];
            end
        end
        %SOUTH
        if curr_y - 1 > 0
            if mat(curr_y-1,curr_x) == 0
                mat(curr_y-1,curr_x) = mat(curr_y,curr_x) + 1;
                children = [children; [curr_y-1,curr_x]];
            end
        end
        %SOUTHWEST
        if curr_y - 1 > 0 && curr_x - 1 > 0
            if mat(curr_y-1,curr_x-1) == 0
                mat(curr_y-1,curr_x-1) = mat(curr_y,curr_x) + 1;
                children = [children; [curr_y-1,curr_x-1]];
            end
        end
        %WEST
        if curr_x - 1 > 0
            if mat(curr_y,curr_x-1) == 0
                mat(curr_y,curr_x-1) = mat(curr_y,curr_x) + 1;
                children = [children; [curr_y,curr_x-1]];
            end
        end
        %NORTHWEST
        if curr_y + 1 < wrows && curr_x - 1 > 0
            if mat(curr_y+1,curr_x-1) == 0
                mat(curr_y+1,curr_x-1) = mat(curr_y,curr_x) + 1;
                children = [children; [curr_y+1,curr_x-1]];
            end
        end
       %{
       fprintf('children')
       disp(children)
       fprintf('visited')
       disp(visited)
       fprintf('fringe')
       disp(fringe)
       [childrenrows,childrencols] = size(children);
       [fringerows,fringecols] = size(fringe);
       fprintf('size children')
       disp([childrenrows,childrencols])
       fprintf('size fringe')
       disp([fringerows,fringecols])
       %}
       %add children not in visited to back of fringe
       if isempty(children) ~= 1
            children = setdiff(children, visited, 'rows');
       end
       %{
       fprintf('after diff children')
       disp(children)
       %}
       fringe = [fringe; children];
       %fprintf('fringe after cacatentation')
       %disp(fringe)
       
       %add node to visited
       if node_x ~= targetx && node_y ~= targety
            visited = [visited; node_array];
       end
       %remove node from fringe
       rowone = fringe(1,:);
       fringe = setdiff(fringe,rowone,'rows');
end

% Search through Filled-Matrix to find path, create waypoints. Store result
% in:
figure
imagesc(mat)
%set(gca, 'YDir', 'Reverse')
%set(gca, 'XDir', 'Reverse')

waypoint_xs = [];
waypoint_ys = []; % vectors


% xzc .