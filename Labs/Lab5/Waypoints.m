function [waypoint_xs, waypoint_ys] = Waypoints(mat, start_x, start_y)
        
[wrows,wcols] = size(mat);

search_x = start_x;
search_y = start_y;

waypoint_xs = [start_x];
waypoint_ys = [start_y]; % vectors

cur_dir = 0;
next_dir = 0;

while mat(search_y,search_x) ~= 2 && mat(search_y,search_x) ~= 3
    curr_num = mat(search_y,search_x);
        fprintf('search_points')
        disp([search_x,search_y])
        north = search_y + 1
        east = search_x +1
        south = search_y - 1
        west = search_x-1
        
       %NORTH
        if search_y + 1 <= wrows
            if mat(search_y+1,search_x) == mat(search_y,search_x) - 1
                search_x = search_x ;
                search_y = search_y + 1;
                waypoint_xs = [waypoint_xs, search_x];
                waypoint_ys = [waypoint_ys, search_y]; 
                continue
            end
        end
        %North East
        if search_y + 1 <= wrows && search_x + 1 <= wcols
            if mat(search_y+1,search_x+1) == mat(search_y,search_x) - 1
                search_x = search_x + 1;
                search_y = search_y + 1;
                waypoint_xs = [waypoint_xs, search_x];
                waypoint_ys = [waypoint_ys, search_y]; 
                continue
            end
        end
        %EAST
        if search_x + 1 <= wcols
            if mat(search_y,search_x+1) == mat(search_y,search_x) - 1
                search_x = search_x + 1;
                search_y = search_y;
                waypoint_xs = [waypoint_xs, search_x];
                waypoint_ys = [waypoint_ys, search_y]; 
                continue
            end
        end
        % SOUTHEAST
        if search_y - 1 > 0 && search_x + 1 <= wcols
            if mat(search_y-1,search_x+1) == mat(search_y,search_x) - 1
                search_x = search_x + 1 ;
                search_y = search_y - 1;
                waypoint_xs = [waypoint_xs, search_x];
                waypoint_ys = [waypoint_ys, search_y]; 
                continue
            end
        end
        %SOUTH
        if search_y - 1 > 0
            if mat(search_y-1,search_x) == mat(search_y,search_x) - 1
                search_x = search_x ;
                search_y = search_y - 1;
                waypoint_xs = [waypoint_xs, search_x];
                waypoint_ys = [waypoint_ys, search_y]; 
                continue
            end
        end
        %SOUTHWEST
        if search_y - 1 > 0 && search_x - 1 > 0
            if mat(search_y-1,search_x-1) == mat(search_y,search_x) - 1
                search_x = search_x - 1;
                search_y = search_y - 1;
                waypoint_xs = [waypoint_xs, search_x];
                waypoint_ys = [waypoint_ys, search_y]; 
                continue
            end
        end
        %WEST
        if search_x - 1 > 0
            if mat(search_y,search_x-1) == mat(search_y,search_x) - 1
                search_x = search_x - 1;
                search_y = search_y;
                waypoint_xs = [waypoint_xs, search_x];
                waypoint_ys = [waypoint_ys, search_y]; 
                continue
            end
        end
        %NORTHWEST
        if search_y + 1 <= wrows && search_x - 1 > 0
            if mat(search_y+1,search_x-1) == mat(search_y,search_x) - 1
                search_x = search_x - 1 ;
                search_y = search_y + 1;
                waypoint_xs = [waypoint_xs, search_x];
                waypoint_ys = [waypoint_ys, search_y]; 
                continue
            end
        end     
end

disp(waypoint_xs)
disp(waypoint_ys)
end