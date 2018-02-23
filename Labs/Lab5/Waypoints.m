start_x = 1;
start_y = 1;
targetx = 5;
targety = 5;
mat = [9,8,7,6,6; ...
       8,1,1,1,5; ...
       7,1,1,1,4; ...
       6,1,1,1,3; ...
       6,5,4,3,2];
        
[wrows,wcols] = size(mat);

search_x = start_x;
search_y = start_y;

waypoint_xs = [start_x];
waypoint_ys = [start_y]; % vectors

while mat(search_x,search_y) ~= 2
    curr_num = mat(search_y,search_x);
        fprintf('search_points')
        disp([search_x,search_y])
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