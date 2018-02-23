function [waypoint_xs, waypoint_ys] = Waypoints(mat, start_x, start_y)
        
[wrows,wcols] = size(mat);

search_x = start_x;
search_y = start_y;


waypoint_xs = [];
waypoint_ys = []; % vectors

%waypoint_xs = [start_x];
%waypoint_ys = [start_y]; % vectors

cur_dir = 0;
next_dir = 0;

while mat(search_y,search_x) ~= 2 && mat(search_y,search_x) ~= 3
        fprintf('search_points')
        disp([search_x,search_y])
        fprintf('curr_dir')
        (cur_dir)
        north_dir = 1;
        northeast_dir = 2;
        east_dir = 3;
        southeast_dir = 4;
        south_dir = 5;
        southwest_dir = 6;
        west_dir = 7;
        northwest_dir = 8;
        
       %NORTH
        if search_y + 1 <= wrows
            if mat(search_y+1,search_x) == mat(search_y,search_x) - 1
                next_dir = north_dir;
                if next_dir ~= cur_dir
                    waypoint_xs = [waypoint_xs; search_x];
                    waypoint_ys = [waypoint_ys; search_y];
                end
                search_x = search_x ;
                search_y = search_y + 1;
                cur_dir = next_dir;
                continue
            end
        end
        %North East
        if search_y + 1 <= wrows && search_x + 1 <= wcols
            if mat(search_y+1,search_x+1) == mat(search_y,search_x) - 1
                next_dir = northeast_dir;
                if next_dir ~= cur_dir
                    waypoint_xs = [waypoint_xs; search_x];
                    waypoint_ys = [waypoint_ys; search_y];
                end
                search_x = search_x + 1;
                search_y = search_y + 1;
                cur_dir = next_dir; 
                continue
            end
        end
        %EAST
        if search_x + 1 <= wcols
            if mat(search_y,search_x+1) == mat(search_y,search_x) - 1
                next_dir = east_dir;
                if next_dir ~= cur_dir
                    waypoint_xs = [waypoint_xs; search_x];
                    waypoint_ys = [waypoint_ys; search_y];
                end
                search_x = search_x + 1;
                search_y = search_y;
                cur_dir = next_dir; 
                continue
            end
        end
        % SOUTHEAST
        if search_y - 1 > 0 && search_x + 1 <= wcols
            if mat(search_y-1,search_x+1) == mat(search_y,search_x) - 1
                next_dir = southeast_dir;
                if next_dir ~= cur_dir
                    waypoint_xs = [waypoint_xs; search_x];
                    waypoint_ys = [waypoint_ys; search_y];
                end
                search_x = search_x + 1 ;
                search_y = search_y - 1;
                cur_dir = next_dir;
                continue
            end
        end
        %SOUTH
        if search_y - 1 > 0
            if mat(search_y-1,search_x) == mat(search_y,search_x) - 1
                next_dir = south_dir;
                if next_dir ~= cur_dir
                    waypoint_xs = [waypoint_xs; search_x];
                    waypoint_ys = [waypoint_ys; search_y];
                end
                search_x = search_x ;
                search_y = search_y - 1;
                cur_dir = next_dir; 
                continue
            end
        end
        %SOUTHWEST
        if search_y - 1 > 0 && search_x - 1 > 0
            if mat(search_y-1,search_x-1) == mat(search_y,search_x) - 1
                next_dir = southwest_dir;
                if next_dir ~= cur_dir
                    waypoint_xs = [waypoint_xs; search_x];
                    waypoint_ys = [waypoint_ys; search_y];
                end
                search_x = search_x - 1;
                search_y = search_y - 1;
                cur_dir = next_dir; 
                continue
            end
        end
        %WEST
        if search_x - 1 > 0
            if mat(search_y,search_x-1) == mat(search_y,search_x) - 1
                next_dir = west_dir;
                if next_dir ~= cur_dir
                    waypoint_xs = [waypoint_xs; search_x];
                    waypoint_ys = [waypoint_ys; search_y];
                end
                search_x = search_x - 1;
                search_y = search_y;
                cur_dir = next_dir; 
                continue
            end
        end
        %NORTHWEST
        if search_y + 1 <= wrows && search_x - 1 > 0
            if mat(search_y+1,search_x-1) == mat(search_y,search_x) - 1
                next_dir = northwest_dir;
                if next_dir ~= cur_dir
                    waypoint_xs = [waypoint_xs; search_x];
                    waypoint_ys = [waypoint_ys; search_y];
                end
                search_x = search_x - 1 ;
                search_y = search_y + 1;
                cur_dir = next_dir; 
                continue
            end
        end
        
        waypoint_xs = [waypoint_xs; search_x];
        waypoint_ys = [waypoint_ys; search_y]; 
        
end
        waypoint_xs = [waypoint_xs; search_x];
        waypoint_ys = [waypoint_ys; search_y];

end