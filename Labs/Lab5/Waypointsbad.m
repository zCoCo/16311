function [waypoint_xs, waypoint_ys] = Waypointsbad(mat, start_x, start_y)
        
[wrows,wcols] = size(mat);

search_x = start_x;
search_y = start_y;

waypoint_xs = [start_x];
waypoint_ys = [start_y]; % vectors

cur_dir = 0;
next_dir = 0;

while mat(search_y,search_x) ~= 2 && mat(search_y,search_x) ~= 3
        %north = search_y + 1
        %east = search_x +1
        %south = search_y - 1
        %west = search_x-1
        north_dir = 1;
        northeast_dir = 2;
        east_dir = 3;
        southeast_dir = 4;
        south_dir = 5;
        southwest_dir = 6;
        west_dir = 7;
        northwest_dir = 8;
        viable = [];
        %NORTH
        if search_y + 1 <= wrows
            if 1 < mat(search_y+1,search_x) < mat(search_y,search_x)
                new_x = search_x ;
                new_y = search_y + 1;
                direct = north_dir;
                viable = [viable;new_x,new_y,mat(search_y+1,search_x),direct];
            end
        end
        %North East
        if search_y + 1 <= wrows && search_x + 1 <= wcols
            if 1 < mat(search_y+1,search_x+1) < mat(search_y,search_x)
                new_x = search_x + 1;
                new_y = search_y + 1;
                direct = northeast_dir;
                viable = [viable;new_x,new_y,mat(search_y+1,search_x+1),direct];
            end
        end
        %EAST
        if search_x + 1 <= wcols
            if 1 < mat(search_y,search_x+1) < mat(search_y,search_x)
                new_x = search_x + 1;
                new_y = search_y;
                direct = east_dir;
                viable = [viable;new_x,new_y,mat(search_y,search_x+1),direct];
            end
        end
        % SOUTHEAST
        if search_y - 1 > 0 && search_x + 1 <= wcols
            if 1 < mat(search_y-1,search_x+1) < mat(search_y,search_x)
                new_x = search_x + 1 ;
                new_y = search_y - 1;
                direct = southeast_dir;
                viable = [viable;new_x,new_y,mat(search_y-1,search_x+1),direct];
            end
        end
        %SOUTH
        if search_y - 1 > 0
            if 1 < mat(search_y-1,search_x) < mat(search_y,search_x)
                new_x = search_x ;
                new_y = search_y - 1;
                direct = south_dir;
                viable = [viable;new_x,new_y,mat(search_y-1,search_x),direct];
            end
        end
        %SOUTHWEST
        if search_y - 1 > 0 && search_x - 1 > 0
            if 1 < mat(search_y-1,search_x-1) < mat(search_y,search_x)
                new_x = search_x - 1;
                new_y = search_y - 1;
                direct = southwest_dir;
                viable = [viable;new_x,new_y,mat(search_y-1,search_x-1),direct];
            end
        end
        %WEST
        if search_x - 1 > 0
            if 1 < mat(search_y,search_x-1) < mat(search_y,search_x)
                new_x = search_x - 1;
                new_y = search_y;
                direct = west_dir;
                viable = [viable;new_x,new_y,mat(search_y,search_x-1),direct];
            end
        end
        %NORTHWEST
        if search_y + 1 <= wrows && search_x - 1 > 0
            if 1 < mat(search_y+1,search_x-1) < mat(search_y,search_x)
                new_x = search_x - 1;
                new_y = search_y + 1;
                direct = northwest_dir;
                viable = [viable;new_x,new_y,mat(search_y+1,search_x-1),direct];
            end
        end
        % See which point has the smallest value in viable
        [rows, cols] = size(viable);
        curr_smallest = 10000;
        small_index = [];
        fprintf('viable')
        disp(viable)
        for row = 1:rows
            for col = 1:cols
                if col == 3
                    %{
                    disp([row,col])
                    disp(viable(row,col))
                    %}
                    if viable(row,col) < curr_smallest
                        curr_smallest = viable(row,col);
                        next_dir = viable(row,col+1);
                        small_index = [viable(row,col-2),viable(row,col-1)];
                    end
                end
            end
        end
        %{
        fprintf('curr_smallest')
        disp(curr_smallest)
        fprintf('next_dir')
        disp(next_dir)
        fprintf('small_index')
        disp(small_index)
        %}
        % Keep or change current direction
        search_x = small_index(1);
        search_y = small_index(2);
        
        % Update waypoint of not on current direct
        if cur_dir ~= next_dir
            waypoint_xs = [waypoint_xs; search_x];
            waypoint_ys = [waypoint_ys; search_y];
        end
        cur_dir = next_dir;
        
end
end