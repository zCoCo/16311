%%%%
% Function that Uses Flood Fill to Identify Objects in a Binary Image where
% Pixels with a Value of 1 are to be Considered for Object Candidacy.

%%%%
function out = flood_fill(img)
    [height, width] = size(img);
    out = img;
    
    no_seeds = 0; % No Viable Seeds Left
    
    obj_idx = 0; % Index of Object being Identified
    
    %% Main Loop:
    while(~no_seeds)
        [x, y, seeds_gone] = next_seed();
        if(~seeds_gone) % Seed Found.
            
            obj_idx = obj_idx + 1; % Increment to Next Object Index
            
            fill_from(x,y, obj_idx+1); % Fill
            
        end % ~seeds_gone?
        no_seeds = seeds_gone;
    end % loop ~no_seeds?

    %% Fill From
    % Recursive Fill of Region Connected (8-point) to the Seed Point (x,y)
    % with the Value, val
    function fill_from(x,y, val)
        if(out(y,x) == 1)
            out(y,x) = val; % Fill

            % Infect:
            if(x+1 <= width)
                fill_from(x+1,y, val); %Right
                if(y-1>0)
                    fill_from(x+1,y-1, val); %Upper Right
                end
                if(y+1<=height)
                    fill_from(x+1,y+1, val); %Lower Right
                end
            end
            if(x-1 > 0)
                fill_from(x-1,y, val); %Left
                if(y-1>0)
                    fill_from(x-1,y-1, val); %Upper Left
                end
                if(y+1<=height)
                    fill_from(x-1,y+1, val); %Lower Left
                end
            end
            if(y-1 > 0)
                fill_from(x,y-1, val); %Down
            end
            if(y+1 <= height)
                fill_from(x,y+1, val); %Up
            end
        end
    end % #fill_from
        
    %% Find Next Seed:
    % Helper Function to Find the Next Position of the Next Seed 
    function [x, y, seeds_gone] = next_seed()
        seeds_gone = 0;
        
        x = 1; % = last_x
        y = 1; % = last_y
        while(y <= height)
            while(x <= width)
                if out(y,x) == 1
%                     last_x = x;
%                     last_y = y;
                    return;
                end
            x = x+1;
            end % x<=width
            x = 1; % Reset at end of row
        y = y+1;
        end % y<=height?
        
%         last_x = x;
%         last_y = y;
        seeds_gone = 1;
    end % #find_seed
end % end_floodfill