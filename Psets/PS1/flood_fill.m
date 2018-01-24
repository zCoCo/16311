%%%%
% Function that Uses Flood Fill to Identify Objects in a Binary Image where
% Pixels with a Value of 1 are to be Considered for Object Candidacy.

%%%%
function out = flood_fill(img)
    [width, height] = size(img);
    out = img;
    
    no_seeds = 0; % No Viable Seeds Left
    
    obj_idx = 0; % Index of Object being Identified
    
    %% Main Loop:
    while(~no_seeds)
        [x, y, seeds_gone] = next_seed();
        if(~seeds_gone) % Seed Found.
            
            obj_idx = obj_idx + 1; % Increment to Next Object Index
            out(y,x) = obj_idx;
            
            fill_from(x,y, obj_idx); % Fill
            
        end % ~seeds_gone?
        no_seeds = seeds_gone;
    end % loop ~no_seeds?

    %% Fill From
    % Recursive Fill of Region Connected (4-point) to the Seed Point (x,y)
    % with the Object Index, idx.
    function fill_from(x,y, idx)
        if(img(y,x) == 1)
            out(y,x) = idx; % Fill

            % Infect:
            fill_from(x+1,y, idx); %Right
            fill_from(x-1,y, idx); %Left
            fill_from(x,y-1, idx); %Down
            fill_from(x,y+1, idx); %Up
        end
    end % #fill_from
    
    %% Number Adjacent Pixels:
    % Count Number of Adjacent Pixels in the Source Image of Value 1
    % (4-point connectivity) to the given point (x,y).
    function n = count_adj(x,y)
        n = ...
            (img(y-1,x) == 1) + ... %Up
            (img(y+1,x) == 1) + ... %Down
            (img(y,x+1) == 1) + ... %Right
            (img(y,x-1) == 1);      %Left
    end % #count_adj
        
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