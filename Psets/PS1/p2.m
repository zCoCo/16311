% 16-311 Homework 1 Framework
% Written by Humphrey Hu 1/16/2014
% humhu@cmu.edu
% Hannah

% Connor Colombo
% cwcolomb

% NOTE: All arguments are OPTIONAL.
function [] = p2(ip, op)
%% Function Parameters:
    if(nargin<1)
        image_path = 'findWaldo.png';
        output_path = 'waldo_locations.txt';
    elseif(nargin<2)
        image_path = ip;
        output_path = 'waldo_locations.txt';
    else
        image_path = ip;
        output_path = op;
    end % nargin?
        
%% Find Waldos:

    waldos = find_waldo(image_path,0);
    
    figure();
    imshow(imread(image_path));
    for(w = waldos)
        w.draw(0);
    end
    title('Indentified Waldos');

%% 3. File I/O

% First we open a file and get a file ID # with fopen(). 'w' specifies we
% want to write.
file_id = fopen(output_path, 'w');
if file_id == -1    % -1 is an invalid file ID and signals failure
    fprintf('File creation failed!\n');
    return;
end

for(w = waldos)
    fprintf(file_id, '%d %d\n', w.center.x, w.center.y);
end % for waldos

% Finally when you're done, you should close the file with fclose()
fclose(file_id);