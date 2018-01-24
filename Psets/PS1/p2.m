% 16-311 Homework 1 Framework
% Written by Humphrey Hu 1/16/2014
% humhu@cmu.edu
% Hannah

% Connor Colombo
% cwcolomb

% Note: Use this file with waldo.png in your active directory
function [] = p2(image_path, output_path)
%% 2. Find Waldo!

% This part is up to you!

%% 3. File I/O

% First we open a file and get a file ID # with fopen(). 'w' specifies we
% want to write.
file_id = fopen(output_path, 'w');
if file_id == -1    % -1 is an invalid file ID and signals failure
    fprintf('File creation failed!\n');
    return;
end

% Now we can write to the file using fprintf (formatted print to file)
% fprintf uses a format specifier string, where %d represents an integer
% and %f represents a floating point number. You can then give fprintf the 
% value you want to substitute into the string as additional arguments.
%'\n' is the special character for newline (or the enter key), and '/t' 
% is the tab character.
a = 1;
b = 2.5;
fprintf('a = %d,\t b = %f\n', a, b);

% If the file ID # is given to fprintf, it will write into the file instead
% of the terminal.
fprintf(file_id, 'Hello world!\n');

% Finally when you're done, you should close the file with fclose()
fclose(file_id);