% @file load_config.m
% @brief Loads in the images.dat configuration file.
%
% @author Oscar Bezi (bezi@cmu.edu)
% @since 19 January 2016
%--------------------------------------------------------------------------
function [filenames, distances, thresholds] = load_config(dir_name)
if ~exist(dir_name, 'dir')
    error('Error: Unable to find ''%s''.', dir_name);
end

fname = fullfile(dir_name, 'images.dat');
if ~exist(fname, 'file')
    error('Error: Unable to find ''%s''.', fname);
end

fileID = fopen(fname);
raw_input = textscan(fileID, '%s %d %f', 'commentStyle', '%');
fclose(fileID);

% Prepend the directory name.
filenames = cellfun(@(x) fullfile(dir_name, x), ...
    raw_input{1}(:), ...
    'UniformOutput', false);

% Validate all files.
for i=1:size(filenames, 1)
   if ~exist(filenames{i}, 'file')
       error('Error: Unable to find file %s', filenames{i});
   end
end

distances = raw_input{2}(:);
thresholds = raw_input{3}(:);
end