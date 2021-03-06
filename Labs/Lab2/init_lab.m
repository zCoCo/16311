% @file init_lab.m
% @brief Initializes the paths and other goodies for the Lab 2 course
% infrastructure.
%
% @author Oscar Bezi (bezi@cmu.edu)
% @since 19 January 2016
%--------------------------------------------------------------------------
function [] = init_lab()
global LEVEL_0_ENABLED;
global LEVEL_1_ENABLED;
global LEVEL_2_ENABLED;
global LEVEL_3_ENABLED;
global LEVEL_4_ENABLED;
% EDIT THIS
% You very likely do not want all 4 levels enabled at once: that's about 90
% images to run through, which would be exceedingly slow.  Turn on the ones
% you're working with, then rerun init_lab.
LEVEL_0_ENABLED = true;
LEVEL_1_ENABLED = true;
LEVEL_2_ENABLED = true;
LEVEL_3_ENABLED = true;
LEVEL_4_ENABLED = true;
% END EDIT 

% Set working directory, which the rest of the code needs to be properly
% set.
cd(fileparts(which(mfilename)));

% Add infrastructure to the path.
addpath(fullfile(pwd, '.', 'infrastructure'));

% Load in images
fprintf('Loading in images...\n');

%% Load Student Images
global student_filenames;
global student_distances;
global student_thresholds;

if LEVEL_0_ENABLED
    [student_filenames, student_distances, student_thresholds] = load_config('./student_images/');
else
    student_filenames = [];
    student_distances = [];
    student_thresholds = [];
end
fprintf('Loaded student images.\n');

%% Load TA Images
global ta_filenames;
global ta_distances;
global ta_thresholds;

ta_filenames = [];
ta_distances = [];
ta_thresholds = [];

enabled_levels = [];
if LEVEL_1_ENABLED
    enabled_levels = [enabled_levels, 1];
end

if LEVEL_2_ENABLED
    enabled_levels = [enabled_levels, 2];
end

if LEVEL_3_ENABLED
    enabled_levels = [enabled_levels, 3];
end

if LEVEL_4_ENABLED
    enabled_levels = [enabled_levels, 4];
end

for i=enabled_levels
    fname = fullfile('.', 'ta_images', sprintf('l%d', i));
    
    [f, d, t] = load_config(fname);
    ta_filenames = [ta_filenames; f];
    ta_distances = [ta_distances; d];
    ta_thresholds = [ta_thresholds; t];
end

fprintf('Loaded TA images.\n');

%% Finish
fprintf('Done!\n');
fprintf('Please make sure to call this again if you add more images!\n');

end