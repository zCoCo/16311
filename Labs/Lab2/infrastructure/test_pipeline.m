% @file test_pipeline.m
% @brief Tests the student pipeline code, comparing the final calculated
% distance to the values in the config files.
%
% @author Oscar Bezi (bezi@cmu.edu)
% @since 19 January 2016
%--------------------------------------------------------------------------
function [grade_info] = test_pipeline(filename, threshold, isTA)

switch nargin
    case 3
        grade_info = run_file_and_grade(filename, threshold, isTA);
        return;
end;

global student_thresholds;
global student_filenames;
global student_distances;

global ta_thresholds;
global ta_filenames;
global ta_distances;

%% Student Images
grade_info = '';
fprintf('Processing student images. . .\n');
for i = 1:size(student_filenames, 1)
    dist = run_file_and_grade(student_filenames{i}, student_thresholds(i), false);
    if student_distances(i) == 0
        fprintf('%s:\tCalculated: %d\n', student_filenames{i}, dist);
    else
        error = round(abs(100 * (student_distances(i) - dist) / student_distances(i)));
        fprintf('%s:\tCalculated: %d,\tExpected: %d,\t Error: %d%%\n', student_filenames{i}, dist, student_distances(i), error);
    end
end
fprintf('  Done.\n');

%% TA Images
fprintf('Processing TA images. . .\n');
for i = 1:size(ta_filenames, 1)
    dist = run_file_and_grade(ta_filenames{i}, ta_thresholds(i), true);
    if ta_distances(i) == 0
        fprintf('%s:\tCalculated: %d\n', ta_filenames{i}, dist);
    else
        error = round(abs(100 * (ta_distances(i) - dist) / ta_distances(i)));
        fprintf('%s:\tCalculated: %d,\tExpected: %d,\t Error: %d%%\n', ta_filenames{i}, dist, ta_distances(i), error);
    end
end
fprintf('  Done.\n');
end

% @function run_file_and_grade
% @brief Does what it says on the tin.
% @returns Runs a file through the student pipeline, then returns a
% struct of grade info.
function [dist] = run_file_and_grade(filename, threshold, isTA)
img = imread(filename);
if isempty(strfind(filename, 'small'))
    ball_separation = 12;
else
    ball_separation = 5;
end
dist = round(image_to_dist(img, threshold, ball_separation, isTA));
end