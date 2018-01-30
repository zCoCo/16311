% @file test_threshold.m
% @brief Tests the student thresholding code, displaying some thresholded
% images to be graded.
%
% @author Oscar Bezi (bezi@cmu.edu)
% @since 19 January 2016
%--------------------------------------------------------------------------
function [] = test_threshold(filename, threshold)

switch nargin
    case 2
        threshold_file_and_display(filename, threshold);
        return;
end;

global student_thresholds;
global student_filenames;

global ta_thresholds;
global ta_filenames;

%% Student Images
fprintf('Thresholding student images. . .\n');
for i = 1:size(student_filenames, 1)
    fprintf('Thresholding %s. . .  ', student_filenames{i});
    hFigure = threshold_file_and_display(student_filenames{i}, student_thresholds(i));
    fprintf('Done.\n');
    fprintf('Press any key to see next file.\n');
    pause;
    close(hFigure);
end
fprintf('  Done.\n');

%% TA Images
fprintf('Thresholding TA images. . .\n');
for i = 1:size(ta_filenames, 1)
    fprintf('Thresholding %s. . .  ', ta_filenames{i});
    hFigure = threshold_file_and_display(ta_filenames{i}, ta_thresholds(i));
    fprintf('Done.\n');
    fprintf('Press any key to see next file.\n');
    pause;
    close(hFigure);
end
fprintf('  Done.\n');
end
% @function threshold_file_and_display
% @brief Does what it says on the tin.
% @returns A handle to the figure that gets displayed.
function [hFigure] = threshold_file_and_display(filename, threshold)
img = imread(filename);
BW = threshold_image(img, threshold);
hFigure = imtool(BW);
set(hFigure,'NumberTitle','off','Name', filename);
end