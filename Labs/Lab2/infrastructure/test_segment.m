% @file test_segment.m
% @brief Tests the student segmenting code, displaying some segmented
% images with overlaid centroids to be graded.
%
% @author Oscar Bezi (bezi@cmu.edu)
% @since 19 January 2016
%--------------------------------------------------------------------------
function [] = test_segment(filename, threshold)

switch nargin
    case 3
        segment_file_and_display(filename, threshold);
        return;
end;

global student_thresholds;
global student_filenames;

global ta_thresholds;
global ta_filenames;

%% Student Images
fprintf('Segmenting student images. . .\n');
for i = 1:size(student_filenames, 1)
    fprintf('Segmenting %s. . .  ', student_filenames{i});
    hFigure = segment_file_and_display(student_filenames{i}, student_thresholds(i));
    fprintf('Done.\n');
    fprintf('Press any key to see next file.\n');
    pause;
    close(hFigure);
end
fprintf('  Done.\n');

%% TA Images
fprintf('Segmenting TA images. . .\n');
for i = 1:size(ta_filenames, 1)
    fprintf('Segmenting %s. . .  ', ta_filenames{i});
    hFigure = segment_file_and_display(ta_filenames{i}, ta_thresholds(i));
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
function [hFigure] = segment_file_and_display(filename, threshold)
img = imread(filename);
BW = threshold_image(img, threshold);
segmented = segment_image(BW);
centroids = calculate_centroids(segmented);

radius = 10;
circlePositions = [centroids (radius * ones(size(centroids, 1), 1))];

labeled = insertShape(segmented, 'circle', circlePositions, 'LineWidth', 10, 'color', 'Red');
hFigure = imtool(labeled);
set(hFigure,'NumberTitle','off','Name', filename);
end
