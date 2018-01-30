% @file verify_config.m
% @brief Verifies the validity of the student's images.dat configuration
% file.
%
% @author Oscar Bezi (bezi@cmu.edu)
% @since 19 January 2016
%--------------------------------------------------------------------------
function [] = verify_config()
[filenames, ~, ~] = load_config('./student_images');
fprintf('Config file has %d files.\n', size(filenames, 1));
filenames
end