function [ ] = p2( imageAddress )
%  Connor Colombo
%  cwcolomb
%
% 16-311 Homework 0
% Hannah
%
% This function takes in the address of a .png image and outputs three
% images.
%   Please display three versions of the image: normal RGB image, green
%   channel only (greyscale is fine), and one RGB image where every other column
%   (vertcal line in the image) is changed to black.

%% Setup

    close all % please keep this

%% Display RGB Image
    figure;
    img = imread(imageAddress);
    imshow(img);

%% Display Green Channel
    figure;
    imshow(img(:,:,2));

%% Display Black-Lined version
    figure;
    [Nrows, Ncols, Nchan] = size(img); % Get Image Size
    Cs = repmat( (1:Ncols), [Nrows,1] ); % Create Equally Sized 2D Matrix, R, where the Value of Each Element is its Column Number
    Cs = ~mod(Cs,2); % Create Stripe Pattern
    Cs = repmat(Cs,[1,1,Nchan]); % Repeat Filter across All Channels
    bs = uint8(img) .* uint8(Cs); % Apply Filter
    
    imshow(bs);
end