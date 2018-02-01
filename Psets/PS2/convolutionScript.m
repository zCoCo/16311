%% Convolution
% 16-311 Introduction to Robotics 
% Homework 2: Part 2
% Hannah

% Connor W. Colombo
% cwcolomb

% TO DO: 
% Write a function called convolve(INPUTIMAGE, MASK)
% that takes in the image array for the specified image and specified
% masks.

% MAKE SURE THE FOLLOWING IS IN THE SAME DIRECTORY AS THIS SCRIPT: 
% image2.pgm (from handout)
% convolve.m (you make this)

% TO RUN THIS:
% Run this script with the image and function in this same folder

% WANT TO SEE:
% Four total figures. One normal, three with different masks.

% SUBMIT:
% Your convolve.m function and pictures of the convolved images with an
% explaination of which mask produced which image and the general effect of
% that mask. These pictures and words should be in the .pdf document for this
% homework.

%% Setup
close all
clc

%% Read in image

inputImageAddress = 'image2.pgm';
inputImageRead = imread(inputImageAddress);
inputImageDouble = double(inputImageRead); %easier to work with than uints
inputImage = inputImageDouble/255; %so we can use imshow

%% Display initial image

figure('Name','Initial Image');
imshow(inputImage);
title('Original input image (converted to double and divided by 255) using imshow');
colorbar;

%% Convolve image with Mask 1

mask1 = [-1 -1 -1; 2 2 2; -1 -1 -1];
convolvedImage1 = convolve(inputImage, mask1); % You must write this function
figure('Name','Image with Mask 1')
imshow(convolvedImage1);
title('Image Convolved with Mask 1')

%% Convolve image with Mask 2

mask2 = [-1 -1 -1 2 2 2 -1 -1 -1;-1 -1 -1 2 2 2 -1 -1 -1;-1 -1 -1 2 2 2 -1 -1 -1;-1 -1 -1 2 2 2 -1 -1 -1;-1 -1 -1 2 2 2 -1 -1 -1;-1 -1 -1 2 2 2 -1 -1 -1;-1 -1 -1 2 2 2 -1 -1 -1;-1 -1 -1 2 2 2 -1 -1 -1;-1 -1 -1 2 2 2 -1 -1 -1];
size(mask2)
convolvedImage2 = convolve(inputImage, mask2); % You must write this function
figure('Name','Image with Mask 2')
imshow(convolvedImage2);
title('Image Convolved with Mask 2')

%% Convolve image with Mask 3

almostMask3 = [1 1 1 1 1 1 1;1 1 1 1 1 1 1;1 1 1 1 1 1 1;1 1 1 1 1 1 1;1 1 1 1 1 1 1;1 1 1 1 1 1 1;1 1 1 1 1 1 1];
productOfDims = size(almostMask3,1)*size(almostMask3,2);
mask3 = (1/productOfDims).*almostMask3; % Normalize so that sum is 1
convolvedImage3 = convolve(inputImage, mask3); % You must write this function
figure('Name','Image with Mask 3')
imshow(convolvedImage3);
title('Image Convolved with Mask 3')

%% (Not necessary) Write images
%
% outputImage = convolvedImage3
% % .pgm is in uint and ranges from 255 to 0
% outputImageInRange = outputImage*255;
% outputImageUInt = uint8(outputImageInRange);
% 
% % Save the image to output.pgm
% imwrite(outputImageUInt, './output.pgm', 'pgm')