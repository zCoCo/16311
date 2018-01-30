% @file threshold_image.m
% @brief Preprocess a given RGB Image Matrix (ie, from imread), turning it
% into a thresholded binary image based on which pixels have similar
% characteristics to those found in an image of a tennis ball.
%
% @author Connor W. Colombo (cwcolomb)
%--------------------------------------------------------------------------
function [output] = threshold_image(input_image, threshold)
    %% Parameters:
    tennis_ball_color = [69.87/360, 0.4365, 0.7098]; % Average HSV Color of the Tennis Ball
    
    %% Convert to HSV:
    hsv_image = double(rgb2hsv(input_image));
    % Get Rid of Low-S Regions (much of the Background):
    % 
    hsv_image = (hsv_image(:,:,2) > 0.14) .* hsv_image;
    
    %% Isolate Channels Windows:
    % Isolate all Pixels in each Channel which are within a Certain Window
    % of the "Tennis-Ball" Color Values.
    % Approx. 1.5-Sigma:
    window_width = 0.409; % Width of Window, Fractional (ie, 0.1 -> 10% tolerance is allowed).
    targets = permute(tennis_ball_color, [3 1 2]);
    
    keep = (hsv_image > ((1-window_width)*targets)) & (hsv_image < ((1+window_width)*targets));
    hsv_image = keep .* hsv_image;
    
    %% Noise Reduction and Jig Isolation:
    % Fork the Image Processing; Perform Extreme Blur on the Saturation 
    % Channel to Clump all the Noise Together in Large, Low-Sat. Chunks.
    % Identify all Pixels in Those Chunks and Remove Them from the Image.
    % This has the nice side-effect of highlighting the jig.
    
    % Define Gaussian Distribution:
    sig = 12; %Sigma of Gaussian Distributions
    G = @(x,y) (1 / (2*pi*sig^2)) .* exp( - (x.^2 + y.^2) ./ (2*sig^2) );
    
    % Generate Kernel for Gaussian:
    x = (-2:1/sig:2)*sig;
    y = x.';
    kern = bsxfun(G,x,y);
    
    % Convolve:
    blur = conv2(kern, hsv_image(:,:,2));
    
    % Center-Crop Blur to Fit Original Image Size:
    [bh,bw,~] = size(blur);
    [ih,iw,~] = size(hsv_image);
    Dh2 = (bh - ih)/2; % Half the Difference in Height
    Dw2 = (bw - iw)/2; % Half the Difference in Width
    
    blur = blur(Dh2:end-Dh2-1, Dw2:end-Dw2-1);
    
    not_noise = (blur > 0.15) .* hsv_image;
    
    %% Greyscale:
    % Linearlly Combine the H, S, and V Channels of the Input Image using
    % the Channel Values of the Average Tennis Ball Color as Weights:
    channel_weights = permute(tennis_ball_color, [3 1 2]);
    
    grey = not_noise .* channel_weights; % Apply Weighting to Each Channel
    grey = sum(grey, 3); % Combine Channels to Produce Greyscale    
    grey = grey ./ sum((1+window_width)*tennis_ball_color.^2); % Normalize Image to Absolute Max. Possible Value

    %% Quasi-Gaussian Blur Output:
    % Performs similar blur effect as with Gaussian filter but, hopefully,
    % a bit faster (and self-made):
    % Define Gaussian Distribution:
    sig = 3; %Sigma of Gaussian Distributions (DIFFERENT SIGMA)
    G = @(x,y) (1 / (2*pi*sig^2)) .* exp( - (x.^2 + y.^2) ./ (2*sig^2) );
    
    % Generate Kernel for Gaussian:
    x = (-2:1/sig:2)*sig;
    y = x.';
    kern = bsxfun(G,x,y);
    % Convolve:
    gauss = conv2(kern, grey);
    
    %% Re-Normalize Image:
    gauss = gauss ./ max(max(gauss)); % Normalize Image to Local Maximum Value
    
    %% Threshold on the Combined Channels:
    post = double(gauss > threshold);% Areas More Tennis-Ball-Colored than Threshold are 1.

output = post;
end
