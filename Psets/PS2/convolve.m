% @file convolve.m
% @brief Performs Convolution of 2-Dimensional Matrices I and K, both of 
% arbitrary size.
% Ostensibly, I is a target image and K is a filter kernel; but,
% convolution is an associative operation, so the designation is merely for
% the user's sake and the function should perform the same if the operators
% are reversed.
%
% @author Connor W. Colombo (cwcolomb)
function C = convolve(I, K)
    %% Handle Sizes, Create "Infinite" Image
    % Product of Kernel is Placed at Top Left of the Kernel.
    % Handle Image Edges by Allowing K to Run up to the Edge of the Image
    % and Assuming that Any Values Required beyond the Image's Edge are
    % Zero (the infinite image space assumption).
    [H_I, W_I] = size(I);
    [H_K, W_K] = size(K);
    infinite_image = [I, zeros(H_I,W_K)];
    infinite_image = [infinite_image; zeros(H_K, W_I+W_K)];
    
    %% Convolve!
    % Loop Through Each Pixel of I inside the infinite_image, get the 
    % Pixels which Land within the Kernel Window, and convolve, placing the
    % result at the Top Left.
    C = ones(size(I)); % Pre-Allocate Space for Output Image to be Same Size as Input.
    
    i = 1;
    while(i<=H_I)
        j = 1;
        while(j<=W_I)
            R = infinite_image(i:(i + H_K - 1), j:(j + W_K - 1));
            C(i,j) = sum(sum(R .* K));
         j = j+1;
        end
     i = i+1;
    end
end % #convolve