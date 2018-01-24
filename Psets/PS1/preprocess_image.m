%%%%
% Preprocess a given RGB Image Matrix (fresh outta imread) to be Nicer to
% Handle
%%%%
function post = preprocess_image(pre)
    % Make Image Greyscale if not already:
    if(length(size(pre)) > 2)
        post = double(rgb2gray(pre));
    else
    % Already Greyscale
        post = double(pre);
    end % length(size(pre))
    
    post = post / 255; % Normalize
    
    %Threshold:
    thresh = 150/255;% mean(double(quantile(post,0.25))); % Toss anything below the first quartile of the distribution.s
%     
%     figure();
%     histogram(post);
%     title(strcat('Mid-Processing Histogram, threshold = ', num2str(thresh)));
    
    post = double(post < thresh);% Areas Darker than Threshold are 1.
end % #preprocess_image