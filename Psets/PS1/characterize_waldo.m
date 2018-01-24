%%%%
% Function to Characterize a Template Waldo Image (stored at Waldo_Template.png)
% by its Inertia Matrix
%%%%
function I = characterize_waldo()
    img = imread('Waldo_Template.png');
    waldo = preprocess_image(img);
end % #characterize_waldo