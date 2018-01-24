% Generates a Blob for the Prototypical Waldo
function waldo = create_waldo_prototype()
    img = imread('Waldo_Template.png');
    proc = preprocess_image(img);
    flood = flood_fill(proc);
    waldo = floodfill2blobs(flood);
end % #characterize_waldo