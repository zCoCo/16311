function find_waldo(addr)
    %% Fetch Image:
    if nargin < 1
        addr = 'findWaldo.png';
    end
    
    src_img = imread(addr); % Get Source Image
    
    %% Process Image:
    proc_img = preprocess_image(src_img);
    
    %% Identify Objects (Flood Fill):
    flood = flood_fill(proc_img);
    
    %% Identify and Create Valid Blobs:
    blobs = floodfill2blobs(flood);
    
    %% Identify Waldo Blobs:
    

    %% Display Outputs:
%     w = 2; h = 4;
    figure();
    imshow(src_img);
    title('Source Image');
    
    figure();
    imshow(proc_img);
    title('Preprocessed Image');
    
    figure();
    imagesc(flood);
    title('Flood-Filled Image');
    
    figure();
    imagesc(flood);
    for(b = blobs)
        b.draw();
    end
    title('Recognized Objects');
    
%     figure();
%     imshow(src_img);
%     title('Identified Waldos');
    
    
end % #find_waldo
