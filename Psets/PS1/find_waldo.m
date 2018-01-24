function find_waldo(addr)
    %% Fetch Image:
    if nargin < 1
        addr = 'findWaldo.png';
    end
    
    src_img = imread(addr); % Get Source Image
    
    %% Process Image:
    proc_img = preprocess_image(src_img);
    
    %% Identify Blobs (Flood Fill):
    
    %% Identify Waldo Blobs:

    %% Display Outputs:
%     w = 2; h = 4;
    figure();
    imshow(src_img);
    title('Source Image');
    
    figure();
    imshow(proc_img);
    title('Preprocessed Image');
    
%     figure();
%     imshow(src_img);
%     title('Flood-Filled Image');
%     
%     figure();
%     imshow(src_img);
%     title('Identified Blobs');
%     
%     figure();
%     imshow(src_img);
%     title('Identified Waldos');
    
    
end % #find_waldo
