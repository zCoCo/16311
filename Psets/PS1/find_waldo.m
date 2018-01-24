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
    the_waldo = create_waldo_prototype();
    waldos = Blob.empty;
    for(b = blobs)
        if is_within_percent(b.il_ratio, 14, the_waldo.il_ratio) ...
        && is_within_percent(b.density, 5, the_waldo.density) ...
        && max(b.size) > 1.5 * max(the_waldo.size)
            waldos = [waldos b]; % Append
            continue;
        end
        if is_within_percent(b.inertia_lambda(1), 12, the_waldo.inertia_lambda(1)) ...
        && is_within_percent(b.inertia_lambda(2), 6.5, the_waldo.inertia_lambda(2)) ...
        && is_within_percent(b.density, 14.5, the_waldo.density)
            waldos = [waldos b]; % Append
            continue;
        end
    end % blobs
    
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
    
    figure();
    imshow(src_img);
    for(w = waldos)
        w.draw(0);
    end
    title('Indentified Waldos');
    
    
end % #find_waldo

% Returns whether the given test value, T, is within P percent of the
% desired value, D
function w = is_within_percent(T,P,D)
    w = (100*abs(T-D)/D < P);
end % #is_within_percent
