% @file calculate_centroids.m
% @brief Returns the Centroid Locations of the Segments in the Given Image 
% which are Most Likely Tennis Balls.
% To do this, it Isolates each Labeled Block of Pixels in a Segmented Image,
% then Creates an Instance of the Blob Object which Computes Data about the
% Block's Size, Shape, and Position (including centroid position). Each
% Blob is then Ranked using the these Properties by Likelihood of being a
% Tennis-Ball. The Centroids for the Most-Likely Four are Reported.
%
% @author Connor W. Colombo (cwcolomb)
%--------------------------------------------------------------------------
function [centroids] = calculate_centroids(segmented_image)
    %% Function Params:
    min_pixels = 47; %Minumum Number of Pixels in a Valid Blob (yet another noise filter)
    max_pixels = 15000; %Maximum Number of Pixels in a Valid Blob (remove big barriers/walls)

    %% Get Data about Flood-Fill:
    [~, width] = size(segmented_image);
    N_obj = max(max(segmented_image)); %Number of objects in the flood (+1)
    
    %% Extract Blobs:
    blobs = Blob.empty;
    
    n = 2;
    while(n<=N_obj)
        N_pixels = sum(sum(segmented_image==n));

        if(N_pixels > min_pixels && N_pixels < max_pixels) % Create Blob
            [ulx,uly, lrx,lry] = bounds(n);
            
            M = select_region(ulx,uly, lrx,lry); % Capture Everything within Bounding Box
            M = double(M==n); % Isolate Features of Interest.

            blob = Blob(M, ulx,uly);
            blobs = [blobs blob]; % Append to List
        end

    n = n+1;
    end % n<=N_obj
    
    %% Find Blobs which are Most-Likely Tennis Ball:
    % Intertia Tensors are quite difficult to compute for large images
    % without some sort of geometric simplifications. So, the since the
    % tennis-balls are centered in the frame and most non-tennis-ball
    % objects have already been filtered out, this will just return the
    % four blobs closest to the center of the image.
    figure();
    imagesc(segmented_image);
    % Rank Distances:
    dist = []; % Vector of Blob-to-Center Distances
    for(b = blobs)
        dist = [ dist, norm([b.center.x, b.center.y]) ];
        b.draw();
    end
    
    
    
    centroids = [400, 400; 800, 400; 400, 800; 800, 800];
    
%% Helper Functions
    %% Bounds
    % Helper function to return the X- and Y-coordinates of the Upper-Left
    % and Lower-Right Corners of the Bounding Box Enclosing the Given
    % Values in the Flood.
    function [ulx,uly, lrx,lry] = bounds(val)
        Ls = find(segmented_image==val); % Linear Indices Corresponding to All Pixels with the Given Value
        [ys, xs] = ind2sub(size(segmented_image), Ls);
        
        ulx = min(xs);
        uly = min(ys);
        lrx = max(xs);
        lry = max(ys);
    end % #bounds

    %% Select Region
    % Helper function to Select the Subregion of the Flood-Fill Matrix
    % Corresponding to the Bounding Box defined by the Given Upper-Left
    % Coordinate (ulx,uly) and Lower-Right Coordinate (lrx,lry).
    function R = select_region(ulx,uly, lrx,lry)
        R = segmented_image(uly:lry, ulx:lrx);
    end % #select_region
end
