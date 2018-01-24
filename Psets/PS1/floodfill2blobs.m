% Returns an Array of Blob Objects Contained within a Given Flood-Filled Image.
function blobs = floodfill2blobs(flood)
    %% Function Params:
    min_pixels = 16; %Minumum Number of Pixels in a Valid Blob

    %% Flood-Fill Data:
    [~, width] = size(flood);
    N_obj = max(max(flood)); %Number of objects in the flood (+1)
    
    %% Extract Blobs:
    blobs = Blob.empty;
    
    n = 2;
    while(n<=N_obj)
        N_pixels = sum(sum(flood==n));

        if(N_pixels > min_pixels) % Create Blob
            [ulx,uly, lrx,lry] = bounds(n);
            
            M = select_region(ulx,uly, lrx,lry); % Capture Everything within Bounding Box
            M = double(M==n); % Isolate Features of Interest.

            blob = Blob(M, ulx,uly);
            blobs = [blobs blob]; % Append to List
        end

    n = n+1;
    end % n<=N_obj
    
    %% Bounds
    % Helper function to return the X- and Y-coordinates of the Upper-Left
    % and Lower-Right Corners of the Bounding Box Enclosing the Given
    % Values in the Flood.
    function [ulx,uly, lrx,lry] = bounds(val)
        Ls = find(flood==val); % Linear Indices Corresponding to All Pixels with the Given Value
        [ys, xs] = ind2sub(size(flood), Ls);
        
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
        R = flood(uly:lry, ulx:lrx);
    end % #select_region
end % #floodfill2blobs