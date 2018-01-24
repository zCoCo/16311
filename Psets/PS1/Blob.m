%% Image Blob
% Basic Container Class for Storing Data on a Blob Detected in an Image
classdef Blob < handle
    %% PROPERTIES
    properties(GetAccess = public, SetAccess = private)
        %% Core Data:
        data; % Image Data of the Blob
        
        size; % Bounding Size of the Blob
        
        % Position of the Upper Left-Corner of the Blob in the Source Image
        pos = struct( ...
            'x', 0, ...
            'y', 0 ...
        );
        
        %% Extrapolated Properties:
        % Position of the Center of the Blob in the Source Image
        center = struct( ...
            'x', 0, ...
            'y', 0 ...
        );
    
        inertia; % Inertia Tensor
        
    end % Blob<-properties(public, private)
    
    %% METHODS
    methods(Access = public)
        %% Constructor
        % Constructs an Image Blob from the Matrix of a Rectangular Cut
        % Around the Blob, M, and the Coordinates of the Upper-Left Corner
        % in the source image.
        function obj = Blob(M, ulx, uly)
            obj.data = M;
            obj.pos.x = ulx;
            obj.pos.y = uly;
            
            obj.size = size(M);
            
            obj.center.x = ulx + floor(obj.size(1)/2);
            obj.center.y = uly + floor(obj.size(2)/2);
            
            obj.inertia = characterize_blob(M);
        end % Blob Constructor
    end % Blob<-methods(public)
end % class Blob
    