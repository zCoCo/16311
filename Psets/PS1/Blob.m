%% Image Blob
% Basic Container Class for Storing Data on a Blob Detected in an Image
classdef Blob < handle
    %% PROPERTIES
    properties(GetAccess = public, SetAccess = private)
        %% Core Data:
        data; % Image Data of the Blob
        
        size; % Bounding Size of the Blob
        width; % Bounding-Box Width of the Blob
        height; % Bounding-Box Height of the Blob
        
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
    
        cloudXs; % X-Coordinates of All Points in the Point-Cloud with Origin at the Blob's Center
        cloudYs; % Y-Coordinates of All Points in the Point-Cloud with Origin at the Blob's Center
        
        inertia_lambda; % Root of the Eigenvector of the Interia Tensor. General Descriptor of Shape / Distribution of Mass.
        
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
            obj.width = obj.size(2);
            obj.height = obj.size(1);
            
            obj.center.x = ulx + floor(obj.width/2);
            obj.center.y = uly + floor(obj.height/2);
            
            Ls = find(M == 1);
            obj.cloudXs = mod(Ls, obj.width) - floor(obj.width/2);
            obj.cloudYs = floor(Ls ./ obj.width) - floor(obj.height/2);
            
            Ixx = obj.cloudXs * obj.cloudXs';
            Iyy = obj.cloudYs * obj.cloudYs';
            Ixy = - obj.cloudXs * obj.cloudYs';
            inertia = [Ixx Ixy; Ixy Iyy] / length(Ls);
            obj.inertia_lambda = sqrt(eig(inertia));
        end % Blob Constructor
        
        %% Draw
        % Draw the Bounding Box for the Blob on the Active Set of Axes.
        function draw(obj)
            hold on
                ulx = obj.pos.x - 1;
                lrx = ulx + obj.width;
                uly = obj.pos.y - 1;
                lry = uly + obj.height;
                
                xs = [ulx lrx lrx ulx ulx];
                ys = [uly uly lry lry uly];
                
                plot(xs,ys, 'r');
            hold off
        end % #draw
    end % Blob<-methods(public)
end % class Blob
    