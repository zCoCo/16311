% @file Blob.m
% @brief Basic Container Class for Storing Data and Computing Geometric
% Information about a Blob of Pixels from a Flood-Filled Image.
% Necessary for calculate_centroids.
%
% @author Connor W. Colombo (cwcolomb)
%--------------------------------------------------------------------------
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
        
        inertia_lambda = [0 0]; % Root of the Eigenvector of the Interia Tensor. General Descriptor of Shape / Distribution of Mass.
        il_ratio = 1; % Ratio of the X Interia Lambda to Y Inertia Lambda
        
        density = 0; % Fraction of Bounding Box which is Occupied by Blob Pixels
        
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
            
            %{
            % Calculate Inertia (.: shape) Characteristics of Blob.
            % (not advisable if large blobs make it through)
            
            Ls = find(M == 1);
            [obj.cloudYs, obj.cloudXs] = ind2sub(size(M), Ls);
            
            Ixx = obj.cloudXs * obj.cloudXs';
            Iyy = obj.cloudYs * obj.cloudYs';
            Ixy = - obj.cloudXs * obj.cloudYs';
            inertia = [Ixx Ixy; Ixy Iyy] / length(Ls);
            obj.inertia_lambda = sqrt(eig(inertia));
            obj.inertia_lambda = obj.inertia_lambda(1:2);
            obj.il_ratio = obj.inertia_lambda(1) / obj.inertia_lambda(2);
            %}
            
            obj.density = length(find(M==1))/(obj.width * obj.height);
        end % Blob Constructor
        
        %% Draw
        % Draw the Bounding Box for the Blob on the Active Set of Axes.
        function draw(obj, dt)
            if nargin < 2
                data_text = 1;
            else
                data_text = dt;
            end
            hold on
                ulx = obj.pos.x - 1;
                lrx = ulx + obj.width + 1;
                uly = obj.pos.y - 1;
                lry = uly + obj.height + 1;
                
                cx = obj.center.x;
                cy = obj.center.y;
                
                xs = [ulx lrx lrx ulx ulx];
                ys = [uly uly lry lry uly];
                
                plot(xs,ys, 'r');
                if(data_text)
                    text(cx,cy-5, num2str(obj.inertia_lambda(1)), 'Color', 'white');
                    text(cx,cy, num2str(obj.inertia_lambda(2)), 'Color', 'white');
                    text(cx,cy+5, num2str(obj.il_ratio), 'Color', 'white');
                    text(cx,cy+10, num2str(obj.density), 'Color', 'white');
                end
            hold off
        end % #draw
    end % Blob<-methods(public)
end % class Blob
    