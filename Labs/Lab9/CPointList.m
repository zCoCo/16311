% List of CPoints, N-Dimensional Points Capable of Printing into to a
% C-Array or an Array of Points.
classdef CPointList
    properties (GetAccess = public, SetAccess = public)
        list;
        name;
    end % properties <- CPointList
    methods
        % Accepts a List of CPoints
        % Optionally, include a name for printing.
        function obj = CPoint(coords, name)
            obj.list = coords(1,:); % Extract First Dmimension from Errant Two-Dimensional Entries
            if nargin > 1
                obj.name = name;
            else
                obj.name = "";
            end
        end %% Constructor
        
        % Creates a String to Create this Point as an Array in C.
        % Optionally printing it to the console.
        function str = toC(obj, print)
            if strcmp(obj.name, "")
                id = inputname(1);
            else
                id = obj.name;
            end
            str = sprintf("float CP_%s[%d] = {", id, numel(obj.pos)) ...
                + sprintf("%0.5f, ", obj.pos(1:end-1)) + sprintf("%0.5f};", obj.pos(end));
            if(print)
                disp(str);
            end
        end
    end % methods <- CPointList
end %CPointList