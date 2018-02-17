classdef Rect_Obj
    properties
        X0; % Coords of Corner
        Y0;
        X1;
        Y1;
    end
    methods
        function obj = Rect_Obj(x0, y0, x1, y1)
            % initialize properties;
            obj.X0 = x0; //etc
        end
        
        %Returns 1 if the give coordinate is contained within
        function bool = isIn(obj, x,y)
            bool = 0;
            if(x> obj.X0) %or however this is done
                bool = 1;
            end
        end
    end
end