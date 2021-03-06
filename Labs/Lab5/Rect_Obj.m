classdef Rect_Obj
    properties
        % Coordinates defined Clockwise
        X1; % Coords of Corner
        Y1;
        X2;
        Y2;
        X3;
        Y3;
        X4;
        Y4;
        a1;
        a2;
        a3;
        a4;
        A;
    end
    methods
        function obj = Rect_Obj(X1, Y1, X2, Y2, X3, Y3,X4,Y4)
            % initialize properties;
            obj.X1 = X1;
            obj.Y1 = Y1;
            obj.X2 = X2;
            obj.Y2 = Y2;
            obj.X3 = X3; 
            obj.Y3 = Y3;
            obj.X4 = X4;
            obj.Y4 = Y4;
            
            %pre-compute data
            obj.a1 = sqrt( (obj.X1-obj.X2)^2 + (obj.Y1-obj.Y2)^2);
            obj.a2 = sqrt( (obj.X2-obj.X3)^2 + (obj.Y2-obj.Y3)^2);
            obj.a3 = sqrt( (obj.X3-obj.X4)^2 + (obj.Y3-obj.Y4)^2);
            obj.a4 = sqrt( (obj.X4-obj.X1)^2 + (obj.Y4-obj.Y1)^2);
            %Area of Rectangle
            obj.A = obj.a1 * obj.a2;
        end
        
        %Returns 1 if the give coordinate is contained within
        function bool = isIn(obj, x,y)
            bool = 0;
            %Use the formula describe by this link
            %https://math.stackexchange.com/questions/190111/how-to-check-if-a-point-is-inside-a-rectangle
            % Side lengths
  
            %length of line segments
            b1 = sqrt( (obj.X1-x)^2 + (obj.Y1 -y)^2);
            b2 = sqrt( (obj.X2-x)^2 + (obj.Y2 -y)^2);
            b3 = sqrt( (obj.X3-x)^2 + (obj.Y3 -y)^2);
            b4 = sqrt( (obj.X4-x)^2 + (obj.Y4 -y)^2);
            %Heron's Formula
            u1 = (obj.a1+b1+b2)/2 ;
            u2 = (obj.a2+b2+b3)/2 ;
            u3 = (obj.a3+b3+b4)/2 ;
            u4 = (obj.a4+b4+b1)/2 ;
            % Area of the Triangles
            A1 = sqrt (u1*(u1-obj.a1)*(u1-b1)*(u1-b2));
            A2 = sqrt (u2*(u2-obj.a2)*(u2-b2)*(u2-b3));
            A3 = sqrt (u3*(u3-obj.a3)*(u3-b3)*(u3-b4));
            A4 = sqrt (u4*(u4-obj.a4)*(u4-b4)*(u4-b1));
            % Sum of the area of the rectangles
            areaRect = A1 + A2 + A3 + A4;
            %fprintf('areaRect')
            %disp(areaRect)
            buffer = 6;
            if(areaRect < obj.A + buffer && areaRect > obj.A - buffer) %or however this is done
                bool = 1;
            end
        end
    end
end