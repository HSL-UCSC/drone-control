classdef Waypoint
    properties
        x double
        y double
        z double
    end
    
    methods
        function obj = Waypoint(x, y, z)
            if nargin > 1
                obj.x = x;
                obj.y = y;
                obj.z = z;
            else
                obj.x = 0.0;
                obj.y = 0.0;
                obj.z = 0.0;
            end
        end
    end
    
end
