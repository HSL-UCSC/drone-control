classdef EventPath < Path.Path
    
    methods
        function obj = EventPath(waypoints)
            % todo: check last point is on the ground
            if nargin >= 1 && size(waypoints, 2) == 3
                obj.waypoints = waypoints;
            else
                obj.waypoints = [];
            end
        end
        
        function waypoints = get_waypoints(obj)
            waypoints = obj.waypoints;
        end

        function [x, y, z] = get_waypoint(obj, vehicle_state, event)
            [x, y, z] = deal(obj.waypoints(1, 1), obj.waypoints(1,2), obj.waypoints(1,3));
        end

        function [waypoint, jump] = waypoint(obj, vehicle_state, event)
            if isempty(obj.waypoints)
                waypoint = [];
            else
                waypoint = obj.waypoints(1,:);
                jump = 0;
                obj.waypoints(1,:) = [];
            end            
        end
    end
    
end


