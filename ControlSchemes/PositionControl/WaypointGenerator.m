classdef WaypointGenerator < handle
    properties 
        waypoints = [0 0 0.7;
                     0.7 0.7 0.7;
                     -0.7 0.7 0.7;
                     -0.7 -0.7 0.7;
                     0.7 -0.7 0.7;]
        currWaypoint = 1;
        numWaypoints = 5;
    end 
    methods 
        function [x,y,z] = getWaypoint(obj)
            x = obj.waypoints(obj.currWaypoint,1);
            y = obj.waypoints(obj.currWaypoint,2);
            z = obj.waypoints(obj.currWaypoint,3);

        end

        function nextWaypoint = nextWaypoint(obj)
            obj.currWaypoint=obj.currWaypoint+1;

            % Wrap to front
            if(obj.currWaypoint > obj.numWaypoints)
                obj.currWaypoint = 1;
            end

        end

        function prevWaypoint = prevWaypoint(obj)
            obj.currWaypoint=obj.currWaypoint-1;

            % Wrap to back
            if(obj.currWaypoint < 1)
                obj.currWaypoint = obj.numWaypoints;
            end

        end
    end
end