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
        function [x,y,z] = getWaypoint(obj, landing_flag, current_position)

            % Landing condition
            if (landing_flag)
                x = current_position.x;
                y = current_position.y;
                z = z_f - 0.10;
            else
                x = obj.waypoints(obj.currWaypoint,1);
                y = obj.waypoints(obj.currWaypoint,2);
                z = obj.waypoints(obj.currWaypoint,3);
            end

        end

        function waypoint = nextWaypoint(obj)
            obj.currWaypoint=obj.currWaypoint+1;

            % Wrap to front
            if(obj.currWaypoint > obj.numWaypoints)
                obj.currWaypoint = 1;
            end
            waypoint = obj.waypoints(obj.currWaypoint,:);
        end

        function waypoint = prevWaypoint(obj)
            obj.currWaypoint=obj.currWaypoint-1;

            % Wrap to back
            if(obj.currWaypoint < 1)
                obj.currWaypoint = obj.numWaypoints;
            end
            waypoint = obj.waypoints(obj.currWaypoint,:);
        end
    end
end