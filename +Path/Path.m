classdef (Abstract) Path < handle
    
    properties
        waypoints;
    end
    
    methods (Abstract)
        waypoints = get_waypoints(obj)
        [waypoint, jump] = waypoint(obj, vehicle_state, event)
    end
    
end

