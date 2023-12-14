classdef (Abstract) LocalizationInterface
    %Interface to a localization provider
    %   A localization provider is any source that can provide position data on a simulation entity.
    %   An example concrete implementation could be a motion capture camera system.
    
    properties
        client
        drone_id
    end
    
    methods (Abstract)

        function obj = init(obj)
        end

        function [t, x, y, z, pitch, roll, yaw] = getPosition(obj)
        end
        
    end
end

