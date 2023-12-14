classdef (Abstract) CommunicationInterface
    %Interface to a localization provider
    %   A localization provider is any source that can provide position data on a simulation entity.
    %   An example concrete implementation could be a motion capture camera system.
    
    methods (Abstract)
        
        function obj = init(obj)
        end
        
        function [data, t] = read(obj, char)
        end
        
        function packet = sendPacket(obj, packet)
        end
        
    end
end

