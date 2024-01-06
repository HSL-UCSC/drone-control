classdef (Abstract) Vehicle

    methods (Abstract)
        
        function obj = initialize(obj)
        end
        
        function mode = control_mode(obj)
        end

        % get vehicle state 12 vector
        function state = state(obj)
        end

        % set a position target, can be three or six?
        function set_position_target(obj, position)
        end
        % 
        
    end
end

