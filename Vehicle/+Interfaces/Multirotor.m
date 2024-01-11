classdef (Abstract) Multirotor < Interfaces.Vehicle

    properties (Abstract)
        control_mode
    end

    methods (Abstract)
        arm()
        is_armed()
        shutdown()
    end

end

