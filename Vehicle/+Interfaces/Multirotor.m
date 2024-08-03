classdef (Abstract) Multirotor < Interfaces.Vehicle

    properties (Abstract)
        control_mode
    end

    methods (Abstract)
        arm()
        disarm()
    end

end

