classdef (Abstract) Multirotor < Interfaces.Vehicle

    properties (Abstract)
        control_mode
        mass_kg
    end

    methods (Abstract)
        arm()
        disarm()
    end

end

