classdef Gains

    properties
        kp
        ki
        kd
    end

    methods

        function obj = Gains(kp, ki, kd)
            obj.kp = kp;
            obj.ki = ki;
            obj.kd = kd;
        end

    end

end
