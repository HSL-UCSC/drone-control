classdef PositionController

    properties
        x_pid PID
        y_pid PID
        z_pid PID
    end

    methods

        function obj = PositionController(sample_frequency, cutoff_frequency, x_gains, y_gains, z_gains)

            arguments
                sample_frequency int = 60;
                cutoff_frequency int = 10;

                x_gains Gains = Gains(300, 15, 300);
                y_gains Gains = Gains(300, 15, 300);
                z_gains Gains = Gains(250, 30, 120);
           end

            obj.x_pid = PID(x_gains.kp, x_gains.ki, x_gains.kd, sample_frequency, cutoff_frequency);
            obj.y_pid = PID(y_gains.kp, y_gains.ki, y_gains.kd, sample_frequency, cutoff_frequency);
            obj.z_pid = PID(z_gains.kp, z_gains.ki, z_gains.kd, sample_frequency, cutoff_frequency);
        end

        function [ax, ay, az] = control(target_state, current_state, dt)

            [ax, ~] = obj.x_pid.control(target_state(1), current_state(1), dt);
            [ay, ~] = obj.y_pid.control(target_state(2), current_state(2), dt);
            [az, ~] = obj.z_pid.control(target_state(3), current_state(3), dt);
        end

    end

end
