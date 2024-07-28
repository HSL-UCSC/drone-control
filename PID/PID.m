classdef PID < handle

    properties
        kp
        ki
        kd
        max
        min
        current_error
        previous_error
        cumulative_error
        
        v_current_error
    end

    methods

        function obj = PID(kp, ki, kd, min, max, sample_frequency, cutoff_frequency)

            arguments
                kp
                ki
                kd
                min
                max
                sample_frequency
                cutoff_frequency
            end

            obj.kp = kp;
            obj.ki = ki;
            obj.kd = kd;

            obj.current_error = 0;
            obj.previous_error = 0;
            obj.v_current_error = 0;
            obj.cumulative_error = 0;
            obj.min = min;
            obj.max = max;

            % obj.filter = Filter.lpf_2_init(sample_frequency, cutoff_frequency, 0.0);
        end

        function reset(obj)
            obj.current_error = 0;
            obj.previous_error = 0;
            obj.cumulative_error = 0;
            obj.v_current_error = 0;
            
        end
        
        function [target_acceleration, pid_outputs] = control(obj, target_state, current_state, dt)
            
            % Proportional
            obj.current_error = target_state - current_state;
            pid_p = obj.kp * obj.current_error;
            
            % Derivative
            derivative = (obj.current_error - obj.previous_error)/dt;
            pid_d = obj.kd * derivative;
            
            % TODO: saturate integral term
            %Integral
            obj.cumulative_error = obj.cumulative_error + obj.ki*dt*(obj.current_error);
            pid_i = obj.cumulative_error;

            % Store previous error
            obj.previous_error = obj.current_error;

            % Final PID result
            pid_outputs = [pid_p, pid_i, pid_d];
            target_acceleration = pid_p + pid_i + pid_d;
            target_acceleration = min(max(obj.min, target_acceleration), obj.max); 
        end
    end

end
