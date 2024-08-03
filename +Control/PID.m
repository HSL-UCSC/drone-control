classdef PID < handle
    
    properties
        kp
        ki
        kd
        saturate_min
        saturate_max
        current_error
        previous_error
        cumulative_error
        v_current_error
    end
    
    methods
        
        function obj = PID(kp, ki, kd, saturate_min, saturate_max, sample_frequency, cutoff_frequency)
            
            arguments
                kp
                ki
                kd
                saturate_min
                saturate_max
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
            obj.saturate_min = saturate_min;
            obj.saturate_max = saturate_max;
            % todo: add simpler digital LPF implementation, and add a filter parameter
            % obj.filter = Filter.lpf_2_init(sample_frequency, cutoff_frequency, 0.0);
        end
        
        function reset(obj)
            obj.current_error = 0;
            obj.previous_error = 0;
            obj.cumulative_error = 0;
            obj.v_current_error = 0;
            
        end
        
        function [output, component_outputs] = control(obj, target_state, current_state, dt)
            
            obj.current_error = target_state - current_state;
            pid_p = obj.kp * obj.current_error;
            
            obj.cumulative_error = obj.cumulative_error + obj.ki*dt*(obj.current_error);
            pid_i = obj.cumulative_error;
            
            derivative = (obj.current_error - obj.previous_error)/dt;
            pid_d = obj.kd * derivative;
            
            
            obj.previous_error = obj.current_error;
            
            % Final PID result
            component_outputs = [pid_p, pid_i, pid_d];
            output = pid_p + pid_i + pid_d;
            output = min(max(obj.saturate_min, output), obj.saturate_max);
        end
    end
    
end
