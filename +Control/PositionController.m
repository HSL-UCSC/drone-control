classdef PositionController
    
    properties
        x_pid
        y_pid
        z_pid
    end
    
    methods
        
        function obj = PositionController(sample_frequency, cutoff_frequency, x_gains, y_gains, z_gains)
            
            arguments
                sample_frequency = 60;
                cutoff_frequency = 10;
                
                x_gains = Control.Gains(.2, 0, .1);
                y_gains = Control.Gains(.2, 0, .1);
                z_gains = Control.Gains(2, 0, -1);
            end
            
            obj.x_pid = Control.PID(x_gains.kp, x_gains.ki, x_gains.kd, -12, 12, sample_frequency, cutoff_frequency);
            obj.y_pid = Control.PID(y_gains.kp, y_gains.ki, y_gains.kd, -12, 12, sample_frequency, cutoff_frequency);
            obj.z_pid = Control.PID(z_gains.kp, z_gains.ki, z_gains.kd, -1, 1, sample_frequency, cutoff_frequency);
        end
        
        function [ax, ay, az] = control(obj, target_state, current_state, dt)
            
            psi = current_state(6);
            ud = current_state(7);
            vd = current_state(8);
            wd = current_state(9);
            
            x_err = (target_state(1) - current_state(1));
            y_err = (target_state(2) - current_state(2));
            
            x_vd = (x_err * cos(psi) + y_err * sin(psi));
            y_vd = (y_err * cos(psi) - x_err * sin(psi));
            
            % x_err = (target_state(1) - current_state(1));
            % y_err = (target_state(2) - current_state(2));
            %
            % xd_b = (target_state(1) * cos(psi) + target_state(2) * sin(psi));
            % yd_b = (target_state(2) * cos(psi) - target_state(1) * sin(psi));
            %
            % x_b = (current_state(1) * cos(psi) + current_state(2) * sin(psi));
            % y_b = (current_state(2) * cos(psi) - current_state(1) * sin(psi));
            
            [ax, ~] = obj.x_pid.control(ud, x_vd, dt);
            [ay, ~] = obj.y_pid.control(vd, y_vd, dt);
            [az, ~] = obj.z_pid.control(target_state(3), current_state(3), dt);
        end
        
    end
    
end
