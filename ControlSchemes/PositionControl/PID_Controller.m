classdef PID_Controller 
    methods     ( Static = true )
        function [x_acc_d, pid_output, X_pid_err] = Xcontroller(X_pid, x_d, x, x_dot, dt)
            MAX_ACC = 900; % Max acceleration using thrust=value(0,255) assuming thrust>=thrustTrim(140)
                
            %% -------------------- FIRST PID BLOCK ------------------------------
            % Gains
            K_p = 125; % Shouldnt do anything with 100 (maybe 1 degree commanded) % 250,0,1 best for just commanding x
            K_i = 15; % 350,5,200 was last gains 300,10,100
            K_d = 50;
            
            % Proportional
            X_pid_err.x_curr_error = x_d - x;
            pid_p = K_p * X_pid_err.x_curr_error; 
            
            % Derivative
            X_pid_err.deriv = -K_d*x_dot; % taking off minus sign
            pid_d = X_pid_err.deriv;
            
            
            %Integral
            X_pid_err.x_cumm_error = X_pid.x_cumm_error + K_i*dt*(X_pid_err.x_curr_error);
            pid_i = X_pid_err.x_cumm_error;
        
                % Integral saturations
        %     if(pid_i > MAX_OUT)
        %         pid_i = MAX_OUT;
        %     end
        %     if(pid_i < -MAX_OUT)
        %         pid_i = -MAX_OUT;
        %     end
            
            % Store previous error
            X_pid_err.x_prev = X_pid_err.x_curr_error;
            
            % Final PID result
            pid_output = [pid_p, pid_i, pid_d];
            
            x_acc_d = pid_p + pid_i + pid_d;
            x_acc_d = min(max(-MAX_ACC, x_acc_d), MAX_ACC);
            
            
        end
        function [y_acc_d, pid_output, Y_pid_err] = Ycontroller(Y_pid, y_d, y, y_dot, dt)
            MAX_ACC = 900; % Max acceleration using thrust=value(0,255) assuming thrust>=thrustTrim(140)
            
            %% -------------------- FIRST PID BLOCK ------------------------------
            % Gains
            K_p = 125; % Shouldnt do anything with 100 (maybe 1 degree commanded)
            K_i = 15;
            K_d = 50; %
            
            % Proportional
            Y_pid_err.y_curr_error = y_d - y;
            pid_p = K_p * Y_pid_err.y_curr_error; 
            
            % Derivative
            Y_pid_err.deriv = -K_d*y_dot;
            pid_d = Y_pid_err.deriv;
            
            
            %Integral
            Y_pid_err.y_cumm_error = Y_pid.y_cumm_error + K_i*dt*(Y_pid_err.y_curr_error);
            pid_i = Y_pid_err.y_cumm_error;
            
                % Integral saturations
        %     if(pid_i > MAX_OUT)
        %         pid_i = MAX_OUT;
        %     end
        %     if(pid_i < -MAX_OUT)
        %         pid_i = -MAX_OUT;
        %     end
            
            % Store previous error
            Y_pid_err.y_prev = Y_pid_err.y_curr_error;
            
            % Final PID result
            pid_output = [pid_p, pid_i, pid_d];
            
            y_acc_d = pid_p + pid_i + pid_d;
            y_acc_d = min(max(-MAX_ACC, y_acc_d), MAX_ACC);
            
        end
        function [T, pid_output, Z_pid_err] = Zcontroller(Z_pid, z_d, z, z_dot, dt)
            MAX_OUT = 3;
            MAX_T = 255;
            
            %% -------------------- FIRST PID BLOCK ------------------------------
            % Gains
            K_p = 150; %60, Increasing Kp can definitely decrease oscillations... it did for on-board tuning
            K_i = 30; % 120,80,100
            K_d = 120;
            
            % Proportional
            Z_pid_err.z_curr_error = z_d - z;
            pid_p = K_p * Z_pid_err.z_curr_error; 
            
            % Derivative
	        % ***NOTE***
            %   Use (Z_pid_err.z_curr_error - Z_pid.z_curr_error) = (measurement - prevMeasurement) once we start changing the
	        %   reference signal
        %     Z_pid_err.deriv = (2*K_d/(2*tau + dt))*(Z_pid_err.z_curr_error - Z_pid.z_curr_error) + ((2*tau - dt)/(2*tau + dt))*Z_pid.deriv;
            Z_pid_err.deriv = -K_d*z_dot;
            pid_d = Z_pid_err.deriv;
            
            % Integral
        %     Z_pid_err.z_cumm_error = Z_pid.z_cumm_error + K_i*0.5*dt*(Z_pid_err.z_curr_error + Z_pid.z_curr_error);
            Z_pid_err.z_cumm_error = Z_pid.z_cumm_error + K_i*dt*(Z_pid_err.z_curr_error);
            pid_i = Z_pid_err.z_cumm_error;
            
            % Integral saturations
        %     if(pid_i > MAX_OUT)
        %         pid_i = MAX_OUT;
        %     end
        %     if(pid_i < -MAX_OUT)
        %         pid_i = -MAX_OUT;
        %     end
            
            % Store previous error
            Z_pid_err.z_prev = Z_pid_err.z_curr_error;
            
            % Final PID result
            pid_output = [pid_p, pid_i, pid_d];
            
            T = pid_p + pid_i + pid_d;
            T = uint8(min(max(0,T), 255));
            
            
            
            % Saturations
        %     output_n = min(max(0.001, output_z), MAX_OUT); % Apply saturations from 0-3(N)
        %     output_n = output_n*(MAX_T/MAX_OUT); % Convert to 0-255
        %     T = uint8(output_n);
            
        end
        function X_pid = Xpid_init(reset_pid, OUT_FREQ, CUT_OFF_FREQ_POS)
            persistent x_curr_error
            persistent vx_curr_error
            persistent x_cumm_error
            persistent x_prev
            persistent deriv
            
            lpf_data = Filter.lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_POS, 0.0);
            
            if reset_pid == 1
                x_curr_error = 0.0;
                vx_curr_error = 0.0;
                x_cumm_error = 0.0;
                x_prev = 0.0;
                deriv = 0.0;
            end
            
            X_pid.x_curr_error = x_curr_error;
            X_pid.vx_curr_error = vx_curr_error;
            X_pid.x_cumm_error = x_cumm_error;
            X_pid.x_prev = x_prev;
            X_pid.deriv = deriv;
            X_pid.lpf_data = lpf_data;
        end
        function Y_pid = Ypid_init(reset_pid, OUT_FREQ, CUT_OFF_FREQ_POS)  
            persistent y_curr_error
            persistent vy_curr_error
            persistent y_cumm_error
            persistent y_prev
            persistent deriv
            
            lpf_data = Filter.lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_POS, 0.0);
            
            if reset_pid == 1
                y_curr_error = 0.0;
                vy_curr_error = 0.0;
                y_cumm_error = 0.0;
                y_prev = 0.0;
                deriv = 0.0;
            end
            
            Y_pid.y_curr_error = y_curr_error;
            Y_pid.vy_curr_error = vy_curr_error;
            Y_pid.y_cumm_error = y_cumm_error;
            Y_pid.y_prev = y_prev;
            Y_pid.deriv = deriv;
            Y_pid.lpf_data = lpf_data;
        end
        function Z_pid = Zpid_init(reset_pid, OUT_FREQ, CUT_OFF_FREQ_POS)
            persistent z_curr_error
            persistent z_cumm_error % This is unique to Z controller
            persistent vz_curr_error
            persistent vz_cumm_error
            persistent vz_prev
            persistent deriv
            
            lpf_data = Filter.lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_POS, 0.0);
            
            if reset_pid == 1
                z_curr_error = 0.0;
                z_cumm_error = 0.0;
                vz_curr_error = 0.0;
                vz_cumm_error = 0.0;
                vz_prev = 0.0;
                deriv = 0.0;
            end
            
            Z_pid.z_curr_error = z_curr_error;
            Z_pid.z_cumm_error = z_cumm_error;
            Z_pid.vz_curr_error = vz_curr_error;
            Z_pid.vz_cumm_error = vz_cumm_error;
            Z_pid.vz_prev = vz_prev;
            Z_pid.deriv = deriv;
            Z_pid.lpf_data = lpf_data;
        end

    end
end